function xyz_final = velocopter(pcap_filename,trajectory_filename,output_filename,varargin)
% VELOCOPTER Short summary of this function goes here
%   Detailed explanation goes here
% 
% Required Inputs:
%	 - pcap_filename         : filename of raw lidar pcap data 
%	 - trajectory_filename   : csv of postprocessed trajcetory (RTpostprocess) 
%	 - output_filename       : csv for output lidar data 
% Optional Inputs:
%	 - 'pcapopt'             : cell of optional inputs to readpcap 
%	 - 'trajcolnum'          : column numbers from trajectory file 
%	 - 'mkplots'             : boolean for whether or not to make plots 
%	 - 'boresightvals'       : boresight from IMU to lidar
%    - 'outputcsv'           : boolean for whether or not to write csv
%
% Outputs:
%   - xyz_final : georectified xyz pointcloud
%   - Saveas a CSV file with the name <output_filename> 
% 
% Examples:
%   - n/a
% 
% Dependencies:
%   - deg2utm.m
%   - interp1nanthresh.m
%   - loopStatus.m
%   - read_pcap.m
% 
% Toolboxes Required:
%   - n/a
% 
% Author        : Chase Simpson
% Email         : simpsoch@oregonstate.edu
% Date Created  : 12-Oct-2017
% Date Modified : 12-Oct-2017
% Github        : 

%% Parse Inputs
[pcap_filename,trajectory_filename,output_filename,pcapopt,trajcolnum,mkplots,boresightvals,outputcsv] = ...
    parseInputs(pcap_filename,trajectory_filename,output_filename,varargin{:});
fprintf('%-35s : %s\n','Parse Inputs Complete',datestr(now));

%% readpcap (x,y,z,i,t) in SOCS
[x_socs,y_socs,z_socs,I,tlidar] = readXYZITpcap(pcap_filename,pcapopt{:});
fprintf('%-35s : %s\n','Read PCAP Complete',datestr(now));

%% read trajectory
[timu,lat,long,alt,head,pitch,roll,latstd,longstd,altstd,headstd,pitchstd,rollstd] = readTRAJECTORY(trajectory_filename,trajcolnum);
fprintf('%-35s : %s\n','Read Trajectory Complete',datestr(now));

%% convert lat long to utm
[UTM_X,UTM_Y] = deg2utm(lat,long);
fprintf('%-35s : %s\n','lat-lon to UTM Complete',datestr(now));

%% apply the boresight
[xyz_imu] = boresight(boresightvals,x_socs,y_socs,z_socs);
fprintf('%-35s : %s\n','Boresight Complete',datestr(now));

%% interpolate trajectory per point
[int_x_utm,int_y_utm,int_z,int_roll,int_pitch,int_head] = interp_pos(UTM_X,UTM_Y,alt,head,pitch,roll,timu,tlidar);
fprintf('%-35s : %s\n','Interpolate Trajectory Complete',datestr(now));

%% apply the trajectory for each point
xyz_final = geolocation(xyz_imu,int_x_utm,int_y_utm,int_z,int_roll,int_pitch,int_head);
fprintf('%-35s : %s\n','Geolocation Complete',datestr(now));

%% output pointcloud
% x,y,z,I,relative Time (seconds)
if outputcsv
    fprintf('%-35s : %s\n','Starting CSV Output...',datestr(now));
    outputlidarcsv(output_filename,[xyz_final double(I(:)) (tlidar(:)-tlidar(1))*24*60*60]);
    fprintf('%-35s : %s\n','CSV Output Complete',datestr(now));
else
    fprintf('Not outputting a csv. If desired, change "outputcsv" argument to true\n');
end
%% output optional plots
% default to save in the same directory as the pcap file
if mkplots
    % determine save directory
    if isstruct(pcap_filename)
       dname = fileparts(pcap_filename{1}); 
    else
       dname = fileparts(pcap_filename);
    end
    
    % make x,y,vertical accuracy plot
    makeXYZstd(UTM_X,UTM_Y,altstd,dname);
    
end


end

function makeXYZstd(UTM_X,UTM_Y,altstd,dname)
%% Makes a scatter plot of UTMX, UTMY, and colored by Std Error of Elevatoin 
    f = figure;
    minX = round(min(UTM_X(:)));
    scatter(UTM_X-minX,UTM_Y-min(UTM_Y(:)),10,altstd,'filled');
    axis equal;
    axis tight;
    grid on
    
    xlabel('x','fontsize',16,'interpreter','latex');
    ylabel('y','fontsize',16,'interpreter','latex');
    title('ALHDFSDJKFHSDJKFH','fontsize',20,'interpreter','latex');
    
    c = colorbar;
    h = ylabel(c,'$\sigma_Z$','fontsize',16,'interpreter','latex','Rotation',0);
    
    xticks(0:10:200)
    yticks(0:10:100)
    
%     xticklabels = get(gca,'XTickLabel');
%     xticklabels{1} = num2str(minX);
%     set(gca,'XTickLabel',xticklabels)
    savename = [dname '/XYdZ.png'];
    saveas(f,savename);
end


function outputlidarcsv(output_filename,xyzit)
%% output a csv file with column titles
% x, y, z, Intensity, time
fid = fopen(output_filename,'w+t');
fprintf(fid,'%s,%s,%s,%s,%s\n','x,y,z,Intensity,time\n');
fprintf(fid,'%.3f,%.3f,%.3f,%d,%.10f\n',xyzit');
fclose(fid);

end

function xyz_final = geolocation(xyz_imu,int_x_utm,int_y_utm,int_z,int_roll,int_pitch,int_head)
%% Used to convert VLP16 Data from IMURF to MappingRF
   
r11 = cosd(int_pitch).*sind(int_head);
r12 = -1*cosd(int_head).*cosd(int_roll)-sind(int_head).*sind(int_pitch).*sind(int_roll);
r13 = cosd(int_head).*sind(int_roll)-sind(int_head).*sind(int_pitch).*cosd(int_roll);
r21 = cosd(int_pitch).*cosd(int_head);
r22 = sind(int_head).*cosd(int_roll)-cosd(int_head).*sind(int_pitch).*sind(int_roll);
r23 = -1*sind(int_head).*sind(int_roll)-cosd(int_head).*sind(int_pitch).*cosd(int_roll);
r31 = sind(int_pitch);
r32 = cosd(int_pitch).*sind(int_roll);
r33 = cosd(int_pitch).*cosd(int_roll);

npts = numel(r11);
xyz_final = nan(npts,3);
%(:) is used to ensure proper dimensions
xyz_final(:,1) = r11(:).*xyz_imu(:,1) + r12(:).*xyz_imu(:,2) + r13(:).*xyz_imu(:,3) + int_x_utm(:);
xyz_final(:,2) = r21(:).*xyz_imu(:,1) + r22(:).*xyz_imu(:,2) + r23(:).*xyz_imu(:,3) + int_y_utm(:);
xyz_final(:,3) = r31(:).*xyz_imu(:,1) + r32(:).*xyz_imu(:,2) + r33(:).*xyz_imu(:,3) + int_z(:);

% IF CODE CRASHES FOR MEMORY IMPLEMENT LIKE THIS, YO
% X = cosd(int_pitch).*sind(int_head).*x + ...
%     -1*cosd(int_head).*cosd(int_roll)-sind(int_head).*sind(int_pitch).*sind(int_roll).*y + ...
%     cosd(int_head).*sind(int_roll)-sind(int_head).*sind(int_pitch).*cosd(int_roll).*z + int_x_utm;

end

function [int_x_utm,int_y_utm,int_z,int_roll,int_pitch,int_head] = interp_pos(UTM_X,UTM_Y,alt,head,pitch,roll,timu,tlidar)
%% Interpolate the IMU data for all VLP16 Data
    int_x_utm = interp1(timu,UTM_X,tlidar);
    int_y_utm = interp1(timu,UTM_Y,tlidar);
    int_z = interp1(timu,alt,tlidar);
    int_roll = interp1(timu,roll,tlidar);
    int_pitch = interp1(timu,pitch,tlidar);
    int_head = interp1(timu,head,tlidar);
      
end

function [xyz_imu] = boresight(boresightvals,x_socs,y_socs,z_socs)
%% Used to convert VLP16 Data from SCRF to IMURF

FLIP = [0 -1 0;     %if there is no redefinition neccassary replace with a
        1 0 0;      % 3x3 identity matrix
        0 0 1];

FLIP2 = [0 0 1;1 0 0;0 1 0];
    
    
tx      = boresightvals(1);
ty      = boresightvals(2);
tz      = boresightvals(3);
rotx    = boresightvals(4);
roty    = boresightvals(5);
rotz    = boresightvals(6);

%Rotation Matrix
Rx = [1 0 0;0 cosd(rotx) sind(rotx);0 -sind(rotx) cosd(rotx)];
Ry = [cosd(roty) 0 -sind(roty);0 1 0;sind(roty) 0 cosd(roty)];
Rz = [cosd(rotz) sind(rotz) 0;-sind(rotz) cosd(rotz) 0;0 0 1];   
    
xyz = Rz*Ry*Rx * [x_socs y_socs z_socs]';

%redefines axis 
xyz = (FLIP * xyz)';

%Translation  
xyz_imu = nan(size(xyz,1),3);
xyz_imu(:,1) = (xyz(:,1)+(tx/1000));    %Divided by 1000 to  get to meters
xyz_imu(:,2) = (xyz(:,2)+(ty/1000));
xyz_imu(:,3) = (xyz(:,3)+(tz/1000));

xyz_imu = (FLIP2*xyz_imu')';

end

function [x,y,z,I,t] = readXYZITpcap(pcap_filename,varargin) 
%% reads pcap data in using readpcap
% handles both string input and cell array of strings
if iscell(pcap_filename)
    nfiles = numel(pcap_filename);
    
    npts = nan(nfiles,1);
    for i = 1:nfiles
       lidar(i) = readpcap(pcap_filename,varargin{:});
       npts(i) = numel(lidar(i).x);
    end
    totalpts = sum(npts);
    
    % preallocate
    x = nan(totalpts,1);
    y = nan(totalpts,1);
    z = nan(totalpts,1);
    I = nan(totalpts,1);
    t = nan(totalpts,1);
    
    indpts = cumsum([0 npts]);
    
    for i = 1:nfiles
       ind = indpts(i)+1:indpts(i+1);
       x(ind)=lidar(i).x;
       y(ind)=lidar(i).y;
       z(ind)=lidar(i).z;
       I(ind)=lidar(i).I;
       t(ind)=lidar(i).t;
    end
    
else % single pcap file as string or character array
    lidar = read_pcap(pcap_filename,varargin{:});
    x = lidar.x;
    y = lidar.y;
    z = lidar.z;
    I = lidar.I;
    t = lidar.t;
end

end

function [timu,lat,long,alt,head,pitch,roll,latstd,longstd,altstd,headstd,pitchstd,rollstd] = readTRAJECTORY(trajectory_filename,trajcolnum)
%% reads in trajectory using importdata
IMUraw = importdata(trajectory_filename);

date     = IMUraw.textdata(2:end,trajcolnum(1));
time     = IMUraw.textdata(2:end,trajcolnum(2));

convertme = strcat(date,time);
timu = datenum(convertme,'m/dd/yyyyHH:MM:SS.FFF');


lat      = IMUraw.data(:,trajcolnum(3));
long     = IMUraw.data(:,trajcolnum(4));
alt      = IMUraw.data(:,trajcolnum(5));
head     = IMUraw.data(:,trajcolnum(6));
pitch    = IMUraw.data(:,trajcolnum(7));
roll     = IMUraw.data(:,trajcolnum(8));
latstd   = IMUraw.data(:,trajcolnum(9));
longstd  = IMUraw.data(:,trajcolnum(10));
altstd   = IMUraw.data(:,trajcolnum(11));
headstd  = IMUraw.data(:,trajcolnum(12));
pitchstd = IMUraw.data(:,trajcolnum(13));
rollstd  = IMUraw.data(:,trajcolnum(14));
end

function [pcap_filename,trajectory_filename,output_filename,pcapopt,trajcolnum,mkplots,boresightvals,outputcsv] = parseInputs(pcap_filename,trajectory_filename,output_filename,varargin)

% Default Values
default_pcapopt        = {'rangegate',[2 100]};
%txtdate,txttime,lat,lon,alt,head,pitch,roll,latstd,lonstd,altstd,headstd,pitchstd,rollstd
default_trajcolnum     = [1 2 1 2 3 15 16 17 18 19 20 24 25 26]; 
default_mkplots        = true;
% x,y,z,rx,ry,rz (mm,degrees)
default_boresightvals  = [-38.956 21.296 118.983 0 -15 0]; 
default_outputcsv    = true;
% Check Values
check_pcap_filename        = @(x) ischar(x) | isstring(x) | iscell(x);
check_trajectory_filename  = @(x) ischar(x) | isstring(x) ;
check_output_filename      = @(x) (ischar(x) | isstring(x)) & strcmp(x(end-3:end),'.csv');
check_pcapopt              = @(x) iscell(x);
check_trajcolnum           = @(x) isnumeric(x) & numel(x)==14;
check_mkplots              = @(x) islogical(x) & numel(x)==1;
check_boresightvals        = @(x) isnumeric(x) & numel(x)==6;
check_outputcsv            = @(x) islogical(x);

% Parser Values
p = inputParser;
% Required Arguments
addRequired(p, 'pcap_filename'       , check_pcap_filename       );
addRequired(p, 'trajectory_filename' , check_trajectory_filename );
addRequired(p, 'output_filename'     , check_output_filename     );
% Parameter Arguments
addParameter(p, 'pcapopt'       , default_pcapopt      , check_pcapopt       );
addParameter(p, 'trajcolnum'    , default_trajcolnum   , check_trajcolnum    );
addParameter(p, 'mkplots'       , default_mkplots      , check_mkplots       );
addParameter(p, 'boresightvals' , default_boresightvals, check_boresightvals );
addParameter(p, 'outputcsv'     , default_outputcsv    , check_outputcsv     );

% Parse
parse(p,pcap_filename,trajectory_filename,output_filename,varargin{:});
% Convert to variables
pcap_filename       = p.Results.('pcap_filename');
trajectory_filename = p.Results.('trajectory_filename');
output_filename     = p.Results.('output_filename');
pcapopt             = p.Results.('pcapopt');
trajcolnum          = p.Results.('trajcolnum');
mkplots             = p.Results.('mkplots');
boresightvals       = p.Results.('boresightvals');
outputcsv           = p.Results.('outputcsv');
end