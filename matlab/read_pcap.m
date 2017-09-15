function [lidar,gps] = read_pcap(filename,varargin)
% READ_PCAP Reads Velodyne VLP16 and HDL32 PCAP data
%   This file reads and decodes the velodyne lidar data from the pcap
%   format based on the details in this manual:
% https://github.com/hokiespurs/velodyne-copter/blob/master/doc/VLP-16%20User%20Manual%20and%20Programming%20Guide%2063-9243%20Rev%20A.pdf
%   * Note: The Acquisition Mode can not change from dual to strongest/last
%           within the pcap file... the algorithm could be improved here
%   * Code is based on 'Read_and_Decode_PCAP_v2.m' from Dr. Craig Glennie
%
% Required Inputs:
%	- filename     : pcap file that will be decoded 
%
% Optional Inputs:
%	- 'npoints'    : Number of points to return 
%	- 'debuglevel' : 0 (default)==no output, 1==status updates, 2==verbose
%	- 'rangegate'  : 2 element vector depicting minimum and maximum range
%	- 'azgate'     : n x 2 element vector depicting min and max azimuth
%   - 'maxtinterp' : time threshold to not interpolate azimuths over
%   
% Outputs:
%   - lidar : lidar structure from data packets 
%   - gps   : gps structure from GPRMC 
% 
% Examples:
%   - Example code goes here
% 
% Dependencies:
%   - interp1nanthresh.m
%   - loopStatus.m
% 
% Toolboxes Required:
%   - n/a
% 
% Author        : Richie Slocum
% Email         : richie@cormorantanalytics.com
% Date Created  : 08-Sep-2017
% Date Modified : 15-Sep-2017
% Github        : https://github.com/hokiespurs/velodyne-copter

%% Parse Inputs
[filename,npoints,debuglevel,rangegate,azgate,maxtinterp] = ...
    parseInputs(filename,varargin{:});

%% Constants
% lidar data
DATA_PORT  = 2368;
DATA_BYTES = 1214;
% gps position data
POS_PORT    = 8308;
POS_BYTES   = 520;

NBYTES_FS = 1.1; % factor of safety when using npoints
% in order to read npoints, the code takes a guess as to how much of the
% full file should be read in. Becuase pos takes some bytes that arent
% accounted for in the math, this factor of safety ensures more bytes than
% needed are read in.
%% Open and Read PCAP File
nbytes2read = (DATA_BYTES+50) + ceil((DATA_BYTES+50) * npoints /32/12 * NBYTES_FS);
allbytes = freadpcap(filename, nbytes2read, debuglevel);

%% Compute Indices of "data" and "pos" blocks
[ind_datablock, ind_posblock] = calc_pcap_inds(allbytes, npoints,...
    DATA_PORT, DATA_BYTES, POS_PORT, POS_BYTES, debuglevel);

%% Determine factory mode and scanner type from first data block
[factory_mode, factory_scanner] = ...
    get_pcap_factory(allbytes, ind_datablock, debuglevel);

%% Parse Data Blocks
lidar = get_pcap_data(allbytes, ind_datablock, factory_mode, ...
    factory_scanner, npoints, maxtinterp, debuglevel);

%% Parse Position Blocks
gps = get_pcap_pos(allbytes, ind_posblock, debuglevel);

%% Trim Data based on Range and Azimuth Gates
indgoodrange = lidar.r>=rangegate(1) & lidar.r<=rangegate(2);
indgoodaz = any(lidar.az>=azgate(:,1) & lidar.az<=azgate(:,2));
lidar.r = lidar.r(indgoodrange & indgoodaz);
lidar.az = lidar.az(indgoodrange & indgoodaz);
lidar.el = lidar.el(indgoodrange & indgoodaz);
lidar.I = lidar.I(indgoodrange & indgoodaz);
lidar.t = lidar.t(indgoodrange & indgoodaz);

end
%%
function allbytes = freadpcap(filename, nbytes, debuglevel)
% read all pcap data bytes into memory as "allbytes" variable

fid = fopenpcap(filename,debuglevel);

%% DEBUG
if debuglevel
   if nbytes==inf
       fprintf('%s... Reading All PCAP File Bytes(this can be slow)\n',...
           datestr(now));
   else
       fprintf('%s... Reading %.2f PCAP File MBytes(this can be slow)\n',...
           datestr(now), nbytes/1e6);
   end
end

%% Reading Data

fseek(fid,24,'bof'); % skip pcap global header
allbytes = fread(fid,nbytes,'*uint8');
fclose(fid);

end

function fid = fopenpcap(filename, debuglevel)
% open pcap file and send error on failure

[fid,errmsg] = fopen(filename,'r','b'); %explicitly declare big endian
if fid < 0 
   fprintf('Unable to Read File: %s',filename);
   error(errmsg)
end

if debuglevel
   fprintf('%s... Opened PCAP File: %s\n',datestr(now),filename) 
end

end

function [dataind,posind] = calc_pcap_inds(allbytes,ndesiredpoints,...
    DATA_PORT, DATA_BYTES, POS_PORT, POS_BYTES, debuglevel)
% parses through the raw bytes from a pcap file and extracts the index
% values for the beginning of the data blocks and position blocks
%
% * function will stop when npoints have been read in
%
% indices are preallocated for speed, and nans are removed at the end...
% I'd rather delete extra indices than have the variable changing size

%% preallocate indices
% these may preallocate too large, but will be trimmed at the end
nbytes = numel(allbytes);

if ndesiredpoints == inf
    max_points = ceil(nbytes/(DATA_BYTES+50)); % 50 = header
else
    max_points = ceil(ndesiredpoints/32/12); 
end

dataind = nan(max_points,1);
posind  = nan(max_points,1);

%% DEBUG
if debuglevel
    fprintf('%s... Looping through to compute indices\n',datestr(now));
end
%% Loop through data computing indices
ifilepos = 0;
ndatapoints = 0;
npospoints = 0;

nstatusupdates = 11;
nextstatus = 1/nstatusupdates * nbytes;

%loop until end of file, or until gathered <ndesiredpoints> indices
startTime = now;
while (ifilepos + 59) < nbytes && (ndatapoints*32*12) < ndesiredpoints
    sourceport   = typecast(allbytes([ifilepos+52 ifilepos+51]),'uint16');
    npacketbytes = typecast(allbytes([ifilepos+56 ifilepos+55]),'uint16');
    
    if sourceport == DATA_PORT && npacketbytes == DATA_BYTES % lidar data
        ndatapoints = ndatapoints + 1;
        dataind(ndatapoints) = ifilepos + 59;
    elseif sourceport == POS_PORT && npacketbytes == POS_BYTES % pos data
        npospoints = npospoints + 1;
        posind(npospoints) = ifilepos + 59;
    else
        if debuglevel==2
            fprintf('Unknown Packet... Filepos: %.010i Port: %05.0f nBytes = %05.0f\n',ifilepos,sourceport,npacketbytes);
        end
    end
    ifilepos = ifilepos + 50 + double(npacketbytes);
    if ifilepos>nextstatus
        if debuglevel==2
            fprintf('\t');
            loopStatus(startTime,ifilepos,nbytes,0);
            nextstatus = nextstatus + 1/nstatusupdates * nbytes;
        end
    end
end

%% Remove Nans from indices
dataind(isnan(dataind)) = [];
posind(isnan(posind)) = [];

%% DEBUG
if debuglevel
    fprintf('%s... Finished Index Calculations\n',datestr(now));
end

end

function [factory_mode, factory_scanner] = ...
    get_pcap_factory(allbytes,ind,debuglevel)
% Determines the factory mode and scanner from the first data block
% Returns the values as strings
% 
% Factory Mode:
%   37h : strongest 
%   38h : last   
%   39h : dual  
%
% Factory Scanner: 
%   21h : HDL-32E 
%   22h : VLP-16

ind_factory_mode    = ind(1)+1204;
ind_factory_scanner = ind(1)+1205;

factory_mode_byte    = allbytes(ind_factory_mode);
factory_scanner_byte = allbytes(ind_factory_scanner);

switch factory_mode_byte
    case hex2dec('37')
        factory_mode = 'strongest';
    case hex2dec('38')
        factory_mode =  'last';
    case hex2dec('39')
        factory_mode =  'dual';
    otherwise
        error('Unknown Factory Mode Byte: %.0f',factory_mode_byte);
end

switch factory_scanner_byte
    case hex2dec('21')
        factory_scanner = 'HDL-32E';
    case hex2dec('22')
        factory_scanner = 'VLP-16';
    otherwise
        error('unknown factory scanner byte: %.0f',factory_scanner_byte);
end
%% DEBUG
if debuglevel
    fprintf('%s... Factory Mode: %s\n',datestr(now), factory_mode);
    fprintf('%s... Factory Scanner: %s\n',datestr(now), factory_scanner);
end

end

function lidar = get_pcap_data(allbytes, ind, factory_mode, ...
    factory_scanner, npoints, maxtinterp, debuglevel)
% parses the velodyne data blocks
% returns structure 'lidar' with: time, azimuth, elevation, range

%% DEBUG
if debuglevel
    fprintf('%s... Computing Data [Time]\n',datestr(now));
end

%% Time Bytes (t) *time is used for azimuth interpolation
% size(t) = [100, 12, npackets]
t = get_pcap_time(allbytes,ind,factory_mode,factory_scanner);

%% DEBUG
if debuglevel
    fprintf('%s... Computing Data [Az, El, R, I]\n',datestr(now));
end

%% Main Data Packet 1200 Bytes (az,el,r,I)
[az,el,r,I]=get_pcap_sphericalcoords(allbytes,ind,t,factory_scanner,...
    maxtinterp,debuglevel);

if npoints~=inf
    lidar.az = az(1:npoints); % azimuth in degrees
    lidar.el = el(1:npoints); % elevation in degrees
    lidar.r  = r(1:npoints);  % range in meters
    lidar.I  = I(1:npoints);  % reflectance {uint8}
    lidar.t  = t(1:npoints);  % time in seconds
else
    lidar.az = az(:); % azimuth in degrees
    lidar.el = el(:); % elevation in degrees
    lidar.r  = r(:);  % range in meters
    lidar.I  = I(:);  % reflectance {uint8}
    lidar.t  = t(:);  % time in seconds
end

end

function t = get_pcap_time(allbytes,ind,factory_mode,factory_scanner)
% computes time for each data packet
% 
% 'Dual' mode returns two data packets for each pulse, therefore packets
% [1-2,3-4,5-6,7-8,9-10,11-12] each have the same time offset

ndatapackets = numel(ind);

%% Compute Time for each Packet
% 4 x npackets
ind_time_bytes = repmat(permute(ind(:),[2 1]),4,1) + ...
    repmat((1200:1203)',1,ndatapackets);
t_packet = typecast(allbytes(ind_time_bytes(:)),'uint32');

%% Compute Offset Time for each packet
if strcmp(factory_scanner,'VLP-16')
    if strcmp(factory_mode,'dual')
        % 32 x 12
        t_r = kron(kron(reshape(0:11,2,6),ones(1,2)),ones(16,1));
        t_c = repmat([0:15 0:15]',1,12);
    else
        % 32 x 12
        t_r = kron(reshape(0:23,2,12),ones(16,1));
        t_c = repmat([0:15 0:15]',1,12);
    end
    t_off = t_r * 55.296 + t_c * 2.304;
elseif strcmp(factory_scanner,'HDL-32E')
    if strcmp(factory_mode,'dual')
        % 32 x 12
        t_r = kron(kron(0:5,ones(1,2)),ones(32,1));
        t_c = repmat([0:31]',1,12);
    else
        % 32 x 12
        t_r = kron(0:11,ones(32,1));
        t_c = repmat([0:31]',1,12);
    end
    t_off = t_r * 46.080 + t_c * 1.152;
end
%% Compute Time for each data point
% 32 x 12 x npackets
t = repmat(permute(double(t_packet(:)),[3 2 1]),32,12,1) + ...
    repmat(t_off,1,1,ndatapackets);
t = t/1000000; %conver to seconds
end

function [az,el,r,I]=get_pcap_sphericalcoords(allbytes,ind,t,...
    factory_scanner,maxtinterp,debuglevel)
% Computes the azimuth, elevation, and range for each data packet

%% Compute Range
if debuglevel
   fprintf('%s... Computing Range\n',datestr(now)) 
end
r = get_pcap_r(allbytes,ind);

%% Compute Intensity
if debuglevel
   fprintf('%s... Computing Intensity\n',datestr(now)) 
end
I = get_pcap_I(allbytes,ind);

%% Compute Elevation Angle
if debuglevel
   fprintf('%s... Computing Elevation Angle\n',datestr(now)) 
end
el = get_pcap_el(numel(ind),factory_scanner);

%% Compute Azimuth Angle
if debuglevel
   fprintf('%s... Computing Azimuth Angle\n',datestr(now)) 
end
az_raw = get_pcap_az(allbytes,ind);

%% Interpolate Azimuth 
if debuglevel
   fprintf('%s... Interpolating Azimuth Angle\n',datestr(now)) 
end
az = interpAz(az_raw,t,maxtinterp);
if debuglevel==2
   %% 
   figure(114);clf
   nonansind = ~isnan(az_raw(:));
   tazraw = sortrows([t(nonansind) az_raw(nonansind)]);
   plot(tazraw(1:end-1,1),diff(tazraw(:,1)),'.-');
   hold on
   plot([min(tazraw(:,1)) max(tazraw(:,1))],[maxtinterp maxtinterp],'r-');
end
end

function r = get_pcap_r(allbytes,ind)
% compute range data (meters) from pcap data given the index for the start
% of each data packet

ndatapackets = numel(ind);
% 100 x 12 x npackets
ind_r = repmat(reshape([4:3:99;5:3:99],64,1),1,12,ndatapackets) + ...
    repmat(0:100:1100,64,1,ndatapackets) + ...
    repmat(permute(ind(:),[3 2 1]),64,12,1);

r_raw = reshape(typecast(allbytes(ind_r(:)),'uint16'),32,12,ndatapackets);

% convert to meters
r = double(r_raw) * 0.002;

end

function I = get_pcap_I(allbytes,ind)
% compute uint8 reflectance data from pcap data given the index for the start
% of each data packet

ndatapackets = numel(ind);
% 100 x 12 x npackets
ind_I = repmat(reshape(6:3:100,32,1),1,12,ndatapackets) + ...
    repmat(0:100:1100,32,1,ndatapackets) + ...
    repmat(permute(ind(:),[3 2 1]),32,12,1);

I = reshape(typecast(allbytes(ind_I(:)),'uint8'),32,12,ndatapackets);

end

function el = get_pcap_el(ndatapackets,factory_scanner)
% compute elevation angle (degrees) data from pcap data given the index for
% the start of each data packet

if strcmp(factory_scanner,'VLP-16')
    VLP16VERTANGLE = [-15 1 -13 -3 -11 5 -9 7 -7 9 -5 11 -3 13 -1 15]';
    el = repmat(VLP16VERTANGLE,2,12,ndatapackets);
elseif strcmp(factory_scanner,'HDL-32E')
    warning('untested for HDL-32E...remove warning once tested');
    VLP32VERTANGLE = zeros(32,1);
    VLP32VERTANGLE(1:2:32) = -92/3:4/3:-32/3;
    VLP32VERTANGLE(2:2:32) = -28/3:4/3:32/3;
    el = repmat(VLP32VERTANGLE,2,12,ndatapackets);
end

end

function az = get_pcap_az(allbytes, ind)
% compute interpolated azimuth values (degrees) from pcap data given the 
% given the index for the start of each data packet
%
% the time of each point is used to assist with the interpolation of data
ndatapackets = numel(ind);

ind_az = repmat(reshape(2:3,2,1),1,12,ndatapackets) + ...
    repmat(0:100:1100,2,1,ndatapackets) + ...
    repmat(permute(ind(:),[3 2 1]),1,12,1);

reported_az = reshape((typecast(allbytes(ind_az(:)),'uint16')),1,12,ndatapackets);

az = nan(32,12,ndatapackets);
az(1,:,:)=reported_az;
az = az/100; %convert to degrees

end

function az = interpAz(az_raw,t,maxtinterp)
% interpolate azimuth for each time
%
% The interpolation method is to:
%  - convert angle from degrees to x,y vector components
%  - interpolate x and y vector components 
%        * with a threshold to avoid interpolating over large gaps
%  - convert vector components back to angle
% 
% There is some inaccuracy in this method, but the average daz = 0.4 deg
%
% run 'showinterperr" to prove that the error is negligible for small gaps

indgood = ~isnan(az_raw(:));
t_az_unique = unique([t(indgood) az_raw(indgood)],'rows');
t_az_unique_sorted = sortrows(t_az_unique,1);

az = interpVectorAz(t_az_unique_sorted(:,1),t_az_unique_sorted(:,2),t,...
    maxtinterp);

end

function azi = interpVectorAz(t,az,ti,maxtinterp)
% interpolate an azimuth value across 360 degrees by using vectors
% az is input in degrees
% see 'showinterperr' to show that this is ok for small angles
% for a dAngle = 0.4 degrees, the max error is 0.0000003127 degrees

vec_x = cosd(az);
vec_y = sind(az);

vec_x_i = interp1nanthresh(t,vec_x,ti,maxtinterp,inf); 
vec_y_i = interp1nanthresh(t,vec_y,ti,maxtinterp,inf); 

azi = atan2d(vec_y_i,vec_x_i);
azi(azi<0)=azi(azi<0)+360;
end

function showinterperr
%% Run this function to show the error associated with the interp method
interpgap = 0.4*10;
t = 0:interpgap:359.99;
anglevals = 0:interpgap:359.99;
ti = 0:.001:270;

trueangles = interp1(t,anglevals,ti);

xvector = cosd(anglevals);
yvector = sind(anglevals);

xivector = interp1(t,xvector,ti);
yivector = interp1(t,yvector,ti);
vectorangles = atan2d(yivector,xivector);

vectorangles(vectorangles<=0)=vectorangles(vectorangles<=0)+360;
trueangles(trueangles<=0)=trueangles(trueangles<=0)+360;

figure(14)
angleerror = trueangles-vectorangles;
plot(ti,trueangles-vectorangles);
xlabel('actual angle (degrees)','interpreter','latex');
ylabel('error (degrees)','interpreter','latex');
titlestr{1} = sprintf('With an interp gap of %.3f degrees',interpgap);
titlestr{2} = sprintf('the max error is %.10f degrees',max(abs(angleerror)));

title(titlestr,'interpreter','latex','fontsize',14);
end

function gps = get_pcap_pos(allbytes, ind_posblock, debuglevel)
% Read GPS info from the pos part of the pcap file
warning('POS packet is not being parsed yet...');
%*% Need to:
%  - parse out the RMC NMEA
%  - report a flag for if time should be adjusted to UTC using timestamp
%  - parse out gyro, accel, temperature

%adjusted
% read time
gps.t = get_pcap_pos_time(allbytes, ind_posblock);

% read nmea
gps.nmea = get_pcap_pos_nmea(allbytes, ind_posblock);

gps.rmc = parse_rmc_nmea(gps.nmea);

% parse gyro, accel, temperature fields 

end

function rmc = parse_rmc_nmea(nmea)
rmc = nan;
end

function t = get_pcap_pos_time(allbytes, ind_posblock)
npackets = numel(ind_posblock);

t_ind = 198:201;

ind = repmat(permute(ind_posblock(:),[2 1]),numel(t_ind),1) + ...
           repmat(t_ind',1,npackets);

t = double(typecast(allbytes(ind(:)),'uint32'))/1e6;

end

function nmea = get_pcap_pos_nmea(allbytes,ind_posblock)
% read nmea data from pcap file
npackets = numel(ind_posblock);

nmea_ind = 206:277;

ind = repmat(permute(ind_posblock(:),[1 2]),1,numel(nmea_ind)) + ...
           repmat(nmea_ind,npackets,1);

nmea = char(allbytes(ind));
end

function [filename,npoints,debuglevel,rangegate,azgate,maxtinterp] = ...
    parseInputs(filename,varargin)
%%	 Call this function to parse the inputs

% Default Values
default_npoints     = inf;
default_debuglevel  = 0;
default_rangegate   = [0.5 130]; %per manual
default_azgate      = [0 360];
default_maxtinterp  = 0.002;

% Check Values
check_filename    = @(x) exist(x,'file');
check_npoints     = @(x) isnumeric(x) & x>0;
check_debuglevel  = @(x) ismember(x,[0 1 2]);
check_rangegate   = @(x) size(x,2)==2;
check_azgate      = @(x) size(x,2)==2;
check_maxtinterp  = @(x) isnumeric(x) & x>0;

% Parser Values
p = inputParser;
% Required 
addRequired(p, 'filename' , check_filename );
% Optional 
% Parameter 
addParameter(p, 'npoints'    , default_npoints   , check_npoints    );
addParameter(p, 'debuglevel' , default_debuglevel, check_debuglevel );
addParameter(p, 'rangegate'  , default_rangegate , check_rangegate  );
addParameter(p, 'azgate'     , default_azgate    , check_azgate     );
addParameter(p, 'maxtinterp' , default_maxtinterp, check_maxtinterp );

% Parse
parse(p,filename,varargin{:});
% Convert to variables
filename   = p.Results.('filename');
npoints    = p.Results.('npoints');
debuglevel = p.Results.('debuglevel');
rangegate  = p.Results.('rangegate');
azgate     = p.Results.('azgate');
maxtinterp = p.Results.('maxtinterp');
end