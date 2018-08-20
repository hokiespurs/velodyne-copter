clear all; close all; clc;

PCAPNAMES = 'G:\20180808_DetroitLake_Bogdan\VLP16\LIDAR_raw_scanline_10.pcap';  %Select PCAP Files (raw lidar data) logged with RPi 
TRAJECTORYNAME = 'G:\20180808_DetroitLake_Bogdan\XNAV\002020\180807_002020.csv'; %Select Processed Trajectory
OUTPUTCSVNAME = 'G:\20180808_DetroitLake_Bogdan\Proc\LIDAR_scanline_10.csv';  %Specify processed lidar file location
OUTPUTTRAJCSVNAME = 'G:\20180808_DetroitLake_Bogdan\Proc\Trajectory_scanline_10.csv';    %Specify trimmed Trajectory location
PCAPOPT = {'rangegate',[2 200]};                                            %Specify Processing options
DOMAKEPLOTS = true;                                                         %If true matlab create plots of the dataset, if false no plots are generated
OUTPUTCSV = true;                                                          %If true matlab exports the processed lidar data, if false the data is only processed but not exported
OUTPUTTRAJCSV = true;                                                      %if true matlab will export the trimmed trajectory, false the trimmed trajectory will not be exported

velocopter(PCAPNAMES, TRAJECTORYNAME, OUTPUTCSVNAME, OUTPUTTRAJCSVNAME, ... %calls the custom matlab functions'velocopter' to georeference the acquired lidar data with the co-acquired trajectory information.
    'pcapopt', PCAPOPT, 'mkplots', DOMAKEPLOTS, 'outputcsv', OUTPUTCSV, ...
    'outputtrajcsv', OUTPUTTRAJCSV);



