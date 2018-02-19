clear all; close all; clc;

% Point to raw lidar PCAP File
PCAPNAMES = ['P:\Simpson_share\Projects\LidarIntegration\07_Data\'...
     '20170926_KittyHawk\VLP16\3_LIDAR_raw_03_25_2015_11_17_44_v98e7.pcap'];
% Point to processed trajectory exported from RT PostProcess 
TRAJECTORYNAME = ['P:\Simpson_share\Projects\LidarIntegration\07_Data\'...
     '20170926_KittyHawk\XNAV\PP_RINEX\170926_212250_Rinex.csv';]
% Specify output location for georeferenced lidar data
OUTPUTCSVNAME = ['P:\Simpson_share\Projects\LidarIntegration\07_Data\'...
     '20170926_KittyHawk\PROC\20170926_modifiedControls_flight3.csv'];
%Variable options for processing
PCAPOPT = {'rangegate',[2 200]};
DOMAKEPLOTS = true;
OUTPUTCSV = true;

%Run velocopter geolocation script
velocopter(PCAPNAMES, TRAJECTORYNAME, OUTPUTCSVNAME, ...
    'pcapopt', PCAPOPT, 'mkplots', DOMAKEPLOTS, 'outputcsv', OUTPUTCSV);