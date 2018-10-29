clear all; close all; clc;

%Point to PCAP file
PCAPNAMES = 'P:\Simpson_share\Projects\LidarIntegration\07_Data\20180417_PactransDemo\VLP16\LIDAR_raw_scanline_3.pcap';
%Point to trajectory.csv (exported from RT-PostProcess)
TRAJECTORYNAME = 'P:\Simpson_share\Projects\LidarIntegration\07_Data\20180417_PactransDemo\XNAV\180417_214521.csv';
%Specify File name/location for georeferenced data to be saved (.csv)
OUTPUTCSVNAME = 'P:\Simpson_share\Projects\LidarIntegration\07_Data\20180417_PactransDemo\Proc\LIDAR_scanline_3.csv';
%Specify File name/location for flightline trajectory file to be saved (.csv)
OUTPUTTRAJCSVNAME = 'P:\Simpson_share\Projects\LidarIntegration\07_Data\20180417_PactransDemo\ProcTrajectory_3.csv';

%%***********************************************************************
PCAPOPT = {'rangegate',[2 200]};
DOMAKEPLOTS = true;
OUTPUTCSV = true; 
OUTPUTTRAJCSV = true;

velocopter(PCAPNAMES, TRAJECTORYNAME, OUTPUTCSVNAME, OUTPUTTRAJCSVNAME, ...
    'pcapopt', PCAPOPT, 'mkplots', DOMAKEPLOTS, 'outputcsv', OUTPUTCSV, ...
    'outputtrajcsv', OUTPUTTRAJCSV);