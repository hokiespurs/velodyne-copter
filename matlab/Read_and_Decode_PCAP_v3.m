function [Xa, Ya, Za, Ia, Ra, La, Aa] = Read_and_Decode_PCAP_v3(filename, Decimate_Rate, Total_Points)
% This function is meant to duplicate the function structure to compare
% output to the function provided by Dr. Craig Glennie.
[lidar,gps] = read_VLP16_pcap(filename,'npoints',Total_Points);
[Xa,Ya,Za]=sph2cart((90-lidar.az)*pi/180,lidar.el*pi/180,lidar.r);
Ia = lidar.I;
Ra = lidar.r;
La = lidar.el;
Aa = lidar.az;

if Decimate_Rate~=1
    ind = 1:Decimate_Rate:numel(Xa);
    Xa = Xa(ind);
    Ya = Ya(ind);
    Za = Za(ind);
    Ia = Ia(ind);
    Ra = Ra(ind);
    La = La(ind);
    Aa = Aa(ind);
end

end