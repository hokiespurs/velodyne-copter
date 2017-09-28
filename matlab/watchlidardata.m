function watchlidardata(lidar, tincr, doI)
%%
R = [0 0 1;0 1 0;1 0 0];
SCATCAX = [0 100];
SCATSIZE = 10;
AX = [-100 100 -100 100 -100 100];
tincrsec = .1;
doI = false;
%
tincr = tincrsec/60/60/24;

tlow = min(lidar.t);
thigh = max(lidar.t);
tind = tlow:tincr:thigh;

figure(100);clf
for i=1:numel(tind)-1
indstart = find(lidar.t>tind(i),1,'first');
indend   = find(lidar.t<tind(i+1),1,'last');

ind = indstart:indend;

xyz = [lidar.x(ind)'; lidar.y(ind)'; lidar.z(ind)'];

xyzR = R * xyz;
if doI
    scatter3(xyzR(1,:),xyzR(2,:),xyzR(3,:),SCATSIZE,lidar.I(ind),'filled');
    caxis(SCATCAX);
else
    plot3(xyzR(1,:),xyzR(2,:),xyzR(3,:),'b.');
end
title([datestr(tind(i),'yyyymmdd-HH:MM:ss.fff') ' + ' sprintf('%.3f',tincrsec) 's'])
axis(AX);
drawnow

end

end