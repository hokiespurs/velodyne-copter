%% This Script visualizes the three potential azimuth interpolation methods
% * Note that Method 3 does not extrapolate at the end, but when other data
%    packets are present those points could be interpolated
% * Method 3 is selected as the more accurate method

%% Generate Sample Data
DELTA_AZ = 0.4;
t_r = kron(reshape(0:23,2,12),ones(16,1));
t_c = repmat([0:15 0:15]',1,12);
t_off = t_r * 55.296 + t_c * 2.304;

az = nan(size(t_off));
az(1,:) = 0:DELTA_AZ:DELTA_AZ*11;

%% Method 1
az1 = repmat(az(1,:),32,1);

%% Method 2
az2 = [repmat(az(1,:),16,1); repmat(az(1,:)+0.2,16,1)];

%% Method 3
az3 = interp1nan(t_off(:),az(:),t_off);

figure(1);clf
subplot 311
plot(t_off(:),az1(:),'.','markersize',10);
hold on
plot(t_off(:),az(:),'.','markersize',20)
xlabel('Time ($\mu$s)','interpreter','latex','fontsize',14)
ylabel('Azimuth (degrees)','interpreter','latex','fontsize',14)
title('Method 1 (Use Previous Azimuth)','interpreter','latex','fontsize',18)
set(gca,'ytick',0:0.4:4.4)
set(gca,'xtick',round(t_off(1,:),0))
grid on
legend({'Interpolated Azimuths','Reported Azimuths'},...
    'location','southeast','interpreter','latex','fontsize',14)

subplot 312
plot(t_off(:),az2(:),'.','markersize',10);
hold on
plot(t_off(:),az(:),'.','markersize',20)
xlabel('Time ($\mu$s)','interpreter','latex','fontsize',14)
ylabel('Azimuth (degrees)','interpreter','latex','fontsize',14)
title('Method 2 (Velodyne pseudocode)','interpreter','latex','fontsize',18)
set(gca,'ytick',0:0.4:4.4)
set(gca,'xtick',round(t_off(1,:),0))
grid on
legend({'Interpolated Azimuths','Reported Azimuths'},...
    'location','southeast','interpreter','latex','fontsize',14)

subplot 313
plot(t_off(:),az3(:),'.','markersize',10);
hold on
plot(t_off(:),az(:),'.','markersize',20)
xlabel('Time ($\mu$s)','interpreter','latex','fontsize',14)
ylabel('Azimuth (degrees)','interpreter','latex','fontsize',14)
title('Method 3 (Proposed)','interpreter','latex','fontsize',18)
set(gca,'ytick',0:0.4:4.4)
set(gca,'xtick',round(t_off(1,:),0))
grid on
legend({'Interpolated Azimuths','Reported Azimuths'},...
    'location','southeast','interpreter','latex','fontsize',14)

bigtitle('Azimuth Interpolation Methods',0.5,0.96,'interpreter','latex','fontsize',20)