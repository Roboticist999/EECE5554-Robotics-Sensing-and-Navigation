clear, close all; clc
stationary = readtable('stationary.csv');
moving = readtable('moving.csv');

% stationary data processing
stationary.Properties.VariableNames(strcmp(stationary.Properties.VariableNames,'field_utm_easting')) = {'easting'};
stationary.Properties.VariableNames(strcmp(stationary.Properties.VariableNames,'field_utm_northing')) = {'northing'};
stationary_centralized = stationary;
stationary_centralized.easting = stationary_centralized.easting - min(stationary_centralized.easting);
stationary_centralized.northing = stationary_centralized.northing - min(stationary_centralized.northing);
stationary_centralized.x_time = stationary_centralized.x_time - min(stationary_centralized.x_time);

% mean of stationary data
mean_northing = mean(stationary_centralized.northing);
mean_easting = mean(stationary_centralized.easting);
mean_lattitude = mean(stationary_centralized.field_latitude);
mean_longitude = mean(stationary_centralized.field_longitude);

% covariance matrix of stationary data
cov_stationary = cov(stationary_centralized.easting,stationary_centralized.northing);

% moving data processing
moving.Properties.VariableNames(strcmp(moving.Properties.VariableNames,'field_utm_easting')) = {'easting'};
moving.Properties.VariableNames(strcmp(moving.Properties.VariableNames,'field_utm_northing')) = {'northing'};
moving_centralized = moving;
moving_centralized.easting = moving_centralized.easting - min(moving_centralized.easting);
moving_centralized.northing = moving_centralized.northing - min(moving_centralized.northing);
moving_centralized.x_time = moving_centralized.x_time - min(moving_centralized.x_time);

% moving data linear regression
p1 = polyfit(moving_centralized.easting,moving_centralized.northing,1);
p2 = polyfit(moving_centralized.northing,moving_centralized.easting,1);
northing_regression = [moving_centralized.easting, ones(height(moving_centralized),1)] * p1';
easting_regression = [moving_centralized.northing, ones(height(moving_centralized),1)] * p2';

figure
subplot(1,2,1)
hold on,grid on
plot3(stationary_centralized.easting,stationary_centralized.northing,stationary_centralized.x_time,'.b')
plot3(mean_easting,mean_northing,stationary_centralized.x_time,'.c')
legend('centralized stationary northing and easting','position mean point')
title('stationary data')
xlabel('easting'),ylabel('northing'),zlabel('time')

subplot(1,2,2)
hold on,grid on
plot3(moving_centralized.easting,moving_centralized.northing,moving_centralized.x_time,'.b')
plot3(moving_centralized.easting,northing_regression,moving_centralized.x_time,'.r')
plot3(easting_regression,moving_centralized.northing,moving_centralized.x_time,'.g')
title('moving data')
legend('centralized moving easting and northing','northing regression based on centralized easting','easting regression based on centralized northing')
xlabel('easting'),ylabel('northing'),zlabel('time')

% moving observations' linear regression with respect to time
p3 = polyfit(moving_centralized.x_time,moving_centralized.northing,1);
p4 = polyfit(moving_centralized.x_time,moving_centralized.easting,1);
northing_time_regression = [moving_centralized.x_time, ones(height(moving_centralized),1)] * p3';
easting_time_regression = [moving_centralized.x_time, ones(height(moving_centralized),1)] * p4';

figure
hold on, grid on
plot(moving_centralized.x_time,moving_centralized.easting,'.r')
plot(moving_centralized.x_time,moving_centralized.northing,'.g')
plot(moving_centralized.x_time,easting_time_regression,'.k')
plot(moving_centralized.x_time,northing_time_regression,'.b')
set(gca,'FontSize',15);
legend({'easting observations','northing observations','easting regresssion','northing regresssion'},'FontSize',15)
title('regression of northing and easting with respect to time','FontSize',22)
xlabel('time','FontSize',22),ylabel('distance','FontSize',22)