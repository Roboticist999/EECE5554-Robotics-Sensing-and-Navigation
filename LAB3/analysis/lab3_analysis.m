clear, close all; clc
stationary_isec_raw = readtable('stationary_isec.csv'  );
moving_isec_raw = readtable('moving_isec.csv');
stationary_field_raw = readtable('stationary_field.csv');
moving_field_raw = readtable('moving_field.csv');
walking_building_data= readtable('walking_isec_5.csv');
% aaa = readtable('sitting_field (2).csv');

% delete rows whose all elements are NaN
% sitting_isec_raw = rmmissing(sitting_isec_raw,'DataVariables',{'field_header_stamp'});
% walking_isec_raw = rmmissing(walking_isec_raw,'DataVariables',{'x_time'});
% sitting_field_raw = rmmissing(sitting_field_raw,'DataVariables',{'x_time'});
% walking_field_raw = rmmissing(walking_field_raw,'DataVariables',{'x_time'});

% change field name and shrink easting and northing values
stationary_isec_raw.Properties.VariableNames(strcmp(stationary_isec_raw.Properties.VariableNames,'field_Easting')) = {'easting'};
stationary_isec_raw.Properties.VariableNames(strcmp(stationary_isec_raw.Properties.VariableNames,'field_Northing')) = {'northing'};
stationary_isec_raw.Properties.VariableNames(strcmp(stationary_isec_raw.Properties.VariableNames,'x_time')) = {'time'};
stationary_isec = stationary_isec_raw;
stationary_isec.easting = stationary_isec.easting - min(stationary_isec.easting);
stationary_isec.northing = stationary_isec.northing - min(stationary_isec.northing);
stationary_isec.time = (stationary_isec.time - min(stationary_isec.time)) .* 10^-9;
fix_good = stationary_isec.field_Fix == 5;
% sitting_isec.time = (sitting_isec.time - min(sitting_isec.time));

moving_isec_raw.Properties.VariableNames(strcmp(moving_isec_raw.Properties.VariableNames,'field_Easting')) = {'easting'};
moving_isec_raw.Properties.VariableNames(strcmp(moving_isec_raw.Properties.VariableNames,'field_Northing')) = {'northing'};
moving_isec_raw.Properties.VariableNames(strcmp(moving_isec_raw.Properties.VariableNames,'x_time')) = {'time'};
moving_isec = moving_isec_raw;
moving_isec.easting = moving_isec.easting - min(moving_isec.easting);
moving_isec.northing = moving_isec.northing - min(moving_isec.northing);
moving_isec.time = (moving_isec.time - min(moving_isec.time)) .* 10^-9;
% walking_isec.time = (walking_isec.time - min(walking_isec.time));

stationary_field_raw.Properties.VariableNames(strcmp(stationary_field_raw.Properties.VariableNames,'field_Easting')) = {'easting'};
stationary_field_raw.Properties.VariableNames(strcmp(stationary_field_raw.Properties.VariableNames,'field_Northing')) = {'northing'};
stationary_field_raw.Properties.VariableNames(strcmp(stationary_field_raw.Properties.VariableNames,'x_time')) = {'time'};
stationary_field = stationary_field_raw;
stationary_field.easting = stationary_field.easting - min(stationary_field.easting);
stationary_field.northing = stationary_field.northing - min(stationary_field.northing);
stationary_field.time = (stationary_field.time - min(stationary_field.time)) .* 10^-9;
% sitting_field.time = (sitting_field.time - min(sitting_field.time));

moving_field_raw.Properties.VariableNames(strcmp(moving_field_raw.Properties.VariableNames,'field_Easting')) = {'easting'};
moving_field_raw.Properties.VariableNames(strcmp(moving_field_raw.Properties.VariableNames,'field_Northing')) = {'northing'};
moving_field_raw.Properties.VariableNames(strcmp(moving_field_raw.Properties.VariableNames,'x_time')) = {'time'};
moving_field = moving_field_raw;
moving_field.easting = moving_field.easting - min(moving_field.easting);
moving_field.northing = moving_field.northing - min(moving_field.northing);
moving_field.time = (moving_field.time - min(moving_field.time)) .* 10^-9;
% walking_field.time = (walking_field.time - min(walking_field.time));

walking_building_data.x_time = (walking_building_data.x_time - min(walking_building_data.x_time)) .* 10^-9;
walking_building_data.field_utm_easting = walking_building_data.field_utm_easting - min(walking_building_data.field_utm_easting);
walking_building_data.field_utm_northing = walking_building_data.field_utm_northing - min(walking_building_data.field_utm_northing);

cov_stationary_isec = cov(stationary_isec.easting,stationary_isec.northing)
cov_stationary_isec_fix_5 = cov(stationary_isec.easting(fix_good),stationary_isec.northing(fix_good))
% cov_moving_isec = cov(moving_isec.easting,moving_isec.northing)
cov_stationary_field = cov(stationary_field.easting,stationary_field.northing)
% cov_moving_field = cov(moving_field.easting,moving_field.northing)

% mean of sitting data
mean_sitting_isec_northing = mean(stationary_isec.northing);
mean_sitting_isec_easting = mean(stationary_isec.easting);

mean_sitting_field_northing = mean(stationary_field.northing);
mean_sitting_field_easting = mean(stationary_field.easting);

% sitting isec data plot
figure,hold on,xlabel('easting m'),ylabel('northing m'),zlabel('time s')
plot3(stationary_isec.easting,stationary_isec.northing,stationary_isec.time,'b.');
plot3(mean_sitting_isec_easting,mean_sitting_isec_northing,stationary_isec.time,'r.')
grid on,title('stationary data collected when GPS is near a building')
legend('stationary data collected near a building','mean of statiionary data collected near a building')

figure,xlabel('time s'),ylabel('fix')
plot(stationary_isec.time,stationary_isec.field_Fix,'b.')
grid on, title('fix of data sitting near ISEC'),xlabel('time s'),ylabel('fix')

figure,hold on,xlabel('easting m'),ylabel('northing m'),zlabel('time s')
plot3(stationary_isec.easting(fix_good),stationary_isec.northing(fix_good),stationary_isec.time(fix_good),'b.');
plot3(mean_sitting_isec_easting,mean_sitting_isec_northing,stationary_isec.time(fix_good),'r.')
grid on,title('stationary data collected when GPS is near a building and fix = 5')
legend('stationary data collected near a building, fix = 5','mean of statiionary data collected near a building')

% sitting field data plot
figure,hold on,xlabel('easting m'),ylabel('northing m'),zlabel('time s')
plot3(stationary_field.easting,stationary_field.northing,stationary_field.time,'b.');
plot3(mean_sitting_field_easting,mean_sitting_field_northing,stationary_field.time,'r.')
grid on,title('stationary data collected when GPS is in an open space')
legend('stationary data collected in an open space','mean of statiionary data collected in an open space')

figure,xlabel('time s'),ylabel('fix')
plot(stationary_field.time,stationary_field.field_Fix,'b.')
grid on, title('fix of data sitting on open field'),xlabel('time s'),ylabel('fix')

% moving data near isec plot
figure,hold on,xlabel('easting m'),ylabel('northing m'),zlabel('time s')
plot3(moving_isec.easting,moving_isec.northing,moving_isec.time,'b.');
% plot3(mean_sitting_isec_easting,mean_sitting_isec_northing,walking_isec.time,'r.')
grid on,title('moving data collected when GPS is near a building')
legend('stationary data collected near a building')

figure,xlabel('time s'),ylabel('fix')
plot(moving_isec.time,moving_isec.field_Fix,'b.')
grid on, title('fix of data moving near ISEC'),xlabel('time s'),ylabel('fix')

% moving data in an open space
figure,hold on,xlabel('easting m'),ylabel('northing m'),zlabel('time s')
plot3(moving_field.easting,moving_field.northing,moving_field.time,'b.');
% plot3(mean_sitting_field_easting,mean_sitting_field_northing,walking_field.time,'r.')
grid on,title('walking data collected when GPS is in an open space')
legend('walking data collected in an open space')

figure,xlabel('time s'),ylabel('fix')
plot(moving_field.time,moving_field.field_Fix,'b.')
grid on, title('fix of data moving on an open field'),xlabel('time s'),ylabel('fix')

% walking data with different resolution in northing and easting
figure
plot3(walking_building_data.field_utm_easting,walking_building_data.field_utm_northing,walking_building_data.x_time,'b.')
grid on, xlabel('easting'),ylabel('northing'),zlabel('time'),title('northing and easting of walking data near a building logged using float32')