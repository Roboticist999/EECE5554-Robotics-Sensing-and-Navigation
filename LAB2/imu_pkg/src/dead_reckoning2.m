clear, close all; clc
%% same as integrating calibrated linear acceleration to get forward velocity part
mag_raw_data = readtable('in_a_car_mag.csv');
imu_raw_data = readtable('in_a_car_imu.csv');
gps_raw_data = readtable('gps.csv');

% time calibration
mag_raw_data.x_time = (mag_raw_data.x_time - min(mag_raw_data.x_time)) * (10^-9);
imu_raw_data.x_time = (imu_raw_data.x_time - min(imu_raw_data.x_time)) * (10^-9);
gps_raw_data.x_time = (gps_raw_data.x_time - min(gps_raw_data.x_time)) * (10^-9);

% forward velocity calculation based on GPS
% shift the starting point of GPS to (0,0)
gps_raw_data.field_utm_easting = gps_raw_data.field_utm_easting - gps_raw_data.field_utm_easting(1);
gps_raw_data.field_utm_northing = gps_raw_data.field_utm_northing - gps_raw_data.field_utm_northing(1);
% forward velocity calculation based on GPS
delta_easting = [gps_raw_data.field_utm_easting;gps_raw_data.field_utm_easting(end)] - ...
    [gps_raw_data.field_utm_easting(1);gps_raw_data.field_utm_easting];
delta_northing = [gps_raw_data.field_utm_northing;gps_raw_data.field_utm_northing(end)] - ...
    [gps_raw_data.field_utm_northing(1);gps_raw_data.field_utm_northing];
fwd_vel_gps = sqrt(delta_easting.^2 + delta_northing.^2);

integrated_vel_from_raw_accel = cumtrapz(imu_raw_data.x_time,imu_raw_data.field_linear_acceleration_x);

figure, hold on, xlabel('time s'), ylabel('forward velocity m/s')
title('forward velocity calculated from GPS and forward velocity integrated from raw accelerometer x data')
plot(imu_raw_data.x_time,integrated_vel_from_raw_accel,'b.')
plot([0;gps_raw_data.x_time],fwd_vel_gps,'r.'), grid on,
legend('forward velocity integrated from raw accelerometer x data','forward velocity calculated from GPS')


figure, hold on
plot(imu_raw_data.x_time,imu_raw_data.field_linear_acceleration_x,'b.'),grid on
% plot(imu_raw_data.x_time,imu_raw_data.field_linear_acceleration_x,'r.'), grid on,
xlabel('time s'), ylabel('linear acceleration x m/s^2')
title('raw linear accelaration data')
legend('linear acceleration x')

% shifting all acceleration x data by the mean of data before engine start
before_engine_start = find(imu_raw_data.x_time <= 10);
stationary_noise_shift = mean(imu_raw_data.field_linear_acceleration_x(before_engine_start));
shifted_linear_accel_x = imu_raw_data.field_linear_acceleration_x - stationary_noise_shift;
integrated_vel_from_shifted_accel = cumtrapz(imu_raw_data.x_time,shifted_linear_accel_x);

figure, hold on
plot(imu_raw_data.x_time,shifted_linear_accel_x,'b.'),grid on
% plot(imu_raw_data.x_time,imu_raw_data.field_linear_acceleration_x,'r.'), grid on,
xlabel('time s'), ylabel('linear acceleration x m/s^2')
title('linear acceleration after calibrating static error')
legend('shifted linear acceleration x', 'raw linear acceleration x')

figure, hold on, xlabel('time s'), ylabel('forward velocity m/s')
title('forward velocity calculated from GPS and forward velocity integrated from shifted linear accel x data')
plot(imu_raw_data.x_time,integrated_vel_from_shifted_accel,'b.')
plot([0;gps_raw_data.x_time],fwd_vel_gps,'r.'), grid on,
legend('forward velocity integrated from shifted linear accel x data','forward velocity calculated from GPS')

% calibration of noise when car stopped
stop_calibration_threshold = 0.3;
stop_part_calibrated_accel_x = shifted_linear_accel_x;
stop_part_calibrated_accel_x(abs(stop_part_calibrated_accel_x) <= 0.3) = 0;
figure
plot(imu_raw_data.x_time,stop_part_calibrated_accel_x,'b.'), grid on
xlabel('time s'), ylabel('linear acceleration x m/s^2')
legend('calibrated linear acceleration x by absolute value method')
title('accelerometer stationary part calibration by setting all values having absolute value less than 0.3 to 0')

% calibration of noise when car stopped using window
window_size = 100;
window_calibration_threshold = 0.5;
window_stop_part_calibrated_accel_x = shifted_linear_accel_x;
jj = 1;
for ii = 1:length(window_stop_part_calibrated_accel_x) - window_size % be careful don't +1 here, ii : ii + window_size is actually window_size+1 values
    if abs(window_stop_part_calibrated_accel_x(ii : ii + window_size)) < window_calibration_threshold % only true when all values are 1
        window_stop_part_calibrated_accel_x(ii : ii + window_size) = 0;
        stop_start_point(jj) = ii;
        jj = jj + 1;
    end
end
figure
plot(imu_raw_data.x_time,window_stop_part_calibrated_accel_x,'b.'), grid on
xlabel('time s'), ylabel('linear acceleration x m/s^2')
legend('calibrated linear acceleration x by window method(better in this particular case)')
title('accelerometer stationary part calibration by moving a window which filters the data')

vel_from_window_accel = cumtrapz(imu_raw_data.x_time,window_stop_part_calibrated_accel_x);
figure, hold on,
plot(imu_raw_data.x_time,vel_from_window_accel,'b.')
plot([0;gps_raw_data.x_time],fwd_vel_gps,'r.'), grid on,
xlabel('time s'), ylabel('forward velocity m/s'),
title('forward velocity calculated from GPS and forward velocity integrated from linear accel x data calibrated by window filter')
legend('forward velocity integrated from linear accel x data calibrated by window filter','forward velocity calculated from GPS')
% %%
% a = [1 2 3 4 5];
% if a<3
%     b = 1
% end

% moving part calibration
aa = 1;
% imu_raw_data = 1;
moving_part_calibrated_accel_x = window_stop_part_calibrated_accel_x;
for kk = 1:length(stop_start_point) - 1
    if stop_start_point(kk + 1) - stop_start_point(kk) >= window_size
        moving_start_point(aa) = stop_start_point(kk) + window_size;
        moving_end_point(aa) = stop_start_point(kk + 1);
%         moving_start_time(aa) = imu_raw_data.x_time(moving_start_point(aa));
%         moving_end_time(aa) = imu_raw_data.x_time(moving_end_point(aa));
        delta_v(aa) = trapz(imu_raw_data.x_time(moving_start_point(aa):moving_end_point(aa)),...
            window_stop_part_calibrated_accel_x(moving_start_point(aa):moving_end_point(aa)));
        delta_t(aa) = imu_raw_data.x_time(moving_end_point(aa)) - imu_raw_data.x_time(moving_start_point(aa));
        moving_part_calibrated_accel_x(moving_start_point(aa):moving_end_point(aa)) = ...
            moving_part_calibrated_accel_x(moving_start_point(aa):moving_end_point(aa)) - delta_v(aa) / delta_t(aa);
        aa = aa + 1;
    end
%     
end
vel_from_moving_adjusted_accel = cumtrapz(imu_raw_data.x_time,moving_part_calibrated_accel_x);


figure, hold on, title('forward velocity calculated from GPS and forward velocity integrated from all parts calibrated linear accel x data')
plot(imu_raw_data.x_time,vel_from_moving_adjusted_accel,'b.')
plot([0;gps_raw_data.x_time],fwd_vel_gps,'r.'),
% plot(moving_start_time,zeros(length(moving_start_time),1),'md')
% plot(moving_end_time,zeros(length(moving_end_time),1),'m*')
xlabel('time s'), ylabel('forward velocity'), grid on
legend('forward velocity integrated from all parts calibrated linear accel x data','forward velocity calculated from GPS')
%% dead reckoning
% intercepting gps data starting from moving straight
gps_start_from_straight = gps_raw_data(120:end,:);
a = find(imu_raw_data.x_time>=gps_start_from_straight.x_time(1));
mag_start_from_straight = mag_raw_data(a(1):end,:);
imu_start_from_straight = imu_raw_data(a(1):end,:);
%% 3.1
fwd_vel_imu_start_from_straight = vel_from_moving_adjusted_accel(a(1):end);
y_2dot_calculated = fwd_vel_imu_start_from_straight .*  imu_start_from_straight.field_angular_velocity_z;
figure
hold on,grid on,xlabel('time s'), ylabel('linear acceleration y m/s^2')
title('imu linear acceleration $y$ reading and linear acceleration $y$ calculated from $\omega * \dot{x}$', 'Interpreter','latex')
plot(imu_start_from_straight.x_time,imu_start_from_straight.field_linear_acceleration_y,'g.')
% plot(imu_start_from_straight.x_time,fwd_vel_imu_start_from_straight,'b.')
% plot(imu_start_from_straight.x_time,imu_start_from_straight.field_angular_velocity_z,'k.')
plot(imu_start_from_straight.x_time,y_2dot_calculated,'r.')
legend('imu linear acceleration $y$ reading','linear acceleration $y$ calculated from $\omega * \dot{x}$', 'Interpreter','latex')

%% 3.2
% quat = [imu_start_from_straight.field_orientation_w(1),imu_start_from_straight.field_orientation_x(1),...
%     imu_start_from_straight.field_orientation_y(1),imu_start_from_straight.field_orientation_z(1)];
% eul = quat2eul(quat,'XYZ')
%% 3.2
% get calibration parameters
mag_circle = mag_raw_data(1500:2100,:);
% hard iron
delta_x = (max(mag_circle.field_magnetic_field_x) + min(mag_circle.field_magnetic_field_x))/2;
delta_y = (max(mag_circle.field_magnetic_field_y) + min(mag_circle.field_magnetic_field_y))/2;
mag_circle.field_magnetic_field_x = mag_circle.field_magnetic_field_x - delta_x;
mag_circle.field_magnetic_field_y = mag_circle.field_magnetic_field_y - delta_y;
% soft iron
radius = sqrt(mag_circle.field_magnetic_field_x.^2 + mag_circle.field_magnetic_field_y.^2);
semi_major_axis_length = max(radius);
semi_minor_axis_length = min(radius);
index = find(radius == semi_major_axis_length);
index1 = find(radius == semi_minor_axis_length);
theta = atan(mag_circle.field_magnetic_field_y(index(1))/mag_circle.field_magnetic_field_x(index(1)));
rotation_matrix = [cos(theta) -sin(-theta); sin(-theta) cos(theta)];
rotated_mag_circle = rotation_matrix * ...
    [mag_circle.field_magnetic_field_x' ; mag_circle.field_magnetic_field_y'];
figure
plot(rotated_mag_circle(1,:),rotated_mag_circle(2,:));axis equal, grid on,hold on
plot(rotated_mag_circle(1,index1),rotated_mag_circle(2,index1),'r.');
ratio = semi_minor_axis_length / semi_major_axis_length;
rotated_mag_circle(1,:) = rotated_mag_circle(1,:) .* ratio;
figure
plot(rotated_mag_circle(1,:),rotated_mag_circle(2,:));axis equal, grid on
backed_mag_circle = [cos(theta) -sin(theta); sin(theta) cos(theta)] * rotated_mag_circle;
figure
subplot(1,2,1)
plot(mag_raw_data.field_magnetic_field_x(1500:2100),mag_raw_data.field_magnetic_field_y(1500:2100));axis equal, grid on
subplot(1,2,2)
plot(backed_mag_circle(1,:),backed_mag_circle(2,:));axis equal, grid on

% calibration
mag_start_from_straight.field_magnetic_field_x = mag_start_from_straight.field_magnetic_field_x - delta_x;
mag_start_from_straight.field_magnetic_field_y = mag_start_from_straight.field_magnetic_field_y - delta_y;
mag_rotation = rotation_matrix * [mag_start_from_straight.field_magnetic_field_x';mag_start_from_straight.field_magnetic_field_y'];
mag_antidistortion = mag_rotation;
mag_antidistortion(1,:) = mag_rotation(1,:) .* ratio;
mag_back = [cos(theta) -sin(theta); sin(theta) cos(theta)] * mag_antidistortion;
mag_start_from_straight.field_magnetic_field_x = mag_back(1,:)';
mag_start_from_straight.field_magnetic_field_y = mag_back(2,:)';

yaw_mag = -180/pi*atan2(mag_start_from_straight.field_magnetic_field_y,mag_start_from_straight.field_magnetic_field_x);
figure, hold on
plot(mag_start_from_straight.field_magnetic_field_x,mag_start_from_straight.field_magnetic_field_y,'b.')
plot(mag_start_from_straight.field_magnetic_field_x(3100:3300),mag_start_from_straight.field_magnetic_field_y(3100:3300),'k.')
grid on
figure
plot(mag_start_from_straight.x_time,yaw_mag,'r.'),grid on,
title('yaw starting from going straight by calibrated magnetometer')
xlabel('time s'),ylabel('yaw wrapped to [-180,+180) degree')
legend('yaw starting from going straight by calibrated magnetometer')

v_e = [fwd_vel_imu_start_from_straight;0] .* cosd(90+14+25/60 - yaw_mag);
v_n = [fwd_vel_imu_start_from_straight;0] .* sind(90+14+25/60 - yaw_mag);
x_e = cumtrapz(mag_start_from_straight.x_time,v_e);
x_n = cumtrapz(mag_start_from_straight.x_time,v_n);

% calcutation of a line showing magnetic north and south pole
k = tand(90+14+25/60);
x_coordinate = min(gps_start_from_straight.field_utm_easting):max(gps_start_from_straight.field_utm_northing);
y_coordinate = k .* x_coordinate;

figure
title('path plotted from IMU and GPS')
subplot(1,2,1),axis equal, grid on, hold on
xlabel('centered easting (m)'), ylabel('centered northing (m)')
title('path plotted from IMU')
plot(x_e,x_n)
plot(x_e(1),x_n(1),'r.')
aaa = find(imu_start_from_straight.x_time>=862&imu_start_from_straight.x_time<1030);
plot(x_e(aaa(1)),x_n(aaa(1)),'kd')
plot(x_e(aaa(end)),x_n(aaa(end)),'k*')
plot(x_coordinate,y_coordinate,'g')
legend('path calculated from accelerometer and magnetometer',...
    'start point of driving straight','start point of part with large error',...
    'end point of part with large error','line showing magnetic north and south')
subplot(1,2,2),axis equal, grid on, hold on
xlabel('centered easting (m)'), ylabel('centered northing (m)')
title('path plotted from GPS')
plot(gps_start_from_straight.field_utm_easting,gps_start_from_straight.field_utm_northing)
plot(gps_start_from_straight.field_utm_easting(1),gps_start_from_straight.field_utm_northing(1),'r.')
plot(x_coordinate,y_coordinate,'g')
plot(gps_raw_data.field_utm_easting(200),gps_raw_data.field_utm_northing(200),'k.')
legend('path drawn from GPS data','start point of driving straight','line showing magnetic north and south')

%%
q = [imu_start_from_straight.field_orientation_w,imu_start_from_straight.field_orientation_x,...
    imu_start_from_straight.field_orientation_y,imu_start_from_straight.field_orientation_z];
angle = quat2eul(q,'ZYX');
angle_degree = rad2deg(angle);
angle_degree_180 = wrapTo180(angle_degree);
figure
plot(imu_start_from_straight.x_time,angle_degree_180(:,1));grid on

%%%%%%%%%%%%%% dead reckoning using shifted and integraded yaw from Gyro
% imu_raw_data = a
integrated_yaw_rad = cumtrapz(imu_raw_data.x_time(a(1):end),imu_raw_data.field_angular_velocity_z(a(1):end));
% yaw_imu_degree = 180/pi .* integrated_yaw_rad; %+ yaw_mag(1);
% deviation = yaw_mag(end) - yaw_imu_degree(end);
% yaw_imu = wrapTo180(yaw_imu_degree + deviation);
yaw_imu_degree = 180/pi .* integrated_yaw_rad + yaw_mag(1);
% deviation = yaw_mag(end) - yaw_imu_degree(end);
% yaw_imu = wrapTo180(yaw_imu_degree + deviation);
yaw_imu = wrapTo180(yaw_imu_degree);
yaw_imu_from_straight = yaw_imu;%(a(1):end);

v_e_imu = fwd_vel_imu_start_from_straight .* cosd(90+14+25/60 - yaw_imu_from_straight);
v_n_imu = fwd_vel_imu_start_from_straight .* sind(90+14+25/60 - yaw_imu_from_straight);
x_e_imu = cumtrapz(imu_raw_data.x_time(a(1):end),v_e_imu);
x_n_imu = cumtrapz(imu_raw_data.x_time(a(1):end),v_n_imu);

% figure
% plot(imu_raw_data.x_time,yaw_imu,'b.'); grid on,title('yaw by integrating angular vel z'),hold on
figure
title('path plotted from IMU(yaw by integrated and shifted Gyro) and GPS')
subplot(1,2,1),axis equal, grid on, hold on
xlabel('centered easting (m)'), ylabel('centered northing (m)')
title('path plotted from IMU(yaw by integrated and shifted Gyro)')
plot(x_coordinate,y_coordinate,'g')
plot(x_e_imu,x_n_imu,'b')
plot(x_e_imu(1),x_n_imu(1),'r.')
aaa = find(imu_start_from_straight.x_time>=862&imu_start_from_straight.x_time<1030);
plot(x_e_imu(aaa(1)),x_n_imu(aaa(1)),'kd')
plot(x_e_imu(aaa(end)),x_n_imu(aaa(end)),'k*')
legend('line showing magnetic north and south','path calculated from accelerometer and magnetometer',...
    'start point of driving straight','start point of part with large error',...
    'end point of part with large error')
subplot(1,2,2),axis equal, grid on, hold on
xlabel('centered easting (m)'), ylabel('centered northing (m)')
title('path plotted from GPS')
plot(gps_start_from_straight.field_utm_easting,gps_start_from_straight.field_utm_northing)
plot(gps_start_from_straight.field_utm_easting(1),gps_start_from_straight.field_utm_northing(1),'r.')
plot(x_coordinate,y_coordinate,'g')
plot(gps_raw_data.field_utm_easting(200),gps_raw_data.field_utm_northing(200),'k.')
legend('path drawn from GPS data','start point of driving straight','line showing magnetic north and south')

%% whole gps plot
figure,axis equal, grid on, hold on
plot(gps_raw_data.field_utm_easting,gps_raw_data.field_utm_northing)
plot(gps_raw_data.field_utm_easting(1),gps_raw_data.field_utm_northing(1),'rd')
plot(gps_raw_data.field_utm_easting(end),gps_raw_data.field_utm_northing(end),'r*')
plot(x_coordinate,y_coordinate,'g')
xlabel('centered easting (m)'), ylabel('centered northing (m)')
legend('complete path plotted from GPS','start point','end point','line showing magnetic north and south')
title('complete path plotted from GPS')