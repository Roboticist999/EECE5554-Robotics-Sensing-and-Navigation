clear, close all; clc
mag_raw_data = readtable('in_a_car_mag.csv'); % used at the end, plotting uncalibrated data
imu = readtable('in_a_car_imu.csv');
gps = readtable('gps.csv');

mag = mag_raw_data;
mag.x_time = (mag.x_time - min(mag.x_time)) * (10^-9);
gps.x_time = (gps.x_time - min(gps.x_time)) * (10^-9);
imu.x_time = (imu.x_time - min(imu.x_time)) * (10^-9);

figure
subplot(1,2,1),
plot(imu.x_time,imu.field_angular_velocity_z),grid on,title('angular vel z')
subplot(1,2,2)
imu.field_angular_velocity_z = imu.field_angular_velocity_z - mean(imu.field_angular_velocity_z(1:600));
yaw_imu_rad = cumtrapz(imu.x_time,imu.field_angular_velocity_z);
plot(imu.x_time,yaw_imu_rad,'.'),grid on, title('integrated yaw')

% mag_circle = mag(1400:3700,:);
% mag_circle = mag(1400:2100,:);
mag_circle = mag(1500:2100,:);
% mag_circle = mag(1400:3700,:);
% figure
% for ii = 1400:3700
%     plot(mag.field_magnetic_field_x(ii),mag.field_magnetic_field_y(ii),'b.'),hold on, axis equal,
%     pause(0.01)
% end

figure
plot(mag_circle.field_magnetic_field_x,mag_circle.field_magnetic_field_y),axis equal,grid on,hold on
plot(mean(mag_circle.field_magnetic_field_x),mean(mag_circle.field_magnetic_field_y),'r.')
plot(mag_circle.field_magnetic_field_x(1),mag_circle.field_magnetic_field_y(1),'kx')
plot(mag_circle.field_magnetic_field_x(end),mag_circle.field_magnetic_field_y(end),'kd')
title('mag data of 1 round')
legend('selected magnetometer data for driving 1 turn','centroid of magnetometer data',...
    'start point of magnetometer data','end point of magnetometer data')
xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
% plot(mag.field_magnetic_field_x,mag.field_magnetic_field_y);
% figure
% plot(gps.field_utm_easting(1:90),gps.field_utm_northing(1:90));
% find(mag.x_time>gps.x_time(90));
% figure

% for i = 1:3562
%     plot(mag.field_magnetic_field_x(1:i),mag.field_magnetic_field_y(1:i));
%     hold on
%     pause(0.01)
% end% figure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mag_3_turns = mag((1400:3550),:);
% figure
% title('mag data when going in 3 circles')
% plot(mag.field_magnetic_field_x(1400:3700),mag.field_magnetic_field_y(1400:3700));axis equal, grid on,hold on
% plot(mag.field_magnetic_field_x(1400),mag.field_magnetic_field_y(1400),'kx')
% plot(mag.field_magnetic_field_x(3550),mag.field_magnetic_field_y(3550),'kd')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% plot(mag.field_magnetic_field_x(1400:2100),mag.field_magnetic_field_y(1400:2100));
% axis equal, grid on

% [A,b,expmfs] = magcal([mag.field_magnetic_field_x(1400:3700),mag.field_magnetic_field_y(1400:3700),zeros(2301,1)])

% calibration
% hard iron
delta_x = (max(mag_circle.field_magnetic_field_x) + min(mag_circle.field_magnetic_field_x))/2;
delta_y = (max(mag_circle.field_magnetic_field_y) + min(mag_circle.field_magnetic_field_y))/2;
% delta_x = mean(mag_circle.field_magnetic_field_x);
% delta_y = mean(mag_circle.field_magnetic_field_y);
mag_circle.field_magnetic_field_x = mag_circle.field_magnetic_field_x - delta_x;
mag_circle.field_magnetic_field_y = mag_circle.field_magnetic_field_y - delta_y;



% soft iron
% delta_x = max(mag_circle.field_magnetic_field_x) - min(mag_circle.field_magnetic_field_x);
% delta_y = max(mag_circle.field_magnetic_field_y) - min(mag_circle.field_magnetic_field_y);
radius = sqrt(mag_circle.field_magnetic_field_x.^2 + mag_circle.field_magnetic_field_y.^2);
semi_major_axis_length = max(radius);
semi_minor_axis_length = min(radius);
index = find(radius == semi_major_axis_length);
index1 = find(radius == semi_minor_axis_length);
theta = atan2(mag_circle.field_magnetic_field_y(index(1)),mag_circle.field_magnetic_field_x(index(1)));
rotation_matrix = [cos(theta) -sin(-theta); sin(-theta) cos(theta)];
rotated_mag_circle = rotation_matrix * ...
    [mag_circle.field_magnetic_field_x' ; mag_circle.field_magnetic_field_y'];
figure
plot(rotated_mag_circle(1,:),rotated_mag_circle(2,:));axis equal, grid on,hold on
plot(rotated_mag_circle(1,index1),rotated_mag_circle(2,index1),'r.');
title('centered at origin and rotated circle')
legend('centered at origin and rotated circle','point whose distance to origin is viewed as semi minor axis length')
xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')

ratio = semi_minor_axis_length / semi_major_axis_length;
rotated_mag_circle(1,:) = rotated_mag_circle(1,:) .* ratio;
figure
plot(rotated_mag_circle(1,:),rotated_mag_circle(2,:));axis equal, grid on,title('after soft iron calibration')
backed_mag_circle = [cos(theta) -sin(theta); sin(theta) cos(theta)] * rotated_mag_circle;
figure
subplot(1,2,1)
plot(mag_circle.field_magnetic_field_x,mag_circle.field_magnetic_field_y);axis equal, grid on, hold on
plot(mag_circle.field_magnetic_field_x(1),mag_circle.field_magnetic_field_y(1),'kx')
plot(mag_circle.field_magnetic_field_x(end),mag_circle.field_magnetic_field_y(end),'kd')
title('before  calibtarion'),xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
legend('original circle','start point of circle','end point of circle')
subplot(1,2,2)
plot(backed_mag_circle(1,:),backed_mag_circle(2,:));axis equal, grid on,hold on,title('after  calibtarion')
plot(backed_mag_circle(1,1),backed_mag_circle(2,1),'kx')
plot(backed_mag_circle(1,end),backed_mag_circle(2,end),'kd')
title('after calibration'),xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
legend('calibrated circle','start point of circle','end point of circle')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 4 turns
mag_4_turns = mag((1400:3550),:);
mag_4_turns.field_magnetic_field_x = mag_4_turns.field_magnetic_field_x - delta_x;
mag_4_turns.field_magnetic_field_y = mag_4_turns.field_magnetic_field_y - delta_y;
rotated_mag_4_turns = rotation_matrix * ...
    [mag_4_turns.field_magnetic_field_x' ; mag_4_turns.field_magnetic_field_y'];
figure
% title('mag data when going in 4 circles')
subplot(2,2,1)
plot(mag_4_turns.field_magnetic_field_x,mag_4_turns.field_magnetic_field_y);axis equal, grid on,hold on
plot(mag_4_turns.field_magnetic_field_x(1),mag_4_turns.field_magnetic_field_y(1),'kx')
plot(mag_4_turns.field_magnetic_field_x(end),mag_4_turns.field_magnetic_field_y(end),'kd')
xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
legend('magnetometer data','start point','end point')
title('4 circles after translation')
subplot(2,2,2)
plot(rotated_mag_4_turns(1,:),rotated_mag_4_turns(2,:)),axis equal, grid on,hold on
plot(rotated_mag_4_turns(1,1),rotated_mag_4_turns(2,1),'kx')
plot(rotated_mag_4_turns(1,end),rotated_mag_4_turns(2,end),'kd')
xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
legend('magnetometer data','start point','end point')
title('4 circles after rotation')
ratio_denominator = sum(abs(mag_4_turns.field_magnetic_field_x(abs(mag_4_turns.field_magnetic_field_y)<10^-6)))...
    /length(mag_4_turns.field_magnetic_field_x(abs(mag_4_turns.field_magnetic_field_y)<10^-6));
ratio_numerator = sum(abs(mag_4_turns.field_magnetic_field_y(abs(mag_4_turns.field_magnetic_field_x)<10^-6)))...
    /length(mag_4_turns.field_magnetic_field_y(abs(mag_4_turns.field_magnetic_field_x)<10^-6));
ratio_4_turns = ratio_numerator / ratio_denominator;
rotated_mag_4_turns(1,:) = rotated_mag_4_turns(1,:) .* ratio_4_turns;
backed_mag_4_turns = [cos(theta) -sin(theta); sin(theta) cos(theta)] * rotated_mag_4_turns;
subplot(2,2,3)
plot(rotated_mag_4_turns(1,:),rotated_mag_4_turns(2,:)),axis equal, grid on,hold on
plot(rotated_mag_4_turns(1,1),rotated_mag_4_turns(2,1),'kx')
plot(rotated_mag_4_turns(1,end),rotated_mag_4_turns(2,end),'kd')
legend('magnetometer data','start point','end point')
xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
title('4 circles after compress in certain direction')
subplot(2,2,4)
plot(backed_mag_4_turns(1,:),backed_mag_4_turns(2,:)),axis equal, grid on,hold on
plot(backed_mag_4_turns(1,1),backed_mag_4_turns(2,1),'kx')
plot(backed_mag_4_turns(1,end),backed_mag_4_turns(2,end),'kd')
legend('magnetometer data','start point','end point')
xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
title('4 circles after back rotation')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% calculating yaw angle
% hard iron
mag.field_magnetic_field_x = mag.field_magnetic_field_x - delta_x;
mag.field_magnetic_field_y = mag.field_magnetic_field_y - delta_y;
% soft iron
mag_rotation = rotation_matrix * [mag.field_magnetic_field_x';mag.field_magnetic_field_y'];
mag_antidistortion = mag_rotation;
mag_antidistortion(1,:) = mag_rotation(1,:) .* ratio; % you can use 'ratio' to use the ratio calculated from 1 circle
mag_back = [cos(theta) -sin(theta); sin(theta) cos(theta)] * mag_antidistortion;
mag.field_magnetic_field_x = mag_back(1,:)';
mag.field_magnetic_field_y = mag_back(2,:)';

figure
% title('raw mag data and calibrated mag data')
subplot(1,2,1)
plot(mag_raw_data.field_magnetic_field_x,mag_raw_data.field_magnetic_field_y), axis equal, grid on, hold on
title('uncalibrated magnetometer data'),xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
% plot(mag_raw_data.field_magnetic_field_x(1:1000),mag_raw_data.field_magnetic_field_y(1:1000),'r.')
subplot(1,2,2)
plot(mag.field_magnetic_field_x,mag.field_magnetic_field_y), axis equal, grid on,hold on
title('calibrated magnetometer data'),xlabel('magnetic flux density x (Gauss)'),ylabel('magnetic flux density y (Gauss)')
plot(mag.field_magnetic_field_x(1:1000),mag.field_magnetic_field_y(1:1000),'r.')
% plot(mag.field_magnetic_field_x(7900:8200),mag.field_magnetic_field_y(7900:8200),'g.')
plot(mag.field_magnetic_field_x(end-100:end),mag.field_magnetic_field_y(end-100:end),'k.')
yaw_mag = -180/pi*atan2(mag.field_magnetic_field_y,mag.field_magnetic_field_x);

figure
plot(mag.x_time,yaw_mag,'r.'),grid on,title('yaw by calibrated magnetometer')
xlabel('time (s)'),ylabel('yaw angle (degree)')

% Yaw calculation using GyroZ
yaw_imu_rad = cumtrapz(imu.x_time,imu.field_angular_velocity_z);
yaw_imu_degree = 180/pi .* yaw_imu_rad + yaw_mag(1); % comment +yag_mag(1) to get intrgration without initial value
yaw_imu = wrapTo180(yaw_imu_degree);

figure
plot(imu.x_time,yaw_imu,'b.'); grid on,title('yaw by integrating angular velocity z'),hold on
xlabel('time (s)'),ylabel('yaw angle (degree)')

%%%%%%%%%%%%%%%%%%%%%%% inverse convertion of quaternion
q = [imu.field_orientation_w,imu.field_orientation_x,...
    imu.field_orientation_y,imu.field_orientation_z];
angle = quat2eul(q,'ZYX');
angle_degree = rad2deg(angle);
angle_degree_plus38 = rad2deg(angle) + 38; % 38 degrees is added to make them match, you can delete the 38
angle_degree_180 = wrapTo180(angle_degree_plus38);
% figure
plot(imu.x_time,angle_degree_180(:,1),'r.');grid on
plot(mag.x_time,yaw_mag,'g.')
legend('yaw by integrating angular vel z','yaw converted from quaternion','yaw by calibrated magnetometer')

%%%%%%%%%%%%%%%%%%%%% using right magnetometer end value to calibrate
%%%%%%%%%%%%%%%%%%%%% quat->yaw and integrated yaw
deviation = yaw_mag(end) - yaw_imu(end);
figure, title('matching different method of yaw calculation'),hold on,grid minor
xlabel('time (s)'),ylabel('yaw angle (degree)')
plot(imu.x_time,wrapTo180(yaw_imu_degree + deviation),'b.')
plot(imu.x_time,wrapTo180(angle_degree_180(:,1) + deviation),'r.')
plot(mag.x_time,yaw_mag,'g.')
legend('yaw by integrating angular vel z','yaw converted from quaternion','yaw by calibrated magnetometer')

%%%%%%%%%%%%%%%%%%%%% angular velocity calculation based on quaternion
% delta_t_imu = [imu.x_time ; imu.x_time(end)] - [0 ; imu.x_time];
% delta_yaw_gyro = [angle(:,1);angle(end,1)] - [0 ; angle(:,1)];
% yaw_ang_vel_gyo = delta_yaw_gyro ./ delta_t_imu;
% figure,
% plot(imu.x_time,yaw_ang_vel_gyo(1:end-1),'b.'),hold on, grid on,ylim([-0.8 0.8])
% plot(imu.x_time,imu.field_angular_velocity_z,'r.')
delta_t_imu = imu.x_time(2:end) - imu.x_time(1:end-1);
delta_yaw_quat = angle(2:end,1) - angle(1:end-1,1);
yaw_ang_vel_quat = delta_yaw_quat ./ delta_t_imu;
figure,
hold on, grid on,ylim([-0.8 0.8])
plot(imu.x_time,imu.field_angular_velocity_z,'r.')
plot(imu.x_time(1:end-1),yaw_ang_vel_quat,'b.')


%%%%%%%%%%%%%%%%%%%%% angular velocity calculation based on magnetometer
% delta_t_mag = [mag.x_time ; mag.x_time(end)] - [0 ; mag.x_time];
% delta_yaw_mag = [yaw_mag ; yaw_mag(end)] - [yaw_mag(1) ; yaw_mag];
% yaw_ang_vel_mag = deg2rad(delta_yaw_mag ./ delta_t_mag);
% plot(mag.x_time,yaw_ang_vel_mag,'g.')
% plot(mag.x_time,lowpass(yaw_ang_vel_mag,0.2),'k.')
delta_t_mag = mag.x_time(2:end) - mag.x_time(1:end-1);
delta_yaw_mag = yaw_mag(2:end) - yaw_mag(1:end-1);
yaw_ang_vel_mag = deg2rad(delta_yaw_mag ./ delta_t_mag);
% plot(mag.x_time(1:end-1),yaw_ang_vel_mag,'g.')
% plot(mag.x_time(1:end-1),lowpass(yaw_ang_vel_mag,0.2),'k.')

% complementary filter
% low pass filter
yaw_mag_unwrap = unwrap(yaw_mag);
% yaw_mag_lowpass = lowpass(yaw_mag_unwrap,0.2);
yaw_mag_lowpass = lowpass(yaw_mag_unwrap,10,50);
% high pass filter
yaw_imu_unwrap = unwrap(yaw_imu);
yaw_imu_highpass = highpass(yaw_imu_unwrap,10,50);
% sum
% yaw_sum = (yaw_mag_lowpass + yaw_imu_
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% grid on,hold on;title('unfiltered yaw')
% plot(mag.x_time,yaw_mag_unwrap,'b.')
% plot(imu.x_time,yaw_imu_unwrap,'r.')
% legend('unfiltered, unwrapped yaw from calibrated magnetometer','unfiltered, unwrapped yaw from Gyro')
% xlabel('time (s)'),ylabel('unfiltered yaw (degree)')
% figure
% grid on,hold on,title('filtered yaw')
% plot(mag.x_time,yaw_mag_lowpass,'b.')
% plot(imu.x_time,yaw_imu_highpass,'r.')
% legend('low pass of yaw calculated from calibrated magetometer','high pass of yaw imtegrated from Gyro')
% xlabel('time (s)'),ylabel('filtered yaw (degree)')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
grid on,hold on;title('yaw from calibrated magnetometer')
plot(mag.x_time,yaw_mag_unwrap,'b.')
plot(mag.x_time,yaw_mag_lowpass,'r.')
legend('unfiltered, unwrapped yaw from calibrated magnetometer','lowpass filtered, unwrapped yaw from calibrated magnetometer')
xlabel('time (s)'),ylabel('yaw (degree)')
figure
grid on,hold on,title('yaw from integrated gyro data')
plot(imu.x_time,yaw_imu_unwrap,'b.')
plot(imu.x_time,yaw_imu_highpass,'r.')
legend('unfiltered, unwrapped yaw from gyro','highpass filtered of yaw imtegrated from Gyro')
xlabel('time (s)'),ylabel('yaw (degree)')

% fft
figure
fs = 50;
y = fft(yaw_imu);
n = length(yaw_imu);
f = (0:n-1)*(fs/n);     % frequency range
power = abs(y).^2/n;
plot(f,power)
xlabel('Frequency')
ylabel('Power')

% not compensated Gyro
%%
yaw_mag_nc = -180/pi*atan2(mag_raw_data.field_magnetic_field_y,mag_raw_data.field_magnetic_field_x);
figure
plot(mag.x_time,yaw_mag_nc,'b.'),grid on,hold on,xlabel('time s'),ylabel('yaw degree')
title('yaw calculated from raw magnetometer data')
% plot complementary filter result