clear, close all; clc
imu = readtable('imu_static2.csv');
magnetic_field = readtable('mag_static2.csv');

% stationary.Properties.VariableNames(strcmp(stationary.Properties.VariableNames,'field_utm_easting')) = {'easting'};
imu.x_time = (imu.x_time - min(imu.x_time)) * (10^-9);
magnetic_field.x_time = (magnetic_field.x_time - min(magnetic_field.x_time)) * (10^-9);
figure(1)
subplot(3,3,1)
plot(imu.x_time,imu.field_angular_velocity_x,'b')
xlabel('time(sec)'),ylabel('angular velocity x (rad/s)'),title('angular velocity x & time'),grid on
subplot(3,3,2)
plot(imu.x_time,imu.field_angular_velocity_y,'b')
xlabel('time(sec)'),ylabel('angular velocity y (rad/s)'),title('angular velocity y & time'),grid on
subplot(3,3,3)
plot(imu.x_time,imu.field_angular_velocity_z,'b')
xlabel('time(sec)'),ylabel('angular velocity z (rad/s)'),title('angular velocity z & time'),grid on

subplot(3,3,4)
plot(imu.x_time,imu.field_linear_acceleration_x,'b')
xlabel('time(sec)'),ylabel('linear acceleration x (m/s^2)'),title('linear acceleration x & time'),grid on
subplot(3,3,5)
plot(imu.x_time,imu.field_linear_acceleration_y,'b')
xlabel('time(sec)'),ylabel('linear acceleration y (m/s^2)'),title('linear acceleration y & time'),grid on
subplot(3,3,6)
plot(imu.x_time,imu.field_linear_acceleration_z,'b')
xlabel('time(sec)'),ylabel('linear acceleration z (m/s^2)'),title('linear acceleration z & time'),grid on

subplot(3,3,7)
plot(magnetic_field.x_time,magnetic_field.field_magnetic_field_x,'b')
xlabel('time(sec)'),ylabel('magnetic field x (Gauss)'),title('magnetic field x & time'),grid on
subplot(3,3,8)
plot(magnetic_field.x_time,magnetic_field.field_magnetic_field_y,'b')
xlabel('time(sec)'),ylabel('magnetic field y (Gauss)'),title('magnetic field y & time'),grid on
subplot(3,3,9)
plot(magnetic_field.x_time,magnetic_field.field_magnetic_field_z,'b')
xlabel('time(sec)'),ylabel('magnetic field z (Gauss)'),title('magnetic field z & time'),grid on

% pd_angular_velocity_x = fitdist(imu.field_angular_velocity_x,'Normal');
% pd_angular_velocity_y = fitdist(imu.field_angular_velocity_y,'Normal');
% pd_angular_velocity_z = fitdist(imu.field_angular_velocity_z,'Normal');
% 
% pd_linear_acceleration_x = fitdist(imu.field_linear_acceleration_x,'Normal');
% pd_linear_acceleration_y = fitdist(imu.field_linear_acceleration_y,'Normal');
% pd_linear_acceleration_z = fitdist(imu.field_linear_acceleration_z,'Normal');
% 
% pd_magnetic_field_x = fitdist(magnetic_field.field_magnetic_field_x,'Normal');
% pd_magnetic_field_y = fitdist(magnetic_field.field_magnetic_field_y,'Normal');
% pd_magnetic_field_z = fitdist(magnetic_field.field_magnetic_field_z,'Normal');

%%
% imu = 1
% quat = [imu.field_orientation_w(500),imu.field_orientation_x(500),...
%     imu.field_orientation_y(500),imu.field_orientation_z(500)];
% eul = quat2eul(quat,'ZYX')
%%

%% histogram analysis
figure
subplot(3,3,1)
histogram(imu.field_angular_velocity_x);
xlabel('angular velocity x rad/s'),ylabel('number of samples'),title('count histogram of angular velocity x'),grid on
subplot(3,3,2)
histogram(imu.field_angular_velocity_y);
xlabel('angular velocity y rad/s'),ylabel('number of samples'),title('count histogram of angular velocity y'),grid on
subplot(3,3,3)
histogram(imu.field_angular_velocity_z);
xlabel('angular velocity z rad/s'),ylabel('number of samples'),title('count histogram of angular velocity z'),grid on

subplot(3,3,4)
histogram(imu.field_linear_acceleration_x);
xlabel('linear acceleration x m/s^2'),ylabel('number of samples'),title('count histogram of linear acceleration x'),grid on
subplot(3,3,5)
histogram(imu.field_linear_acceleration_y);
xlabel('linear acceleration y m/s^2'),ylabel('number of samples'),title('count histogram of linear acceleration y'),grid on
subplot(3,3,6)
histogram(imu.field_linear_acceleration_z);
xlabel('linear acceleration z m/s^2'),ylabel('number of samples'),title('count histogram of linear acceleration z'),grid on

subplot(3,3,7)
histogram(magnetic_field.field_magnetic_field_x);
xlabel('magnetic flux density x Gauss'),ylabel('number of samples'),title('count histogram of magnetic flux density x'),grid on
subplot(3,3,8)
histogram(magnetic_field.field_magnetic_field_y);
xlabel('magnetic flux density y Gauss'),ylabel('number of samples'),title('count histogram of magnetic flux density y'),grid on
subplot(3,3,9)
histogram(magnetic_field.field_magnetic_field_z);
xlabel('magnetic flux density z Gauss'),ylabel('number of samples'),title('count histogram of magnetic flux density z'),grid on


%% mean and variance calculation
% angular velocity
mean_angular_vel_x = mean(imu.field_angular_velocity_x);
variance_angular_vel_x = var(imu.field_angular_velocity_x);
disp(['mean of angular velocity x is ',num2str(mean_angular_vel_x)])
disp(['variance of angular velocity x is ',num2str(variance_angular_vel_x)])

mean_angular_vel_y = mean(imu.field_angular_velocity_y);
variance_angular_vel_y = var(imu.field_angular_velocity_y);
disp(['mean of angular velocity y is ',num2str(mean_angular_vel_y)])
disp(['variance of angular velocity y is ',num2str(variance_angular_vel_y)])

mean_angular_vel_z = mean(imu.field_angular_velocity_z);
variance_angular_vel_z = var(imu.field_angular_velocity_z);
disp(['mean of angular velocity z is ',num2str(mean_angular_vel_z)])
disp(['variance of angular velocity z is ',num2str(variance_angular_vel_z)])

% linear acceleration
mean_linear_accel_x = mean(imu.field_linear_acceleration_x);
variance_linear_accel_x = var(imu.field_linear_acceleration_x);
disp(['mean of linear acceleration x is ',num2str(mean_linear_accel_x)])
disp(['variance of linear acceleration x is ',num2str(variance_linear_accel_x)])

mean_linear_accel_y = mean(imu.field_linear_acceleration_y);
variance_linear_accel_y = var(imu.field_linear_acceleration_y);
disp(['mean of linear acceleration y is ',num2str(mean_linear_accel_y)])
disp(['variance of linear acceleration y is ',num2str(variance_linear_accel_y)])

mean_linear_accel_z = mean(imu.field_linear_acceleration_z);
variance_linear_accel_z = var(imu.field_linear_acceleration_z);
disp(['mean of linear acceleration z is ',num2str(mean_linear_accel_z)])
disp(['variance of linear acceleration z is ',num2str(variance_linear_accel_z)])

% magnetic flux density
mean_mag_x = mean(magnetic_field.field_magnetic_field_x);
variance_mag_x = var(magnetic_field.field_magnetic_field_x);
disp(['mean of magnetic flux density x is ',num2str(mean_mag_x)])
disp(['variance of magnetic flux density x is ',num2str(variance_mag_x)])

mean_mag_y = mean(magnetic_field.field_magnetic_field_y);
variance_mag_y = var(magnetic_field.field_magnetic_field_y);
disp(['mean of magnetic flux density y is ',num2str(mean_mag_y)])
disp(['variance of magnetic flux density y is ',num2str(variance_mag_y)])

mean_mag_z = mean(magnetic_field.field_magnetic_field_z);
variance_mag_z = var(magnetic_field.field_magnetic_field_z);
disp(['mean of magnetic flux density z is ',num2str(mean_mag_z)])
disp(['variance of magnetic flux density z is ',num2str(variance_mag_z)])

%%
q = [imu.field_orientation_w,imu.field_orientation_x,...
    imu.field_orientation_y,imu.field_orientation_z];
angle = quat2eul(q,'ZYX');
angle_degree = rad2deg(angle);
angle_degree_180 = wrapTo180(angle_degree);
figure
plot(imu.x_time,angle_degree_180(:,1));grid on,hold on
plot(imu.x_time,angle_degree_180(:,2))
plot(imu.x_time,angle_degree_180(:,3)),legend('yaw','pitch','roll')