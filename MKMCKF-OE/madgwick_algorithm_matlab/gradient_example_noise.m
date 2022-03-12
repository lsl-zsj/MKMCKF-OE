function gradient_example_noise()

% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches ?90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected

%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data

load('sensorlog_20200313_204454.mat');
Accelerometer = [Acceleration.X,Acceleration.Y,Acceleration.Z];
Gyroscope =[AngularVelocity.X,AngularVelocity.Y,AngularVelocity.Z];
Magnetometer=[MagneticField.X,MagneticField.Y,MagneticField.Z];
elu_phone=[Orientation.X,Orientation.Y,Orientation.Z];
len=length(Accelerometer);
fs=100;
time=0:1/fs:1/fs*(len-1);
figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.1);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
%     AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
     AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.




%% calculate angle using imufilter

data=orientation_estimation_fun(Accelerometer,Gyroscope,fs);
elu=data.elu;


figure('Name', 'Euler Angles');
hold on;
x1=subplot(3,1,1);
plot(time, euler(:,1),time,elu(:,3) ,time, -elu_phone(:,2), 'r');
legend('gradient','matlab','phone')
x2=subplot(3,1,2);
plot(time, euler(:,2),time,elu(:,2), time, elu_phone(:,3),'g');
legend('gradient','matlab','phone')
x3=subplot(3,1,3);
plot(time, euler(:,3),time,elu(:,1),time, -elu_phone(:,1)-90,'b');
legend('gradient','matlab','phone')
xlabel('Time (s)');
ylabel('Angle (deg)');
linkaxes([x1,x2,x3],'x');
hold off;
%% End of script

% figure('Name', 'Euler Angles');
% hold on;
% angle_g=sqrt(euler(:,1).^2+euler(:,2).^2);
% angle_m=sqrt(elu(:,3).^2+elu(:,2).^2);
% angle_p=sqrt((-elu_phone(:,2)).^2+(elu_phone(:,3)).^2)
% x1=subplot(2,1,1);
% plot(time, angle_g,time,angle_m ,time, angle_p, 'r');
% legend('gradient','matlab','phone')
% xlabel('Time (s)');
% ylabel('Angle (deg)');
% x2=subplot(2,1,2);
% plot(time, euler(:,3),time,elu(:,1),time, -elu_phone(:,1)-90,'b');
% legend('gradient','matlab','phone')
% xlabel('Time (s)');
% ylabel('Angle (deg)');
% linkaxes([x1,x2],'x');
% hold off;



end
