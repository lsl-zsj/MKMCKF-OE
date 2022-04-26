function AHRS_compare_rot2()

%%
% This function is used to compare the performance of orientation
% estimation using ESKF, GD, and MKMCKF-OE.
%
% The raw data is sampled at gait frequency f=0.2hz with 
% magnetic disturbance.
%%

clear all
%% add path
addpath('MKMCKF-OE');
addpath('ESKF');
addpath('madgwick_algorithm_matlab');
addpath('data100hz');
addpath('madgwick_algorithm_matlab/quaternion_library');

%% load the data
load('gait_02_100hz.mat');
IMU=gait;

%
Accelerometer=IMU.Acceleration;
Gyroscope=IMU.Gyroscope;
fs=IMU.fs;
Magnetic=IMU.Magnetic;

len=length(Accelerometer);
% 
for i=1:len
Accelerometer_norm(i)= norm(Accelerometer(i,:)); 
Magnetic_norm(i)= norm(Magnetic(i,:)); 
end

% plot the raw data
t=0:1/fs:1/fs*(length(Accelerometer)-1);
time=[t;t;t];
time=time';
figure
x1=subplot(3,1,1);
plot(time,Accelerometer)
legend('acc')
set(gca,'FontSize',16)
x2=subplot(3,1,2);
plot(time,Gyroscope)
legend('gyro')
set(gca,'FontSize',16)
x3=subplot(3,1,3);
plot(time,Magnetic)
legend('mag')
set(gca,'FontSize',16)
linkaxes([x1,x2,x3],'x')


%% GD method

AHRS = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', 0.1);

time=0:1/fs:1/fs*(len-1);
quat = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetic(t,:));	% gyroscope units must be radians
    quat(t, :) = AHRS.Quaternion;
end

% Plot algorithm output as Euler angles
for i=1:length(quat)
Quat(i)=quaternion(quat(i,1),quat(i,2),quat(i,3),quat(i,4));
end

euler=eulerd(Quat,'ZXY','frame');



%% ahrs
ahrs=orientation_estimation_ahrs_fun(Accelerometer,Gyroscope,Magnetic,fs);
euler_ahrs=eulerd(ahrs.Quat,'ZXY','frame');


%% mkmc ahrs


sigma_1=1.6188;
sigma_2=0.4234;

sigma1=2*sigma_1*sigma_1;
sigma2=2*sigma_2*sigma_2;
xigma_x=[10^8 10^8 10^8 10^8 10^8 10^8 sigma1 sigma1 sigma1 sigma2 sigma2 sigma2]; 
xigma_y=[10^8 10^8 10^8 10^8 10^8 10^8];
mkmc_ahrs=orientation_estimation_ahrs_mkmc_fun_(Accelerometer,Gyroscope,Magnetic,fs,xigma_x,xigma_y);
euler_mkmc_ahrs=eulerd(mkmc_ahrs.Quat,'ZXY','frame');




figure
x1=subplot(3,1,1);
plot(time,euler(:,1),'blue',time,euler_ahrs(:,1),'red',time,euler_mkmc_ahrs(:,1),'black')
legend('GD Yaw','AHRS Yaw','MKMC Yaw')
set(gca,'FontSize',12)
x2=subplot(3,1,2);
plot(time,euler(:,2),'blue',time,euler_ahrs(:,2),'red',time,euler_mkmc_ahrs(:,2),'black')
legend('GD Roll','AHRS Roll','MKMC Roll')
set(gca,'FontSize',12)
x3=subplot(3,1,3);
plot(time,euler(:,3),'blue',time,euler_ahrs(:,3),'red',time,euler_mkmc_ahrs(:,3),'black')
legend('GD Pitch','AHRS Pitch','MKMC Pitch')
set(gca,'FontSize',12)
linkaxes([x1,x2,x3],'x')





end