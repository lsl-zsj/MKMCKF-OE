function data_processing()

% load('gait_linear6.mat')
% load('gait_0.2.mat')
% load('gait_0.2_mag.mat')
% load('gait_0.3.mat')
% load('gait_0.3_mag.mat')
% load('gait_0.4.mat')
% load('gait_0.4_mag.mat')
% load('enc_gait_0.2.mat')
% load('enc_gait_0.2_magd.mat')
% load('enc_gait_0.3.mat')
% load('enc_gait_0.3_magd.mat')
% load('enc_gait_0.4.mat')
% load('enc_gait_0.4_magd.mat')



addpath('..\data');

%%
load('gait_linear6.mat')

gait1=gait;
clear gait

gait.fs=100;
gait.Acceleration=-gait1.Acceleration(1:4:end,:);
gait.Gyroscope=gait1.Gyroscope(1:4:end,:);
gait.Magnetic=100*gait1.Magnetic(1:4:end,:);

save gait_linear6_100hz.mat gait

%%
clear all
load('gait_0.2.mat')

gait.fs=100;
gait.Acceleration=-gait_02.Acceleration(1:4:end,:);
gait.Gyroscope=gait_02.Gyroscope(1:4:end,:);
gait.Magnetic=100*gait_02.Magnetic(1:4:end,:);

save gait_02_100hz.mat gait

%%

clear all
load('gait_0.2_mag.mat')

gait.fs=100;
gait.Acceleration=-gait_02_mag.Acceleration(1:4:end,:);
gait.Gyroscope=gait_02_mag.Gyroscope(1:4:end,:);
gait.Magnetic=100*gait_02_mag.Magnetic(1:4:end,:);

save gait_02_mag_100hz.mat gait
plot(gait.Magnetic,'DisplayName','gait.Magnetic')

%%
clear all
load('gait_0.3.mat')

gait.fs=100;
gait.Acceleration=-gait_03.Acceleration(1:4:end,:);
gait.Gyroscope=gait_03.Gyroscope(1:4:end,:);
gait.Magnetic=100*gait_03.Magnetic(1:4:end,:);

save gait_03_100hz.mat gait
plot(gait.Magnetic,'DisplayName','gait.Magnetic')

%%
clear all
load('gait_0.3_mag.mat')

gait.fs=100;
gait.Acceleration=-gait_03_mag.Acceleration(1:4:end,:);
gait.Gyroscope=gait_03_mag.Gyroscope(1:4:end,:);
gait.Magnetic=100*gait_03_mag.Magnetic(1:4:end,:);

save gait_03_mag_100hz.mat gait
plot(gait.Magnetic,'DisplayName','gait.Magnetic')



%%
clear all
load('gait_0.4.mat')

gait.fs=100;
gait.Acceleration=-gait_04.Acceleration(1:4:end,:);
gait.Gyroscope=gait_04.Gyroscope(1:4:end,:);
gait.Magnetic=100*gait_04.Magnetic(1:4:end,:);

save gait_04_100hz.mat gait
plot(gait.Magnetic,'DisplayName','gait.Magnetic')



%%
clear all
load('gait_0.4_mag.mat')

gait.fs=100;
gait.Acceleration=-gait_04_mag.Acceleration(1:4:end,:);
gait.Gyroscope=gait_04_mag.Gyroscope(1:4:end,:);
gait.Magnetic=100*gait_04_mag.Magnetic(1:4:end,:);

save gait_04_mag_100hz.mat gait
plot(gait.Magnetic,'DisplayName','gait.Magnetic')


end