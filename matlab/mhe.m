%% load data
clc;clear;
addpath(genpath(pwd))
prog = @progation_lineardrag;
lopt = 38;
data = load('sample.mat'); kappa=[1.2;2.4;4];
useflow=1;

%% 25 Hz, store with mat filetv
time = data.time;       % time
gtd = data.gtd;        % position from vicon
uwb = data.uwb;        % uwb distance
flow = data.flow;
imu = data.imu;        % imu data
dt = mean(diff(time));
vel = gtd(4:6,:);
y = uwb;
%%
%------------- preprocessing-----------------%
%% initialize
delta = 3; opt_range = length(time);
xt = gtd(:,1:lopt+delta);
k=1;
tic
for i = k*lopt+delta+1:opt_range  % i: current time-step
    disp(['Step ' , num2str(i)])
    x1_ = xt(:,i-k*lopt-1:k:i-1); % estimated initial value
    MHE_imu = imu(:,i-k*lopt:i-1);
    MHE_uwb = y(i-lopt*k:i);
    MHE_flow = flow(:,i-lopt*k:i);
    sms = solveonce(x1_, MHE_imu, MHE_uwb, MHE_flow, dt, kappa, k,useflow);
    xt(:,i) = sms(:,end);
    if(i+2>length(y))
        break
    end
end
save('results')
disp(['Done'])
toc

plot_position_estimation
