%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:       Daniel Ossig
%   edited by:  Kevin Kurzenberger (06.08.2018 MATLAB 2018a)
%
% Date:         26.07.2018
%
% Description:  This script initializes the SIMULINK model 'experiment.slx'
%               containing the ideal linear KF vor validation.
%
% MATLAB Vers.: 2017a
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%clear variables        % should be done separately in other file 

addpath('lstm')

%% build continuous state space model (linear single track model)

% get vehicle parameters
[I_zz,c_alpha_fl,c_alpha_fr,c_alpha_rl,c_alpha_rr,l_f,l_r,m,v] = getParameter;

% get continuous state space matrices 
A = lstm1_getContinuousA(I_zz,c_alpha_fl,c_alpha_fr,c_alpha_rl,c_alpha_rr,l_f,l_r,m,v);

B = lstm1_getContinuousB(I_zz,c_alpha_fl,c_alpha_fr,c_alpha_rl,c_alpha_rr,l_f,l_r,m,v);

C_psi_dot = lstm1_getContinuousC_psi_dot;
C_ay = lstm1_getContinuousC_ay(c_alpha_fl,c_alpha_fr,c_alpha_rl,c_alpha_rr,l_f,l_r,m,v);
C=[C_psi_dot;C_ay];

D_psi_dot = lstm1_getContinuousD_psi_dot;
D_ay = lstm1_getContinuousD_ay(c_alpha_fl,c_alpha_fr,c_alpha_rl,c_alpha_rr,m);
D=[D_psi_dot;D_ay];

%% build discrete state space model
td = 0.001;
sys = ss(A,B,C,D);
sysd = c2d(sys,td);

Ad=sysd.a;
Bd=sysd.b;
Cd=sysd.c;
Dd=sysd.d;

%% noise definition

% real noise of the continuous system
% covariances
Q_real=diag([0.005,0.001]);     % Process-Noise Covariance
R_real=diag([0.05,0.03]);       % Measurement-Noise Covariance
N_real=zeros(2,2);
% mean
qm_real = [0.01;0.01]*0; 
rm_real = [0.1;0.1]*0;

% real noise of the discrete system
Qd_real=Q_real*td^2;
Rd_real=R_real;
Nd_real=N_real;
% mean
qmd_real = qm_real*td;
rmd_real = rm_real;

% kalman filter noise
% create Noise Factor-Variable if not done by invoking script
if ~exist('noise_factor','var')
    noise_factor = 1;       % real noise
end
Qd_kf=Qd_real*noise_factor;
Rd_kf=Rd_real;
Nd_kf=Nd_real;

%% set manueover

% set input
delta_f = deg2rad(5);
delta_r = 0;
M_z = 0;

% build input vector
u = lstm1_buildU(delta_f,delta_r,M_z);

% create Offset-Variable if not done by invoking script
if ~exist('Simulated_Sensor_Offset','var')
    Simulated_Sensor_Offset = [0;...        % yaw rate-Offset
                                0];         % lateral acceleration-Offset
end

% set initial state
psi_dot = 0;
beta = 0;
x0 = lstm1_buildX(psi_dot,beta);

% set initial kalman state
x0_hat = x0;
P0 = zeros(2,2);

%% simulate model
sim('experiment')

%% Logged Signals:
x       = logsout.getElement('xbus').Values.x;          % Car States: 1: Gierrate, 2: Schwimmwinkel
x_hat   = logsout.getElement('xbus').Values.x_hat;      % predicted by KF: 1: Gierrate, 2: Schwimmwinkel 
y       = logsout.getElement('ybus').Values.y;          % Car States: 1: Gierrate, 2: Querbeschleunigung
y_hat   = logsout.getElement('ybus').Values.y_hat;      % predicted by KF: 1: Gierrate, 2: Querbeschleunigung
z       = logsout.getElement('zbus').Values.z;          % Sensor values: 1: Gierrate, 2: Querbeschleunigung
S       = logsout.getElement('S').Values;               % Covariance-Matrix of the innovation-sequence

% calculate covariance of the systems output (for Q=0, R_sim --> R_real)
R_sim = cov(z.Data(:,1),z.Data(:,2));                % Measurement-Noise Covariance


% plot solutions
plot_active = 1;            % set to 1 if plots should be plotted

if plot_active == 1
    figure('name','states')
    subplot(2,1,1)
        hold on
        plot(x.Time,x.Data(1,:))
        plot(x_hat.Time,x_hat.Data(1,:))
        title('Gierrate')
        legend('Car','KF')
    subplot(2,1,2)
        hold on
        plot(x.Time,x.Data(2,:))
        plot(x_hat.Time,x_hat.Data(2,:))
        title('Schwimmwinkel')
        legend('Car','KF')
    
    figure('name','measurement')
    subplot(2,1,1)
        hold on
        plot(z.Time,z.Data(1,:))
        plot(y_hat.Time,y_hat.Data(1,:))
     plot(y.Time,y.Data(1,:),'-.')
        title('Gierrate')
        legend('Sensor','KF','Car')
    subplot(2,1,2)
        hold on
        plot(z.Time,z.Data(2,:))
        plot(y_hat.Time,y_hat.Data(2,:))
        plot(y.Time,y.Data(2,:),'-.')
        title('Querbeschleunigung')
        legend('Sensor','KF','Car')
end