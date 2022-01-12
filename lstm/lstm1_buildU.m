function u = lstm1_buildU(M_z,delta_f,delta_r)
%LSTM1_BUILDU
%    U = LSTM1_BUILDU(M_Z,DELTA_F,DELTA_R)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    27-Aug-2018 16:16:26

% 
% Linear Single Track Model lstm1
% 
% STATES
% x=[psi_dot;beta] (yaw rate and side slip angle)
% INPUTS
% u=[delta_f;delta_r;M_z] (wheel steering angles, additional yaw moment)
% OUTPUTS
% psi_dot, beta, ay --> use C_psi_dot D_psi_dot, C_beta D_beta, C_ay D_ay
% INFORMATION on parameters
% the cornering stiffness parameters are "tire-wise" so there is no confusion weather it has to be multiplied by 2 or not
% 
% x_dot = A*x + B*u
% y     = C*x + D*u
% 
% Single Track Modell with
% - front steering
% - rear steering
% - cog on road height
% - small angle approximation
% - constant v
% - x = [psi_dot; beta]
% 
u = [delta_f;delta_r;M_z];
