function A = lstm1_getContinuousA(I_zz,c_alpha_fl,c_alpha_fr,c_alpha_rl,c_alpha_rr,l_f,l_r,m,v)
%LSTM1_GETCONTINUOUSA
%    A = LSTM1_GETCONTINUOUSA(I_ZZ,C_ALPHA_FL,C_ALPHA_FR,C_ALPHA_RL,C_ALPHA_RR,L_F,L_R,M,V)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    27-Aug-2018 16:16:25

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
t2 = l_f.^2;
t3 = l_r.^2;
t4 = 1.0./I_zz;
t5 = c_alpha_fl.*l_f;
t6 = c_alpha_fr.*l_f;
t7 = t5+t6-c_alpha_rl.*l_r-c_alpha_rr.*l_r;
t8 = 1.0./m;
t9 = 1.0./v;
A = reshape([-t4.*t9.*(c_alpha_fl.*t2+c_alpha_fr.*t2+c_alpha_rl.*t3+c_alpha_rr.*t3),-t7.*t8.*1.0./v.^2-1.0,-t4.*t7,-t8.*t9.*(c_alpha_fl+c_alpha_fr+c_alpha_rl+c_alpha_rr)],[2,2]);
