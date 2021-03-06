function V_a_i_cog0_y = lstm1_getay(beta,c_alpha_fl,c_alpha_fr,c_alpha_rl,c_alpha_rr,delta_f,delta_r,l_f,l_r,m,psi_dot,v)
%LSTM1_GETAY
%    V_A_I_COG0_Y = LSTM1_GETAY(BETA,C_ALPHA_FL,C_ALPHA_FR,C_ALPHA_RL,C_ALPHA_RR,DELTA_F,DELTA_R,L_F,L_R,M,PSI_DOT,V)

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
t2 = 1.0./v;
t3 = l_f.*psi_dot.*t2;
t4 = beta+t3;
t5 = beta-l_r.*psi_dot.*t2;
V_a_i_cog0_y = psi_dot.*v-(-c_alpha_fl.*delta_f-c_alpha_fr.*delta_f-c_alpha_rl.*delta_r-c_alpha_rr.*delta_r+c_alpha_fl.*t4+c_alpha_fr.*t4+c_alpha_rl.*t5+c_alpha_rr.*t5+m.*psi_dot.*v)./m;
