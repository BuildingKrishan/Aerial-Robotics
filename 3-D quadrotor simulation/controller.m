function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%   params: robot parameters
%   Using these current and desired states, you have to compute the desired
%   controls
% =================== Your code goes here ===================

F = 0;
M = zeros(3,1);


kp_1=9;kv_1=4.2;
kp_2=12;kv_2=4.2;
Kp_3=600;Kv_3=15;

Kp_phi=6*100;Kd_phi=1.9*sqrt(Kp_phi);
Kp_theta=6*400;Kd_theta=4*sqrt(Kp_phi);
Kp_psi=6*45;Kd_psi=1.9*sqrt(Kp_psi);

%kp_1=6;kv_1=3.4;
%kp_2=6;kv_2=3.4;
%Kp_3=6000;Kv_3=150;

%Kp_phi=6*84;Kd_phi=1.75*sqrt(Kp_phi);
%Kp_theta=6*84;Kd_theta=1.75*sqrt(Kp_phi);
%Kp_psi=6*40;Kd_psi=1.75*sqrt(Kp_psi);

%kp_1=6;kv_1=2.5;
%kp_2=6;kv_2=2.5;
%Kp_3=20;Kv_3=8;

%Kp_phi=50;Kd_phi=8;
%Kp_theta=50;Kd_theta=8;
%Kp_psi=10;Kd_psi=10;




rddot(1) = des_state.acc(1) + kv_1*(des_state.vel(1) - state.vel(1)) + kp_1*(des_state.pos(1) - state.pos(1));
rddot(2) = des_state.acc(2) + kv_2*(des_state.vel(2) - state.vel(2)) + kp_2*(des_state.pos(2) - state.pos(2));
rddot(3) = des_state.acc(3) + Kv_3*(des_state.vel(3) - state.vel(3)) + Kp_3*(des_state.pos(3) - state.pos(3));

phi_des=(1/params.gravity)*(rddot(1)*sin(des_state.yaw)-rddot(2)*cos(des_state.yaw));
theta_des=(1/params.gravity)*(rddot(1)*cos(des_state.yaw)+rddot(2)*sin(des_state.yaw));

K=[Kp_phi*(phi_des-state.rot(1,1))+Kd_phi*(-state.omega(1,1));
   Kp_theta*(theta_des-state.rot(2,1))+Kd_theta*(-state.omega(2,1));
   Kp_psi*(des_state.yaw-state.rot(3,1))+Kd_psi*(des_state.yawdot-state.omega(3,1));];
M=params.I*K;
%assuming small angles so that cos phi,theta=1
F=params.mass*(params.gravity+rddot(3));

% =================== Your code ends here ===================

end
