function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_sta te.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;
Kp1=300;
Kv1=200;
Kp2=30;
Kv2=0.3;
Kp3=4000;
Kv3=30;
u1=min(2*params.maxF,params.mass*(params.gravity+des_state.acc(2,1)+Kv1*(des_state.vel(2,1)-state.vel(2,1))+Kp1*(des_state.pos(2,1)-state.pos(2,1))));
phi_C=(-1/params.gravity)*(des_state.acc(1,1)+Kv2*(des_state.vel(1,1)-state.vel(1,1))+Kp2*(des_state.pos(1,1)-state.pos(1,1)));
u2=min(params.maxF*params.arm_length,params.Ixx*(Kp3*(phi_C-state.rot(1,1))+Kv3*(-state.omega(1,1))));
% FILL IN YOUR CODE HERE

end

