function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
scaling=1;
Kp=62;
Kv=9;
u=min(params.mass*(params.gravity+Kp*(s_des(1,1)-s(1,1))-Kv*(s(2,1)-s_des(2,1))),params.u_max);
end

