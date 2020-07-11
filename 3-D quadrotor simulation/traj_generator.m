function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves

% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.




%% Fill in your code here

k=[0 0;0 0;0 0;]
desired_state.pos = 0;
desired_state.vel = zeros(3,1);
desired_state.acc = zeros(3,1);
desired_state.yaw = 0;
desired_state.yawdot = 0;
persistent waypoints0 traj_time d0
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
   % x=sym('x')
    %y=sym('y')
    %z=sym('z')
    %B=[waypoints(1,1);
    %   waypoints(1,2);
     %  0;
     % 0;
     %  0;
     %  x;
     %  y;
     %  z;]
    %A=[0 0 0 0 0 0 0 1;
     %  1 1 1 1 1 1 1 1;
     % 0 0 0 0 0 0 1 0;
     %  0 0 0 0 0 1 0 0;
     %  0 0 0 0 1 0 0 0;
     %  7 6 5 4 3 2 1 0;
     %  42 30 20 12 6 2 0 0;
     %  210 120 60 24 6 0 0 0;]
   %X=inv(A)*B
   %can be done by making 4th,5th,6th derivative calculator and then
   %inputing in next matrix initial values, here we will have to take
   %x,y,x as 4th derivative.... so matrix will be  different. A is
   %constantfor all matrix and in end relation for x,y,z is obtained and
   %feeded into matrices. Above steps will take more than 100 code lines so
   %avoiding that and writing 4 lines
   
    waypoints0 = waypoints;
    
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    %if(t_index > 1)
    %    t = t - traj_time(t_index-1);
    %end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    else
       
        pp=spline(traj_time,[k(:,1) waypoints0 k(:,2)]);
 
        desired_state.pos = ppval(pp,t);
        desired_state.vel=(ppval(pp,t+.001)-ppval(pp,t))/(.001);
        desired_state.acc =zeros(3,1); 
        %(((ppval(pp,t+.001)-ppval(pp,t))/(.001))-((ppval(pp,t)-ppval(pp,t-0.001))/(.001)))/(0.001);;
        desired_state.yaw = 0;
         desired_state.yawdot = 0;
    end
end
end

