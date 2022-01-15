% 3D Quadrotor Control

% Using Linearised Equations 

function [F, M] = controller(t, state, des_state, params)

%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust

% PD Constants for Position
Kp_pos = [200; 200; 100];
Kd_pos = [40; 40; 20];

% Errors in Position Control
e_pos = des_state.pos - state.pos;
e_vel = des_state.vel - state.vel;

% Command Acceleration
cmd_acc = des_state.acc + Kd_pos.*(e_vel) + Kp_pos.*(e_pos);

% Control Variable -> Thrust
F = params.mass*(params.gravity + cmd_acc(3));

% Moment
M = zeros(3, 1);

% PD Constants for Attitude
Kp_att = [100; 100; 100];
Kd_att = [2; 2; 2];

% Desired Pitch
phi_des = (1/params.gravity)*(cmd_acc(1)*sin(des_state.yaw) - cmd_acc(2)*cos(des_state.yaw));

% Desired Roll
theta_des = (1/params.gravity)*(cmd_acc(1)*cos(des_state.yaw) + cmd_acc(2)*sin(des_state.yaw));

% Desired Angles
rot_des = [phi_des; theta_des; des_state.yaw];

% Desired Angular Velocities
omega_des = [0; 0; des_state.yawdot];

% Errors in Attitude Control
e_rot = rot_des - state.rot;
e_omega = omega_des - state.omega;

% Control Variable -> Moment
M = Kp_att.*(e_rot) + Kd_att.*(e_omega);

% =================== Your code ends here ===================

end