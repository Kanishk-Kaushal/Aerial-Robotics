%   1 D QUADROTOR CONTROL

function [ u ] = controller(~, s, s_des, params, ~, ~)

    %   PD_CONTROLLER  PD controller for the height
    %
    %   s: 2x1 vector containing the current state [z; v_z]
    %   s_des: 2x1 vector containing desired state [z; v_z]
    %   params: robot parameters

    u = 0;

    % FILL IN YOUR CODE HERE

    % PD Gains
    Kp = 200;
    Kv = 20;
    Z_ddot = 0;

    % Error
    e = (s_des(1) - s(1));
    e_dot = s_des(2) - s(2);

    % Control Variable
    u = (params.mass)*(Z_ddot + (Kp*e) + (Kv*e_dot) + params.gravity);

    % u = z_ddot + Kp*e(1) + Kv*e(2) + m*g

    % Capping the Maximum Control Variable
    if (u > params.u_max)
        u = params.u_max;
    end

    % Capping the Minimum Control Variable
    if (u < params.u_min)
        u = params.u_min;
    end

end

