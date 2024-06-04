vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID > -1)
    disp('Connected to remote API server');
    [~, left_motor] = vrep.simxGetObjectHandle(clientID, 'motor_left', vrep.simx_opmode_blocking);
    [~, right_motor] = vrep.simxGetObjectHandle(clientID, 'motor_right', vrep.simx_opmode_blocking);
    [~, gps] = vrep.simxGetObjectHandle(clientID, 'GPS', vrep.simx_opmode_blocking);
    [~, gyro] = vrep.simxGetObjectHandle(clientID, 'GyroSensor', vrep.simx_opmode_blocking);
    [~, Orierobot] = vrep.simxGetObjectOrientation(clientID, gyro, -1, vrep.simx_opmode_streaming);
    [~, Posirobot] = vrep.simxGetObjectPosition(clientID, gps, -1, vrep.simx_opmode_streaming);


    pathway = getPath;
    pathway = pathway/4;
    disp(pathway);
    % PID controller parameters
    Kp = 1500;       % Increased Proportional gain
    Ki = 0.09;      % Integral gain
    Kd = 30;       % Increased Derivative gain
    
    previous_error = 0;
    integral_error = 0;
    dt = 0.05;  % Set the sampling time
    
    for i = 1:size(pathway, 1)
        goal = pathway(i, :);
        while (true)
            
    
            a = move2goal(clientID, vrep, left_motor, right_motor, gps, gyro, goal, Kp, Ki, Kd, previous_error, integral_error, dt);
            previous_error = a.error;
            integral_error = a.integral_error;
%               % Ensure the loop runs at the correct sampling time
            [~, robotPos] = vrep.simxGetObjectPosition(clientID, gps, -1, vrep.simx_opmode_buffer);
            robotOxy = [robotPos(1), robotPos(2)];
            dist2goal = norm(goal - robotOxy);
            if (dist2goal < 0.1)
                [~] = vrep.simxSetJointTargetVelocity(clientID, left_motor, 0, vrep.simx_opmode_oneshot);
                [~] = vrep.simxSetJointTargetVelocity(clientID, right_motor,0, vrep.simx_opmode_oneshot);
                break;  % Move to the next goal
            end
        end
    end
else 
    disp('Failed connecting to remote API server');
end

function [out] = move2goal(clientID, vrep, left_motor, right_motor, gps, gyro, goal, Kp, Ki, Kd, previous_error, integral_error, dt)
%% Initialize 
    d_m2g = 0.1;
    a_m2g = 70;
    R = 0.03;
    L = 0.1665;
    MAX_SPEED = 5;
%     a_tol = 0.1;  % Angular tolerance

%% Get current pos 
    [~, robotPos] = vrep.simxGetObjectPosition(clientID, gps, -1, vrep.simx_opmode_buffer);
    robotOxy = [robotPos(1), robotPos(2)];
    [~, robotHeading] = vrep.simxGetObjectOrientation(clientID, gyro, -1, vrep.simx_opmode_buffer);
    
%%
    V_m2g = goal - robotOxy;
    dist2goal = norm(V_m2g);

    if (dist2goal ~= 0)
        V_m2g = a_m2g * (V_m2g / dist2goal);
    end
%%
    if (dist2goal < d_m2g)
        isReached = true;
    else
        isReached = false;
    end
%%
    desiredOrientation = atan2(V_m2g(2), V_m2g(1));
    error = desiredOrientation - robotHeading(3);

    if (abs(error) > pi)
        if (error < 0)
            error = error + 2 * pi;
        else
            error = error - 2 * pi;
        end
    end

    integral_error = integral_error + error * dt;
    derivative_error = (error - previous_error) / dt;

    omega = Kp * error + Ki * integral_error + Kd * derivative_error;
    v = norm(V_m2g);

    vr = (2 * v + omega * L) / (2 * R);
    vl = (2 * v - omega * L) / (2 * R);

    vr = max(min(vr, MAX_SPEED), -MAX_SPEED);
    vl = max(min(vl, MAX_SPEED), -MAX_SPEED);


    [~] = vrep.simxSetJointTargetVelocity(clientID, left_motor, vl, vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetVelocity(clientID, right_motor, vr, vrep.simx_opmode_oneshot);

  

    out.error = error;
    out.integral_error = integral_error;
end
