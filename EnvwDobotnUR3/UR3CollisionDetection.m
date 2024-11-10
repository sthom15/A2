function Ass2EnvironmentWithCollisionDetection
    % Clear previous simulations
    close all; clear; clc;

    % Set up the environment
    figure;
    axis([-3, 3, -3, 3, 0, 2]);
    hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Robot Serving Simulation with Collision Detection');

    % Add floor texture
    surf([-3, -3; 3, 3], [-3, 3; -3, 3], [0, 0; 0, 0], ...
        'CData', imread('floor.jpg'), 'FaceColor', 'texturemap');

    % Place environment objects
    PlaceObject('table2.ply', [-0.5, 1, 0]);
    PlaceObject('counter2.ply', [1, -1, 0]);
    PlaceObject('stool3.ply', [-1, -2, 0]);
    PlaceObject('stool3.ply', [0, -2, 0]);
    PlaceObject('stool3.ply', [1, -2, 0]);
    PlaceObject('stool3.ply', [2, -1, 0]);
    PlaceObject('stool3.ply', [2, 0, 0]);
    PlaceObject('stool3.ply', [2, 1, 0]);
    PlaceObject('stool3.ply', [2, 2, 0]);
    PlaceObject('personMaleCasual.ply', [-1.5, 1.5, 0]);

    % Place safety equipment
    PlaceObject('fence.ply', [-1, -0.6, 0]);
    PlaceObject('fence.ply', [-1, 2.5, 0]);
    PlaceObject('emergencyStopButton.ply', [-0.8, 1.3, 0.8]);

    % Red Solo Cup setup
    RedSoloCup = PlaceObject('RedSoloCup.ply', [0, 0, 0]);
    RedSoloCupVertices = get(RedSoloCup, 'Vertices');
    cupInitialPosition = transl(1.2, 1.2, 1.0);
    transformedVertices = [RedSoloCupVertices, ones(size(RedSoloCupVertices, 1), 1)] * cupInitialPosition';
    set(RedSoloCup, 'Vertices', transformedVertices(:, 1:3));

    % Dobot Setup
    r_dobot = dobot();
    r_dobot.model.base = transl(0.0, 2.0, 1.0);
    q0_dobot = [pi/6, -pi/2, pi/3, 0, -pi/4, pi/2];
    r_dobot.model.animate(q0_dobot);

    % UR3 Setup
    r_ur3 = UR3();
    rail_base_position = transl(-0.3, 1, 1.0);
    r_ur3.model.base = rail_base_position;
    q0_ur3 = zeros(1, 6);
    r_ur3.model.animate(q0_ur3);

    % Draw the rail for UR3 visualization
    plot3([-0.3, -0.3], [1, -1], [1.0, 1.0], 'k-', 'LineWidth', 3);

    % Dobot pouring task
    dobotPouringTask(r_dobot);

    % UR3 pick-and-place task with collision detection
    executeUR3PickPlace(r_ur3, RedSoloCup, RedSoloCupVertices);

    %% Dobot pouring task function
    function dobotPouringTask(robot)
        disp('Starting simulated Dobot pouring task...');
        pourPos1 = transl(0.2, 2.0, 1.2);
        pourPos2 = transl(0.0, 1.8, 1.2);
        steps = 50;

        % Move Dobot to pour positions
        qMatrix1 = jtraj(q0_dobot, robot.model.ikcon(pourPos1), steps);
        qMatrix2 = jtraj(robot.model.ikcon(pourPos1), robot.model.ikcon(pourPos2), steps);

        for i = 1:steps
            robot.model.animate(qMatrix1(i, :));
            pause(0.05);
        end
        for i = 1:steps
            robot.model.animate(qMatrix2(i, :));
            pause(0.05);
        end

        disp('Pouring complete.');
    end

    %% UR3 pick-and-place function
    function executeUR3PickPlace(robot, cupHandle, cupVertices)
        pickupPos = transl(-0.2, 1, 1.1);
        placePos = transl(-0.3, -1, 1.0);

        % Move to pickup position and grip
        moveUR3OnSimulatedRail(robot, rail_base_position, pickupPos, 'grip', cupHandle, cupVertices);

        % Move to place position and release
        moveUR3OnSimulatedRail(robot, transl(-0.3, -1.0, 1.0), placePos, 'release', cupHandle, cupVertices);

        disp('UR3 task complete.');
    end

    %% Move UR3 on simulated rail with collision detection
    function moveUR3OnSimulatedRail(robot, baseTarget, targetTransform, action, cupHandle, cupVertices)
        steps = 50;
        qCurrent = robot.model.getpos();
        qTarget = robot.model.ikcon(targetTransform, qCurrent);

        for i = 1:steps
            s = i / steps;
            interpolatedBase = trinterp(rail_base_position, baseTarget, s);
            robot.model.base = interpolatedBase;

            % Interpolate joint angles
            qInterp = qCurrent * (1 - s) + qTarget * s;

            % Check joint limits
        if any(qInterp < robot.model.qlim(:, 1), 'all') || any(qInterp > robot.model.qlim(:, 2), 'all')


                disp('Joint limits exceeded. Stopping simulation.');
                return;
            end

            % Animate robot
            robot.model.animate(qInterp);

            % Update cup position
            endEffectorPos = robot.model.fkine(qInterp).t;
            transformedVertices = [cupVertices, ones(size(cupVertices, 1), 1)] * transl(endEffectorPos)';
            set(cupHandle, 'Vertices', transformedVertices(:, 1:3));

            % Collision detection
            if checkCollisionWithTable(robot, endEffectorPos)
                disp('Collision detected with table! Stopping simulation...');
                return;
            end

            pause(0.05);
        end
    end

    %% Collision detection function
    function collision = checkCollisionWithTable(robot, endEffectorPos)
        collision = false;
        tableXBounds = [-0.8, 0.8];
        tableYBounds = [1.0, 1.6];
        tableZHeight = 1.0;

        if endEffectorPos(1) > tableXBounds(1) && endEffectorPos(1) < tableXBounds(2) && ...
           endEffectorPos(2) > tableYBounds(1) && endEffectorPos(2) < tableYBounds(2) && ...
           endEffectorPos(3) < tableZHeight + 0.05
            collision = true;
        end
    end


 %% Gui Code
       qc = q0_ur3; % Joint configuration
% Initialize the robot models and joint configurations
qc_ur3 = q0_ur3; % Initial joint configuration for UR3
qc_dobot = q0_dobot; % Initial joint configuration for Dobot

% Set up the end-effector initial transformation and position for both robots
end_effector_pos_ur3 = transl(r_ur3.model.fkine(qc_ur3)); % UR3
end_effector_pos_dobot = transl(r_dobot.model.fkine(qc_dobot)); % Dobot
% Display debugging info
disp('Starting GUI and initializing end-effector positions for UR3 and Dobot...');

   % GUI Controls for Emergency Stop and Resume
    uicontrol('Style', 'pushbutton', 'Position', [600, 100, 100, 40], ...
              'String', 'E-Stop', 'Callback', @emergency_stop);
    uicontrol('Style', 'pushbutton', 'Position', [600, 50, 100, 40], ...
              'String', 'Resume', 'Callback', @resume_operation);
% Cartesian position inputs for UR3
uicontrol('Style', 'text', 'Position', [50, 300, 100, 20], 'String', 'UR3 X Position');
x_input_ur3 = uicontrol('Style', 'edit', 'Position', [150, 300, 100, 20], ...
                        'String', num2str(end_effector_pos_ur3(1)), 'Callback', @(src, ~) update_cartesian_position_ur3(src, 1));
uicontrol('Style', 'text', 'Position', [50, 270, 100, 20], 'String', 'UR3 Y Position');
y_input_ur3 = uicontrol('Style', 'edit', 'Position', [150, 270, 100, 20], ...
                        'String', num2str(end_effector_pos_ur3(2)), 'Callback', @(src, ~) update_cartesian_position_ur3(src, 2));
uicontrol('Style', 'text', 'Position', [50, 240, 100, 20], 'String', 'UR3 Z Position');
z_input_ur3 = uicontrol('Style', 'edit', 'Position', [150, 240, 100, 20], ...
                        'String', num2str(end_effector_pos_ur3(3)), 'Callback', @(src, ~) update_cartesian_position_ur3(src, 3));

% Cartesian position inputs for Dobot
uicontrol('Style', 'text', 'Position', [300, 300, 100, 20], 'String', 'Dobot X Position');
x_input_dobot = uicontrol('Style', 'edit', 'Position', [400, 300, 100, 20], ...
                          'String', num2str(end_effector_pos_dobot(1)), 'Callback', @(src, ~) update_cartesian_position_dobot(src, 1));
uicontrol('Style', 'text', 'Position', [300, 270, 100, 20], 'String', 'Dobot Y Position');
y_input_dobot = uicontrol('Style', 'edit', 'Position', [400, 270, 100, 20], ...
                          'String', num2str(end_effector_pos_dobot(2)), 'Callback', @(src, ~) update_cartesian_position_dobot(src, 2));
uicontrol('Style', 'text', 'Position', [300, 240, 100, 20], 'String', 'Dobot Z Position');
z_input_dobot = uicontrol('Style', 'edit', 'Position', [400, 240, 100, 20], ...
                          'String', num2str(end_effector_pos_dobot(3)), 'Callback', @(src, ~) update_cartesian_position_dobot(src, 3));

% Joint angle control for UR3
joint_index_ur3 = 1; % Default joint index for UR3

uicontrol('Style', 'text', 'Position', [50, 200, 100, 20], 'String', 'UR3 Select Joint');
joint_dropdown_ur3 = uicontrol('Style', 'popupmenu', 'Position', [150, 200, 100, 20], ...
                               'String', {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'}, ...
                               'Callback', @update_joint_index_ur3);
uicontrol('Style', 'text', 'Position', [50, 170, 100, 20], 'String', 'UR3 Joint Angle');
joint_input_ur3 = uicontrol('Style', 'edit', 'Position', [150, 170, 100, 20], ...
                            'String', num2str(qc_ur3(joint_index_ur3)), 'Callback', @(src, ~) update_joint_angle_ur3(src));
angle_display_ur3 = uicontrol('Style', 'text', 'Position', [260, 170, 100, 20], 'String', 'Angle: 0');

% Joint angle control for Dobot
joint_index_dobot = 1; % Default joint index for Dobot

uicontrol('Style', 'text', 'Position', [300, 200, 100, 20], 'String', 'Dobot Select Joint');
joint_dropdown_dobot = uicontrol('Style', 'popupmenu', 'Position', [400, 200, 100, 20], ...
                                 'String', {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7', 'Joint 8'}, ...
                                 'Callback', @update_joint_index_dobot);
uicontrol('Style', 'text', 'Position', [300, 170, 100, 20], 'String', 'Dobot Joint Angle');
joint_input_dobot = uicontrol('Style', 'edit', 'Position', [400, 170, 100, 20], ...
                              'String', num2str(qc_dobot(joint_index_dobot)), 'Callback', @(src, ~) update_joint_angle_dobot(src));
angle_display_dobot = uicontrol('Style', 'text', 'Position', [510, 170, 100, 20], 'String', 'Angle: 0');

%% Callback Functions for UR3
function update_cartesian_position_ur3(src, axis)
    new_value = str2double(src.String);
    if isnan(new_value)
        disp('Please enter a valid number for UR3.');
        return;
    end
    end_effector_pos_ur3(axis) = new_value;

    target_tform = transl(end_effector_pos_ur3);
    new_qc = r_ur3.model.ikcon(target_tform, qc_ur3);

    if isempty(new_qc)
        disp('UR3: No valid joint configuration found for the target position.');
        return;
    end

    steps = 50;
    qMatrix = jtraj(qc_ur3, new_qc, steps);

    for i = 1:steps
        r_ur3.model.animate(qMatrix(i, :));
        pause(0.05);
    end
    qc_ur3 = new_qc;
    disp('UR3 moved to new position.');
end

function update_joint_index_ur3(src, ~)
    joint_index_ur3 = src.Value;
    joint_input_ur3.String = num2str(qc_ur3(joint_index_ur3));
end

function update_joint_angle_ur3(src)
    joint_angle = str2double(src.String);
    qc_ur3(joint_index_ur3) = joint_angle;
    r_ur3.model.animate(qc_ur3);
end

%% Callback Functions for Dobot
function update_cartesian_position_dobot(src, axis)
    new_value = str2double(src.String);
    if isnan(new_value)
        disp('Please enter a valid number for Dobot.');
        return;
    end
    end_effector_pos_dobot(axis) = new_value;

    target_tform = transl(end_effector_pos_dobot);
    new_qc = r_dobot.model.ikcon(target_tform, qc_dobot);

    if isempty(new_qc)
        disp('Dobot: No valid joint configuration found for the target position.');
        return;
    end

    steps = 50;
    qMatrix = jtraj(qc_dobot, new_qc, steps);

    for i = 1:steps
        r_dobot.model.animate(qMatrix(i, :));
        pause(0.05);
    end
    qc_dobot = new_qc;
    disp('Dobot moved to new position.');
end

function update_joint_index_dobot(src, ~)
    joint_index_dobot = src.Value;
    joint_input_dobot.String = num2str(qc_dobot(joint_index_dobot));
end

function update_joint_angle_dobot(src)
    joint_angle = str2double(src.String);
    qc_dobot(joint_index_dobot) = joint_angle;
    r_dobot.model.animate(qc_dobot);
end

           function emergency_stop(~, ~)
        isPaused = true;  % Set the pause flag
        if isStopped == false; % Track that E-Stop was triggered
        disp('Emergency Stop activated.');
        isStopped = true;
        else
                  isStopped = false;
          disp('Emergency Stop Deactivated.');

        end

    end

    % Callback for Resume button
    function resume_operation(~, ~)
        if isStopped == false;
            isPaused = false; % Clear the pause flag
            disp('Resuming operation...');
    move_robot();

        end
        if section3run == true;
            section1run = false
            section2run = false
            section3run = false
        end
             if section3rund == true;
            section1rund = false
            section2rund = false
            section3rund = false
        end
    end
  end