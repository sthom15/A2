function Ass2Environmentv6
    % Clear previous simulations
    close all; clear; clc;
    
    % Set up the environment
    figure;
    axis([-3, 3, -3, 3, 0, 2]);
    hold on;

    % Add floor texture
    surf([-3, -3; 3, 3], [-3, 3; -3, 3], [0, 0; 0, 0], ...
        'CData', imread('floor.jpg'), 'FaceColor', 'texturemap');

    % Place the main table and environment objects
    PlaceObject('table2.ply', [-0.5, 1, 0]);    % Main table
    PlaceObject('counter2.ply', [1, -1, 0]);    % Side counter
    PlaceObject('stool3.ply', [-1, -2, 0]);     % Stools
    PlaceObject('stool3.ply', [0, -2, 0]);
    PlaceObject('stool3.ply', [1, -2, 0]);
    PlaceObject('stool3.ply', [2, -1, 0]);
    PlaceObject('stool3.ply', [2, 0, 0]);
    PlaceObject('stool3.ply', [2, 1, 0]);
    PlaceObject('stool3.ply', [2, 2, 0]);
    PlaceObject('personMaleCasual.ply', [-1.5, 1.5, 0]); % Person

    % Place safety equipment and objects on the table
    PlaceObject('fence.ply', [-1, -0.6, 0]);    % Safety fencing
    PlaceObject('fence.ply', [-1, 2.5, 0]);
    PlaceObject('emergencyStopButton.ply', [-0.8, 1.3, 0.8]);  % E-stop button
    PlaceObject('vodkabottle.ply', [1.35, 1.9, 1.0]);          % Vodka bottle
    PlaceObject('rumbottle.ply', [1.5, 1.9, 1.0]);             % Rum bottle
    PlaceObject('greenbottle.ply', [1.65, 1.9, 1.0]);          % Green bottle
    cupHandle = PlaceObject('RedSoloCup.ply', [1.2, 1.2, 1.0]); % Position Red Solo Cup near the Dobot
    cupVertices = get(cupHandle, 'Vertices');                  % Store original vertices of the cup

    % Dobot Setup
    r_dobot = dobot();
    r_dobot.model.base = transl(0.0, 2.2, 1.0);   % Updated position: 1 unit forward on the X-axis
    q0_dobot = [pi/6, -pi/2, pi/3, 0, -pi/4, pi/2];
    r_dobot.model.animate(q0_dobot);

    % UR3 Setup on Simulated Linear Rail
    r_ur3 = UR3();
    rail_base_position = transl(-0.3, 1, 1.0); % Base position on table level, aligned along Y-axis
    r_ur3.model.base = rail_base_position;
    q0_ur3 = zeros(1, 6);                       
    r_ur3.model.animate(q0_ur3);

    % Draw the rail as a line beneath UR3 for visualization
    plot3([-0.3, -0.3], [1, -1], [1.0, 1.0], 'k-', 'LineWidth', 3);

    % Initialize flags
    isStopped = false;

    % GUI Controls for Emergency Stop and Resume
    uicontrol('Style', 'pushbutton', 'Position', [600, 100, 100, 40], ...
              'String', 'E-Stop', 'Callback', @emergency_stop);
    uicontrol('Style', 'pushbutton', 'Position', [600, 50, 100, 40], ...
              'String', 'Resume', 'Callback', @resume_operation);

    % Begin Tasks
    dobotTask(r_dobot);                   % Perform Dobot Task
    executeUR3PickPlace(r_ur3, cupHandle, cupVertices); % UR3 pick and place task

    %% Function to Perform Dobot Task
    function dobotTask(robot)
        disp('Starting Dobot task...');
        % Define target positions for Dobot
        targetPos1 = transl(0.2, 2.2, 1.2);
        targetPos2 = transl(0.0, 2.0, 1.2);

        % Trajectory and movement
        qPos1 = robot.model.ikcon(targetPos1);
        qPos2 = robot.model.ikcon(targetPos2);
        steps = 50;
        qMatrix1 = jtraj(q0_dobot, qPos1, steps);
        qMatrix2 = jtraj(qPos1, qPos2, steps);

        % Animate movement
        for i = 1:steps
            robot.model.animate(qMatrix1(i, :));
            pause(0.05);
        end
        for i = 1:steps
            robot.model.animate(qMatrix2(i, :));
            pause(0.05);
        end
        disp('Dobot task complete.');
    end

    %% Function to Execute UR3 Pick-and-Place Task
    function executeUR3PickPlace(robot, cupHandle, cupVertices)
        pickupPos = transl(0.0, 2.2, 1.1);   % Adjusted above the Red Solo Cup near the Dobot
        placePos = transl(-0.3, 0, 1.0);     % Placement position on the bar table

        % Move to pickup and grip
        moveUR3OnSimulatedRail(robot, rail_base_position, pickupPos, 'grip', cupHandle, cupVertices);

        % Move to place and release
        moveUR3OnSimulatedRail(robot, transl(-0.3, -0.5, 1.0), placePos, 'release', cupHandle, cupVertices);
        disp('UR3 task complete.');
    end

    %% Function to Move UR3 on Simulated Linear Rail with Pick-and-Place Action
    function moveUR3OnSimulatedRail(robot, baseTarget, targetTransform, action, cupHandle, cupVertices)
        steps = 50;
        qCurrent = robot.model.getpos();
        qTarget = robot.model.ikcon(targetTransform, qCurrent);

        % Interpolate base position and joint configuration
        for i = 1:steps
            s = i / steps;
            interpolatedBase = rail_base_position * (1 - s) + baseTarget * s;
            robot.model.base = interpolatedBase;

            % Interpolate joint angles and animate
            qInterp = qCurrent * (1 - s) + qTarget * s;
            robot.model.animate(qInterp);

            % Attach or detach cup based on action at final position
            endEffectorTransform = robot.model.fkine(qInterp).T;
            rotationAndTranslation = endEffectorTransform(1:3, :); % Extract 3x4 part of the transform

            if strcmp(action, 'grip') && i == steps
                disp('Gripping cup...');
                % Transform cup vertices to the end-effector's location
                transformedVertices = [cupVertices, ones(size(cupVertices, 1), 1)] * rotationAndTranslation';
                set(cupHandle, 'Vertices', transformedVertices(:, 1:3));  % Attach cup to gripper
            elseif strcmp(action, 'release') && i == steps
                disp('Releasing cup...');
                % Drop cup at the final position
                releaseTransform = targetTransform(1:3, :); % Extract the 3x4 part for the drop
                transformedVertices = [cupVertices, ones(size(cupVertices, 1), 1)] * releaseTransform';
                set(cupHandle, 'Vertices', transformedVertices(:, 1:3));  % Release cup
            end
            pause(0.05);  % Pause for smooth animation
        end
    end

    %% Emergency Stop Callback
    function emergency_stop(~, ~)
        isStopped = true;
        disp('Emergency Stop activated.');
    end

    %% Resume Operation Callback
    function resume_operation(~, ~)
        if isStopped
            disp('Resuming operation...');
            isStopped = false;
            executeUR3PickPlace(r_ur3, cupHandle, cupVertices);
        end
    end
end
