function Ass2Environmentv9
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

    % Position the Red Solo Cup initially on the main table near UR3's start
    RedSoloCup = PlaceObject('RedSoloCup.ply', [0 0 0]);
    RedSoloCupVertices = get(RedSoloCup, 'Vertices');
    RedSoloCuptr = transl(-0.2, 1, 1.0); % Adjusted initial position near UR3
    transformedVertices = [RedSoloCupVertices, ones(size(RedSoloCupVertices,1), 1)] * RedSoloCuptr';
    set(RedSoloCup, 'Vertices', transformedVertices(:,1:3));

    % Dobot Setup
    r_dobot = dobot();
    r_dobot.model.base = transl(0.0, 2.0, 1.0);   % Slightly adjusted position
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

    % Begin Tasks
    dobotTask(r_dobot);                   % Perform Dobot Task
    executeUR3PickPlace(r_ur3, RedSoloCup, RedSoloCupVertices); % UR3 pick and place task

    %% Function to Perform Dobot Task
    function dobotTask(robot)
        disp('Starting Dobot task...');
        targetPos1 = transl(0.2, 2.0, 1.2);
        targetPos2 = transl(0.0, 1.8, 1.2);
        steps = 50;

        % Trajectory and movement
        qMatrix1 = jtraj(q0_dobot, robot.model.ikcon(targetPos1), steps);
        qMatrix2 = jtraj(robot.model.ikcon(targetPos1), robot.model.ikcon(targetPos2), steps);

        % Animate Dobot movement
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
        pickupPos = transl(-0.2, 1, 1.1);   % Above the Red Solo Cup on the main table
        placePos = transl(-0.3, -1, 1.0);   % End of the rail on the other table

        % Move to pickup and grip
        moveUR3OnSimulatedRail(robot, rail_base_position, pickupPos, 'grip', cupHandle, cupVertices);

        % Move to place and release
        moveUR3OnSimulatedRail(robot, transl(-0.3, -1.0, 1.0), placePos, 'release', cupHandle, cupVertices);
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

            % Continuously update cup's position to follow the UR3's end-effector
            endEffectorTransform = robot.model.fkine(qInterp).T;
            transformedVertices = [cupVertices, ones(size(cupVertices, 1), 1)] * endEffectorTransform';
            set(cupHandle, 'Vertices', transformedVertices(:, 1:3)); 

            % Release cup at final position if specified
            if strcmp(action, 'release') && i == steps
                disp('Releasing cup...');
                transformedVertices = [cupVertices, ones(size(cupVertices, 1), 1)] * targetTransform.T';
                set(cupHandle, 'Vertices', transformedVertices(:, 1:3)); 
            end
            pause(0.05);  
        end
    end
end
