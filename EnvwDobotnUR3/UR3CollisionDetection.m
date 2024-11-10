function Ass2EnvironmentWithCollisionDetection
    % Clear previous simulations
    close all; clear; clc;
    
    % Set up the environment
    figure;
    axis([-3, 3, -3, 3, 0, 2]);
    hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Environment and Collision Detection Showcase');

    % Add floor texture
    surf([-3, -3; 3, 3], [-3, 3; -3, 3], [0, 0; 0, 0], ...
        'CData', imread('floor.jpg'), 'FaceColor', 'texturemap');

    % Place main table and environment objects
    PlaceObject('table2.ply', [-0.5, 1, 0]);         % Main table
    PlaceObject('counter2.ply', [1, -1, 0]);         % Side counter
    PlaceObject('stool3.ply', [-1, -2, 0]);          % Stools
    PlaceObject('stool3.ply', [0, -2, 0]);
    PlaceObject('stool3.ply', [1, -2, 0]);
    PlaceObject('stool3.ply', [2, -1, 0]);
    PlaceObject('stool3.ply', [2, 0, 0]);
    PlaceObject('stool3.ply', [2, 1, 0]);
    PlaceObject('stool3.ply', [2, 2, 0]);
    PlaceObject('personMaleCasual.ply', [-1.5, 1.5, 0]);  % Person

    % Place safety equipment and objects on the table
    PlaceObject('fence.ply', [-1, -0.6, 0]);    % Safety fencing
    PlaceObject('fence.ply', [-1, 2.5, 0]);
    PlaceObject('emergencyStopButton.ply', [-0.8, 1.3, 0.8]);  % E-stop button
    PlaceObject('vodkabottle.ply', [1.35, 1.9, 1.0]);          % Vodka bottle
    PlaceObject('rumbottle.ply', [1.5, 1.9, 1.0]);             % Rum bottle
    PlaceObject('greenbottle.ply', [1.65, 1.9, 1.0]);          % Green bottle

    % Red Solo Cup setup
    RedSoloCup = PlaceObject('RedSoloCup.ply', [0, 0, 0]);
    RedSoloCupVertices = get(RedSoloCup, 'Vertices');
    cupInitialPosition = transl(1.2, 1.2, 1.0);
    transformedVertices = [RedSoloCupVertices, ones(size(RedSoloCupVertices, 1), 1)] * cupInitialPosition';
    set(RedSoloCup, 'Vertices', transformedVertices(:, 1:3));

    % UR3 Setup on Simulated Linear Rail
    r_ur3 = UR3();
    rail_base_position = transl(-0.3, 1, 1.0); % Base position on table level, aligned along Y-axis
    r_ur3.model.base = rail_base_position;
    q0_ur3 = zeros(1, 6);                       
    r_ur3.model.animate(q0_ur3);

    % Draw the rail as a line beneath UR3 for visualization
    plot3([-0.3, -0.3], [1, -1], [1.0, 1.0], 'k-', 'LineWidth', 3);

    % Wait briefly to ensure the environment is fully built
    pause(2);

    % Simulate the UR3 moving towards the table to showcase collision detection
    simulateCollisionDetection(r_ur3, RedSoloCup);

    %% Function to Simulate Collision Detection by Moving UR3 Towards Table
    function simulateCollisionDetection(robot, cupHandle)
        disp('Starting collision detection showcase...');
        
        % Define target position deliberately set to collide with the table
        collisionPos = transl(-0.3, 1.5, 1.0);
        
        % Move the robot towards the collision point
        moveUR3WithCollision(robot, rail_base_position, collisionPos, cupHandle);
    end

    %% Function to Move UR3 and Stop on Collision
    function moveUR3WithCollision(robot, baseTarget, targetTransform, cupHandle)
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

            % Move cup with the UR3 gripper for demonstration purposes
            endEffectorPos = robot.model.fkine(qInterp).t;
            transformedVertices = [RedSoloCupVertices, ones(size(RedSoloCupVertices, 1), 1)] * transl(endEffectorPos)';
            set(cupHandle, 'Vertices', transformedVertices(:, 1:3));

            % Collision detection with the environment (table)
            if checkCollisionWithTable(robot, endEffectorPos)
                disp('Collision detected with table! Stopping simulation...');
                return;  % Stop movement on collision detection
            end

            pause(0.05);  % Pause for smooth animation
        end
    end

    %% Function for Collision Detection with Table
    function collision = checkCollisionWithTable(robot, endEffectorPos)
        collision = false;
        
        % Define table boundary for collision detection (example dimensions)
        tableXBounds = [-0.8, 0.8];  % X range of the table in world coordinates
        tableYBounds = [1.0, 1.6];   % Y range of the table in world coordinates
        tableZHeight = 1.0;          % Z height for table top surface

        % Check if the end-effector is within table bounds (collision condition)
        if endEffectorPos(1) > tableXBounds(1) && endEffectorPos(1) < tableXBounds(2) && ...
           endEffectorPos(2) > tableYBounds(1) && endEffectorPos(2) < tableYBounds(2) && ...
           endEffectorPos(3) < tableZHeight + 0.05  % Allow small margin above table
            collision = true;
        end
    end
end
