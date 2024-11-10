function Ass2EnvironmentWithCollisionDetection
    % Clear previous simulations
    close all; clear; clc;
    
    % Set up the environment
    figure;
    axis([-3, 3, -3, 3, 0, 2]);
    hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Environment with Collision Detection');

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

    % Set up the UR3 as a substitute for the Dobot (moving robot for collision detection)
    r_dobot = UR3();
    r_dobot.model.base = transl(0.0, 2.2, 1.0);  % Position near the table
    q0_dobot = zeros(1, 6);
    r_dobot.model.animate(q0_dobot);

    % Set up the UR3 on a Linear Rail (stationary robot for demonstration)
    r_ur3 = UR3();
    rail_base_position = transl(-0.3, 1, 1.0); % Base position on table level, aligned along Y-axis
    r_ur3.model.base = rail_base_position;
    q0_ur3 = zeros(1, 6);
    r_ur3.model.animate(q0_ur3);

    % Draw the rail as a line beneath UR3 for visualization
    plot3([-0.3, -0.3], [1, -1], [1.0, 1.0], 'k-', 'LineWidth', 3);

    % Initialize flags
    isStopped = false;

    % Wait briefly to ensure the environment is fully built
    pause(2);

    % Simulate the UR3 (as Dobot) moving towards the table to demonstrate collision detection
    simulateDobotCollisionDetection(r_dobot, RedSoloCup);

    %% Function to Simulate Collision Detection for Dobot
    function simulateDobotCollisionDetection(robot, cupHandle)
        disp('Starting Dobot collision detection showcase...');
        
        % Define target position deliberately set to collide with an object
        collisionPos = transl(1.2, 1.2, 1.0);  % Near the Red Solo Cup
        
        % Move the UR3 towards the collision point
        moveRobotWithCollision(robot, q0_dobot, collisionPos, cupHandle);
    end

    %% Function to Move UR3 and Stop on Collision
    function moveRobotWithCollision(robot, qStart, targetTransform, cupHandle)
        steps = 50;
        qCurrent = qStart;
        qTarget = robot.model.ikcon(targetTransform, qCurrent);

        % Trajectory from start to target
        qMatrix = jtraj(qCurrent, qTarget, steps);
        
        % Move along trajectory with collision checking
        for i = 1:steps
            robot.model.animate(qMatrix(i, :));

            % End-effector position for collision detection
            endEffectorPos = robot.model.fkine(qMatrix(i, :)).t;

            % Check collision with table or objects
            if checkCollisionWithTable(robot, endEffectorPos)
                disp('Collision detected! Stopping Dobot movement...');
                return;  % Stop movement on collision detection
            end

            % Update cup position with the end-effector (for visual demonstration)
            transformedVertices = [RedSoloCupVertices, ones(size(RedSoloCupVertices, 1), 1)] * transl(endEffectorPos)';
            set(cupHandle, 'Vertices', transformedVertices(:, 1:3));

            pause(0.05);  % Pause for smooth animation
        end
    end

    %% Function for Collision Detection with Table
    function collision = checkCollisionWithTable(robot, endEffectorPos)
        collision = false;
        
        % Define table boundary for collision detection
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
