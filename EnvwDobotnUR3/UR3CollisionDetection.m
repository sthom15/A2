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
end
