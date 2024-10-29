function Ass2Environmentv2
    % Clear previous simulations
    clear; clc;
    global eStopActive;  % Define global variable
    eStopActive = false; % Initialize e-stop to inactive

    % Set up the environment
    figureHandle = figure;
    axis([-3, 8, -3, 3, 0, 2]);
    hold on;

    % Set up the key press callback for the emergency stop (when "0" is pressed)
    set(figureHandle, 'WindowKeyPressFcn', @keyPressCallback);

    % Add floor texture
    surf([-3, -3; 3, 3], [-3, 3; -3, 3], [0, 0; 0, 0], ...
        'CData', imread('floor.jpg'), 'FaceColor', 'texturemap');

    % Place the main table
    PlaceObject('table2.ply', [-0.5, 1, 0]);
    PlaceObject('counter2.ply', [1.2, -1, 0]);

    % Position the Dobot on the table
    r_dobot = dobot;
    r_dobot.model.base = transl(1.2, 1, 1.0);
    q0_dobot = [pi/6, -pi/2, pi/3, 0, -pi/4, pi/2, 0, 0];
    r_dobot.model.animate(q0_dobot);

    % Position environment objects on the table
    PlaceObject('shaker.ply', [1.1, 1.2, 1.0]);
    PlaceObject('emergencyStopButton.ply', [2.0, -1.2, 1.0]);  % E-stop button moved 1 step behind
    PlaceObject('vodkabottle.ply', [2.05, 1.9, 1.0]);          % Vodka bottle moved 1 forward and right
    PlaceObject('rumbottle.ply', [2.15, 1.9, 1.0]);            % Rum bottle
    PlaceObject('greenbottle.ply', [2.25, 1.9, 1.0]);          % Green bottle
    PlaceObject('RedSoloCup.ply', [1.35, 0.5, 1.0]);           % Red Solo Cup

    %% Dobot Movements
    disp('Starting Dobot movements...');
    targetPositions = {[1, 1, 2], [1, 0, 2], [1.35, 0.12, 1.1]}; % Define target positions
    qCurrent = q0_dobot; % Initialize current position

    for t = 1:length(targetPositions)
        if eStopActive, disp('E-Stop Activated: Dobot Motion Stopped'); break; end  % Check e-stop
        % Calculate the target joint angles
        targetTransform = transl(targetPositions{t});
        qTarget = r_dobot.model.ikcon(targetTransform, qCurrent);
        
        % Generate trajectory for the movement
        steps = 50;
        qMatrix = jtraj(qCurrent, qTarget, steps);  % Calculate the trajectory

        % Animate Dobot to the target position
        for i = 1:steps
            if eStopActive, disp('E-Stop Activated: Dobot Motion Stopped'); break; end  % Check e-stop
            r_dobot.model.animate(qMatrix(i, :));
            pause(0.05);
        end

        qCurrent = qTarget; % Update current position
        if eStopActive, break; end  % Check e-stop after each movement
    end
    disp('Dobot movements complete.');

    %% UR3 Movements
    r_ur3 = UR3();
    r_ur3.model.base = transl(1.5, -1.0, 1.0);  % Moved -1 unit along the y-axis
    q0_ur3 = zeros(1, 6);
    r_ur3.model.animate(q0_ur3);

    % Define pick-up and drop positions for UR3
    pickupPosition = transl(1.5, -0.9, 1.05);
    dropPosition = transl(1.6, -1.1, 1.05);

    % Calculate joint configurations for the UR3
    qPickup = r_ur3.model.ikcon(pickupPosition, q0_ur3);
    qDrop = r_ur3.model.ikcon(dropPosition, qPickup);

    % Trajectory for UR3 from initial position to pickup position
    steps = 50;
    qMatrixPickup = jtraj(q0_ur3, qPickup, steps);
    qMatrixDrop = jtraj(qPickup, qDrop, steps);

    % Animate UR3 to the pickup location
    for i = 1:steps
        if eStopActive, disp('E-Stop Activated: UR3 Motion Stopped'); break; end  % Check e-stop
        r_ur3.model.animate(qMatrixPickup(i, :));
        pause(0.05);
    end
    
    % Simulate gripping the cup
    if ~eStopActive
        disp('Gripping cup...');
        pause(1);
    end

    % Animate UR3 to the drop location
    for i = 1:steps
        if eStopActive, disp('E-Stop Activated: UR3 Motion Stopped'); break; end  % Check e-stop
        r_ur3.model.animate(qMatrixDrop(i, :));
        pause(0.05);
    end
    
    % Simulate releasing the cup
    if ~eStopActive
        disp('Releasing cup...');
        pause(1);
    end

    % Return UR3 to the initial position
    qMatrixReset = jtraj(qDrop, q0_ur3, steps);
    for i = 1:steps
        if eStopActive, disp('E-Stop Activated: UR3 Motion Stopped'); break; end  % Check e-stop
        r_ur3.model.animate(qMatrixReset(i, :));
        pause(0.05);
    end

    disp('Simulation complete.');
end

function keyPressCallback(~, event)
    global eStopActive;
    if strcmp(event.Key, '0')
        eStopActive = true;
        disp('Emergency Stop Activated by pressing "0"!');
    end
end
