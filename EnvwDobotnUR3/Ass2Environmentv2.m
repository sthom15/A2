function Ass2Environmentv2
    % Clear previous simulations
    clear; clc;
    global eStopActive;  % Define a global variable for emergency stop
    eStopActive = false; % Initialize e-stop to inactive

    % Set up the environment
    figure;
    axis([-3, 8, -3, 3, 0, 2]);
    hold on;

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

    % Position objects on the table, including e-stop button
    PlaceObject('shaker.ply', [1.1, 1.2, 1.0]);
    PlaceObject('emergencyStopButton.ply', [1.0, 1.3, 1.0]);  % Emergency button

    % Add an emergency stop button UI
    uicontrol('Style', 'pushbutton', 'String', 'Emergency Stop', ...
              'Position', [20 20 100 40], 'Callback', @triggerEStop);

    % Place other objects
    PlaceObject('vodkabottle.ply', [1.05, 0.9, 1.0]);
    PlaceObject('rumbottle.ply', [1.15, 0.9, 1.0]);
    PlaceObject('greenbottle.ply', [1.25, 0.9, 1.0]);
    PlaceObject('RedSoloCup.ply', [1.0, 1.1, 1.0]);

    % Position UR3 Robot
    r_ur3 = UR3();
    r_ur3.model.base = transl(1.5, 0.0, 1.0);
    q0_ur3 = zeros(1, 6);
    r_ur3.model.animate(q0_ur3);

    % Prompt for simulation start
    disp('Press Enter to start UR3 motion');
    pause;

    % Define pick-up and drop positions
    pickupPosition = transl(1.5, 0.1, 1.05);
    dropPosition = transl(1.6, -0.1, 1.05);

    % Calculate joint configurations
    qPickup = r_ur3.model.ikcon(pickupPosition, q0_ur3);
    qDrop = r_ur3.model.ikcon(dropPosition, qPickup);

    % Trajectory for UR3 from initial position to pickup position
    steps = 50;
    qMatrixPickup = jtraj(q0_ur3, qPickup, steps);
    qMatrixDrop = jtraj(qPickup, qDrop, steps);

    % Animate UR3 to the pickup location
    for i = 1:steps
        if eStopActive, disp('E-Stop Activated: Motion Stopped'); break; end  % Check e-stop
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
        if eStopActive, disp('E-Stop Activated: Motion Stopped'); break; end  % Check e-stop
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
        if eStopActive, disp('E-Stop Activated: Motion Stopped'); break; end  % Check e-stop
        r_ur3.model.animate(qMatrixReset(i, :));
        pause(0.05);
    end

    disp('Simulation complete.');
end

function triggerEStop(~, ~)
    % Callback function for the emergency stop button
    global eStopActive;
    eStopActive = true;
    disp('Emergency Stop Activated!');
end
