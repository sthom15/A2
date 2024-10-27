function Ass2Environmentv2
    % Clear previous simulations
    close all; clear; clc;
    
    % Set up the environment
    figure;
    axis([-3, 3, -3, 3, 0, 2]);
    hold on;
    
    % Add floor texture
    surf([-3, -3; 3, 3], [-3, 3; -3, 3], [0, 0; 0, 0], ...
        'CData', imread('floor.jpg'), 'FaceColor', 'texturemap');
    
    % Place the main table
    PlaceObject('table2.ply', [-0.5, 1, 0]);  % Position the main table
    PlaceObject('counter2.ply', [1.2, -1, 0]);  % Side counter
    
    % Position the Dobot on the table
    r_dobot = dobot;
    r_dobot.model.base = transl(1.2, 1, 1.0);  % Adjusted height for table top
    q0_dobot = [pi/6, -pi/2, pi/3, 0, -pi/4, pi/2, 0, 0];
    r_dobot.model.animate(q0_dobot);

    % Position objects on the table
    PlaceObject('shaker.ply', [-0.5, 1.2, 0.75]);  % Shaker
    PlaceObject('emergencyStopButton.ply', [-0.8, 1.3, 0.8]);  % Emergency button

    % Move bottles to the back of the table
    PlaceObject('vodkabottle.ply', [-0.6, 1.0, 0.85]);  % Vodka bottle
    PlaceObject('rumbottle.ply', [-0.4, 1.0, 0.85]);    % Rum bottle
    PlaceObject('greenbottle.ply', [-0.2, 1.0, 0.85]);  % Green bottle
    
    % Position RedSoloCup in front of the table
    PlaceObject('RedSoloCup.ply', [0.5, 0.8, 0.75]);  % Adjusted closer to front edge

    % Position UR3 Robot in front of the table, closer to the cup
    r_ur3 = UR3();  % Instantiate the UR3 class from the toolbox
    r_ur3.model.base = transl(0.5, 0.5, 1.0); % Place UR3 in front of the table
    q0_ur3 = zeros(1, 6); % Set initial joint positions
    r_ur3.model.animate(q0_ur3);
    
    disp('Press Enter to start UR3 motion');
    pause;

    % Define pick-up and drop positions for the UR3
    pickupPosition = transl(0.5, 0.8, 0.9);  % Above the RedSoloCup
    dropPosition = transl(-0.6, 1.0, 0.9);   % Drop location near bottles
    
    % Calculate joint configurations
    qPickup = r_ur3.model.ikcon(pickupPosition, q0_ur3);
    qDrop = r_ur3.model.ikcon(dropPosition, qPickup);

    % Trajectory for UR3 from initial position to pickup position
    steps = 50;
    qMatrixPickup = jtraj(q0_ur3, qPickup, steps);
    qMatrixDrop = jtraj(qPickup, qDrop, steps);

    % Animate the UR3 to the pickup location
    for i = 1:steps
        r_ur3.model.animate(qMatrixPickup(i, :));
        pause(0.05);
    end
    
    % Simulate gripping the cup (virtual gripper)
    disp('Gripping cup...');
    pause(1);

    % Animate the UR3 to the drop location
    for i = 1:steps
        r_ur3.model.animate(qMatrixDrop(i, :));
        pause(0.05);
    end
    
    % Simulate releasing the cup
    disp('Releasing cup...');
    pause(1);
    
    % Return UR3 to the initial position
    qMatrixReset = jtraj(qDrop, q0_ur3, steps);
    for i = 1:steps
        r_ur3.model.animate(qMatrixReset(i, :));
        pause(0.05);
    end

    disp('Simulation complete.');
end
