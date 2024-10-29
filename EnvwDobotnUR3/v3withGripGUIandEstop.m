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
  %  r_dobot = dobot;
   % r_dobot.model.base = transl(1.2, 1, 1.0);  % Adjusted height for table top
   % q0_dobot = [pi/6, -pi/2, pi/3, 0, -pi/4, pi/2, 0, 0];
   % r_dobot.model.animate(q0_dobot);

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
    r_ur3 = UR3()  % Instantiate the UR3 class from the toolbox
    r_ur3.model.base = transl(0.5, 0.5, 1.0); % Place UR3 in front of the table
    q0_ur3 = zeros(1, 6); % Set initial joint positions
    r_ur3.model.animate(q0_ur3);
    gripperOrigin = r_ur3.model.fkine(r_ur3.model.getpos());
gripperL = Gripper(gripperOrigin.T * trotx(pi/2));
gripperR = Gripper(gripperOrigin.T *trotx(-pi/2) * trotz(pi));
g0 = deg2rad(45);

 % Initialize control flags
    isPaused = false;    % Flag for emergency stop
    isStopped = false;   % Track if E-Stop has been triggered

    % E-Stop Button
    uicontrol('Style', 'pushbutton', 'Position', [600, 100, 100, 40], ...
              'String', 'E-Stop', 'Callback', @emergency_stop);

    % Resume Button
    uicontrol('Style', 'pushbutton', 'Position', [600, 50, 100, 40], ...
              'String', 'Resume', 'Callback', @resume_operation);
     disp('Press Enter to start UR3 motion');
    pause;
        section1run = true;
    section2run = true;
    section3run = true;
        qc = q0_ur3;

    move_robot();

     function move_robot

    % Define pick-up and drop positions for the UR3
    pickupPosition = transl(0.5, 0.8, 0.9);  % Above the RedSoloCup
    dropPosition = transl(-0.6, 1.0, 0.9);   % Drop location near bottles
    
    % Calculate joint configurations
    qPickup = r_ur3.model.ikcon(pickupPosition, qc);
    qDrop = r_ur3.model.ikcon(dropPosition, qc);

    % Trajectory for UR3 from initial position to pickup position
    steps = 50;
    qMatrixPickup = jtraj(qc, qPickup, steps);
    q00 = deg2rad(45);

    % Animate the UR3 to the pickup location
    for i = 1:steps
                if isStopped == false;
 if section1run == false;

        r_ur3.model.animate(qMatrixPickup(i, :));
        qc = qMatrixPickup(i, :);

    gripperTransform = r_ur3.model.fkine(r_ur3.model.getpos());
              gripperLTransform = gripperTransform.T * trotx(pi/2);
    gripperRTransform = gripperTransform.T *trotx(-pi/2) * trotz(pi);
    % Set the base transformation of each gripper to follow the end-effector
    gripperL.model.base = gripperLTransform;
    gripperR.model.base = gripperRTransform;
        gripperL.model.animate(q00); % Default pose for left gripper
    gripperR.model.animate(q00); % Default pose for right gripper
        pause(0.05);
                end
                                if i == steps
                    section1run = true
                end

                end
    end
    % Simulate gripping the cup (virtual gripper)
    disp('Gripping cup...');
    pause(1);

    qMatrixDrop = jtraj(qc, qDrop, steps);

    % Animate the UR3 to the drop location
    for i = 1:steps
                if isStopped == false;
if section2run == false;

        r_ur3.model.animate(qMatrixDrop(i, :));
                qc = qMatrixDrop(i, :);

    gripperTransform = r_ur3.model.fkine(r_ur3.model.getpos());
            gripperLTransform = gripperTransform.T * trotx(pi/2);
    gripperRTransform = gripperTransform.T *trotx(-pi/2) * trotz(pi);
    % Set the base transformation of each gripper to follow the end-effector
    gripperL.model.base = gripperLTransform;
    gripperR.model.base = gripperRTransform;
        gripperL.model.animate(q00); % Default pose for left gripper
    gripperR.model.animate(q00); % Default pose for right gripper
    
        pause(0.05);
end
                if i == steps
                    section2run = true
                end

                end
    end
    
    % Simulate releasing the cup
    disp('Releasing cup...');
    pause(1);
    
    % Return UR3 to the initial position
    qMatrixReset = jtraj(qc, q0_ur3, steps);
    for i = 1:steps
                if isStopped == false;
if section3run == false;
        r_ur3.model.animate(qMatrixReset(i, :));
                        qc = qMatrixReset(i, :);

    gripperTransform = r_ur3.model.fkine(r_ur3.model.getpos());
               gripperLTransform = gripperTransform.T * trotx(pi/2);
    gripperRTransform = gripperTransform.T *trotx(-pi/2) * trotz(pi);
    % Set the base transformation of each gripper to follow the end-effector
    gripperL.model.base = gripperLTransform;
    gripperR.model.base = gripperRTransform;
        gripperL.model.animate(q00); % Default pose for left gripper
    gripperR.model.animate(q00); % Default pose for right gripper
        pause(0.05);
                end
                if i == steps
                    section3run = true
                end
                                
                end
    end

    disp('Simulation complete.');
     end
       qc = q0_ur3; % Joint configuration

 end_effector_tform = r_ur3.model.fkine(qc);

% Extract the position of the end-effector from the transformation matrix
end_effector_pos = transl(end_effector_tform); % Extracts [x, y, z] position

 % Cartesian sliders for X, Y, Z movement
    uicontrol('Style', 'text', 'Position', [50, 200, 100, 20], 'String', 'X Position');
    x_slider = uicontrol('Style', 'slider', 'Position', [150, 200, 300, 20], ...
                         'Min', -1, 'Max', 1, 'Value', end_effector_pos(1), 'Callback', @(src, ~) update_cartesian_position(src, 1));
    
    uicontrol('Style', 'text', 'Position', [50, 170, 100, 20], 'String', 'Y Position');
    y_slider = uicontrol('Style', 'slider', 'Position', [150, 170, 300, 20], ...
                         'Min', -1, 'Max', 1, 'Value', end_effector_pos(2), 'Callback', @(src, ~) update_cartesian_position(src, 2));

    uicontrol('Style', 'text', 'Position', [50, 140, 100, 20], 'String', 'Z Position');
    z_slider = uicontrol('Style', 'slider', 'Position', [150, 140, 300, 20], ...
                         'Min', 0.5, 'Max', 1.5, 'Value', end_effector_pos(3), 'Callback', @(src, ~) update_cartesian_position(src, 3));
    

    % GUI dropdown, slider, and text display for joint angle control
    qc = q0_ur3; % Initial joint configuration for control
    joint_index = 1; % Default joint index (first joint)

    % Dropdown for selecting which joint to control
    uicontrol('Style', 'text', 'Position', [50, 100, 100, 20], 'String', 'Select Joint');
    joint_dropdown = uicontrol('Style', 'popupmenu', 'Position', [150, 100, 100, 20], ...
                               'String', {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'}, ...
                               'Callback', @update_joint_index);

    % Slider for adjusting the selected joint's angle
    uicontrol('Style', 'text', 'Position', [50, 50, 100, 20], 'String', 'Joint Angle');
    joint_slider = uicontrol('Style', 'slider', 'Position', [150, 50, 300, 20], ...
                             'Min', -pi, 'Max', pi, 'Value', qc(joint_index), 'Callback', @(src, ~) update_joint_angle(src));
    angle_display = uicontrol('Style', 'text', 'Position', [460, 50, 100, 20], 'String', 'Angle: 0');
    
        % Callback to update the selected joint index based on the dropdown
    function update_joint_index(src, ~)
        joint_index = src.Value; % Update joint index based on dropdown selection
        joint_slider.Value = qc(joint_index); % Reset slider to the current angle of the selected joint
        angle_display.String = sprintf('Angle: %.2f rad', joint_slider.Value);
    end

    % Callback to update the robot configuration and display based on the slider value
    function update_joint_angle(src)
        joint_angle = src.Value; % Get the current slider value (joint angle in radians)
        qc(joint_index) = joint_angle; % Update the selected joint's angle in the configuration
        r_ur3.model.animate(qc);
        angle_display.String = sprintf('Angle: %.2f rad', joint_angle); % Update the angle display
    end

 % Callback to update the robot's end-effector position based on Cartesian sliders
  
         function update_cartesian_position(src, axis)
        % Update the end-effector position based on the slider value
        end_effector_pos(axis) = src.Value

        % Calculate the new joint angles to achieve the target end-effector position
        ogqc = qc;
        target_tform = transl(end_effector_pos); % Create transformation matrix from target position
        qc = r_ur3.model.ikine(target_tform, qc); % Solve inverse kinematics

        % Update the robot visualization with the new joint angles
    steps = 50;
    qMatrixrobot = jtraj(ogqc, qc, steps);

    % Animate the UR3 to the pickup location
    for i = 1:steps
        r_ur3.model.animate(qMatrixrobot(i, :));
        pause(0.05);
    end
            angle_display.String = sprintf(src.Value); % Update the angle display

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
    end
     end
