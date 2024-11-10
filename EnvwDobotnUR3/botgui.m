  function Ass2Environmentv2
    % Clear previous simulations
    close all; clear; clc;
        % Define collision plane for the table
    tablePoint = [-0.5, 1, 1.0];
    tableNormal = [0, 0, 1];
   
    % Set up the environment
    figure;
    axis([-3, 3, -3, 3, 0, 2]);
    hold on;
    
    % Add floor texture
    surf([-3, -3; 3, 3], [-3, 3; -3, 3], [0, 0; 0, 0], ...
        'CData', imread('floor.jpg'), 'FaceColor', 'texturemap');
    
    % Place the main table
       % Place the main table
    PlaceObject('table2.ply', [-0.5, 1, 0]);  % Position the main table
    PlaceObject('counter2.ply', [1, -1, 0]);  % Side counter
    PlaceObject('stool3.ply',[-1,-2,0]);      % Stools
    PlaceObject('stool3.ply',[0,-2,0]);
    PlaceObject('stool3.ply',[1,-2,0]);
    PlaceObject('stool3.ply',[2,-1,0]);
    PlaceObject('stool3.ply',[2,0,0]);
    PlaceObject('stool3.ply',[2,1,0]);
    PlaceObject('stool3.ply',[2,2,0]);
    PlaceObject('personMaleCasual.ply',[-1.5,1.5,0]);  % Person to scale

        % Place Safety Fencing around Robot Zone
    f1 = PlaceObject('fence.ply',[-1,-0.6,0]);
    verts = [get(f1,'Vertices'), ones(size(get(f1,'Vertices'),1),1)];   
    verts(:,3) = verts(:,3) * 2; % Scale the z-coordinates to make the object taller
    set(f1,'Vertices',verts(:,1:3));
    f2 = PlaceObject('fence.ply',[-1,2.5,0]);
    verts = [get(f2,'Vertices'), ones(size(get(f2,'Vertices'),1),1)];
    verts(:,3) = verts(:,3) * 2;
    set(f2,'Vertices',verts(:,1:3));

  %   Position the Dobot on the table
    r_dobot = dobot;
    r_dobot.model.base = transl(1.2, 1, 1.0);  % Adjusted height for table top
    q0_dobot = [pi/6, -pi/2, pi/3, 0, -pi/4, pi/2];
    r_dobot.model.animate(q0_dobot);
        gripperOrigin1 = r_dobot.model.fkine(r_dobot.model.getpos());
gripperL1 = Gripper(gripperOrigin1.T * trotx(pi/2));
gripperR1 = Gripper(gripperOrigin1.T *trotx(-pi/2) * trotz(pi));

      % Position environment objects on the table
  Cup =  PlaceObject('shaker.ply', [1.1, 1.2, 1.0]);
    PlaceObject('emergencyStopButton.ply', [-0.8, 1.3, 0.8]);  % E-stop button moved 1 step behind
    PlaceObject('vodkabottle.ply', [1.35, 1.9, 1.0]);          % Vodka bottle moved 1 forward and right
    PlaceObject('rumbottle.ply', [1.5, 1.9, 1.0]);            % Rum bottle
    PlaceObject('greenbottle.ply', [1.65, 1.9, 1.0]);          % Green bottle
RedSoloCup=PlaceObject('RedSoloCup.ply', [0 0 0]);
        RedSoloCup_vertices=get(RedSoloCup,'Vertices');
    RedSoloCuptr=transl(1.35, 0.5, 1.0);
    transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*RedSoloCuptr';
    set(RedSoloCup,'Vertices',transformedVertices(:,1:3));
    PlaceObject('vodkabottle.ply', [1, -1.35, 1.0]);          
    PlaceObject('rumbottle.ply', [1, -1.55, 1.0]);
    PlaceObject('vodkabottle.ply', [-0.25, 1, 0.8]);          
    PlaceObject('rumbottle.ply', [-0.25, 0.8, 0.8]);            
    PlaceObject('greenbottle.ply', [-0.25, 0.6, 0.8]);

    % Position UR3 Robot in front of the table, closer to the cup
    r_ur3 = UR3()  % Instantiate the UR3 class from the toolbox
    r_ur3.model.base = transl(1.5, -1.0, 1.0); % Place UR3 in front of the table
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
   %  disp('Press Enter to start UR3 motion');
  %  pause;
        section1run = true;
    section2run = true;
    section3run = true;
         section1rund = true;
    section2rund = true;
    section3rund = true;
        qc = q0_ur3;
        qd = q0_dobot
%

    move_robot();

     function move_robot
%% Dobot Code
  disp('Starting Dobot movements...');
    targetPosition1 = transl(1, 1, 2);
     targetPosition2 = transl(1, 0, 2);
     targetPosition3 = transl(1.35, 0.12, 1.1); % Define target positions
    qd = q0_dobot; % Initialize current position

  qPos1 = r_dobot.model.ikcon(targetPosition1, qd);
    qPos2 = r_dobot.model.ikcon(targetPosition2, qPos1);
    qPos3 = r_dobot.model.ikcon(targetPosition3, qPos2);

    % Trajectory for UR3 from initial position to pickup position
    steps = 50;
    qMatrix1 = jtraj(qc, qPos1, steps);
    q00 = deg2rad(45);

    % Animate the UR3 to the pickup location
    for i = 1:steps
                if isStopped == false;

 if section1rund == false;

        r_dobot.model.animate(qMatrix1(i, :));
        qd = qMatrix1(i, :);

    gripperTransform1 = r_dobot.model.fkine(r_dobot.model.getpos());
              gripperLTransform1 = gripperTransform1.T * trotx(pi/2);
    gripperRTransform1 = gripperTransform1.T *trotx(-pi/2) * trotz(pi);
    % Set the base transformation of each gripper to follow the end-effector
    gripperL1.model.base = gripperLTransform1;
    gripperR1.model.base = gripperRTransform1;
        gripperL1.model.animate(q00); % Default pose for left gripper
    gripperR1.model.animate(q00); % Default pose for right gripper
           tr=r_dobot.model.fkine(r_dobot.model.getpos()).T * transl(0.05,0,-0.15);
                        transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*(double(tr))';
                        set(RedSoloCup,'Vertices',transformedVertices(:,1:3));
    
        pause(0.05);
                end
                                if i == steps
                    section1rund = true
                end
%{ 
 if checkCollisionWithTable(r_dobot, qMatrix1(i, :), tablePoint, tableNormal)
                    disp('Collision detected! Stopping Dobot.');
                    return;
                end 
%}
                end
    end
    % Simulate gripping the cup (virtual gripper)
    disp('Gripping cup...');
    pause(1);

    qMatrix2 = jtraj(qd, qPos2, steps);

    % Animate the UR3 to the drop location
    for i = 1:steps
                if isStopped == false;

 if section1rund == false;

        r_dobot.model.animate(qMatrix2(i, :));
        qd = qMatrix1(i, :);

    gripperTransform1 = r_dobot.model.fkine(r_dobot.model.getpos());
              gripperLTransform1 = gripperTransform1.T * trotx(pi/2);
    gripperRTransform1 = gripperTransform1.T *trotx(-pi/2) * trotz(pi);
    % Set the base transformation of each gripper to follow the end-effector
    gripperL1.model.base = gripperLTransform1;
    gripperR1.model.base = gripperRTransform1;
        gripperL1.model.animate(q00); % Default pose for left gripper
    gripperR1.model.animate(q00); % Default pose for right gripper
         tr=r_dobot.model.fkine(r_dobot.model.getpos()).T * transl(0.05,0,-0.15);
                        transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*(double(tr))';
                        set(RedSoloCup,'Vertices',transformedVertices(:,1:3));        pause(0.05);
                end
                                if i == steps
                    section1rund = true
                end
%{ 
 if checkCollisionWithTable(r_dobot, qMatrix1(i, :), tablePoint, tableNormal)
                    disp('Collision detected! Stopping Dobot.');
                    return;
                end 
%}
                end
    end
    
    % Simulate releasing the cup
    disp('Releasing cup...');
    pause(1);
    
    % Return UR3 to the initial position
    qMatrix3 = jtraj(qd, qPos3, steps);
    for i = 1:steps
                 if isStopped == false;

 if section3rund == false;

        r_dobot.model.animate(qMatrix3(i, :));
        qd = qMatrix1(i, :);

    gripperTransform1 = r_dobot.model.fkine(r_dobot.model.getpos());
              gripperLTransform1 = gripperTransform1.T * trotx(pi/2);
    gripperRTransform1 = gripperTransform1.T *trotx(-pi/2) * trotz(pi);
    % Set the base transformation of each gripper to follow the end-effector
    gripperL1.model.base = gripperLTransform1;
    gripperR1.model.base = gripperRTransform1;
        gripperL1.model.animate(q00); % Default pose for left gripper
    gripperR1.model.animate(q00); % Default pose for right gripper
         tr=r_dobot.model.fkine(r_dobot.model.getpos()).T * transl(0.05,0,-0.15);
                        transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*(double(tr))';
                        set(RedSoloCup,'Vertices',transformedVertices(:,1:3));
        pause(0.05);
                end
                                if i == steps
                    section3rund = true
                end
%{ 
 if checkCollisionWithTable(r_dobot, qMatrix1(i, :), tablePoint, tableNormal)
                    disp('Collision detected! Stopping Dobot.');
                    return;
                end 
%}
                end
    end

     

%% UR3 Code
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
       tr=r_ur3.model.fkine(r_ur3.model.getpos()).T * transl(0.05,0,-0.15);
                        transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*(double(tr))';
                        set(RedSoloCup,'Vertices',transformedVertices(:,1:3));
        pause(0.05);
                end
                                if i == steps
                    section1run = true
                end
%{ 
 if checkCollisionWithTable(r_dobot, qMatrix1(i, :), tablePoint, tableNormal)
                    disp('Collision detected! Stopping Dobot.');
                    return;
                end 
%}
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
       tr=r_ur3.model.fkine(r_ur3.model.getpos()).T * transl(0.05,0,-0.15);
                        transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*(double(tr))';
                        set(RedSoloCup,'Vertices',transformedVertices(:,1:3));
    
        pause(0.05);
end
                if i == steps
                    section2run = true
                end
%{ 
 if checkCollisionWithTable(r_dobot, qMatrix1(i, :), tablePoint, tableNormal)
                    disp('Collision detected! Stopping Dobot.');
                    return;
                end 
%} 
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
       tr=r_ur3.model.fkine(r_ur3.model.getpos()).T * transl(0.05,0,-0.15);
                        transformedVertices=[RedSoloCup_vertices,ones(size(RedSoloCup_vertices,1),1)]*(double(tr))';
                        set(RedSoloCup,'Vertices',transformedVertices(:,1:3));
        pause(0.05);
                end
                if i == steps
                    section3run = true
                end
                  %{ 
 if checkCollisionWithTable(r_dobot, qMatrix1(i, :), tablePoint, tableNormal)
                    disp('Collision detected! Stopping Dobot.');
                    return;
                end 
%}            
                end
    end

    disp('Simulation complete.');
 end

 %% Gui Code
       qc = q0_ur3; % Joint configuration
% Initialize the robot models and joint configurations
qc_ur3 = q0_ur3; % Initial joint configuration for UR3
qc_dobot = q0_dobot; % Initial joint configuration for Dobot

% Set up the end-effector initial transformation and position for both robots
end_effector_pos_ur3 = transl(r_ur3.model.fkine(qc_ur3)); % UR3
end_effector_pos_dobot = transl(r_dobot.model.fkine(qc_dobot)); % Dobot

% Ensure positions are numeric arrays with three elements
if ~isnumeric(end_effector_pos_ur3) || numel(end_effector_pos_ur3) ~= 3
    error('end_effector_pos_ur3 must be a numeric array with three elements [x, y, z].');
end
if ~isnumeric(end_effector_pos_dobot) || numel(end_effector_pos_dobot) ~= 3
    error('end_effector_pos_dobot must be a numeric array with three elements [x, y, z].');
end

% Display debugging info
disp('Starting GUI and initializing end-effector positions for UR3 and Dobot...');

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
  end