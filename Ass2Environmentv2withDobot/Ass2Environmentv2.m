function[] = Ass2Environment()
    % clear all
    close all
    clc

    % Creating Robots
    % Linear C5 Model
    r = dobot;
    % For example, to move it to [x, y, z] = [1, 0, 0.5], create a transformation matrix
    newBaseTransform = transl(1.2, 1, 1); % transl(x, y, z) for position [x, y, z]
    r.model.base = newBaseTransform; % Assign the new base transformation
    assignin('base', 'r', r);
q0 = [pi/6, -pi/2, pi/3, 0, -pi/4, pi/2, 0, 0]; % The last two entries control the gripper
r.model.animate(q0);


    % Dobot C5 Model
    %r = Dobot CR5;
    %assignin('base', 'r', r);

    % Creating rest of environment


axis([-3,3,-3,3,0,2]);
    hold on

    % Floor
     surf([-3,-3;3,3], [-3,3;-3,3], [0,0;0,0], 'CData', imread('floor.jpg'), 'FaceColor', 'texturemap');

    % Bar counters
    PlaceObject('table2.ply',[-0.5,1,0]); 
    PlaceObject('counter2.ply',[1,-1,0]);
    PlaceObject('shaker.ply',[1.38,0.15,1]);
    shakerPosition = [1.35,0.12,1.1];

    % Stools
    PlaceObject('stool2.ply',[-1,-2,0]);
    PlaceObject('stool2.ply',[0,-2,0]);
    PlaceObject('stool2.ply',[1,-2,0]);
    PlaceObject('stool2.ply',[2,-1,0]);
    PlaceObject('stool2.ply',[2,0,0]);
    PlaceObject('stool2.ply',[2,1,0]);
    PlaceObject('stool2.ply',[2,2,0]);
    input('Press enter to continue')
        movePosition = [1, 1, 2];

    % First Movement: Move to the bottle (shaker position)
    targetPosition1 = transl(movePosition); % Homogeneous transformation matrix
    qTarget1 = r.model.ikcon(targetPosition1, q0); % Solve using inverse kinematics

    % Generate trajectory for the first movement
    steps1 = 50; % Number of steps for smooth motion
    qMatrix1 = jtraj(q0, qTarget1, steps1); % Trajectory from q0 to qTarget1

    % Animate the robot's movement to the first target position (the bottle)
    for i = 1:steps1
        r.model.animate(qMatrix1(i, :)); % Animate each step
        pause(0.05); % Pause for smooth visualization
    end
 input('Press enter to continue')

    % Second Movement: Move to [1, 1, 0]
    targetPosition2 = transl(1, 0, 2); % New target position [1, 1, 0]
    qTarget2 = r.model.ikcon(targetPosition2, qTarget1); % Solve for new joint angles

    % Generate trajectory for the second movement
    steps2 = 50; % Number of steps for smooth motion
    qMatrix2 = jtraj(qTarget1, qTarget2, steps2); % Trajectory from qTarget1 to qTarget2

    % Animate the robot's movement to the second target position
    for i = 1:steps2
        r.model.animate(qMatrix2(i, :)); % Animate each step
        pause(0.05); % Pause for smooth visualization
    end
 input('Press enter to continue')

     targetPosition3 = transl(shakerPosition); % Homogeneous transformation matrix
    qTarget3 = r.model.ikcon(targetPosition3, qTarget2); % Solve using inverse kinematics

    % Generate trajectory for the first movement
    steps3 = 50; % Number of steps for smooth motion
    qMatrix3 = jtraj(qTarget2, qTarget3, steps3); % Trajectory from q0 to qTarget1

    % Animate the robot's movement to the first target position (the bottle)
    for i = 1:steps1
        r.model.animate(qMatrix3(i, :)); % Animate each step
        pause(0.05); % Pause for smooth visualization
    end
end

