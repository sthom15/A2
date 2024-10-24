function[] = Ass2Environment()
    % clear all
    close all
    clc

    % Creating Robots
    % Linear UR3e Model
    r = dobot;
    % Move the robot's base to a new position and height
    % For example, to move it to [x, y, z] = [1, 0, 0.5], create a transformation matrix
    newBaseTransform = transl(1.2, 1, 1); % transl(x, y, z) for position [x, y, z]
    r.model.base = newBaseTransform; % Assign the new base transformation

    assignin('base', 'r', r);
%%    robot = UR3();
    q = [pi/6, -pi/2, pi/3, 0, -pi/4, pi/2];
r.model.animate(q);


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

    % Stools
    PlaceObject('stool2.ply',[-1,-2,0]);
    PlaceObject('stool2.ply',[0,-2,0]);
    PlaceObject('stool2.ply',[1,-2,0]);
    PlaceObject('stool2.ply',[2,-1,0]);
    PlaceObject('stool2.ply',[2,0,0]);
    PlaceObject('stool2.ply',[2,1,0]);
    PlaceObject('stool2.ply',[2,2,0]);

end