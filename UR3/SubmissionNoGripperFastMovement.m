close all
clear all
clf


hold on

%Calling the environment function
environment()

robot = LinearUR3(eye(4));

robot5 = LinearUR5(eye(4)*transl(4,0,0));

% Loading Brick file
[f,v,data] = plyread('HalfSizedRedGreenBrick.ply','tri');
% Get vertex count
brickVertexCount = size(v,1);
% Move center point to origin
midPoint = sum(v)/brickVertexCount;
brickVerts = v - repmat(midPoint,brickVertexCount,1);
% Create a transform to describe the location (at the origin, since it's centered
brickPose = eye(4);
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% Then plot the trisurf of each brick
brickMesh1_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
brickMesh2_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
brickMesh3_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
brickMesh4_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
brickMesh5_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
brickMesh6_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
brickMesh7_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
brickMesh8_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
brickMesh9_h = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


%Brick Starting Position Transforms


BP1 = eye(4) * transl(-0.7,-0.3,0)*trotz(pi/2)*trotx(pi);
BP2 = eye(4) * transl(-0.5,-0.3,0)*trotz(pi/2)*trotx(pi);
BP3 = eye(4) * transl(-0.3,-0.5,0)*trotz(pi/2)*trotx(pi);
BP4 = eye(4) * transl(-0.7,-0.4,0)*trotz(pi/2)*trotx(pi);
BP5 = eye(4) * transl(-0.5,-0.4,0)*trotz(pi/2)*trotx(pi);
BP6 = eye(4) * transl(-0.3,-0.4,0)*trotz(pi/2)*trotx(pi);
BP7 = eye(4) * transl(-0.7,-0.5,0)*trotz(pi/2)*trotx(pi);
BP8 = eye(4) * transl(-0.5,-0.5,0)*trotz(pi/2)*trotx(pi);
BP9 = eye(4) * transl(-0.3,-0.3,0)*trotz(pi/2)*trotx(pi);

%Brick Ending Position Transforms


EP1 = eye(4) * transl(-0.6,0.4,0)*trotz(pi/2)*trotx(pi);
EP2 = eye(4) * transl(-0.4666,0.4,0)*trotz(pi/2)*trotx(pi);
EP3 = eye(4) * transl(-0.3332,0.4,0)*trotz(pi/2)*trotx(pi);
EP4 = eye(4) * transl(-0.6,0.4,0.0334)*trotz(pi/2)*trotx(pi);
EP5 = eye(4) * transl(-0.4666,0.4,0.0334)*trotz(pi/2)*trotx(pi);
EP6 = eye(4) * transl(-0.3332,0.4,0.0334)*trotz(pi/2)*trotx(pi);
EP7 = eye(4) * transl(-0.6,0.4,0.0334*2)*trotz(pi/2)*trotx(pi);
EP8 = eye(4) * transl(-0.4666,0.4,0.0334*2)*trotz(pi/2)*trotx(pi);
EP9 = eye(4) * transl(-0.3332,0.4,0.0334*2)*trotz(pi/2)*trotx(pi);


%Plotting starting bricks
updatedPoints1 = [BP1 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh1_h.Vertices = updatedPoints1(:,1:3);

updatedPoints2 = [BP2 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh2_h.Vertices = updatedPoints2(:,1:3);

updatedPoints3 = [BP3 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh3_h.Vertices = updatedPoints3(:,1:3);

updatedPoints4 = [BP4 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh4_h.Vertices = updatedPoints4(:,1:3);

updatedPoints5 = [BP5 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh5_h.Vertices = updatedPoints5(:,1:3);

updatedPoints6 = [BP6 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh6_h.Vertices = updatedPoints6(:,1:3);

updatedPoints7 = [BP7 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh7_h.Vertices = updatedPoints7(:,1:3);

updatedPoints8 = [BP8 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh8_h.Vertices = updatedPoints8(:,1:3);

updatedPoints9 = [BP9 * [brickVerts,ones(brickVertexCount,1)]']'; 
brickMesh9_h.Vertices = updatedPoints9(:,1:3);



%Initial q values given to ikcon function to ensure good robot movement
qStart = [-0.3971 0 -1.0996 -0.9495 0.3770 1.5708 0];

qEnd = [-0.3655 0 1.2566 0.5341 -0.2513 -1.5708 0];

%%%Brick coordinates
qb1 = robot.model.ikcon(BP1,qStart);
qe1 = robot.model.ikcon(EP1,qEnd);

qb2 = robot.model.ikcon(BP2,qStart);
qe2 = robot.model.ikcon(EP2,qEnd);

qb3 = robot.model.ikcon(BP3,qStart);
qe3 = robot.model.ikcon(EP3,qEnd);

qb4 = robot.model.ikcon(BP4,qStart);
qe4 = robot.model.ikcon(EP4,qEnd);

qb5 = robot.model.ikcon(BP5,qStart);
qe5 = robot.model.ikcon(EP5,qEnd);

qb6 = robot.model.ikcon(BP6,qStart);
qe6 = robot.model.ikcon(EP6,qEnd);

qb7 = robot.model.ikcon(BP7,qStart);
qe7 = robot.model.ikcon(EP7,qEnd);

qb8 = robot.model.ikcon(BP8,qStart);
qe8 = robot.model.ikcon(EP8,qEnd);

qb9 = robot.model.ikcon(BP9,qStart);
qe9 = robot.model.ikcon(EP9,qEnd);


%defining steps for each movement
steps = 50;

%%%%%% Defining trajectories from brick to brick

qMatrix0b1 = jtraj(qZ,qb1,steps);

qMatrixb1e1 = jtraj(qb1,qe1,steps);

qMatrixe1b2 = jtraj(qe1,qb2,steps);

qMatrixb2e2 = jtraj(qb2,qe2,steps);

qMatrixe2b3 = jtraj(qe2,qb3,steps);

qMatrixb3e3 = jtraj(qb3,qe3,steps);

qMatrixe3b4 = jtraj(qe3,qb4,steps);

qMatrixb4e4 = jtraj(qb4,qe4,steps);

qMatrixe4b5 = jtraj(qe4,qb5,steps);

qMatrixb5e5 = jtraj(qb5,qe5,steps);

qMatrixe5b6 = jtraj(qe5,qb6,steps);

qMatrixb6e6 = jtraj(qb6,qe6,steps);

qMatrixe6b7 = jtraj(qe6,qb7,steps);

qMatrixb7e7 = jtraj(qb7,qe7,steps);

qMatrixe7b8 = jtraj(qe7,qb8,steps);

qMatrixb8e8 = jtraj(qb8,qe8,steps);

qMatrixe8b9 = jtraj(qe8,qb9,steps);

qMatrixb9e9 = jtraj(qb9,qe9,steps);

qMatrixe90 = jtraj(qe9,qZ,steps);




%Move from start position to first brick
display(['Retrieving brick 1'])
for i = 1:steps
     robot.model.animate(qMatrix0b1(i,:));
     gripperRight.base = robot.model.fkine(qMatrix0b1(i,:));
     gripperLeft.base = robot.model.fkine(qMatrix0b1(i,:));
     % gripperRight.animate(qRight);
     % gripperLeft.animate(qLeft);
     drawnow();   
end


dylan = robot.model.fkine(robot.model.getpos())


display(['Depositing brick 1'])
%Move brick 1 from start to end position then from brick 1 end position to
%brick 2 start position
for i = 1:steps
       
        
         robot.model.animate(qMatrixb1e1(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb1e1(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb1e1(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP1 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints1 = (BP1 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh1_h.Vertices = updatedPoints1(:,1:3);
         drawnow()
         
end

display(['Retrieving brick 2'])
for i = 1:steps

         robot.model.animate(qMatrixe1b2(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe1b2(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe1b2(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end

display(['Depositing brick 2'])
%Move brick 2 from start to end position then from brick 2 end position to
%brick 3 start position
for i = 1:steps

         robot.model.animate(qMatrixb2e2(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb2e2(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb2e2(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP2 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints2 = (BP2 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh2_h.Vertices = updatedPoints2(:,1:3);
         drawnow()
         
end

display(['Retrieving brick 3'])
for i = 1:steps

         robot.model.animate(qMatrixe2b3(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe2b3(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe2b3(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end

display(['Depositing brick 3'])
%Move brick 3 from start to end position then from brick 3 end position to
%brick 4 start position
for i = 1:steps

         robot.model.animate(qMatrixb3e3(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb3e3(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb3e3(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP3 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints3 = (BP3 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh3_h.Vertices = updatedPoints3(:,1:3);
         drawnow()
         
end

display(['Retrieving brick 4'])
for i = 1:steps

         robot.model.animate(qMatrixe3b4(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe3b4(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe3b4(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end

display(['Depositing brick 4'])
%Move brick 4 from start to end position then from brick 4 end position to
%brick 5 start position
for i = 1:steps

         robot.model.animate(qMatrixb4e4(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb4e4(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb4e4(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP4 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints4 = (BP4 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh4_h.Vertices = updatedPoints4(:,1:3);
         drawnow()
         
end

display(['Retrieving brick 5'])
for i = 1:steps

         robot.model.animate(qMatrixe4b5(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe4b5(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe4b5(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end

display(['Depositing brick 5'])
%Move brick 5 from start to end position then from brick 5 end position to
%brick 6 start position
for i = 1:steps

         robot.model.animate(qMatrixb5e5(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb5e5(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb5e5(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP5 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints5 = (BP5 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh5_h.Vertices = updatedPoints5(:,1:3);
         drawnow()
         
end

display(['Retrieving brick 6'])
for i = 1:steps

         robot.model.animate(qMatrixe5b6(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe5b6(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe5b6(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end

display(['Depositing brick 6'])
%Move brick 6 from start to end position then from brick 6 end position to
%brick 7 start position
for i = 1:steps

         robot.model.animate(qMatrixb6e6(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb6e6(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb6e6(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP6 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints6 = (BP6 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh6_h.Vertices = updatedPoints6(:,1:3);
         drawnow()
         
end

display(['Retrieving brick 7'])
for i = 1:steps

         robot.model.animate(qMatrixe6b7(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe6b7(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe6b7(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end

display(['Depositing brick 7'])
%Move brick 7 from start to end position then from brick 7 end position to
%brick 8 start position
for i = 1:steps

         robot.model.animate(qMatrixb7e7(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb7e7(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb7e7(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP7 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints7 = (BP7 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh7_h.Vertices = updatedPoints7(:,1:3);
         drawnow()
         
end

display(['Retrieving brick 8'])
for i = 1:steps

         robot.model.animate(qMatrixe7b8(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe7b8(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe7b8(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end

display(['Depositing brick 8'])
%Move brick 8 from start to end position then from brick 8 end position to
%brick 9 start position
for i = 1:steps

         robot.model.animate(qMatrixb8e8(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb8e8(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb8e8(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP8 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints8 = (BP8 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh8_h.Vertices = updatedPoints8(:,1:3);
         drawnow()
         
end

display(['Retrieving brick 9'])
for i = 1:steps

         robot.model.animate(qMatrixe8b9(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe8b9(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe8b9(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end

display(['Depositing brick 9'])
%Move brick 9 from start to end position then from brick 9 end position to
%robot home position
for i = 1:steps

         robot.model.animate(qMatrixb9e9(i,:));
         gripperRight.base = robot.model.fkine(qMatrixb9e9(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixb9e9(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         BP9 = robot.model.fkine(robot.model.getpos()).T;
         updatedPoints9 = (BP9 * [brickVerts,ones(brickVertexCount,1)]')';
         brickMesh9_h.Vertices = updatedPoints9(:,1:3);
         drawnow()
         
end

display(['Back to home position'])
for i = 1:steps

         robot.model.animate(qMatrixe90(i,:));
         gripperRight.base = robot.model.fkine(qMatrixe90(i,:));
         gripperLeft.base = robot.model.fkine(qMatrixe90(i,:));
         % gripperRight.animate(qRight);
         % gripperLeft.animate(qLeft);
         drawnow()
end