function[] = environment()


axis ([-4 8 -6 6 -0.5 3])
hold on


%Concrete Floor
surf([-6 -6; 8, 8] ...
    , [-6, 6; -6, 6] ...
    , [-0.5, -0.5; -0.5, -0.5] ...
    , 'CData', imread('concrete.jpg') ...
    , 'FaceColor', 'texturemap');

%Table for ur3
table1 = PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0,0,-0.5]);

%Table for ur5
table2 = PlaceObject('tableBrown2.1x1.4x0.5m.ply', [4,0,-0.5]);




%Top fencing
fence = PlaceObject('barrier1.5x0.2x1m.ply', [-1.75, 3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [-0.25, 3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [1.25, 3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [2.75, 3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [4.25, 3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [5.75, 3, -0.5]);


%Bottom fencing
fence = PlaceObject('barrier1.5x0.2x1m.ply', [-1.75, -3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [-0.25, -3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [1.25, -3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [2.75, -3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [4.25, -3, -0.5]);
fence = PlaceObject('barrier1.5x0.2x1m.ply', [5.75, -3, -0.5]);



%Right fencing
fence1 = PlaceObject('barrier1.5x0.2x1m.ply', [-2.2, 6.45, -0.5]);
fence2 = PlaceObject('barrier1.5x0.2x1m.ply', [-0.7, 6.45, -0.5]);
fence3 = PlaceObject('barrier1.5x0.2x1m.ply', [0.8, 6.45, -0.5]);
fence4 = PlaceObject('barrier1.5x0.2x1m.ply', [2.3, 6.45, -0.5]);


fence1Vert = [get(fence1, 'Vertices'), ones(size(get(fence1,'Vertices'),1),1)] *trotz(pi/2);
set(fence1, 'Vertices', fence1Vert(:,1:3));	

fence2Vert = [get(fence2, 'Vertices'), ones(size(get(fence2,'Vertices'),1),1)] *trotz(pi/2);
set(fence2, 'Vertices', fence2Vert(:,1:3));

fence3Vert = [get(fence3, 'Vertices'), ones(size(get(fence3,'Vertices'),1),1)] *trotz(pi/2);
set(fence3, 'Vertices', fence3Vert(:,1:3));

fence4Vert = [get(fence4, 'Vertices'), ones(size(get(fence4,'Vertices'),1),1)] *trotz(pi/2);
set(fence4, 'Vertices', fence4Vert(:,1:3));


%Left fencing
fence5 = PlaceObject('barrier1.5x0.2x1m.ply', [-2.2, 2.4, -0.5]);
fence6 = PlaceObject('barrier1.5x0.2x1m.ply', [-0.7, 2.4, -0.5]);
fence7 = PlaceObject('barrier1.5x0.2x1m.ply', [0.8, 2.4, -0.5]);
fence8 = PlaceObject('barrier1.5x0.2x1m.ply', [2.3, 2.4, -0.5]);


fence5Vert = [get(fence5, 'Vertices'), ones(size(get(fence5,'Vertices'),1),1)] *trotz(-pi/2);
set(fence5, 'Vertices', fence5Vert(:,1:3));	

fence6Vert = [get(fence6, 'Vertices'), ones(size(get(fence6,'Vertices'),1),1)] *trotz(-pi/2);
set(fence6, 'Vertices', fence6Vert(:,1:3));

fence7Vert = [get(fence7, 'Vertices'), ones(size(get(fence7,'Vertices'),1),1)] *trotz(-pi/2);
set(fence7, 'Vertices', fence7Vert(:,1:3));

fence8Vert = [get(fence8, 'Vertices'), ones(size(get(fence8,'Vertices'),1),1)] *trotz(-pi/2);
set(fence8, 'Vertices', fence8Vert(:,1:3));



%Roundtable
tableRound = PlaceObject('tableRound0.3x0.3x0.3m.ply', [0,-5,-0.5]);

%Persons
maleCon = PlaceObject('personMaleConstruction.ply', [-1,-5,-0.5]);

femBusi = PlaceObject('personFemaleBusiness.ply', [1,-5,-0.5]);

maleCas = PlaceObject('personMaleCasual.ply', [0,4,-0.5]);


%Estops
wallEStop1 = PlaceObject('emergencyStopWallMounted.ply', [2,-3,-0.1]);

Estop3 = PlaceObject('emergencyStopButton.ply', [0.8,0.5,0]);

Estop5 = PlaceObject('emergencyStopButton.ply', [4.9,0.5,0]);

%Fire Extinguisher
fireExt = PlaceObject('fireExtinguisher.ply', [2,3,-0.5]);

end