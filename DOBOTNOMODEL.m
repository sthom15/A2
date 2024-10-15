%% Robotics
% Lab 6 
% Updated syntax and tested for V10 compatibility (2023)
close all;

%% Initialize the figure
set(0,'DefaultFigureWindowStyle','docked')
clf

% Define the robot's links with DH parameters
link1 = Link('alpha',-pi/2,'a',0,'d',-0.38,'offset',0,'qlim',[deg2rad(-117),deg2rad(117)]);
link2 = Link('alpha',pi,'a',0.385,'d',0,'offset',pi/2,'qlim',[deg2rad(-115),deg2rad(115)]);
link3 = Link('alpha',pi/2,'a',0,'d',0,'offset',-pi/2,'qlim',[deg2rad(-110),deg2rad(110)]);
link4 = Link('alpha',pi/2,'a',0,'d',-0.445,'offset',0,'qlim',[deg2rad(-200),deg2rad(200)]);
link5 = Link('alpha',-pi/2,'a',0,'d',0,'offset',0,'qlim',[deg2rad(-107),deg2rad(107)]);
link6 = Link('alpha',pi,'a',0,'d',-0.2106,'offset',0,'qlim',[deg2rad(-200),deg2rad(200)]);

% Create the SerialLink robot object
robot = SerialLink([link1 link2 link3 link4 link5 link6], 'name', 'CustomBot');


% Define the initial joint configuration
q = [0, pi/2, 0, 0, 0, 0];

% Plot the robot at the initial configuration
robot.plot(q);

% Set view to 3D and add lighting
view(3);
camlight;
axis equal;
hold on;

% Animate the robot moving up and down while rotating link 1
for i = 1:100
    % Generate a sinusoidal motion for joint 2 (up and down)
    q(2) = pi/2 + sin(i/10) * pi/8; % Sinusoidal motion for vertical movement
    
    % Rotate joint 1 (rotation of the base)
    q(1) = q(1) + deg2rad(2);  % Continuous rotation of link 1 (2 degrees per iteration)
    
    % Animate the robot with the updated joint configuration
    robot.animate(q);
    
    pause(0.05);  % Pause for a short time to control animation speed
end