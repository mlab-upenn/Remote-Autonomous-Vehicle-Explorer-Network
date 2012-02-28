%% Simulate Follower Model
clc;
close all;
clear all;

%% Define Variables
hoverThrust = 0.900*9.81;
fiducialHeight = 0.1; % m
xBound = 20;
yBound = 20;
zBound = 15;
scrsz = get(0, 'ScreenSize');
figHeight = 700;
figWidth = 1400;
fig = figure('Position',[0,200,figWidth,figHeight]);

totalTime = 30; % seconds
dT = 0.1; % seconds

%% Event List for Leader
% [time(sec) thrust yaw pitch roll]
events = [  1 hoverThrust+1 0 0 0;
    3 hoverThrust-1 0 0 0;
    5 hoverThrust+0 0 0 0;
    6 hoverThrust+0 0 10 0;
    7 hoverThrust+0 0 0 0;
    9 hoverThrust+0 0 -10 0;
    11 hoverThrust+.1 0 0 0;
    12 hoverThrust+.1 0 0 10;
    14 hoverThrust+.1 0 0 -10;
    15 hoverThrust+.2 0 10 -10;
    16 hoverThrust+.3 20 5 5;
    16.5 hoverThrust+.1 40 0 5;
    17 hoverThrust+.1 60 0 5;
    17.5 hoverThrust+.1 80 0 -10;
    18 hoverThrust+0 90 0 0;
    22 hoverThrust+.3 90 10 0;
    24 hoverThrust+.1 70 5 -10];

lastEvent = 0;

%% Initialize Quadrotor Units
% quad 1 [X,Y,Z,YAW,PITCH,ROLL,ARMLENGTH]
quad1 = Quad(5,5,0,0,0,0);
% quad 2
quad2 = Quad(2,4.5,0,0,0,0);
quad2 = quad2.setParent(quad1,2,1,0);
% quad 3
quad3 = Quad(1,6,0,0,0,0);
quad3 = quad3.setParent(quad1,2,-1,0);
% quad 4
quad4 = Quad(0,3,0,0,0,0);
quad4 = quad4.setParent(quad2,2,1,0);
% quad 5
quad5 = Quad(0,4,0,0,0,0);
quad5 = quad5.setParent(quad2,2,-1,0);
% quad 6
quad6 = Quad(0,8,0,0,0,0);
quad6 = quad6.setParent(quad3,2,-1,0);


% -- DEFINE FIGURE PLOTS --
axis_perspective = subplot(1,1,1);
set(axis_perspective, 'CameraPosition', [xBound yBound zBound], 'CameraTarget', [0 0 0]);
xlabel(axis_perspective,'X');
ylabel(axis_perspective,'Y');
zlabel(axis_perspective,'Z');

%% MAIN LOOP
pause(0.5);
clear mymovie;
info_11 = zeros(1,totalTime/dT);
info_12 = zeros(1,totalTime/dT);
times = (0:dT:totalTime);
for t = 0:dT:totalTime
    count = round(t/dT + 1);
    % -- HANDLE EVENTS --
    for i = 1:length(events(:,1))
        if t >= events(i,1) && events(i,1) > lastEvent
            lastEvent = events(i,1);
            quad1 = quad1.setThrust(events(i,2));
            quad1 = quad1.setYaw(events(i,3));
            quad1 = quad1.setPitch(events(i,4));
            quad1 = quad1.setRoll(events(i,5));
        end
    end
    
    
    
    % -- PERSPECTIVE VIEW --
    axes(axis_perspective);
    cla;
    hold on;
    % plot quad1
    quad1.drawArms();
    quad1.drawBody();
    % plot quad2
    quad2.drawArms();
    quad2.drawBody();
    % plot quad3
    quad3.drawArms();
    quad3.drawBody();
    % plot quad4
    quad4.drawArms();
    quad4.drawBody();
    % plot quad5
    quad5.drawArms();
    quad5.drawBody();
    % plot quad6
    quad6.drawArms();
    quad6.drawBody();
    
    axis([0 8 2 10 0 8]);
    grid on;
    colormap gray
    
    
    % -- VISION PROCESSING FOR FOLLOWERS --
    quad2 = quad2.parseCamera(dT,0);
    quad3 = quad3.parseCamera(dT,0);
    quad4 = quad4.parseCamera(dT,0);
    quad5 = quad5.parseCamera(dT,0);
    quad6 = quad6.parseCamera(dT,0);
    
   
    
    % -- UPDATE ALL PARAMETERS --
    % quad1
    quad1 = quad1.update(dT,20,0,0.6);
    % quad2
    quad2 = quad2.simulateRF(0);
    quad2 = quad2.respond(dT);
    quad2 = quad2.setParent(quad1,2,1,0);
    quad2 = quad2.update(dT,20,0,0.6);
    % quad3
    quad3 = quad3.simulateRF(0);
    quad3 = quad3.respond(dT);
    quad3 = quad3.setParent(quad1,2,-1,0);
    quad3 = quad3.update(dT,20,0,0.6);
    % quad4
    quad4 = quad4.simulateRF(0);
    quad4 = quad4.respond(dT);
    quad4 = quad4.setParent(quad2,2,1,0);
    quad4 = quad4.update(dT,20,0,0.6);
    % quad5
    quad5 = quad5.simulateRF(0);
    quad5 = quad5.respond(dT);
    quad5 = quad5.setParent(quad2,2,-1,0);
    quad5 = quad5.update(dT,20,0,0.6);
    % quad6
    quad6 = quad6.simulateRF(0);
    quad6 = quad6.respond(dT);
    quad6 = quad6.setParent(quad3,2,-1,0);
    quad6 = quad6.update(dT,20,0,0.6);
    
end

% to save movie: movie2avi(mymovie,'test.avi', 'fps', 10);

