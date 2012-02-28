%% Simulate Follower Model
clc;
close all;
clear all;

%% Define Variables
hoverThrust = 0.900*9.81;
fiducialHeight = 0.1; % m
xBound = 20;
yBound = 20;
zBound = 7;
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

% -- DEFINE FIGURE PLOTS --
axis_perspective = subplot(3,2,[1 3 5]);
set(axis_perspective, 'CameraPosition', [xBound yBound zBound], 'CameraTarget', [0 0 0]);
xlabel(axis_perspective,'X');
ylabel(axis_perspective,'Y');
zlabel(axis_perspective,'Z');
axis_info1 = subplot(3,2,2);
title(axis_info1,'Follower 1 Distance (Desired in red)');
xlabel(axis_info1,'Time (sec)');
ylabel(axis_info1,'Distance (m)');
axis_info2 = subplot(3,2,4);
title(axis_info2,'-unused-');
xlabel(axis_info2,'Time (sec)');
ylabel(axis_info2,'Distance (m)');
axis_info3 = subplot(3,2,6);
title(axis_info3,'-unused-');
xlabel(axis_info3,'Time (sec)');
ylabel(axis_info3,'Distance (m)');

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
    
    axis([0 xBound 0 yBound 0 zBound]);
    grid on;
    colormap gray
    
    
    % -- VISION PROCESSING FOR FOLLOWERS --
    quad2 = quad2.parseCamera(dT,0);
    quad3 = quad3.parseCamera(dT,0);

    % -- ERROR INFORMATION PLOTS --
    axes(axis_info1);
    cla;
    hold on;
     
    % Follower 1 x distance
    info_11(count) = quad2.dist_x_des;
    info_12(count) = quad2.cam_x;
    plot(times(1:count),info_11(1:count),'r','LineWidth',2);
    plot(times(1:count),info_12(1:count),'.-b','LineWidth',1);
    xlim([(t-5) (t+1)]);
    ylim([quad2.dist_x_des-5 quad2.dist_x_des+5]);
    
    % -- SAVE AS MATLAB MOVIE --
    %mymovie(round(t/dT+1)) = getframe(fig);
    
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

end

% to save movie: movie2avi(mymovie,'test.avi', 'fps', 10);

