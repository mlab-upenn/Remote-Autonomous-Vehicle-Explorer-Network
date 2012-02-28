% R.A.V.E.N. Quadrotor
% Wii IR Camera Visualizer
% William Etter (EE '11) and Paul Martin (EE '11)

% INFO
% This script reads the serial data output from the microcontroller and
% visually plots the infrared points as seen by the Wii Remote IR Camera

%% Setup
clear;
clc;
close all;
%% Open USB and select device
ports = instrhwinfo('serial');
if length(ports.AvailableSerialPorts) < 1
    error('No Serial Ports Available');
else
    PORT = ports.AvailableSerialPorts{1};
end

wiiSerial = serial(PORT);
set(wiiSerial,'Baudrate',115200);
set(wiiSerial,'Timeout',5);

%% Open Serial Port
disp('Open Port');
fopen(wiiSerial);
%% Set Up Figure
f1 = figure('Name','Wii IR Camera');
axis([0 1023 0 768])
hold on;
blob1 = plot(-10,-10,'ob','MarkerEdgeColor','b','MarkerFaceColor','b',...
    'MarkerSize',7);
blob2 = plot(-10,-10,'og','MarkerEdgeColor','g','MarkerFaceColor','g',...
    'MarkerSize',7);
blob3 = plot(-10,-10,'or','MarkerEdgeColor','r','MarkerFaceColor','r',...
    'MarkerSize',7);
blob4 = plot(-10,-10,'oy','MarkerEdgeColor','y','MarkerFaceColor','y',...
    'MarkerSize',7);
set(get(f1,'CurrentAxes'),'Color',[0 0 0]);
ylabel('Y Coordinate (px)');
xlabel('X Coordinate (px)');
title('IR Camera Point Tracking');
hold off;
x1array = [-10];
y1array = [-10];
x2array = [-10];
y2array = [-10];

%% Main Loop
clear mymovie;
while(1)
    %for frame = 1:350
    pause(.02);
    if(wiiSerial.BytesAvailable)
        st = fgetl(wiiSerial);
        sts = regexp(st,'\,','split');
        x1 = str2num(char(sts(1)));%#ok<ST2NM>
        y1 = str2num(char(sts(2))); %#ok<ST2NM>
        x2 = str2num(char(sts(3))); %#ok<ST2NM>
        y2 = str2num(char(sts(4))); %#ok<ST2NM>
        x3 = str2num(char(sts(5))); %#ok<ST2NM>
        y3 = str2num(char(sts(6))); %#ok<ST2NM>
        x4 = str2num(char(sts(7))); %#ok<ST2NM>
        y4 = str2num(char(sts(8))); %#ok<ST2NM>
        if(x1>0 && y1>0)
            x1array = [x1array x1];
            y1array = [y1array (768-y1)];
            %y1 = 768-y1;
        else
            x1 = -10;
            y1 = -10;
        end
        
        %         if(x1 == 0 && y1 == 0)
        %             x1 = -1;
        %             y1 = -1;
        %         else
        %             x = [x x1];
        %             y = [y y1];
        %         end
        
        if(x2>0 && y2>0)
            x2array = [x2array x2];
            y2array = [y2array (768-y2)];
            %y1 = 768-y1;
        else
            x2 = -10;
            y2 = -10;
        end
        %         if(x2 == 0 && y2 == 0)
        %             x2 = -100;
        %             y2 = -100;
        %         else
        %             y2 = 768 - y2;
        %         end
        if(x3 == 0 && y3 == 0)
            x3 = -100;
            y3 = -100;
        else
            y3 = 768 - y3;
        end
        if(x4 == 0 && y4 == 0)
            x4 = -100;
            y4 = -100;
        else
            y4 = 768 - y4;
        end
        
        
        if(length(x1array)>35)
            x1array = x1array(2:length(x1array));
            y1array = y1array(2:length(y1array));
        end
        if(length(x2array)>35)
            x2array = x2array(2:length(x2array));
            y2array = y2array(2:length(y2array));
        end
        
        set(blob1,'XData',x1array);
        set(blob1,'YData',y1array);
        set(blob2,'XData',x2array);
        set(blob2,'YData',y2array);
        set(blob3,'XData',x3);
        set(blob3,'YData',y3);
        set(blob4,'XData',x4);
        set(blob4,'YData',y4);
        %mymovie(frame) = getframe(f1);
    end
end
%movie2avi(mymovie,'test.avi', 'fps', 10);



