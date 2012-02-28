clc;
close all;

perceivedD = [-1 -1 -1 78 78 -1 -1 -1;
            -1 -1 110 105 104 -1 -1 -1;
            -1 143 134 131 131 131 -1 -1;
            182 167 160 160 157 155 167 -1;
            207 188 188 188 183 180 185 -1;
            226 220 213 211 211 210 209 215];
 
realX = 2.54*[30 30 30 30 30 30 30 30;
            40 40 40 40 40 40 40 40;
            50 50 50 50 50 50 50 50;
            60 60 60 60 60 60 60 60;
            70 70 70 70 70 70 70 70;
            80 80 80 80 80 80 80 80];
realY = [-35 -25 -15 -5 5 15 25 35;
    -35 -25 -15 -5 5 15 25 35;
    -35 -25 -15 -5 5 15 25 35;
    -35 -25 -15 -5 5 15 25 35;
    -35 -25 -15 -5 5 15 25 35;
    -35 -25 -15 -5 5 15 25 35];

errors = zeros(6,8);
for i = 1:length(perceivedD(1,:))
    errors(:,i) = perceivedD(:,i)-(realD);
end

for i = 1:6
    for j = 1:8
        if perceivedD(i,j) == -1
            errors(i,j) = inf;
        end
    end
end

surf(realX,realY,errors);
colorbar;
xlabel('Distance to Leader');
ylabel('Lateral Displacement');
zlabel('Error (cm)');