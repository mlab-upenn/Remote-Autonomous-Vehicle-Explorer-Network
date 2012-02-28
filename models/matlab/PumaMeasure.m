% William Etter, Paul Martin, Mohit Bhoite
%        10-21-2009
% University of Pennsylvania SEAS
% CIS 390 Assignment 1 - Fall 2009


%% Draw Art
simulation = true;
if simulation
    pauselength = 0.5;
else
    pauselength=2;
end

% potential camera offsets
xd = 0*25.4;
zd = 0*25.4;

% Square Coordinates
x = 11*25.4+xd;
y1 = 0*25.4;
y2 = 5*25.4;
y3 = 10*25.4;
z1 = 20*25.4+zd;
z2 = 16.7*25.4+zd;
z3 = 13.3*25.4+zd;

% initial positions
d1=13.0*25.4;
b2=2.5*25.4;
d2=8.0*25.4;
b3=2.5*25.4;
d3=8.0*25.4;
d4=2.5*25.4;


if simulation
    currAngles = [0 0 0 0 0 0];
else
    currAngles = pumaAngles();
end

%Euler Angles
phi=180;
theta=-90;
sci=0;

%% Go to home position
disp('Please position close to initial position, press enter to continue');
pause();
if simulation
    pumaAnimate(0,0,0,0,pi/2,0);
else
    pumaServo(0,0,0,0,pi/2,0);
end

%% Position
disp('Press enter to go to starting position');
pause();
disp('Going to Starting Position');
pumaLine2(d2+xd,b2+b3,d1+d3+d4+zd, x, y1, z1, 180, 0, 0);
pause(pauselength);
while(true)
    disp('Taking picture');
    if simulation
        image = imread('circle.jpg');
    else
        image = pumaCapture();
    end
   
    
    %% Measure Distance
    height_inch = 4;
    %height_pix = 0;
    field_of_view = 13*pi/180;

    imorig=image;
    redimage=imred(imorig);

    figure;
    imshow(redimage);

    % Call bwlabel -- this idenitifies and counts that blobs.
    %   Note that the second argument indicates whether blobs are
    %   considered contiguous if they share an edge (if the value is
    %   four) or if they share either an edge or a corner (if the value
    %   is eight).
    RLL = bwlabel(redimage,8);

    % Use regionprops to compute the properties of the blobs.
    stats = regionprops(RLL,'Area','Centroid','MajorAxisLength','MinorAxisLength','Perimeter')

    height_pix = stats.MinorAxisLength; % calculate circle height in pixels
    distance = (height_inch*480)/ (2*height_pix*tan(field_of_view));%calculate distance from camera to the image plane
    disp(sprintf('Distance to the image = %d',distance));

    inch_to_pix_ratio = height_inch/height_pix;
    %finding distance between center of the image to object centroid in pixels
    x2 = stats.Centroid(1);
    y2 = stats.Centroid(2);

    %center of the image
    xc = 320;
    yc = 240;

    z_add = (yc-y2)*inch_to_pix_ratio;
    y_add = (xc-x2)*inch_to_pix_ratio;

    %offset from center in pixels
    d= sqrt(((x2-xc)^2)+((y2-yc)^2));

    %offset in inches
    offset_inch = d*inch_to_pix_ratio;

    dist_centroid = sqrt((distance^2)+(offset_inch^2));
    disp(sprintf('Distance to the centroid = %d',dist_centroid));
    x_add = distance;

    x = x/25.4;
    y1 = y1/25.4;
    z1 = z1/25.4;
    x_base = x + x_add;
    y_base = y1 + y_add;
    z_base = z1 + z_add;

    disp('Circle position in base coordinates:');
    disp(sprintf('X: %g', x_base));
    disp(sprintf('Y: %g', y_base));
    disp(sprintf('Z: %g', z_base));
    
    disp('To take another picture, press <enter>');
    disp('To terminate, press <CTRL+C>');
    pause();
end

