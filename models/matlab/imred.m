function redImage = imred (im)
 %%%%%%%% redimage (:,:,1) is red channel
 imHSV=rgb2hsv(im);
 redImage=[];
 HEIGHT=length (imHSV (1,:,:));
 WIDTH=length (imHSV (:,1,:));
 SPACING = 1; % px

%% Sample Image
imSample = [];
i = 0;
for w = 1:SPACING:WIDTH
    i = i + 1;
    j = 0;
    for h = 1:SPACING:HEIGHT
        j = j + 1;
        imSample (i,j,1) = imHSV (w,h,1);
        imSample (i,j,2) = imHSV (w,h,2);
        imSample (i,j,3) = imHSV (w,h,3);
    end
end

%% Change to Red or Not Image
for w = 1:length (imSample (:,1,:))
    for h = 1:length (imSample (1,:,:))
        if isRed (imSample (w,h,1), imSample (w,h,2), imSample (w,h,3))
            redImage(w,h)=0;
        else redImage(w,h)=1;
        end
    end
end

%% Determine if Red
function bool = isRed (hue,sat,val)
    bool = false;
    if ((hue > 0.90 || hue < 0.1) && sat > 0.4)
    %if ((hue > 0.90 || hue < 0.01) && sat > 0.4 && val > 0.2)
        bool = true;
    end
end

end