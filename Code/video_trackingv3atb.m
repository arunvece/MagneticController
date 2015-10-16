function video_trackingv3atb
%ways to speed it up:
% Add a ROI --
% change parameters on imfindcircles
% find a faster detector than imfindcircles  (using a ROI, a very simple
% blob detector, tuned to find circles of a given size, may work)
% I'm not finding the end of the video correctly

%TODO:  ROI should grow if the detection fails or if the velocity is large
% my velocity estimator is horrible
%  imfindcircles is still too slow -- 0.0233s per frame.
% we can try just finding the average position of the dark spot.

addpath('../Test Videos/')
format compact
close all
clear all
clc
videoName = 'output3.mp4';% 'output3.ogg';
display(['trying to load ', videoName])
vidBead = VideoReader(videoName);
numframes = vidBead.NumberOfFrames;

%obj = imaq.VideoDevice('dcam',1,'F7_Y8_640x480_mode0'); %camera link and object 
%a = arduino('/dev/ttyACM0','Mega2560'); %Ccreate object link for arduino
%controller; not needed for dummy testing

%numframes = 50;
xyCMD = [400,400];
%yCMD = 300;
%I_CMD = 0;
%I_OUT = 0;

ballpos = zeros(numframes,2);
error_xy = [0,0];
%f = figure(1);
%I=step(obj);
%I = step(vidBead);
%h = imshow(I);
%e = imellipse(gca, [333 208 96 89]);
%ballCMDpos = [xCMD,yCMD];
%B2 = createMask(e,h_im);


% Ball is being tracked by colour intensity. Light colours are filtered out, and avareage of X and Y coordinates of the rest makes the output. Only region covered by blue square (generated with X and Y coordinates from previous frame) is taken into calculations for better performance

ballRad= 9.5;

centers = [327.3083  200.2870];%hardcoded initial position.
vel = [0,0]; %simple velocity estimate


for frame = 1:numframes
    try
       %I=step(obj);
       I = im2double(read(vidBead,frame)); %read frame
    catch e
        display (e);
    end

    
    [ym,yM,xm,xM] = setROI(centers+1/2*vel);
    BW = im2bw(I(ym:yM,xm:xM,:),.7);%BW=im2bw(I,.7);
        
        %d = imdistline;  %Run this to find object's diameter is about 20.
        %Divide by two to get radius
        %12.8 7 to 13
        %12.8 7 to 13
        
        % thanks: http://www.mathworks.com/help/images/examples/detect-and-measure-circular-objects-in-an-image.html?refresh=true
        %[centers, radii] = imfindcircles(I,[7
        %10],'ObjectPolarity','dark','Sensitivity',0.95); Camera Feed only
        centersp = centers; %update previous position
        [centers, radii] = imfindcircles(BW,[ballRad-2.5 ballRad+2.5],'ObjectPolarity','dark','Sensitivity',0.93);
        centers = [xm,ym]+centers;  %add ROI offset
        vel = 1/3*(vel+2*(centers-centersp)); %simple velocity estimator
        %Motor Controller
        if numel(centers)>1
            error_xy = xyCMD - centers(1,1:2);  %find error in position
        else
            display('using old data')
        end

        error_gain = Controller(error_xy);
        
        I_CMD_x = error_gain(1,1);
        I_CMD_y = error_gain(1,2);
        
        DummyArduino(I_CMD_x,I_CMD_y);
           
     ballpos(frame,:) = centers(1,1:2);

     %display e;
     %display "Error found";
     %end
     imshow(I)
     %set(h,'CData',I);
     viscircles( centers,radii);
     rH = rectangle('Position',[xm,ym,xM-xm,yM-ym]);
     set(rH,'EdgeColor','b');
     title(['Frame ', num2str(frame), ', Centroid: [',num2str(centers(1,1)),',',num2str(centers(1,2)),']']);
     drawnow();
     %e = imellipse(gca, [sx sy 96 89]);
     %flag=1;
end
 
figure(1);
plot(ballpos(:,1), ballpos(:,2),'.-',...
    ballpos(1,1), ballpos(1,2),'go', ballpos(end,1), ballpos(end,2),'rx');
axis equal  %necessary since this is x and y position
legend('Path','start','end');
xlabel('x, (px)');
ylabel('y, (px)');


function [ym,yM,xm,xM] = setROI(centers)
yR = 480; %height of image
xR = 864; %width of image
% centers = 327.3083  200.2870 for ball 1
rROI = 30; %radius of ROI
ym = max(1,floor(centers(2) - rROI));
yM = min(yR,ceil(centers(2) + rROI));
xm = max(1,floor(centers(1) - rROI));
xM = min(xR,ceil(centers(1) + rROI));
end

end