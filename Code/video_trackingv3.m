%ways to speed it up:
% Add a ROI --
% change parameters on imfindcircles
% find a faster detector than imfindcircles  (using a ROI, a very simple
% blob detector, tuned to find circles of a given size, may work)
% I'm not finding the end of the video correctly


format compact
close all
clear all
clc
videoName = 'output3.ogg';
display(['trying to load ', videoName])
vidBead = VideoReader(videoName);
numframes = vidBead.NumberOfFrames;
frame = 1;

%obj = imaq.VideoDevice('dcam',1,'F7_Y8_640x480_mode0'); %camera link and object 
%a = arduino('/dev/ttyACM0','Mega2560'); %Ccreate object link for arduino
%controller; not needed for dummy testing

numframes = 210;
xyCMD = [400,400];
rect = [0 0 480 864];
%yCMD = 300;
I_MAX = 20;
%I_CMD = 0;
%I_OUT = 0;
error_gain = [0,0];

ballpos = zeros(numframes,2);
error_xy = [0,0];
%f = figure(1);
%I=step(obj);
%I = step(vidBead);
%h = imshow(I);
%e = imellipse(gca, [333 208 96 89]);
%ballCMDpos = [xCMD,yCMD];
%B2 = createMask(e,h_im);

%rROI = 25; %radius of ROI
sx = 400;
sy = 400;

for frame = 1:numframes
    try
       %I=step(obj);
       I = im2double(read(vidBead,frame)); %read frame
    catch e
        display (e);
    end
   if (frame>5)
            %e = imellipse(gca, [sx sy 50 50]);
            %h_im = imshow(I);
            %h = imrect(gca, [(sx-50) (sy-50) 100 100]);
            %mask = createMask(h,h_im);
            Ymin = max(100,round(sy - 100));
            Ymax = max(400,round(sy + 200));
            Xmin = max(100,round(sx - 100));
            Xmax = max(400,round(sx + 200));
            level = graythresh(I);
            BW = im2bw(I,0.7);
            imshow(BW((Ymin:Ymax),(Xmin:Xmax)));
            %BW2 = BW((Ymin:Ymax),(Xmin:Xmax));
            %I2 = imcrop(I,h);
            %level = graythresh(I2);
            %BW = im2bw(I2,.7);
            %imshow(I);  
            
    else
            level = graythresh(I);
            BW = im2bw(I,.7);
            Xmin = 1;
            Xmax = 850;
            Ymin = 1;
            Ymax = 480;
            imshow(BW((Ymin:Ymax),(Xmin:Xmax)));
            
    end 
        
        
        %d = imdistline;  %Run this to find object's diameter is about 20.
        %Divide by two to get radius
        %12.8 7 to 13
        %12.8 7 to 13
        
        % thanks: http://www.mathworks.com/help/images/examples/detect-and-measure-circular-objects-in-an-image.html?refresh=true
        %[centers, radii] = imfindcircles(I,[7
        %10],'ObjectPolarity','dark','Sensitivity',0.95); Camera Feed only
        [centers, radii] = imfindcircles(BW((Ymin:Ymax),(Xmin:Xmax)),[7 15],'ObjectPolarity','dark','Sensitivity',0.93);
        %Motor Controller
        if numel(centers)>1
            error_xy = xyCMD - centers(1,1:2);  %find error in position
            sx = centers(1,1);
            sy = centers(1,2);
            %rect = [(sx-rROI) (sy-rROI) (sx+rROI) (sy+rROI)]; 
        else
            display('using old data')
        end
        %sx = centers(1,1);
        %sy = centers(1,2);
        
        error_gain = Controller(error_xy);
        
        I_CMD_x = error_gain(1,1);
        I_CMD_y = error_gain(1,2);
        
        DummyArduino(I_CMD_x,I_CMD_y);
           
     ballpos(frame,:) = centers(1,1:2);

     %display e;
     %display "Error found";
     %end
     %imshow(I2);
     %set(h,'CData',I);
     h = viscircles(centers,radii); %need to scale?
     title(['Frame ', num2str(frame), ', Centroid: [',num2str(centers(1,1)),',',num2str(centers(1,2)),']']);
     drawnow();
     frame = frame+1;
     %e = imellipse(gca, [sx sy 96 89]);
     %flag=1;
end
 
figure(1);
plot(ballpos(:,1), ballpos(:,2),'.-',...
    ballpos(1,1), ballpos(1,2),'go', ballpos(end,1), ballpos(end,2),'rx');
legend('Path','start','end');
xlabel('x, (px)');
ylabel('y, (px)');

