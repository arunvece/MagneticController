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
%videoName = 'output_new.avi';
%display(['trying to load ', videoName])
%vidBead = VideoReader(videoName);
%numframes = vidBead.NumberOfFrames;
%frame = 1;

obj = imaq.VideoDevice('dcam',1,'F7_Y8_640x480_mode0'); %camera link and object 

numframes = 50;
%init plot:

ballpos = zeros(numframes,2);
f = figure(1);
I=step(obj);
h = imshow(I);
e = imellipse(gca, [333 208 96 89]);
%B2 = createMask(e,h_im);

sx = 333;
sy = 208;
for frame = 1:50
    try
        I=step(obj);
        
        %I = im2double(read(vidBead,frame)); %read frame
        
        %imshow(I)
       level = graythresh(I);
       BW = im2bw(I,.7);
        
        %d = imdistline;  %Run this to find object's diameter is about 20.
        %Divide by two to get radius
        %12.8 7 to 13
        %12.8 7 to 13
        
        % thanks: http://www.mathworks.com/help/images/examples/detect-and-measure-circular-objects-in-an-image.html?refresh=true
        [centers, radii] = imfindcircles(I,[7 10],'ObjectPolarity','dark','Sensitivity',0.95);
        try
            
        ballpos(frame,:) = centers(1,1:2);
        catch e
            imshow(I);
            display e;
            display "Error found";
        end
        imshow(I)
        
        %set(h,'CData',I);
        h = viscircles(centers,radii);
        title(['Frame ', num2str(frame), ', Centroid: [',num2str(centers(1,1)),',',num2str(centers(1,2)),']'])
    drawnow()
    catch error
        display(error)
    end
   
   % frame = frame+1;  
    
end


figure(1)
plot(ballpos(:,1), ballpos(:,2),'.-',...
    ballpos(1,1), ballpos(1,2),'go', ballpos(end,1), ballpos(end,2),'rx');
legend('Path','start','end')
xlabel('x, (px)')
ylabel('y, (px)')