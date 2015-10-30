addpath('../Test Videos/')
format compact
close all
clear all
clc
%videoName = 'output_new.avi' %'output3.mp4'% 'output3.ogg';
%display(['trying to load ', videoName])
%vidBead = VideoReader(videoName);
%I = step(vidBead);
%numframes = vidBead.NumberOfFrames;

obj = imaq.VideoDevice('dcam',1,'F7_Y8_640x480_mode0'); %camera link and object 
I=step(obj);
numframes = 399;
ballpos = zeros(numframes,2);


for frame = 1:numframes
    try
       I=step(obj);
       %I = im2double(read(vidBead,frame)); %read frame
    catch e
        display (e);
    end
    
    level = graythresh(I);
    BW = im2bw(I,level);
%imshow(BW2);

%se = strel('disk',15);
%BW5 = imfill(imdilate(BW,se),'holes');
%figure, imshow(BW5)

%figure; imshow(boxImage);
%title('Image of a Box');

%e = imellipse(gca,[829 1240 336 312]);
%BW = createMask(e,BW2);
%BW = bwareaopen(BW2,300);

    

    hblob = vision.BlobAnalysis;
    hblob.AreaOutputPort = false;
    hblob.BoundingBoxOutputPort = false;
    centroid = step(hblob, BW); % [x y] coordinates of the centroid
    
    ballpos(frame,:) = centroid;

    figure(1);
    plot(ballpos(:,1), ballpos(:,2),'.-',...
    ballpos(1,1), ballpos(1,2),'go', ballpos(end,1), ballpos(end,2),'rx');
    legend('Path','start','end');
    xlabel('x, (px)');
    ylabel('y, (px)');

end