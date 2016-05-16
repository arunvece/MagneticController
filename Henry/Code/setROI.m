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