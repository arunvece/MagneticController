%Main (fast) loop for control of sphere
function MainLoop()
format compact
close all
clear all
clc
%SETUP for loops
t1 = 0; %begin time for ODE solver
t2  = 0.100; %end time for ODE solver
minPulse = 1000e-6; %Min pulse width for hardware controller, Standard RC Mode
maxPulse = 2000e-6; %Max pulse width for hardware controller, Standard RC Mode
%limit = 0.8;
%s = 0;
display('Connecting to Camera');
obj = imaq.VideoDevice('dcam',1,'F7_Y8_640x480_mode0'); %camera link and object 
display('Connecting to Arduino');
a=arduino('/dev/ttyS101','Mega2560'); %create object for controller
%controller; not needed for dummy testing

%s = servo(a, 'D4', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse); %connect PWN control wire to digital pin #4.
%s1 = servo(a, 'D5', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse);%connect PWN control wire to digital pin #5.
%s2 = servo(a, 'D6', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse);%connect PWN control wire to digital pin #6.
%s3 = servo(a, 'D7', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse);%connect PWN control wire to digital pin #7.

xyCMD = [400,400]; %Desired poistion for sphere. Can be an array of points but no support built in yet
I_CMD_x = 0;
I_CMD_y = 0;
I_MAX = 20;
error_xy = [0,0];
parameters();   %Load model parameters
I=step(obj);    %Create link between I (image file) and camera object (RAW image stream) 
trigger = 0;
velx = 0;
vely = 0;

%MAIN Loop
while 1
    tic = timerVal;
    tmp = rand;
    if tmp > limit %Break loop condition
        break
    end
    %s = s + tmp;
    %estimator run first and call the DummyArduino or RealArduino
    %functions, Controller function required as well
    
    [G_x,G_y] = MagGrad(centers(1,1),centers(1,2),I_MES_1_x,I_MES_2_x,I_MES_2_y,I_MES_1_y);
        

    options = odeset('RelTol',1e-4,'AbsTol',[1e-4 1e-4]);
    [T,X] = ode45(@motion,[t1 t2],[(centers(1,1)) velx],options);
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4 1e-4]);
    [T,Y] = ode45(@motion1,[t1 t2],[(centers(1,2)) vely],options);
    
    
    error_xy = centers-xyCMD;
    
    %insert Contoller here
    error_gain = Controller(error_xy);
    
    I_CMD_1_x = error_gain(1,1)*I_MAX;
    I_CMD_2_x = error_gain(1,1)*I_MAX;
    I_CMD_1_y = error_gain(1,2)*I_MAX;
    I_CMD_2_y = error_gain(1,2)*I_MAX;
    
    %insert DummyArduino here
    %RealArduino(I_CMD_1_x,I_CMD_2_x,I_CMD_1_y,I_CMD_2_y,s,s1,s2,s3);
    DummyArduino(I_CMD_1_x,I_CMD_2_x,I_CMD_1_y,I_CMD_2_y);
    %insert image capture here
   
    if (elapsedTime == 0.0100)  %run image program every 100 ms
        centers = ImageTrack(obj);
    end
   
    %x_pos_inital = centers(1,1);    %scale pixels to cm? May not be needed since the pixels may be good 
    %y_pos_inital = centers(1,2);    %scale pixels t cm? May not be needed since the pixels may be good  
    
    elapsedTime = toc(timerVal);
end

    function dy = motion1(t,y)
    dy = zeros(2,1);        %a column vector for equation set
    dy(1) = y(2);           %position
    dy(2) = S1*y(2) + G_y;   %velocity
    end

    function dx = motion(t,x)
    dx = zeros(2,1);      %a column vector for equation set
    dx(1) = x(2);         %position
    dx(2) = S1*x(2) +G_x;   %velocity
    end

end