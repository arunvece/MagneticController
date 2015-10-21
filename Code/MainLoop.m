%Main (fast) loop for control of sphere

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
obj = imaq.VideoDevice('dcam',1,'F7_Y8_640x480_mode0'); %camera link and object 
display('Connecting to Arduino');
a=arduino('/dev/ttyS101','Mega2560'); %create object for controller
%controller; not needed for dummy testing
s = servo(a, 'D4', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse); %connect PWN control wire to digital pin #4.
xyCMD = [400,400]; %Desired poistion for sphere. Can be an array of points but no support built in yet
I_CMD_x = 0;
I_CMD_y = 0;
error_xy = [0,0];
parameters();   %Load model parameters
I=step(obj);    %Create link between I (image file) and camera object (RAW image stream) 

%MAIN Loops
while 1
    tmp = rand;
    if tmp > limit %Break loop condition
        break
    end
    %s = s + tmp;
    %estimator run first and call the DummyArduino or RealArduino
    %functions, Controller function required as well
    function dx = motion(t,x)
    dx = zeros(3,1);    % a column vector for equation set
    dx(1) = x(2) * x(3);
    dx(2) = -x(1) * x(3);
    dx(3) = -0.51 * x(1) * x(2);
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4 1e-4 1e-5]);
    [~,X] = ode45(@motion,[t1 t2],[(centers(1,1)) 1 1],options);
        
    function dy = motion1(t,y)
    dy = zeros(3,1);    % a column vector for equation set
    dy(1) = y(2) * y(3);
    dy(2) = -y(1) * y(3);
    dy(3) = -0.51 * y(1) * y(2);
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4 1e-4 1e-5]);
    [T,Y] = ode45(@motion,[t1 t2],[(centers(1,2)) 1 1],options);
    %insert Contoller here
    
    %insert DummyArduino here
    %RealArduino(I_CMD_x,I_CMD_y,s);
    DummyArduino( I_CMD_x,I_CMD_y)
    %insert image capture here
    if (timer expired)
        ImageTrack(centers);
        reset timer
    end
    
    x_pos_inital = centers(1,1);    %scale pixels to cm? May not be needed since the pixels may be good 
    y_pos+inital = centers(1,2);    %scale pixels t cm? May not be needed since the pixels may be good
    
    
    
end