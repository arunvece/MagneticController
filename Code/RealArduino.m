function [ angle_x,angle_y ] = RealArduino( I_CMD_x,I_CMD_y,s)
%Command program to send PWM signals to the current controller

I_MAX = 20;

%ln -s /dev/ttyACM0 /dev/ttyS101 - CMD used to make link

%minPulse = 1000e-6;
%maxPulse = 2000e-6;


if (abs(I_CMD_x) <= 20) 
    angle_x = (I_CMD_x/I_MAX);
    writePosition(s,angle_x);
    %display(angle_x);

else
    I_CMD_x = 10; %Current output set to 0.5*max allowed
    %display 'Too Much X Current';
    angle_x = (I_CMD_x/I_MAX);
    writePosition(s,angle_x);
    %display(angle_x);
end
    
if (abs(I_CMD_y)<=20)
    angle_y = (I_CMD_y/I_MAX);
    writePosition(s,angle_y);
    %display(angle_y);
    
else
   I_CMD_y = 10; %Current output set to 0.5*max allowed
   %display 'Too Much Y Current';
   angle_y = (I_CMD_y/I_MAX);
   writePosition(s,angle_y);
   %display(angle_y);
end
clear s;
end

