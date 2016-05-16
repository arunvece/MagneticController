function [ angle_x_1,angle_x_2,angle_y_1,angle_y_2 ] = RealArduino(I_CMD_1_x,I_CMD_2_x,I_CMD_1_y,I_CMD_2_y,s,s1,s2,s3)
%Command program to send PWM signals to the current controller

I_MAX = 20;

%ln -s /dev/ttyACM0 /dev/ttyS101 - CMD used to make link

%minPulse = 1000e-6;
%maxPulse = 2000e-6;


if ((abs(I_CMD_1_x) <= 20) || (abs(I_CMD_2_x) <= 20))
    angle_x_1 = (I_CMD_1_x/I_MAX);
    angle_x_2 = (I_CMD_2_x/I_MAX);
    writePosition(s,angle_x_1);
    writePosition(s1,angle_x_2);
    %display(angle_x);

else
    I_CMD_1_x = 10; %Current output set to 0.5*max allowed
    I_CMD_2_X = 10; %Current output set to 0.5*max allowed
    %display 'Too Much X Current';
    angle_x_1 = (I_CMD_1_x/I_MAX);
    angle_x_2 = (I_CMD_2_x/I_MAX);
    writePosition(s,angle_x);
    writePosition(s1,angle_x_2);
    %display(angle_x);
end
    
if ((abs(I_CMD_1_y) <= 20) || (abs(I_CMD_2_y) <= 20))
    angle_y_1 = (I_CMD_1_y/I_MAX);
    angle_y_2 = (I_CMD_2_y/I_MAX);
    writePosition(s2,angle_y_1);
    writePosition(s3,angle_y_2);
    %display(angle_x);
    
else
    I_CMD_1_y = 10; %Current output set to 0.5*max allowed
    I_CMD_2_y = 10; %Current output set to 0.5*max allowed
    %display 'Too Much X Current';
    angle_y_1 = (I_CMD_1_y/I_MAX);
    angle_y_2 = (I_CMD_2_y/I_MAX);
    writePosition(s2,angle_y);
    writePosition(s3,angle_y_2);
end
end

