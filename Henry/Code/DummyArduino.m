function [ I_OUT_x_1,I_OUT_y_1,I_OUT_x_2,I_OUT_2_y ] = DummyArduino(I_CMD_1_x,I_CMD_2_x,I_CMD_1_y,I_CMD_2_y)
%Dummy Arduino Controller link for Testng Control Loop
%   This function allowes the control loop to be run and output a valid
%   current command to the arduino controller without the controller being
%   connected to the PC. Make sure the following line is commented out out
%   of the control loop when using this dummy fucntion: 
%a = arduino('/dev/ttyACM0','Mega2560') - line 20
if ((abs(I_CMD_1_x) <= 20) || (abs(I_CMD_2_x <= 20))
    I_OUT_x_1 = 1;
    I_OUT_x_2 = 1;
    %display (I_CMD_x);

else
    I_OUT_x_1 = 0;
    I_OUT_x_2 = 0;
    %Current output set to max allowed
    %display 'Too Much X Current';
    %display (I_CMD_x);
end
    
if ((abs(I_CMD_1_y)<=20) || (abs(I_CMD_2_y)<=20))
    I_OUT_y_1 = 1;
    I_OUT_y_2 = 1;
    %display (I_CMD_y);
else
    I_OUT_y_1 = 0;
    I_OUT_y_2 = 0;
    %display 'Too Much Y Current';
    %display (I_CMD_y);
end


end

