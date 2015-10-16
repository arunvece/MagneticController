function [ I_OUT_x,I_OUT_y ] = DummyArduino( I_CMD_x,I_CMD_y )
%Dummy Arduino Controller link for Testng Control Loop
%   This function allowes the control loop to be run and output a valid
%   current command to the arduino controller without the controller being
%   connected to the PC. Make sure the following line is commented out out
%   of the control loop when using this dummy fucntion: 
%a = arduino('/dev/ttyACM0','Mega2560') - line 20
if (abs(I_CMD_x) <= 20) 
    I_OUT_x = 1;
    %display (I_CMD_x);

else
    I_OUT_x = 0;
    I_CMD_x = 20; %Current output set to max allowed
    %display 'Too Much X Current';
    %display (I_CMD_x);
end
    
if (abs(I_CMD_y)<=20)
    I_OUT_y = 1;
    %display (I_CMD_y);
else
    I_OUT_y = 0;
    I_CMD_y = 20;
    %display 'Too Much Y Current';
    %display (I_CMD_y);
end


end

