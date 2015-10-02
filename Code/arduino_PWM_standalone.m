%Command program to send PWM signals to the current controller
%if nargin < 1
 %   I_CMD = 0;
%end

%if I_CMD>25
%    display('Too much current')
%    I_CMD = 25;
%end

a=arduino('/dev/ttyS101','Mega2560'); %create object for controller
%ln -s /dev/ttyACM0 /dev/ttyS101 - CMD used to make link

minPulse = 1000e-6;
maxPulse = 2000e-6;

s = servo(a, 'D4', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse); %connect PWN control wire to digital pin #4.
%display(I_CMD)

for angle = 0:0.05:1
    writePosition(s, angle);
    current_pos = readPosition(s);
    current_pos = current_pos*180;
    fprintf('Current motor position is %d degrees\n', current_pos);
    pause(2);
end

for angle = 1:0.05:0
    writePosition(s, angle);
    current_pos = readPosition(s);
    current_pos = current_pos*180;
    fprintf('Current motor position is %d degrees\n', current_pos);
    pause(2);
end

clear all;