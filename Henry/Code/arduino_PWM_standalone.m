%Command program to send PWM signals to the current controller
%if nargin < 1
 %   I_CMD = 0;
%end
%D7 - Coil 1 - x1
%D8 - Coil 2 (Top)
%D9 - Coil 3 - y1
%D10 -Coil 4 (Bottom)
%D11 - Coil 5 - x2
%D12 - Coil 6 - y2


%if I_CMD>25
%    display('Too much current')
%    I_CMD = 25;
%end

 a=arduino('/dev/ttyS101','Mega2560'); %create object for controller
% ln -s /dev/ttyACM0 /dev/ttyS101 - CMD used to make link

minPulse = 1000e-6;
maxPulse = 2000e-6;

s = servo(a, 'D7', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse); %PWM - Coil 1
s1 = servo(a, 'D9', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse);%Pwm - COil 3
s2 = servo(a, 'D11', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse);%Pwm - COil 5
s3 = servo(a, 'D12', 'MinPulseDuration', minPulse, 'MaxPulseDuration', maxPulse);%Pwm - COil 6
%display(I_CMD)

writePosition(s,0.46);  %Set output to STOP
writePosition(s1,0.46); %Setotuptu to STOP
writePosition(s2,0.46);  %Set output to STOP
writePosition(s3,0.46); %Set output to STOP

for angle = 0:0.05:0.46
    writePosition(s, angle);
    %writePosition(s1, angle);
    current_pos = readPosition(s);
    current_pos = current_pos*180;
    fprintf('Current motor position is %d degrees\n', current_pos);
    pause(1);
end

writePosition(s, 0.46);
current_pos = readPosition(s);
fprintf('Current motor position is %d degrees\n', current_pos);
pause(1);

for angle1 = 0:0.05:0.46
    writePosition(s2, angle1);
    %writePosition(s3, angle);
    current_pos1 = readPosition(s2);
    current_pos1 = current_pos1*180;
    fprintf('Current motor position is %d degrees\n', current_pos1);
    pause(1);
end

writePosition(s2,0);
%writePosition(s3, 0.5);
%clear all;