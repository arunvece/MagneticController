function [error_gain] = Controller(error_xy)
%PID Controller used for tracking program
%   Detailed explanation goes here
Kp = 1; %Prop term
Ki = 1; %Int term
Kd = 0; %Der term
%error_mem = 0;
%dt = 0.01 %sample time
error_x = error_xy(1,1);
error_y = error_xy(1,2);

 %Prop(i+1) = Error(i+1);% error of proportional term
 %Der(i+1)  = (Error(i+1) - Error(i))/dt; % derivative of the error
 %Der = (error_x - error_x)/dt;
 %Int(i+1)  = (Error(i+1) + Error(i))*dt/2; % integration of the error
 %I(i+1)    = sum(Int); % the sum of the integration of the error
    
 %PID(i+1)  = Kp*Prop(i) + Ki*I(i+1)+ Kd*Der(i); % the three PID terms
    
 error_gain(1,1)  = Kp*error_x;
 display (error_gain(1,1));
 error_gain(1,2)  = Kp*error_y;
 display (error_gain(1,2));