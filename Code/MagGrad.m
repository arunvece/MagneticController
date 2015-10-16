function MagGrad [G_x,G_y,I_CMD_1_x,I_CMD_2_x,I_CMD_2_y,I_CMD_1_y] = BGrad(x,y,x_err,y_err,I_MES_1_x,I_MES_2_x,I_MES_2_y,I_MES_1_y)
%#codegen
%Model of Magnetix Fields
N = 510; %number of coils
M=1;
W = 0.001;
u_o =(4*3.14*10^(-7));
uk = 0.47;
FN = 9.81*W
FF = (uk*FN)/W;
%I_1 = 17 %Max current through coil in A
%I_2 = 17
R =0.08; %radius of coil in meters
R_1=0.01;
%s = linspace(0,10)
%b_z=0:99
%b_z_2=0:99
%b_z_total=0:99
%z = 0;
%z_2 = 0.1;

%for k=1:100

V = (4/3)*(3.14)*(R_1^3);
x=x+0.0001;
y=y+0.0001;

x1=x;
y1=y;

x2=0.1-x;
y2=0.1-y;
%z_2 = z_2-0.001
%b_x = (u_o*N*I_MES*(R^2))/((2*((R^2)+(x^2)))^1.5);
%b_z_2(k) = (u_o*N*I_1*(R^2))/((2*((R^2)+(z_2^2)))^1.5)
%b_z_total(k) = b_z(k)+b_z_2(k)
%db_z = (-3*u_o*N*I*(R^2)*z)/(((R^2)+(z^2))^2.5)\

%db_z_1(k) = (u_o*N*I_1*(R^2)*((-3*z)))/(2*(((R^2)+(z^2))^(2.5)));

db_x_1 = M*V*(u_o*N*I_MES_1_x*(R^2)*((-3*x1)))/(2*((R^2)+(x1^2))^(2.5));
db_y_1 = M*V*(u_o*N*I_MES_1_y*(R^2)*((-3*y1)))/(2*((R^2)+(y1^2))^(2.5));

db_x_2 = M*V*(u_o*N*I_MES_2_x*(R^2)*((-3*x2)))/(2*((R^2)+(x2^2))^(2.5));
db_y_2 = M*V*(u_o*N*I_MES_2_y*(R^2)*((-3*y2)))/(2*((R^2)+(y2^2))^(2.5));

%G_x = ((db_x_1+db_x_2)/W)*x_err;
%G_y = ((db_y_1+db_y_2)/W)*y_err;

G_x = ((db_x_1+db_x_2)/W);
G_y = ((db_y_1+db_y_2)/W);

%I_CMD_1 = x_err*(db_x_1*((R^2)+(x1^2))^(2.5))/(-3*x1*u_o*N*(R^2));
%I_CMD_2 = y_err*(db_x_2*((R^2)+(x2^2))^(2.5))/(-3*x2*u_o*N*(R^2));

I_CMD_1_x = x_err*(-17);
I_CMD_2_x = x_err*(-17);
I_CMD_1_y = y_err*(-17);
I_CMD_2_y = y_err*(-17);


%I_CMD = -5*I_set;

%I_CMD = x_err*17
%db_z_2(k) = (u_o*N*I_2*(R^2)*((-3*z_2)))/(((R^2)+(z_2^2))^(2.5))
%db_z_total(k) = db_z_1(k)+db_z_2(k)
%end
%plot(s,b_z,s,b_z_2,s,b_z_total)
%plot(s,db_z_1,s,db_z_2,s,db_z_total)
%ylabel('T - Telsa') 
%xlabel('distance - cm')
%I_CMD = 0.01;
