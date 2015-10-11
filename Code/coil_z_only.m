%Model of Magnetix Fields
N = 510 %number of coils
u_o =(4*3.14*10^(-7))
I_1 = 17 %Max current through coil in A
I_2 = 17
R =0.08 %radius of coil in meters
s = linspace(0,10)
b_z=0:99
b_z_2=0:99
b_z_total=0:99
z = 0;
z_2 = 0.1;

for k=1:100
z=z+0.001
z_2 = z_2-0.001
b_z(k) = (u_o*N*I_1*(R^2))/((2*((R^2)+(z^2)))^1.5)
b_z_2(k) = (u_o*N*I_1*(R^2))/((2*((R^2)+(z_2^2)))^1.5)
b_z_total(k) = b_z(k)+b_z_2(k)
%db_z = (-3*u_o*N*I*(R^2)*z)/(((R^2)+(z^2))^2.5)\
db_z_1(k) = (u_o*N*I_1*(R^2)*((-3*z)))/(((R^2)+(z^2))^(2.5))
db_z_2(k) = (u_o*N*I_2*(R^2)*((-3*z_2)))/(((R^2)+(z_2^2))^(2.5))
db_z_total(k) = db_z_1(k)+db_z_2(k)
end
%plot(s,b_z,s,b_z_2,s,b_z_total)
plot(s,db_z_1,s,db_z_2,s,db_z_total)
ylabel('T - Telsa')
xlabel('distance - cm')