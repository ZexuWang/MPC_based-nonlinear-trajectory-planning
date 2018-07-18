function x = car_update(z_curr,u_curr,params,theta_c,C_c)
a           = u_curr(3);
beta        = u_curr(2);
kappa       = u_curr(1);%1/params.lane_radius;%
s           = z_curr(1);
y           = z_curr(2);
theta       = z_curr(3); %theta_m %%%%%%% theta = theta_m-theta_c
v           = z_curr(4);
dt          = params.Ts;
cc          = C_c;
% vehicle kinematic equations
% z_new(1) = s + (v*cos(theta+beta)/(1-cc*y))*dt;
% z_new(2) = y + v*sin(theta+beta)*dt;
z_new(1) = s + (v*cos(theta-theta_c+beta)/(1-cc*y))*dt;
z_new(2) = y + v*sin(theta-theta_c+beta)*dt;
z_new(3) = theta + kappa*v*dt;
z_new(4) = v+a*dt;
x = [z_new(1);z_new(2);z_new(3);z_new(4);];
end