function poses_c = frenet2cartesian(poses_f, path)
% Description: coordiante transformation from curvilinear frenet frame to
% cartesian

% input:
% poses_f - matrix of poses in the curviliear frame (each row is a pose)
% path    - path in cartesian coordinates (each row is a point [X,Y] ) 

% output:
% poses_c - matrix of poses in cartesian coordinates (each row is a pose)

% note: this function assumes a path of equidistant points of a
% sufficiently fine discretization

[N,~] = size(poses_f);

% compute S
[path_S, dX, dY, theta_c, ~] = getPathProperties(path);
% [path_S, dX, dY] = getSfromCartesianPath(path(:,1), path(:,2));

% compute tangent (as a function of S)
% theta_c = atan2(dY, dX);% + pi/2; % normaliza to [-pi pi] ?

% debug fig
%figure; plot(path_S, theta_c,'r*')
%xlabel('S')
%ylabel('theta_c')

% interpolate to compute the X, Y and theta_c of the T-frame (inria paper notation)
theta_c_poses = interp1(path_S,theta_c,poses_f(:,1), 'linear', 'extrap');
X_T_poses = interp1(path_S,path(:,1),poses_f(:,1), 'linear', 'extrap'); % if X or Y < 0 , set to 0
Y_T_poses = interp1(path_S,path(:,2),poses_f(:,1), 'linear', 'extrap');
X_poses = zeros(1,N);
Y_poses = zeros(1,N);
psi_poses = zeros(1,N);

% transform poses to cartesian
for i = 1:N
    eyi = poses_f(i,2);
    psi_fi = poses_f(i,3);
    theta_ci = theta_c_poses(i);
    XTi = X_T_poses(i);
    YTi = Y_T_poses(i);    
    Xi = XTi - eyi*sin(theta_ci);
    Yi = YTi + eyi*cos(theta_ci);
    X_poses(i) = Xi;
    Y_poses(i) = Yi;
    psi_poses(i) = psi_fi;%+theta_ci-pi/2;
end
poses_c= [X_poses' Y_poses' psi_poses'];
