function [S, dX, dY, theta_c, C_c] = getPathProperties(path)
% Description: returns path properties relevant for curvilinear
% transformations

% input: path in cartesian coordinates, rows are X,Y pairs
% output: 
% S - curvilinear abscissa
% dX, dY - per point diff in X and Y
% theta_c - tangent
% C_c - curvature

% path_X and path_Y should be column vectors

path_X = path(:,1);
path_Y = path(:,2);

%dX = [diff(path_X); path_X(end)-path_X(end-1)];
%dY = [diff(path_Y); path_Y(end)-path_Y(end-1)];
dX = [path_X(2)-path_X(1); diff(path_X)];
dY = [path_Y(2)-path_Y(1); diff(path_Y)];

S = cumsum(sqrt(dX.^2 + dY.^2));
theta_c = atan2(dY, dX);% + pi/2;
%C_c = diff(theta_c)./diff(S); C_c = [C_c; C_c(end)];
C_c = diff(theta_c)./diff(S); C_c = [C_c(1); C_c];