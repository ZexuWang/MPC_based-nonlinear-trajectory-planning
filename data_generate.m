function [X,Y,varargout] = data_generate(road)

switch road
    case 'parking'
        dev = 4;
        A = 8;
        Y = 0:0.01:20;
        w = pi/20;
        X = dev+A-A*cos(w*Y);
    case 'circle'
        rho = 40;
        theta = 0:0.01:pi/2;
        X = rho*cos(theta);
        Y = rho*sin(theta);
    case 'real'
        dev = 4;
        A = 8;
        Y = 0:0.01:20;
        w = pi/20;
        X = dev+A-A*cos(w*Y);
        r = 8;
        theta = 0:0.01:pi/2;
        X_park = X(end)+r-r*cos(theta);
        Y_park = Y(end)+r*sin(theta);
        X_park = X_park(2:end);
        Y_park = Y_park(2:end);
        dx = X_park(end):0.01:(X_park(end)+4);
        dy = Y_park(end)*ones(1,length(dx));
        varargout{1} = [X_park,dx(2:end)];
        varargout{2} = [Y_park,dy(2:end)];
    case 'DLC'
        dev = 4;
        Y1 = 0:0.1:50;
%         Xin = linspace(4,8,10);
%         Yin = 50*ones(1,10);
        Y2 = 50:0.1:100;
        X1 = 4*ones(length(Y1),1);
        X2 = 8*ones(length(Y2),1);
        X = X1;
        Y = Y1';
        varargout{1} = X2;
        varargout{2} = Y2;
%         varargout{3} = X2;
%         varargout{4} = Y2';
end