addpath(genpath('YALMIP-master'));
clear; close all;clc;

% constraints that is representing a car or a thiner road 
% final state with a higher weight to track the fianl state reference

%% Parameters definition
% model parameters
params.a_acc                = 1;                 % acceleration limit
params.a_dec                = 2;                   % decelerationlimit
params.delta_max            = 0.4;                 % maximum steering angle 
params.beta_max             = pi;                  % crabbing slip angle limit
params.beta_dot_max         = (pi/180)*(100);      % crabbing slip angle rate limit
params.l_f                  = 2;                   % distance between center of gravity and front axle
params.l_r                  = 2;                   % distance between center of gravity and rear axle
params.vehicle_width        = 2;                   % vehicle width
params.Ts                   = 0.1;                 % sampling time (both of MPC and simulated vehicle)
params.nstates              = 4;                   % number of states
params.ninputs              = 3;                   % number of inputs

% environment parameters
params.road                 = 'DLC';
params.activate_obstacles   = 0; 
params.obstacle_centers     = [10 -2; 20  0; 30 -2; 40 6]; % x and y coordinate of the 4 obstacles
params.obstacle_size        = [2 6];                    % size of the obstacles
switch params.road
    case 'real'
        [X,Y,X_park,Y_park]                   = data_generate(params.road);
    case 'DLC'
        [X,Y,X2,Y2]                   = data_generate(params.road);
    otherwise
        [X,Y]                   = data_generate(params.road);
end
params.X                    = X;
params.Y                    = Y;
params.lane_semiwidth       = 4;                        % semi-width of the lane
params.track_end_x          = X(end);                   % Tracking end X coordinate of the planner
params.track_end_y          = Y(end);                   % Tracking end Y coordinate of the planner
params.xlim                 = max(X);                   % x limit in the plot
params.ylim                 = max(Y);                   % y limit in the plot



% simulation parameters in frenet coord
params.s0                   = 0;                           % initial s coordinate
params.y0                   = params.lane_semiwidth/2;     % initial y coordinate
params.theta0               = pi/2;                        % initial relative heading angle
params.v0                   = 5;                          % initial speed
params.theta_c0             = pi/2;                        % initial heading angle of the curve
params.N_max                = 1000;                        % maximum number of simulation steps
params.vn                   = params.v0/2;                 % average speed during braking
params.vs                   = 1;                           % Stopping speed during the last stop 

%%% in this case, theta = theta_m (angle of the velocity vector)- theta_c (angle of the tangent vector of the curve)

% simulation parameters in cartesian coord %%%%% change according to
% different road
params.x_c0                 = X(1)-params.y0; %initial x coordinate
params.y_c0                 = 0;                            % initial y coordinate


% plotting parameters
params.window_size          = 10;                       % plot window size for the
%during simulation, centered at the
%current position (x,y) of the vehicle

params.plot_full            = 1;                        % 0 for plotting only the window size,


% 1 for plotting from 0 to track_end
%% Simulation environment
% initialization
sref         = zeros(params.N_max,1);
z            = zeros(params.N_max,params.nstates);             % z(k,j) denotes state j at step k-1(s and y)
z_cart       = zeros(params.N_max,3);                          % x,y coordinate in cartesian system and the heading angle
theta_c      = zeros(params.N_max,1);                          % initial value of theta_c
u            = zeros(params.N_max,params.ninputs);             % z(k,j) denotes input j at step k-1
z(1,:)       = [params.s0,params.y0,params.theta0,params.v0];  % definition of the intial state
z_cart(1,:)  = [params.x_c0,params.y_c0,params.theta0];
%theta_c(1)   = pi/2;    %%%%%% This value is changable according to different roads
comp_times   = zeros(params.N_max);                            % total computation time each sample
solve_times  = zeros(params.N_max);                            % optimization solver time each sample
i = 0;
N = 10;                                                 %predicted horizon length
m = params.N_max;
% formulating the reference of the system
switch params.road
    case 'real'
        path1 = [X;Y]';
        path2 = [X_park;Y_park]';
        [S1, ~, ~, ~, ~] = getPathProperties(path1);
        num        = S1(end)/(params.v0*params.Ts);
        n_step1    = round(num);
        [S2, ~, ~, ~, ~] = getPathProperties(path2);
        num        = S2(end)/(params.vn*params.Ts);
        n_step2    = round(num);
        n_steps    = n_step1+n_step2;
        vq1         = linspace(Y(1),Y(end),n_step1);    % this need to be the 'variable' of the path function
        centerX1    = interp1(Y,X,vq1,'pchip');      % initialize the centerline y coordinate
        centerY1    = vq1;
        path_new_1   = [centerX1;centerY1]';
        vq2         = linspace(X_park(1),X_park(end),n_step2);
        centerY2    = interp1(X_park,Y_park,vq2,'pchip');      % initialize the centerline y coordinate
        centerX2    = vq2;
        path_new_2   = [centerX2;centerY2]';
        path_new     = [path_new_1;path_new_2];
        [S, dX, dY, theta_c, C_c] = getPathProperties(path_new);
        theta_c_ref    = zeros(n_steps+N,1);
        C_c_ref        = zeros(n_steps+N,1);
        v_ref          = zeros(n_steps+N,1);
        for ii = 1:length(theta_c_ref)
            if ii<n_steps
                theta_c_ref(ii) = theta_c(ii);
                C_c_ref(ii)     = C_c(ii);
            else
                theta_c_ref(ii) = theta_c(end);
                C_c_ref(ii)     = C_c(end);
            end
        end
        for ii = 1:length(v_ref)
            if ii<n_step1+18
                v_ref(ii) = params.v0;
            else
                v_ref(ii) = 0;
            end
        end
        theta_c = theta_c_ref;
        C_c     = C_c_ref;
    case 'DLC'
%         pathX       = [X ;X2];
%         pathY       = [Y,Y2]';
%         path        = [pathX,pathY];
        num1        = Y(end)/(params.v0*params.Ts);
%         numin       = (Xin(end)-Xin(1))/(params.v0*params.Ts);
        num2        = Y2(end)/(params.v0*params.Ts);
        n_steps1    = round(num1);
%         n_stepsin   = round(numin);
        n_steps2    = round(num2)-n_steps1;
        centerX1    = linspace(X(1),X(end),n_steps1)';
%         centerXin   = linspace(Xin(1),Xin(end),n_stepsin)';
        centerX2    = linspace(X2(1),X2(end),n_steps2)';
        centerY1    = linspace(Y(1),Y(end),n_steps1)';
%         centerYin   = linspace(Yin(1),Yin(end),n_stepsin)';
        centerY2    = linspace(Y2(1),Y2(end),n_steps2)';
        centerX     = [centerX1;centerX2];
        centerY     = [centerY1;centerY2];
        path_new    = [centerX,centerY ];
        [S, dX, dY, theta_c, C_c] = getPathProperties(path_new);
        theta_c_ref    = zeros(n_steps1+n_steps2+N,1);
        C_c_ref        = zeros(n_steps1+n_steps2+N,1);
        v_ref          = zeros(n_steps1+n_steps2+N,1);
        for ii = 1:length(theta_c_ref)
            if ii<(n_steps1+n_steps2)
                v_ref(ii)       = params.v0;
                theta_c_ref(ii) = theta_c(ii);
                C_c_ref(ii)     = C_c(ii);
            else
                v_ref(ii)       = 0;
                theta_c_ref(ii) = theta_c(end);
                C_c_ref(ii)     = C_c(end);
            end
        end
        theta_c = theta_c_ref;
        C_c     = C_c_ref;
    otherwise
        path = [X;Y]';
        [S, ~, ~, ~, ~] = getPathProperties(path);
        num        = S(end)/(params.v0*params.Ts);
        n_steps    = round(num);
        vq         = linspace(Y(1),Y(end),n_steps);    % this need to be the 'variable' of the path function
        % centerX    = pchip(Y,X,vq);
        centerX    = interp1(Y,X,vq,'pchip');      % initialize the centerline y coordinate
        centerY    = vq;
        path_new   = [centerX;centerY]';
        [S, dX, dY, theta_c, C_c] = getPathProperties(path_new);
        theta_c_ref    = zeros(n_steps+2*N,1);
        C_c_ref        = zeros(n_steps+2*N,1);
        v_ref          = zeros(n_steps+2*N,1);
        for ii = 1:length(theta_c_ref)
            if ii<length(theta_c)
                theta_c_ref(ii) = theta_c(ii);
                C_c_ref(ii)         = C_c(ii);
            else
                theta_c_ref(ii) = theta_c(end);
                C_c_ref(ii)         = C_c(end);
            end
        end
        for ii = 1:length(v_ref)
            if ii<0.75*n_steps
                v_ref(ii) = params.v0;
            else
                v_ref(ii) = 0;
            end
        end
        theta_c = theta_c_ref;
        C_c     = C_c_ref;
end

% centerX, centerY are the references that the car want to track
% plot(path_new(:,1),path_new(:,2))
%%
% hold on;
% plot(theta_c,'.');
% plot(path_new(:,2));


%%

% while (z_cart(i+1,1)<centerX(end) && z_cart(i+1,2)<centerY(end))
% while (i<length(theta_c_ref)) 
    while(z(i+1,1)<S(end))
    % while (z_cart(i+1,1)>params.track_end_x)&& (i<params.N_max)
    % initialize the MPC controller
    i = i+1;                                                % MPC controller definition
    yalmip('clear');
    Q = [1,0,0,0;0,20,0,0;0,0,10,0;0,0,0,1];
    Qf= [1,0,0,0;0,20,0,0;0,0,10,0;0,0,0,1];
    R = 0.1*eye(3);
    nu = 3;
    nx = 4;
    constraints = [];
    cost = 0;
    u_MPC = sdpvar(nu,N);
    x_MPC = sdpvar(nx,N+1);
    switch params.road
        case 'DLC'
            if z(i,1)>S(n_steps1)&&z(i,1)<S(n_steps1)+params.Ts*z(i,4)
                m = i;
                z(m,2) = z(m,2)-4;
            end
        otherwise
    end

    %formulate the cost from n = 1 to N-1
    for k = 1:N-1

        % formulate the reference at i+k in the predicted horizon
        theta_c_dummy = theta_c(i);
        sref(i+k) = sref(i+k-1)+z(i,4)*params.Ts;
        Nref = 0;
        theta_ref = theta_c_dummy;                                      % reference angle difference
%         v_ref = params.v0;          % reference velocity
        x_ref = [sref(i+k);Nref;theta_ref;v_ref(i)];
       %%%%%%%%%%%%%% Try s(t+1) = s(t) + v*dt with y = 0;   check!
        
        % formulate the cost function
        cost = cost+ (x_MPC(:,k)-x_ref)'*Q*(x_MPC(:,k)-x_ref)...
            +u_MPC(:,k)'*R*u_MPC(:,k);
        constraints = [constraints, x_MPC(:,1)==z(i,:)'];
        x_MPC_dummy = car_update(x_MPC(:,k),u_MPC(:,k),params,theta_c_dummy,0);
        constraints = [constraints, x_MPC(:,k+1)==x_MPC_dummy];
        
        % fromulate the constraints 
        constraints = [constraints,...
            - params.lane_semiwidth-params.vehicle_width<=x_MPC(2,k)<=params.lane_semiwidth+params.vehicle_width];
         constraints = [constraints,...
            -params.a_dec<=u_MPC(3,k)<=params.a_acc];
        %avoid betad_max 2
%          constraints = [constraints,...
%              implies(0.0001<=x_MPC(2,k)<=0.1,0.001<=u_MPC(2,k)<=0.1)];
                 constraints = [constraints,...
            -params.beta_max<=u_MPC(2,k)<=params.beta_max];%avoid betad_max 1
%         constraints = [constraints,(pi/2)*u_MPC(1,k) + u_MPC(2,k) <= params.kappaBetaMax];
%         constraints = [constraints,(pi/2)*u_MPC(1,k) - u_MPC(2,k) <= params.kappaBetaMax];
%         constraints = [constraints,-(pi/2)*u_MPC(1,k) + u_MPC(2,k) <= params.kappaBetaMax];
%         constraints = [constraints,-(pi/2)*u_MPC(1,k) - u_MPC(2,k) <= params.kappaBetaMax];
          constraints = [constraints, -params.delta_max<= u_MPC(2,k)+params.l_f*u_MPC(1,k)  <= params.delta_max];
          constraints = [constraints, -params.delta_max<= u_MPC(2,k)-params.l_r*u_MPC(1,k)  <= params.delta_max];
    end
    % formulate the final cost
    theta_c_dummy = theta_c(i);
    sref(i+N) = sref(i+N-1)+z(i,4)*params.Ts;
    Nref = 0;
%     v_ref = params.v0;     % reference velocity
    theta_ref = theta_c_dummy;
    x_ref = [sref(i+N);Nref;theta_ref;v_ref(i)];
    cost = cost + (x_MPC(:,N)-x_ref)'*Q*(x_MPC(:,N)-x_ref);
     constraints = [constraints,...
            -params.a_dec<=u_MPC(3,N)<=params.a_acc];
    %     constraints = [constraints,-params.a_max<=u_MPC(1,N)<=params.a_max,...
    %         -params.beta_max <=u_MPC(2,N)<=params.beta_max];
    %     constraints = [constraints,x_MPC(1,N)>=0];
    %     constraints = [constraints,...
    %         1<=(x_MPC(1,N)-ob_c(1,1))^2/a^2+(x_MPC(2,N)-ob_c(1,2))^2/b^2,...
    %         1<=(x_MPC(1,N)-ob_c(2,1))^2/a^2+(x_MPC(2,N)-ob_c(2,2))^2/b^2,...
    %         1<=(x_MPC(1,N)-ob_c(3,1))^2/a^2+(x_MPC(2,N)-ob_c(3,2))^2/b^2,...
    %         1<=(x_MPC(1,N)-ob_c(4,1))^2/a^2+(x_MPC(2,N)-ob_c(4,2))^2/b^2];%ellipse
    %     constraints = [constraints,...
    %         -8<=x_MPC(2,N)<=8];% road boundary
    ops = sdpsettings('solver','fmincon','usex0',1,'verbose',0);
% ops = sdpsettings('usex0',1,'verbose',0);
    if i > 1
        assign(u_MPC,u_predict);   % solver use x0 part, use this from the second step
        assign(x_MPC,z_predict);
    end
    tic;
    diagnostic = optimize(constraints,cost,ops);
    comp_times(i) = toc;
    solve_times(i) = diagnostic.solvertime;
    
    % Use the first computed input signal as the actual input signal
    u(i,:) = value(u_MPC(:,1))';
    
    %% store the prediction values
    z_predict = value(x_MPC);
    u_predict = value(u_MPC);
    
    %% Simulating the plot
    % find the centerline of the path
    

    
    % find the updated new state of the car
    z(i+1,:) = car_update(z(i,:),u(i,:),params,theta_c(i),C_c(i));
%     if i==m-4
%         z(i+1,2) = z(i+1,2)+4;
%     end
    % coordinate transformation
    poses_f = [z(1:i+1,1),z(1:i+1,2),z(1:i+1,3)];
    poses_c = frenet2cartesian(poses_f, path_new);
    z_cart(i+1,1) = poses_c(i+1,1);
    z_cart(i+1,2) = poses_c(i+1,2);
    z_cart(i+1,3) = poses_c(i+1,3);%+u(i+1,2);
%     disp([z(i,1),z(i,2)])
%     disp([z_cart(i,1),z_cart(i,2)]);

    

    
%     [Cart_x,Cart_y] = sn2xy(z(i+1,1),z(i+1,2),path_new(:,1),path_new(:,2));
%     z_cart(i+1,1) = Cart_x;
%     z_cart(i+1,2) = Cart_y;
%     z_cart(i+1,3) = z(i+1,3);%+u(i+1,2);
    plot_environment(z_cart(1:i,:),params);
end

%%
comp_times = comp_times(comp_times>0);
solve_times = solve_times(solve_times>0);
racetime                    = i*params.Ts;                          % race time
time = 0:params.Ts:racetime;
fprintf('completed the race in %f seconds!\n',racetime);
params.plot_full            = 1;                                    % plot the full trajectory
plot_environment(z_cart(1:i,:),params);
% figure(2);
% plot(z_cart(1:i,1),z_cart(1:i,2));

fprintf('The computing time is %f seconds\n',sum(comp_times));
