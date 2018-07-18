function plot_environment(z,params,varargin)

%% parsing the parameters structure
vehicle_length      = params.l_f + params.l_r;
vehicle_width       = params.vehicle_width;
window_size         = params.window_size;
nstates             = params.nstates;
xlim                = params.xlim;
ylim                = params.ylim;
X                   = params.X;
Y                   = params.Y;
fs                  = 16;
semi                = params.lane_semiwidth;
if nstates < 3
    error('The number of states cannot be less than 3!\n');
end
activate_obstacles  = params.activate_obstacles;
obstacle_centers    = params.obstacle_centers;
obstacle_size       = params.obstacle_size;
lane_semiwidth      = params.lane_semiwidth;
plot_full           = params.plot_full;

%% parsing the arguments
current_pos         = z(end,:);
if length(varargin) == 2
    z_horizon           = varargin{1};
    u_horizon           = varargin{2};
else
    z_horizon = [];
    u_horizon = [];
end
%% car description
car_x = [current_pos(1)+vehicle_width/2 current_pos(1)+...
    vehicle_width/2 current_pos(1)-vehicle_width/2 current_pos(1)-vehicle_width/2];
car_y = [current_pos(2)-vehicle_length/2 current_pos(2)+...
    vehicle_length/2 current_pos(2)+vehicle_length/2 current_pos(2)-vehicle_length/2];

%% window size limits
if plot_full == 1
    windowlimits_x = [0, lane_semiwidth+xlim];
    windowlimits_y = [0 ,ylim+lane_semiwidth];
elseif plot_full == 0
    if ~isempty(window_size) % if there are window_size limits
        windowlimits_x = [current_pos(1)-window_size current_pos(1)+window_size];
        windowlimits_y = [-lane_semiwidth lane_semiwidth];
    else % if there are no window_size limits then plot everything
        error('The window size cannot be an empty vector if plot_full is 0\n')
    end
else
    error('The flag plot_full must be set to 0 or 1\n')
end
windowlimits = [windowlimits_x,windowlimits_y];


%% define obstacles
if activate_obstacles == 1 % define obstacles
    obs_x = zeros(size(obstacle_centers,1),4); % edges x coordinates
    obs_y = zeros(size(obstacle_centers,1),4); % edges y coordinates
    for i = 1:size(obstacle_centers,1)
        xc = obstacle_centers(i,1); % obstacle center x coordinate
        xsize = obstacle_size(1)/2; % obstacle x size
        obs_x(i,:) = [xc-xsize, xc+xsize, xc+xsize, xc-xsize];
        
        yc = obstacle_centers(i,2); % obstacle center y coordinate
        ysize = obstacle_size(2)/2; % obstacle y size
        obs_y(i,:) = [yc+ysize, yc+ysize, yc-ysize, yc-ysize];
    end
    
elseif activate_obstacles == 0
    % do nothing
else
    error('The flag activate_obstacles must be set to 0 or 1\n')
end


figure(1);
%% plot simulation environment
% subplot(4,4,1:10); % plot road and vehicle
theta = linspace(0,pi/2,1000);
hold off; plot(0,0); hold on;



% plot the car (x,y) and handle the pose of the car
car_handle=patch(car_x,car_y,'white','EdgeColor','black');
rotate(car_handle,[0 0 1],rad2deg(current_pos(3)+pi/2),[current_pos(1),current_pos(2), 0]);



% plot the lane limits

switch params.road
    
    case 'real'
        se   = 2.4;
        [X,Y,X_park,Y_park] = data_generate('real');
        X_tot = [X,X_park];
        Y_tot = [Y,Y_park];
        [~, ~, ~, theta_c, ~] = getPathProperties([X;Y]');
        [~, ~, ~, theta_c_p, ~] = getPathProperties([X_park;Y_park]');
        dx = semi*sin(theta_c);
        dx_p = se*sin(theta_c_p);
        dy = semi*cos(theta_c);
        dy_p = se*cos(theta_c_p);
        lane_xin = X-dx';
        lane_xin_p = X_park-dx_p';
        lane_yin = Y+dy';
        lane_yin_p = Y_park+dy_p';
        lane_xin_tot = [lane_xin,lane_xin_p];
        lane_yin_tot = [lane_yin,lane_yin_p];
        hold on;
        plot(lane_xin_tot,lane_yin_tot);
        lane_xout = X+dx';
        lane_xout_p = X_park+dx_p';
        lane_yout = Y-dy';
        lane_yout_p = Y_park-dy_p';
        lane_xout_tot = [lane_xout,lane_xout_p];
        lane_yout_tot = [lane_yout,lane_yout_p];
        plot(lane_xout_tot,lane_yout_tot);
        % plot the centerline
        plot(X_tot,Y_tot,'--k','linewidth',1);
        % plot the garage
        plot([X_park(end)-4,X_park(end)-4],[Y_park(end)-2,Y_park(end)+2],'b--');
        plot([X_park(end),X_park(end)],[Y_park(end)-2,Y_park(end)+2],'b--');
        plot([X_park(end)-4,X_park(end)],[Y_park(end)-2,Y_park(end)-2],'b--');
        plot([X_park(end)-4,X_park(end)],[Y_park(end)+2,Y_park(end)+2],'b--');
        plot(z(:,1),z(:,2),'g');
    case 'DLC'
        [X,Y,X2,Y2] = data_generate('DLC');
        X_tot = [X;X2];
        Y_tot = [Y;Y2'];
        % plot the centerline
        plot(X_tot,Y_tot,'--k','linewidth',1);
    otherwise
        [~, ~, ~, theta_c, ~] = getPathProperties([X;Y]');
        dx = semi*sin(theta_c);
        dy = semi*cos(theta_c);
        lane_xin = X-dx';
        lane_yin = Y+dy';
        plot(lane_xin,lane_yin);
        lane_xout = X+dx';
        lane_yout = Y-dy';
        plot(lane_xout,lane_yout);
        % plot the centerline
        plot(X,Y,'--k','linewidth',3)
        plot(z(:,1),z(:,2),'g');
        switch params.road
            case 'parking'
                lane_xend = X(end);
                plot([lane_xend-lane_semiwidth,lane_xend+lane_semiwidth],[Y(end),Y(end)], '-r', 'linewidth',10)
                axis(windowlimits)
                xlabel('x','FontSize',fs);
                ylabel('y','FontSize',fs);
                drawnow;
            case 'circle'
                lane_xend = X(end);
                plot([lane_xend,lane_xend],[Y(end)-lane_semiwidth,Y(end)+lane_semiwidth], '-r', 'linewidth',10)
                axis(windowlimits)
                xlabel('x','FontSize',fs);
                ylabel('y','FontSize',fs);
                drawnow;
            case 'real'
                lane_xend = X(end);
                plot([lane_xend,lane_xend],[Y(end)-lane_semiwidth,Y(end)+lane_semiwidth], '-r', 'linewidth',10)
                axis(windowlimits)
                xlabel('x','FontSize',fs);
                ylabel('y','FontSize',fs);
                drawnow;
            case 'DLC'
                
        end
end




% % % % % % % plot the obstacles
% % % % % % if activate_obstacles == 1
% % % % % %     for i = 1:size(obstacle_centers,1)
% % % % % %         patch(obs_x(i,:),obs_y(i,:),'black','EdgeColor','black');
% % % % % %     end
% % % % % % end


% plot the car path from the beggining of time
plot(z(:,1),z(:,2),'g');
switch params.road
    case 'DLC'
        axis equal;
    otherwise
end


% % plot predicted vehicle states (x,y) if given
% if ~isempty(z_horizon)
%     plot(z_horizon(:,1),z_horizon(:,2),'-om');
% end



% plot the "end" of the track (when x==track_end)

%% plot the heading, velocity and the vehicle inputs (if z/u_horizon vectors are given)
% if ~isempty(z_horizon) && ~isempty(u_horizon)
%     %% plot vehicle heading
%     subplot(4,4,13);
%     hold off; plot(0,0); hold on;
%     plot(z_horizon(:,1),z_horizon(:,4),'-om')
%     ylim([-pi/3 pi/3]);
%     xlim([min(z_horizon(:,1)) max(z_horizon(:,1))]);
%     xlabel('x');
%     ylabel('\psi');
%     drawnow;
%
%     %% plot vehicle velocity
%     subplot(4,4,14);
%     hold off; plot(0,0); hold on;
%     plot(z_horizon(:,1),z_horizon(:,3),'-om')
%     ylim([0 20])
%     xlim([min(z_horizon(:,1)) max(z_horizon(:,1))]);
%     xlabel('x');
%     ylabel('v');
%     drawnow;
%
%     %% plot vehicle acceleration
%     subplot(4,4,15);
%     hold off; plot(0,0); hold on;
%     plot(z_horizon(1:end-1,1),u_horizon(:,1),'-om')
%     ylim([-2 2])
%     xlim([min(z_horizon(:,1)) max(z_horizon(:,1))]);
%     xlabel('x');
%     ylabel('a');
%     drawnow;
%
%     %% plot vehicle \beta
%     subplot(4,4,16);
%     hold off; plot(0,0); hold on;
%     plot(z_horizon(1:end-1,1),u_horizon(:,2),'-om')
%     ylim([-pi/4 pi/4])
%     xlim([min(z_horizon(:,1)) max(z_horizon(:,1))]);
%     xlabel('x');
%     ylabel('\beta');
%     drawnow;
%
% else
%     % do nothing
% end
end