%% trajectory parameters
d_yaw = 2*pi*0;
v_des = 1;
dt = 0.01;

%% heli parameters
om0 = 2*[1 1 1]';
zeta = [1 1 1]';
v_max = [1 1 1]'*v_des;
a_max = [1 1 1]'*2;

%% generate ellipse

a = 2; 
b = 1; 

h = 1;
offset = [0 0 0.5 + h/2]';

x = -a:0.01:a;
y = sqrt(b^2 - x.^2 * b^2 / a^2);

wp = [x fliplr(x(1:end-1)); ...
      y -fliplr(y(1:end-1)); ...
      zeros(1, length(x)*2 - 1)];
  
rpy = [ 0 -asin(h/(b*2)) 0]';
R = quat2rot(rpy2quat(rpy));

R_90 = quat2rot(rpy2quat([0 0 pi/2]'));
% R = R_90*R;
  
wp = R*wp + offset(:,ones(1,size(wp,2)));  
  


[wp t_wp] = equalizeWaypoints(wp, v_des, dt);

% shift by x such that we start right on the ellipse
wp(1,:) = wp(1,:) + a;



[p v a t] = simulateRefmodel(wp, t_wp, om0, zeta, v_max, a_max, wp(:,1), [0 0 0]', 24);

% uncomment this to use wps smoothed by the reference model 
% wp = p(:,1:10:end);
% t_wp = t(1:10:end);

if(d_yaw == 0)
    yaw = ones(1,size(wp, 2))*0;
else
    yaw = 0:d_yaw/(size(wp,2)-1):d_yaw;
end




generateTrajectoryFile([wp; yaw; v_max(1:2, ones(1, size(wp, 2))); [0 diff(t_wp)]], 'ellipse.txt');
% generateTrajectoryFile([wp; yaw; v_max(1:2, ones(1, size(wp, 2))); [0 diff(t_wp)]], 'ellipse.txt', 5); % wait at first wp to make sure we reach it
 

% generateTrajectoryFile([wp; yaw; 10*(ones(2, size(wp, 2))); [0 diff(t)]], 'ellipse.txt');


% simulateRefmodel(wp, t, om0, zeta, v_max, a_max, wp(:,1), [0 0 0]', 24);
