function [p v a w t] = simulateRefmodel3rdOrder(wpts, t_wpts, om0, v_max, p0, v0, simtime)

if nargin == 3
    v_max = ones(3,1) * inf;
    p0 = zeros(3,1);
    v0 = zeros(3,1);
    simtime = t_wpts(end);
elseif nargin == 4
    p0 = zeros(3,1);
    v0 = zeros(3,1);
    simtime = t_wpts(end);
elseif nargin == 5
    v0 = zeros(3,1);
    simtime = t_wpts(end);
elseif nargin == 6
    simtime = t_wpts(end);
end

dt_sim = 0.001;
t_sim = 0:dt_sim:t_wpts(end);
wpts_sim = interp1(t_wpts, wpts', t_sim)';




t_sim = [t_sim (t_sim(end)+dt_sim):dt_sim:simtime];
wpts_sim = [wpts_sim repmat(wpts_sim(:,end), 1, length(t_sim)-size(wpts_sim,2))];

n = length(t_sim);

% upsample waypts with zero order hold
wpts_sim2 = zeros(size(wpts_sim));
wp_cnt = 1;
t_wpts2 = [t_wpts t_wpts(end)];
for i=1:n
    wpts_sim2(:,i) =  wpts(:,wp_cnt);
    if(abs(t_wpts2(wp_cnt+1)-t_sim(i))<dt_sim*0.5 && length(t_wpts)>wp_cnt)
        wp_cnt = wp_cnt+1;
    end    
end



% figure(32)
% clf
% hold on
% plot(t_sim, wpts_sim, '--')
% plot(t_sim, wpts_sim2)





p = zeros(3, n);
v = zeros(3, n);
a = zeros(3, n);
w = zeros(3, n);
w_d = zeros(3, n);

% initial condition
p(:,1) = p0;
v(:,1) = v0;

    global al;

wp_tmp = wpts_sim2(:,1);


for i=1:n-1        
	v_tmp = (wpts_sim2(:,i+1) - wp_tmp) / dt_sim;
    for k=1:length(v_tmp)
        if(v_tmp(k) > v_max(k))
            wp_tmp(k) = wp_tmp(k) + dt_sim*v_max(k);
        elseif(v_tmp(k) < -v_max(k))
            wp_tmp(k) = wp_tmp(k) - dt_sim*v_max(k);
        else
            wp_tmp(k) = wpts_sim2(k,i+1);
        end
    end
    
%     err = wpts_sim2(:,i+1)-p(:,i);
    err = wp_tmp - p(:,i);
    om_v = 10;
    
%     w(:,i+1) = om0.^3.*(err) - 3*(om0.^2.*v(:,i)) - 3*(om0.*a(:,i));
%     
%     p(:,i+1) = p(:,i) + dt_sim*v(:,i);
%     v(:,i+1) = v(:,i) + dt_sim*a(:,i);
%     a(:,i+1) = a(:,i) + dt_sim*w(:,i);
    
    
    l = [10 35 50 24];
%     l = [5 10 10 4];
    l=[12 49 78 40];
    

    l=-al;
    
    p(:,i+1) = p(:,i) + dt_sim*v(:,i);
    v(:,i+1) = v(:,i) + dt_sim*a(:,i);
    a(:,i+1) = a(:,i) + dt_sim*w(:,i);
    w(:,i+1) = w(:,i) + dt_sim*w_d(:,i);
    
    w_d(:,i+1) = l(1)*(err) - l(2)*v(:,i) - l(3)*a(:,i) - l(4)*w(:,i);
    
    
%     v(:,i+1) = limit(v(:,i+1), v_max);
end

t = t_sim;

figure(30)
clf
hold on
grid on 
axis equal
plot3(wpts(1,:), wpts(2,:), wpts(3,:), '.r')
% plot3(p(1,:), p(2,:), p(3,:), '.r')
xlabel('x')
ylabel('y')
zlabel('z')

figure(31)
clf
subplot 511
hold on
plot(t_sim, p)
plot(t_wpts, wpts, '--')
grid on 

subplot 512
plot(t_sim, v)
grid on 

subplot 513
plot(t_sim, a)
grid on

subplot 514
plot(t_sim, w)
grid on

subplot 515
plot(t_sim, w_d)
grid on

end