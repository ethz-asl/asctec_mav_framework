function x = simulateRefmodelNthOrder(wpts, t_wpts, v_max, N, poles, x_init)


dt_sim = 0.001;
t_sim = 0:dt_sim:t_wpts(end);
wpts_sim = interp1(t_wpts, wpts', t_sim);

n = length(t_sim);

x = zeros(N,n);
u = zeros(1,n);

if(nargin < 6)
    x(:,1) = zeros(N, 1);
end 


A = diag(ones(1,N-1),1);
B = [zeros(N-1,1) ; -1];
K = place(A, B, poles)

% alternative pole placement, only valid for this model
lA = zeros(N, N);
lB = zeros(N, 1);

for i=0:(N-1)
    lA(:,i+1) = poles.^i ;
    lB(i+1) = poles(i+1)^N;
end

K_=K;
K = (lA\lB)'

K-K_

wp_tmp = wpts_sim(:,1);


for i=1:n-1        
	v_tmp = (wpts_sim(i+1) - wp_tmp) / dt_sim;
    
    if(v_tmp > v_max)
        wp_tmp = wp_tmp + dt_sim*v_max;
    elseif(v_tmp < -v_max)
        wp_tmp = wp_tmp - dt_sim*v_max;
    else
        wp_tmp = wpts_sim(i+1);
    end
    
    x_c = [wp_tmp];
    N_c = length(x_c);

    u(i) = -K*(x(:,i) - [x_c; zeros(N-N_c,1)]);
    x(:,i+1) = x(:,i) + (A * x(:,i) + B*u(:,i))*dt_sim; 
    
% %     err = wpts_sim2(:,i+1)-p(:,i);
%     err = wp_tmp - p(:,i);
%     
%     
%     l = [10 35 50 24];
% %     l = [5 10 10 4];
%     l=[12 49 78 40];
%     
% 
%     l=-al;
%     
%     p(:,i+1) = p(:,i) + dt_sim*v(:,i);
%     v(:,i+1) = v(:,i) + dt_sim*a(:,i);
%     a(:,i+1) = a(:,i) + dt_sim*w(:,i);
%     w(:,i+1) = w(:,i) + dt_sim*w_d(:,i);
%     
%     w_d(:,i+1) = l(1)*(err) - l(2)*v(:,i) - l(3)*a(:,i) - l(4)*w(:,i);
    
    
%     v(:,i+1) = limit(v(:,i+1), v_max);
end

t = t_sim;

figure(31)
clf
subplot(N+1,1,1)
hold on
plot(t_sim, x(1,:))
plot(t_wpts, wpts, '--')
grid on 

for i=2:N
    subplot(N+1, 1, i)
    plot(t_sim, x(i,:))
    grid on
end

subplot(N+1, 1, i+1)
plot(t_sim, -u)
grid on
