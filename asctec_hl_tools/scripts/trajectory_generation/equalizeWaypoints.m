function [wpts t] = equalizeWaypoints(wpts, v_des, dt)

wpts_d = diff(wpts, 1, 2);
wpts_d_abs = sqrt(sum(wpts_d.^2, 1));
dt_v = wpts_d_abs/v_des;

% interpolate to constant dt
t_tmp = [0 cumsum(dt_v)];
t = 0:dt:t_tmp(end);
wpts = interp1(t_tmp, wpts', t)';