syms dt real

x_old = sym('x_old', [16 1]);
x_old = sym(x_old, 'real');

x_new = sym('x_new', [16 1]);
x_new = sym(x_new, 'real');

acc = sym('acc', [3 1]);
acc = sym(acc, 'real');

om = sym('om', [3 1]);
om = sym(om, 'real');

g = [0 0 9.81]';

% attitude
x_new(7:10) = (eye(4) + dt/2*quatskew_sym([0; om-x_old(11:13)]))*x_old(7:10);

% position
x_new(1:3) = x_old(1:3) + x_old(4:6)*dt;

% velocity
x_new(4:6) = x_old(4:6) + (quat2rot(x_old(7:10))*(acc-x_old(14:16)) - g)*dt;

x_new(11:end) = x_old(11:end);