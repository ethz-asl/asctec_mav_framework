function y = limit(x, x_max, x_min)

if nargin == 2
    x_min = -x_max;
end

    y = min([x_max max([x_min x], [],2)], [], 2);
end