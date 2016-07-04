function [ x_out ] = Trans_Period( x, x_max, x_min )
%TRANS_TO_PERIOD Summary of this function goes here
%   Detailed explanation goes here
if x_max < x_min
    tmp = x_min;
    x_min = x_max;
    x_max = tmp;
end
period = x_max - x_min;
while x > x_max
    x = x - period;
end
while x <= x_min
    x = x + period;
end
x_out = x;
end

