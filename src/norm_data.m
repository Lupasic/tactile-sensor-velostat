function outputArg1 = norm_data(a)
%NORM_DATA Summary of this function goes here
%   Detailed explanation goes here
range = max(a) - min(a);
 outputArg1 = (a - min(a)) / range;
end

