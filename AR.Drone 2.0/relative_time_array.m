function [x] = relative_time_array(x,op)
%RELATIVE_TIME soutai jikan. kijun ha x(1). return x & base_time(kijun)
%   詳細説明をここに記述
% op = x(1,1);
for i = 1:numel(x(:,1))
    x(i,1) = x(i,1) - op;
end
end

