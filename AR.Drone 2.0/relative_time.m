function [x] = relative_time(x)
%RELATIVE_TIME soutai jikan. kijun ha x(1). return x & base_time(kijun)
%   詳細説明をここに記述
op = x.Time(1);
for i = 1:size(x.Time,1)
    x.Time(i) = x.Time(i) - op;
end
end

