function [x] = cut_data_timebase_array(x,min_time,max_time,op_time)
%CUT_DATA_TIMEBASE min~max made wo kiritoru. return x & basetime(kijun)
%   詳細説明をここに記述
i=1;
while x(i,1) < min_time
    i = i + 1;
end
x(1:i-1,:) = [];

i=numel(x(:,1));
while x(i,1) > max_time
    i = i - 1;
end
x(i+1:numel(x(:,1)),:) = [];
x = relative_time_array(x,op_time);
end

