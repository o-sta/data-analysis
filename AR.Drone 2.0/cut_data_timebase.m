function [x] = cut_data_timebase(x,min_time,max_time)
%CUT_DATA_TIMEBASE min~max made wo kiritoru. return x & basetime(kijun)
%   詳細説明をここに記述
i=1;
while x.Time(i) < min_time
    i = i + 1;
end
x = delsample(x,'Index', (1:i-1));

i=size(x.Time,1);
while x.Time(i) > max_time
    i = i - 1;
end
x = delsample(x,'Index', (i+1:size(x.Time,1)));
x = relative_time(x);


end

