function seconds = teleop_clock_seconds(clock, origin)
% Subtract uint64 nanosecond clocks before conversion; support times < origin.
clock = uint64(clock);
origin = uint64(origin);
seconds = nan(size(clock));
after = clock >= origin;
seconds(after) = double(clock(after) - origin) * 1e-9;
seconds(~after) = -double(origin - clock(~after)) * 1e-9;
seconds(clock == 0) = NaN;
end
