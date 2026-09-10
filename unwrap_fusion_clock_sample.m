function [time_value, previous, offset] = unwrap_fusion_clock_sample( ...
        time_value, previous, offset, format)
%UNWRAP_FUSION_CLOCK_SAMPLE Stateful midnight expansion for file readers.

if ~strcmpi(format, 'hms') || ~isfinite(time_value)
    return;
end
candidate = time_value + offset;
if isfinite(previous) && candidate < previous - 12 * 3600
    offset = offset + 24 * 3600;
    candidate = time_value + offset;
end
time_value = candidate;
previous = candidate;
end
