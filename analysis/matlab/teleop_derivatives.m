function [acceleration, jerk, valid] = teleop_derivatives(t, velocity, episode, windowSeconds)
% TELEOP_DERIVATIVES Offline local-quadratic derivatives of velocity.
% Uses actual sample times; never differentiates across episode boundaries.
% The centered window is noncausal and is for analysis, not robot feedback.
% No Signal Processing Toolbox required. Outputs remain NaN near boundaries,
% invalid input, rank-deficient fits or insufficient temporal support.
validateattributes(windowSeconds, {'numeric'}, {'scalar', 'finite', 'positive'});
t = t(:);
episode = episode(:);
assert(size(velocity, 1) == numel(t) && numel(episode) == numel(t));
acceleration = nan(size(velocity));
jerk = nan(size(velocity));
valid = false(size(t));
half = windowSeconds / 2;
if isempty(t)
    return
end
breaks = [true; diff(episode) ~= 0 | diff(t) <= 0 | ~isfinite(diff(t))];
starts = find(breaks);
ends = [starts(2:end) - 1; numel(t)];
for segment = 1:numel(starts)
    first = starts(segment);
    last = ends(segment);
    if episode(first) <= 0
        continue
    end
    lo = first;
    hi = first;
    for i = first:last
        if ~isfinite(t(i))
            continue
        end
        while lo < i && t(lo) < t(i) - half
            lo = lo + 1;
        end
        while hi < last && t(hi + 1) <= t(i) + half
            hi = hi + 1;
        end
        idx = lo:hi;
        good = isfinite(t(idx)) & all(isfinite(velocity(idx, :)), 2);
        idx = idx(good);
        if numel(idx) < 5 || sum(t(idx) < t(i)) < 2 || sum(t(idx) > t(i)) < 2
            continue
        end
        if t(i) - t(idx(1)) < .8 * half || t(idx(end)) - t(i) < .8 * half
            continue
        end
        u = (t(idx) - t(i)) / half;
        design = [ones(numel(idx), 1), u, u.^2];
        if rcond(design' * design) < 1e-8
            continue
        end
        coefficients = design \ velocity(idx, :);
        acceleration(i, :) = coefficients(2, :) / half;
        jerk(i, :) = 2 * coefficients(3, :) / (half^2);
        valid(i) = true;
    end
end
end
