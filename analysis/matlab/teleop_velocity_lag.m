function result = teleop_velocity_lag(requestTime, requestVelocity, requestEpisode, ...
                                      measuredTime, measuredVelocity, measuredEpisode, maximumLag)
% Positive lag means measured velocity trails the request. Descriptive only:
% a human-in-the-loop, nonlinear system need not have one constant delay.
% All candidates use the same time support. No xcorr/Signal Toolbox required.
step = .005;
candidates = 0:step:maximumLag;
numerator = zeros(size(candidates));
energyA = numerator;
energyB = numerator;
for id = unique(requestEpisode(:))'
    if id <= 0
        continue
    end
    a = requestEpisode == id & all(isfinite(requestVelocity), 2) & isfinite(requestTime);
    b = measuredEpisode == id & all(isfinite(measuredVelocity), 2) & isfinite(measuredTime);
    ta = requestTime(a);
    va = requestVelocity(a, :);
    tb = measuredTime(b);
    vb = measuredVelocity(b, :);
    if numel(ta) < 5 || numel(tb) < 5 || any(diff(ta) <= 0) || any(diff(tb) <= 0)
        continue
    end
    first = max(ta(1), tb(1));
    last = min(ta(end), tb(end) - maximumLag);
    if last - first < .5
        continue
    end
    grid = (first:step:last)';
    reference = interp1(ta, va, grid, 'linear');
    reference = reference - mean(reference, 1);
    referenceEnergy = sum(reference(:).^2);
    for k = 1:numel(candidates)
        measured = interp1(tb, vb, grid + candidates(k), 'linear');
        measured = measured - mean(measured, 1);
        numerator(k) = numerator(k) + sum(reference(:) .* measured(:));
        energyA(k) = energyA(k) + referenceEnergy;
        energyB(k) = energyB(k) + sum(measured(:).^2);
    end
end
correlation = numerator ./ sqrt(energyA .* energyB);
correlation(energyA < 1e-10 | energyB < 1e-10) = NaN;
result = struct('seconds', NaN, 'correlation', NaN, 'at_search_boundary', false, ...
                'maximum_search_seconds', maximumLag, 'grid_seconds', step);
if any(isfinite(correlation))
    [score, index] = max(correlation, [], 'omitnan');
    result.seconds = candidates(index);
    result.correlation = score;
    result.at_search_boundary = index == numel(candidates);
end
end
