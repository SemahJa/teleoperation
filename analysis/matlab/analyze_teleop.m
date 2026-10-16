function [summary, data] = analyze_teleop(csvFile, outputDirectory, varargin)
% ANALYZE_TELEOP Analyze one arm's export_tracking CSV using base MATLAB.
% [summary,data] = analyze_teleop(csvFile, outputDirectory, ...
%     'DerivativeWindow', .08, 'MaximumGap', .05, 'MakePlots', true, 'DPI', 600)
% Raw requested/ref/measured signals are retained. Derivatives are estimates
% from velocity, not IMU readings. This function never connects to ROS/robots.
parser = inputParser;
addParameter(parser, 'DerivativeWindow', .08, @(x) isscalar(x) && isfinite(x) && x > 0);
addParameter(parser, 'MaximumGap', .05, @(x) isscalar(x) && isfinite(x) && x > 0);
addParameter(parser, 'MakePlots', true, @(x) islogical(x) && isscalar(x));
addParameter(parser, 'DPI', 600, @(x) isscalar(x) && isfinite(x) && x >= 72);
parse(parser, varargin{:});
options = parser.Results;
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end
optionsImport = detectImportOptions(csvFile, 'VariableNamingRule', 'preserve');
names = optionsImport.VariableNames;
integerColumns = {'control_cycle', 'previous_control_cycle', 'sequence', ...
                  'reference_space_generation', 'monitor_queue_drops'};
integerNames = names(endsWith(names, '_ns') | ismember(names, integerColumns));
optionsImport = setvartype(optionsImport, integerNames, 'uint64');
optionsImport = setvartype(optionsImport, {'frame_id', 'session_id', 'reason'}, 'string');
numericNames = setdiff(names, [integerNames, {'frame_id', 'session_id', 'reason'}]);
optionsImport = setvartype(optionsImport, numericNames, 'double');
optionsImport = setvaropts(optionsImport, numericNames, 'TreatAsMissing', {'nan', 'NaN'});
raw = readtable(csvFile, optionsImport);
assert(height(raw) >= 5, 'Tracking CSV needs at least five samples.');
required = {'state_time_pc_monotonic_ns', 'cartesian_reference_valid', 'requested_twist_valid', ...
            'previous_command_issue_time_pc_monotonic_ns', 'robot_torque_valid'};
assert(all(ismember(required, raw.Properties.VariableNames)), 'Unsupported tracking CSV schema.');
origin = raw.state_time_pc_monotonic_ns(1);
t = teleop_clock_seconds(raw.state_time_pc_monotonic_ns, origin);
assert(all(isfinite(t)) && all(diff(t) > 0), ...
       'State timestamps must strictly increase; export separate runs if clocks reset.');
active = logical(raw.motion_enabled) & logical(raw.cartesian_reference_valid) & ...
    ~logical(raw.homing_active) & ~logical(raw.safety_fallback);
changed = [true; raw.session_id(2:end) ~= raw.session_id(1:end - 1) | ...
           raw.reference_space_generation(2:end) ~= raw.reference_space_generation(1:end - 1) | ...
           raw.control_cycle(2:end) <= raw.control_cycle(1:end - 1) | ...
           diff(t) > options.MaximumGap | diff(active) ~= 0];
episode = cumsum(changed);
episode(~active) = 0;
% Retain unique, valid mapped commands. Never differentiate repeated held
% commands at the 1 kHz control rate as if they were new Quest measurements.
newRequest = [true; raw.sequence(2:end) ~= raw.sequence(1:end - 1) | ...
              raw.command_source_time_pc_monotonic_ns(2:end) ~= raw.command_source_time_pc_monotonic_ns(1:end - 1) | ...
              diff(episode) ~= 0];
requestIndex = find(active & logical(raw.requested_pose_valid) & newRequest);
data = struct;
data.raw = raw;
data.time = t;
data.active = active;
data.episode = episode;
data.request_index = requestIndex;
data.request_episode = episode(requestIndex);
data.request_time = teleop_clock_seconds(raw.command_source_time_pc_monotonic_ns(requestIndex), origin);
data.reference_time = teleop_clock_seconds(raw.command_issue_time_pc_monotonic_ns, origin);
data.previous_command_time = teleop_clock_seconds(raw.previous_command_issue_time_pc_monotonic_ns, origin);
for kind = {'requested', 'reference', 'measured'}
    name = kind{1};
    data.([name '_position']) = columns(raw, [name '_position_'], 'xyz');
    data.([name '_orientation']) = columns(raw, [name '_orientation_'], 'xyzw');
    data.([name '_velocity']) = [columns(raw, [name '_linear_velocity_'], 'xyz'), ...
                                 columns(raw, [name '_angular_velocity_'], 'xyz')];
end
% Compare operator request and response on their own timestamps. ZOH expresses
% the sampled requested trajectory; it does not pretend a smooth interpolator
% exists in the actual controller. No extrapolation beyond recorded commands.
requestOnState = nan(height(raw), 3);
requestQuatOnState = nan(height(raw), 4);
for id = unique(data.request_episode(:))'
    indices = requestIndex(data.request_episode == id);
    targets = find(episode == id);
    times = teleop_clock_seconds(raw.command_source_time_pc_monotonic_ns(indices), origin);
    [times, keep] = unique(times, 'stable');
    indices = indices(keep);
    if numel(times) < 2 || any(diff(times) <= 0)
        continue
    end
    requestOnState(targets, :) = interp1(times, data.requested_position(indices, :), t(targets), 'previous', NaN);
    requestQuatOnState(targets, :) = interp1(times, data.requested_orientation(indices, :), ...
                                             t(targets), 'previous', NaN);
end
data.request_position_on_state_time = requestOnState;
data.reference_position_error = data.reference_position - data.measured_position;
data.operator_position_error = requestOnState - data.measured_position;
data.reference_rotation_error = teleop_rotation_angle(data.reference_orientation, data.measured_orientation);
data.operator_rotation_error = teleop_rotation_angle(requestQuatOnState, data.measured_orientation);
% For derivatives of Quest-sampled velocity use OpenXR location-time intervals
% when provided, preserving the distinction from acquisition/transport clocks.
requestDerivativeTime = data.request_time;
for id = unique(data.request_episode(:))'
    mask = data.request_episode == id;
    indices = requestIndex(mask);
    location = raw.openxr_location_time_ns(indices);
    if ~isempty(location) && all(location > 0) && all(diff(location) > 0)
        requestDerivativeTime(mask) = data.request_time(find(mask, 1)) + ...
            teleop_clock_seconds(location, location(1));
    end
end
requestVelocity = data.requested_velocity(requestIndex, :);
requestVelocity(~logical(raw.requested_twist_valid(requestIndex)), :) = NaN;
requestBreak = [true; diff(data.request_episode) ~= 0 | ...
                diff(requestDerivativeTime) <= 0 | diff(requestDerivativeTime) > options.MaximumGap];
if isempty(requestIndex)
    requestBreak = false(0, 1);
end
data.request_derivative_episode = cumsum(requestBreak);
[data.request_acceleration, data.request_jerk, data.request_derivative_valid] = ...
    teleop_derivatives(requestDerivativeTime, requestVelocity, ...
                       data.request_derivative_episode, options.DerivativeWindow);
[data.reference_acceleration, data.reference_jerk, data.reference_derivative_valid] = ...
    teleop_derivatives(data.reference_time, data.reference_velocity, episode, options.DerivativeWindow);
[data.measured_acceleration, data.measured_jerk, data.measured_derivative_valid] = ...
    teleop_derivatives(t, data.measured_velocity, episode, options.DerivativeWindow);
% Metrics use time weights within contiguous active intervals, never idle/home
% samples (whose old reference would otherwise give misleading zero errors).
weights = [diff(t); 0];
weights(~active | [diff(episode) ~= 0; true] | weights > options.MaximumGap) = 0;
positionNorm = sqrt(sum(data.reference_position_error.^2, 2));
operatorNorm = sqrt(sum(data.operator_position_error.^2, 2));
velocityError = data.reference_velocity - data.measured_velocity;
summary = struct('schema_version', 1, 'csv', char(csvFile), 'matlab_release', version('-release'), ...
                 'samples', height(raw), 'active_seconds', sum(weights), ...
                 'active_episodes', numel(unique(episode(episode > 0))), ...
                 'hardware_data', all(logical(raw.hardware_backend)), ...
                 'position_rmse_m', weighted_rms(positionNorm, weights), ...
                 'position_peak_m', finite_max(positionNorm(active)), ...
                 'operator_position_rmse_m', weighted_rms(operatorNorm, weights), ...
                 'orientation_rmse_rad', weighted_rms(data.reference_rotation_error, weights), ...
                 'operator_orientation_rmse_rad', weighted_rms(data.operator_rotation_error, weights), ...
                 'linear_velocity_rmse_m_s', weighted_rms(sqrt(sum(velocityError(:, 1:3).^2, 2)), weights), ...
                 'angular_velocity_rmse_rad_s', weighted_rms(sqrt(sum(velocityError(:, 4:6).^2, 2)), weights), ...
                 'torque_intervention_rms_normalized', weighted_rms(raw.normalized_torque_intervention, weights), ...
                 'torque_rate_limit_time_fraction', weighted_fraction(logical(raw.torque_rate_limited), weights), ...
                 'torque_magnitude_limit_time_fraction', ...
                 weighted_fraction(logical(raw.torque_magnitude_limited), weights), ...
                 'command_age_p95_ms', percentile(raw.command_age_ms(active), .95), ...
                 'monitor_queue_drops_max', double(max(raw.monitor_queue_drops)), ...
                 'control_cycles_missing', sum(max(0, double(diff(raw.control_cycle)) - 1)), ...
                 'maximum_sample_gap_seconds', max(diff(t)), ...
                 'derivative_window_seconds', options.DerivativeWindow, ...
                 'derivative_method', 'Centered quadratic local fit of velocity; estimated acceleration and jerk', ...
                 'plot_dpi', options.DPI);
summary.velocity_lag = teleop_velocity_lag(data.request_time, requestVelocity(:, 1:3), data.request_episode, ...
                                           t, data.measured_velocity(:, 1:3), episode, .25);
gainColumns = {'lambda_translation', 'lambda_rotation', 'gain_translation', 'gain_rotation'};
summary.impedance_parameters = unique(raw(:, gainColumns), 'rows');
summary.impedance_parameters = table2struct(summary.impedance_parameters);
summary.notes = {'Requested twist is pre-low-pass mapped/blended velocity, not raw IMU data.', ...
                 'Reference twist may differ from the derivative of held pose targets.', ...
                 'Measured torque includes gravity; robot desired torque excludes gravity.', ...
                 'Previous command is a candidate for readback alignment, not an acknowledgement.', ...
                 'Derivative values depend on window/bandwidth; compare matched tasks and processing.', ...
                 'Mock data is a fixed fixture and cannot establish physical tracking quality.'};
write_json(fullfile(outputDirectory, 'summary.json'), summary);
flat = rmfield(summary, {'impedance_parameters', 'velocity_lag', 'notes'});
writetable(struct2table(flat), fullfile(outputDirectory, 'summary.csv'));
write_derivatives(outputDirectory, 'requested', data.request_time, data.request_derivative_episode, ...
                  data.request_acceleration, data.request_jerk, data.request_derivative_valid);
write_derivatives(outputDirectory, 'reference', data.reference_time, episode, ...
                  data.reference_acceleration, data.reference_jerk, data.reference_derivative_valid);
write_derivatives(outputDirectory, 'measured', t, episode, ...
                  data.measured_acceleration, data.measured_jerk, data.measured_derivative_valid);
if options.MakePlots
    teleop_figures(data, summary, outputDirectory, options.DPI);
end
save(fullfile(outputDirectory, 'analysis.mat'), 'summary', 'options');
fprintf('Analyzed %d samples, %.3f active seconds. Output: %s\n', height(raw), sum(weights), outputDirectory);
end

function values = columns(T, prefix, suffixes)
names = arrayfun(@(axis) [prefix axis], suffixes, 'UniformOutput', false);
values = T{:, names};
end

function value = weighted_rms(x, w)
keep = isfinite(x) & w > 0;
if any(keep)
    value = sqrt(sum(w(keep) .* x(keep).^2) / sum(w(keep)));
else
    value = NaN;
end
end

function value = weighted_fraction(x, w)
if sum(w) > 0
    value = sum(w .* double(x)) / sum(w);
else
    value = NaN;
end
end

function value = finite_max(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = max(x);
end
end

function value = percentile(x, p)
x = sort(x(isfinite(x)));
if isempty(x)
    value = NaN;
    return
end
index = 1 + p * (numel(x) - 1);
lo = floor(index);
hi = ceil(index);
value = x(lo) + (index - lo) * (x(hi) - x(lo));
end

function write_json(path, value)
file = fopen(path, 'w');
assert(file >= 0, 'Cannot write report JSON.');
cleanup = onCleanup(@() fclose(file)); %#ok<NASGU>
fprintf(file, '%s\n', jsonencode(value, 'PrettyPrint', true));
end

function write_derivatives(directory, name, t, episode, a, j, valid)
T = table(t, episode, valid, 'VariableNames', {'time_s', 'episode', 'estimate_valid'});
axes = {'x', 'y', 'z', 'rx', 'ry', 'rz'};
for k = 1:6
    T.(['estimated_acceleration_' axes{k}]) = a(:, k);
    T.(['estimated_jerk_' axes{k}]) = j(:, k);
end
writetable(T, fullfile(directory, [name '_derivatives.csv']));
end
