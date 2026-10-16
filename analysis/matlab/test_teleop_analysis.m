function tests = test_teleop_analysis
tests = functiontests(localfunctions);
end

function testPolynomialDerivativesAndIrregularTimes(testCase)
t = (0:.002:1)';
t = t + .0001 * sin(13 * t);
v = [3 * t.^2 + 2 * t + 1, -t.^2];
episode = ones(size(t));
[a, j, valid] = teleop_derivatives(t, v, episode, .08);
verifyGreaterThan(testCase, sum(valid), 400);
verifyEqual(testCase, a(valid, :), [6 * t(valid) + 2, -2 * t(valid)], 'AbsTol', 1e-9);
verifyEqual(testCase, j(valid, :), repmat([6 -2], sum(valid), 1), 'AbsTol', 1e-7);
end

function testDoesNotBridgeEpisodeDiscontinuity(testCase)
t = (0:.002:1)';
v = zeros(numel(t), 1);
v(t >= .5) = 100;
episode = ones(size(t));
episode(t >= .5) = 2;
[a, j, valid] = teleop_derivatives(t, v, episode, .08);
verifyLessThan(testCase, max(abs(a(valid))), 1e-7);
verifyLessThan(testCase, max(abs(j(valid))), 1e-5);
verifyFalse(testCase, any(valid(abs(t - .5) < .02)));
end

function testUint64TimePrecisionAndPreOriginTimes(testCase)
origin = uint64(9007199254740992) + uint64(1);
clock = [origin - uint64(1000000); origin; origin + uint64(1000000)];
verifyEqual(testCase, teleop_clock_seconds(clock, origin), [-.001; 0; .001], 'AbsTol', 1e-15);
end

function testQuaternionSigns(testCase)
q = [0 0 sin(.1) cos(.1)];
identity = [0 0 0 1];
verifyEqual(testCase, teleop_rotation_angle(q, identity), .2, 'AbsTol', 1e-12);
verifyEqual(testCase, teleop_rotation_angle(-q, identity), .2, 'AbsTol', 1e-12);
end

function testEmptyAndInsufficientDerivativeInput(testCase)
[a, j, valid] = teleop_derivatives(zeros(0, 1), zeros(0, 6), zeros(0, 1), .08);
verifyEmpty(testCase, a);
verifyEmpty(testCase, j);
verifyEmpty(testCase, valid);
[~, ~, valid] = teleop_derivatives((0:3)', ones(4, 1), ones(4, 1), 1);
verifyFalse(testCase, any(valid));
end

function testKnownVelocityDelay(testCase)
t = (0:.005:3)';
signal = @(t) [sin(9 * t) + .2 * sin(17 * t), cos(5 * t), sin(3 * t)];
result = teleop_velocity_lag(t, signal(t), ones(size(t)), t, signal(t - .06), ones(size(t)), .2);
verifyEqual(testCase, result.seconds, .06, 'AbsTol', .005);
verifyGreaterThan(testCase, result.correlation, .99);
end

function testReportRoundtrip(testCase)
root = tempname;
mkdir(root);
cleanup = onCleanup(@() rmdir(root, 's')); %#ok<NASGU>
t = (0:.002:2)';
n = numel(t);
origin = uint64(9007199254740992) + uint64(1);
clock = origin + uint64(round(t * 1e9));
active = t >= .1 & t <= 1.8;
T = table(t, 'VariableNames', {'time_s'});
for name = {'state_time_pc_monotonic_ns', 'control_time_pc_monotonic_ns', ...
            'command_issue_time_pc_monotonic_ns', 'command_source_time_pc_monotonic_ns', ...
            'command_receipt_time_pc_monotonic_ns', 'openxr_location_time_ns', ...
            'previous_command_issue_time_pc_monotonic_ns'}
    T.(name{1}) = clock;
end
T.frame_id = repmat("panda/base", n, 1);
T.session_id = repmat("test", n, 1);
T.reason = repmat("active", n, 1);
T.control_cycle = uint64((1:n)');
T.previous_control_cycle = uint64((0:n - 1)');
T.sequence = T.control_cycle;
T.reference_space_generation = ones(n, 1, 'uint64');
for name = {'hardware_backend', 'requested_twist_valid', 'requested_pose_valid', 'cartesian_reference_valid', ...
            'motion_enabled', 'robot_torque_valid', 'gravity_valid', 'previous_command_valid', 'solver_succeeded'}
    T.(name{1}) = ones(n, 1);
end
for name = {'motion_enabled', 'requested_twist_valid', 'requested_pose_valid', 'cartesian_reference_valid'}
    T.(name{1}) = double(active);
end
for name = {'homing_active', 'safety_fallback', 'cbf_enabled', 'torque_rate_limited', 'torque_magnitude_limited', ...
            'monitor_queue_drops', 'normalized_torque_intervention', 'command_age_ms'}
    T.(name{1}) = zeros(n, 1);
end
T.period_seconds = .002 * ones(n, 1);
T.lambda_translation = 50 * ones(n, 1);
T.lambda_rotation = 40 * ones(n, 1);
T.gain_translation = 20 * ones(n, 1);
T.gain_rotation = 2.5 * ones(n, 1);
for kind = {'requested', 'reference', 'measured'}
    for axis = 'xyz'
        T.([kind{1} '_position_' axis]) = .01 * sin(5 * t);
        T.([kind{1} '_linear_velocity_' axis]) = .05 * cos(5 * t);
        T.([kind{1} '_angular_velocity_' axis]) = zeros(n, 1);
    end
    for axis = 'xyz'
        T.([kind{1} '_orientation_' axis]) = zeros(n, 1);
    end
    T.([kind{1} '_orientation_w']) = ones(n, 1);
end
for name = {'nominal_torque', 'commanded_torque', 'robot_desired_torque', 'measured_torque', ...
            'previous_commanded_torque', 'gravity_torque'}
    for k = 1:7
        T.([name{1} '_' num2str(k)]) = zeros(n, 1);
    end
end
csv = fullfile(root, 'fixture.csv');
writetable(T, csv);
[summary, data] = analyze_teleop(csv, fullfile(root, 'report'), 'MakePlots', false);
verifyEqual(testCase, summary.position_rmse_m, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, summary.linear_velocity_rmse_m_s, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, data.raw.state_time_pc_monotonic_ns(1), origin);
verifyTrue(testCase, isfile(fullfile(root, 'report', 'summary.json')));
verifyTrue(testCase, any(data.measured_derivative_valid));
% Also exercise headless tiled plotting/export at a small test raster size.
analyze_teleop(csv, fullfile(root, 'figures'), 'DPI', 72);
verifyTrue(testCase, isfile(fullfile(root, 'figures', 'pose_velocity.pdf')));
verifyTrue(testCase, isfile(fullfile(root, 'figures', 'four_torques.png')));
% Every saved panel must identify all of its curves without generic dataN
% labels, including seven-joint residuals and single-series diagnostics.
pages = {'pose_velocity', 'four_torques', 'diagnostics', ...
         'estimated_linear_derivatives', 'estimated_angular_derivatives'};
counts = [12 8 6 6 6];
for k = 1:numel(pages)
    fig = openfig(fullfile(root, 'figures', [pages{k} '.fig']), 'new', 'invisible');
    cleanupFigure = onCleanup(@() close(fig));
    axesHandles = findall(fig, 'Type', 'axes');
    verifyNumElements(testCase, axesHandles, counts(k));
    for a = 1:numel(axesHandles)
        ax = axesHandles(a);
        verifyNotEmpty(testCase, ax.Legend);
        if isempty(ax.Legend)
            continue
        end
        verifyEqual(testCase, ax.Legend.Visible, 'on');
        names = cellstr(string(ax.Legend.String));
        verifyNumElements(testCase, names, numel(findall(ax, 'Type', 'line')));
        verifyTrue(testCase, all(~cellfun(@isempty, names)));
        verifyTrue(testCase, all(cellfun(@isempty, regexp(names, '^data\d+$', 'once'))));
    end
    clear cleanupFigure;
end
% No engaged request and no hardware torque readback must still keep the
% requested/reference/measured and four-torque legend entries distinguishable.
T.motion_enabled(:) = 0;
T.cartesian_reference_valid(:) = 0;
T.requested_pose_valid(:) = 0;
T.requested_twist_valid(:) = 0;
T.robot_torque_valid(:) = 0;
T.gravity_valid(:) = 0;
T.hardware_backend(:) = 0;
T.reason(:) = "position_hold";
for name = {'robot_desired_torque', 'measured_torque', 'gravity_torque'}
    for k = 1:7
        T.([name{1} '_' num2str(k)])(:) = NaN;
    end
end
csv = fullfile(root, 'inactive.csv');
writetable(T, csv);
analyze_teleop(csv, fullfile(root, 'inactive'), 'DPI', 72);
for page = {'pose_velocity', 'four_torques'}
    fig = openfig(fullfile(root, 'inactive', [page{1} '.fig']), 'new', 'invisible');
    cleanupFigure = onCleanup(@() close(fig));
    axesHandles = findall(fig, 'Type', 'axes');
    for a = 1:numel(axesHandles)
        ax = axesHandles(a);
        names = cellstr(string(ax.Legend.String));
        if startsWith(ax.Title.String, 'Joint ')
            verifyEqual(testCase, names(:), ...
                        {'Nominal'; 'After safety'; 'Robot desired (no gravity)'; 'Measured (includes gravity)'});
        elseif startsWith(ax.Title.String, 'Position ') || contains(ax.Title.String, ' velocity ')
            verifyEqual(testCase, names(:), {'Requested'; 'Reference'; 'Measured'});
        end
    end
    clear cleanupFigure;
end
end
