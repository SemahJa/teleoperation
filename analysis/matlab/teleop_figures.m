function teleop_figures(D, summary, directory, dpi)
% TELEOP_FIGURES Headless dashboard pages and vector/raster publication exports.
% All units are explicit. Quaternion error is sign invariant; angular velocity
% is in the robot base frame. All four raw torque stages remain distinguishable.
colors = [0 .447 .698; .902 .624 0; 0 .620 .451; .800 .475 .655];
trackingLabels = {'Requested', 'Reference', 'Measured'};
torqueLabels = {'Nominal', 'After safety', 'Robot desired (no gravity)', 'Measured (includes gravity)'};
jointLabels = arrayfun(@(j) sprintf('Joint %d', j), 1:7, 'UniformOutput', false);
jointColors = lines(7);
raw = D.raw;
r = D.request_index;
state = D.time;
issue = D.reference_time;
[rt, rp] = break_trace(D.request_time, D.requested_position(r, :), D.request_episode);
[~, rv] = break_trace(D.request_time, D.requested_velocity(r, :), D.request_episode);
[fig, tiles] = page('Pose and velocity', 4, 3);
for k = 1:3
    ax = nexttile(tiles);
    hold(ax, 'on');
    plot(ax, rt, rp(:, k), '--', 'Color', colors(1, :));
    plot(ax, issue, D.reference_position(:, k), ':', 'Color', colors(2, :));
    plot(ax, state, D.measured_position(:, k), '-', 'Color', colors(3, :));
    label(ax, ['Position ' axis_name(k)], 'm', trackingLabels);
end
for k = 1:6
    ax = nexttile(tiles);
    hold(ax, 'on');
    plot(ax, rt, rv(:, k), '--', 'Color', colors(1, :));
    plot(ax, issue, D.reference_velocity(:, k), ':', 'Color', colors(2, :));
    plot(ax, state, D.measured_velocity(:, k), '-', 'Color', colors(3, :));
    if k <= 3
        unit = 'm/s';
        part = 'Linear';
    else
        unit = 'rad/s';
        part = 'Angular';
    end
    label(ax, [part ' velocity ' axis_name(mod(k - 1, 3) + 1)], unit, trackingLabels);
end
ax = nexttile(tiles);
hold(ax, 'on');
plot(ax, state, D.reference_rotation_error * 180 / pi, 'Color', colors(2, :));
plot(ax, state, D.operator_rotation_error * 180 / pi, '--', 'Color', colors(1, :));
label(ax, 'Orientation error', 'deg', {'Reference error', 'Requested error'});
ax = nexttile(tiles);
plot(ax, state, raw.command_age_ms);
label(ax, 'Command age at evaluation', 'ms', {'Command age'});
ax = nexttile(tiles);
plot(ax, state, raw.normalized_torque_intervention);
label(ax, 'Safety intervention', 'normalized norm', {'Normalized torque intervention'});
if summary.hardware_data
    source = 'HARDWARE';
else
    source = 'MOCK FIXTURE: not a dynamics model';
end
title(tiles, sprintf('%s | active %.2f s | pose RMSE %.2f mm', ...
                     source, summary.active_seconds, 1000 * summary.position_rmse_m));
export_page(fig, tiles, directory, 'pose_velocity', dpi);

[fig, tiles] = page('Four torque stages', 2, 4);
for joint = 1:7
    ax = nexttile(tiles);
    hold(ax, 'on');
    plot(ax, issue, raw.(['nominal_torque_' num2str(joint)]), '--', 'Color', colors(1, :));
    plot(ax, issue, raw.(['commanded_torque_' num2str(joint)]), ':', 'Color', colors(2, :));
    plot(ax, state, raw.(['robot_desired_torque_' num2str(joint)]), '-', 'Color', colors(3, :));
    plot(ax, state, raw.(['measured_torque_' num2str(joint)]), '-', 'Color', colors(4, :));
    label(ax, sprintf('Joint %d', joint), 'Nm', torqueLabels);
end
ax = nexttile(tiles);
hold(ax, 'on');
stairs(ax, state, raw.robot_torque_valid);
stairs(ax, state, raw.previous_command_valid);
label(ax, 'Readback availability', 'valid', {'Robot torque valid', 'Previous command valid'});
title(tiles, 'Four raw torque stages: different clocks and gravity conventions; measured is link-side torque');
export_page(fig, tiles, directory, 'four_torques', dpi);

[fig, tiles] = page('Diagnostic context', 3, 2);
ax = nexttile(tiles);
hold(ax, 'on');
stairs(ax, state, raw.motion_enabled);
stairs(ax, state, raw.reason == "position_hold");
stairs(ax, state, raw.homing_active);
stairs(ax, state, raw.safety_fallback);
label(ax, 'Controller modes', 'active', {'Teleoperation', 'Position hold', 'Homing', 'Safety fallback'});
ax = nexttile(tiles);
hold(ax, 'on');
stairs(ax, state, raw.torque_rate_limited);
stairs(ax, state, raw.torque_magnitude_limited);
label(ax, 'Nominal actuator pressure', 'flag', {'Torque rate limited', 'Torque magnitude limited'});
ax = nexttile(tiles);
plot(ax, state, raw.monitor_queue_drops);
label(ax, 'Monitoring queue drops', 'cumulative count', {'Queue drops'});
ax = nexttile(tiles);
plot(ax, state(2:end), 1000 * diff(state));
label(ax, 'Recorded state intervals', 'ms', {'State sample interval'});
ax = nexttile(tiles);
hold(ax, 'on');
for joint = 1:7
    previous = raw.(['previous_commanded_torque_' num2str(joint)]);
    residual = raw.(['robot_desired_torque_' num2str(joint)]) - previous;
    residual(~logical(raw.previous_command_valid) | ~logical(raw.robot_torque_valid)) = NaN;
    plot(ax, state, residual, 'Color', jointColors(joint, :));
end
label(ax, 'Robot desired minus previous output: candidate alignment', 'Nm', jointLabels);
ax = nexttile(tiles);
hold(ax, 'on');
for joint = 1:7
    residual = raw.(['measured_torque_' num2str(joint)]) - raw.(['gravity_torque_' num2str(joint)]) - ...
        raw.(['robot_desired_torque_' num2str(joint)]);
    residual(~logical(raw.gravity_valid) | ~logical(raw.robot_torque_valid)) = NaN;
    plot(ax, state, residual, 'Color', jointColors(joint, :));
end
label(ax, 'Measured - model gravity - robot desired: approximate residual', 'Nm', jointLabels);
title(tiles, ['Supporting diagnostics: residuals include model/contact/dynamic effects; ' ...
              'no exact acknowledgement assumed']);
export_page(fig, tiles, directory, 'diagnostics', dpi);

for angular = 0:1
    if angular
        suffix = 'angular';
        unitA = 'rad/s^2';
        unitJ = 'rad/s^3';
    else
        suffix = 'linear';
        unitA = 'm/s^2';
        unitJ = 'm/s^3';
    end
    [fig, tiles] = page(['Estimated ' suffix ' derivatives'], 2, 3);
    [ta, aa] = break_trace(D.request_time, D.request_acceleration, D.request_episode);
    [tj, jj] = break_trace(D.request_time, D.request_jerk, D.request_episode);
    for row = 1:2
        for k = 1:3
            axis = k + 3 * angular;
            ax = nexttile(tiles);
            hold(ax, 'on');
            if row == 1
                plot(ax, ta, aa(:, axis), '--', 'Color', colors(1, :));
                plot(ax, issue, D.reference_acceleration(:, axis), ':', 'Color', colors(2, :));
                plot(ax, state, D.measured_acceleration(:, axis), '-', 'Color', colors(3, :));
                label(ax, ['Estimated acceleration ' axis_name(k)], unitA, trackingLabels);
            else
                plot(ax, tj, jj(:, axis), '--', 'Color', colors(1, :));
                plot(ax, issue, D.reference_jerk(:, axis), ':', 'Color', colors(2, :));
                plot(ax, state, D.measured_jerk(:, axis), '-', 'Color', colors(3, :));
                label(ax, ['Estimated jerk ' axis_name(k)], unitJ, trackingLabels);
            end
        end
    end
    title(tiles, sprintf(['Supporting estimates only | centered velocity fit | ' ...
                          'window %.0f ms | invalid boundaries omitted'], ...
                         1000 * summary.derivative_window_seconds));
    export_page(fig, tiles, directory, ['estimated_' suffix '_derivatives'], dpi);
end
end

function [fig, tiles] = page(name, rows, cols)
fig = figure('Name', name, 'Visible', 'off', 'Color', 'w', 'Units', 'inches', 'Position', [0 0 14 9]);
tiles = tiledlayout(fig, rows, cols, 'TileSpacing', 'compact', 'Padding', 'compact');
end

function label(ax, name, unit, seriesLabels)
title(ax, name, 'Interpreter', 'none');
xlabel(ax, 'Time [s]');
ylabel(ax, unit, 'Interpreter', 'none');
grid(ax, 'on');
ax.FontSize = 10;
% Every tile has its own explicit legend, including single-series diagnostics
% and unavailable (NaN) signals. Saved FIG/PDF/PNG exports retain these labels.
lgd = legend(ax, seriesLabels, 'Location', 'best', 'Interpreter', 'none', 'FontSize', 7);
lgd.AutoUpdate = 'off';
lgd.Visible = 'on';
end

function value = axis_name(i)
axes = 'xyz';
value = axes(i);
end

function export_page(fig, tiles, directory, name, dpi)
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
exportgraphics(tiles, fullfile(directory, [name '.pdf']), 'ContentType', 'vector');
exportgraphics(tiles, fullfile(directory, [name '.png']), 'Resolution', dpi);
savefig(fig, fullfile(directory, [name '.fig']));
end

function [time, values] = break_trace(t, y, episode)
% NaNs break lines at engagement boundaries even after command de-duplication.
if isempty(t)
    % Keep an explicitly unavailable curve/legend entry even in recordings
    % containing only position hold, with no engaged requested trajectory.
    time = NaN;
    values = nan(1, size(y, 2));
    return
end
n = numel(t);
boundary = [false; diff(episode) ~= 0];
positions = (1:n)' + cumsum(boundary);
time = nan(n + sum(boundary), 1);
values = nan(n + sum(boundary), size(y, 2));
time(positions) = t;
values(positions, :) = y;
end
