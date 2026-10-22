function teleop_figures(D, summary, directory, dpi)
% TELEOP_FIGURES Headless dashboard pages and vector/raster publication exports.
% All units are explicit. Angular velocity is in the robot base frame.
colors = [0 .447 .698; .902 .624 0; 0 .620 .451; .800 .475 .655];
trackingLabels = {'Requested', 'Reference', 'Measured'};
r = D.request_index;
state = D.time;
issue = D.reference_time;
[rt, rp] = break_trace(D.request_time, D.requested_position(r, :), D.request_episode);
[~, rv] = break_trace(D.request_time, D.requested_velocity(r, :), D.request_episode);
[fig, tiles] = page('Pose and velocity', 3, 3);
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
if summary.hardware_data
    source = 'HARDWARE';
else
    source = 'MOCK FIXTURE: not a dynamics model';
end
title(tiles, sprintf('%s | active %.2f s | pose RMSE %.2f mm', ...
                     source, summary.active_seconds, 1000 * summary.position_rmse_m));
export_page(fig, tiles, directory, 'pose_velocity', dpi);

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
% Saved FIG/PDF/PNG exports retain explicit labels, including NaN signals.
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
