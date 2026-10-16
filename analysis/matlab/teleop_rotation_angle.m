function angle = teleop_rotation_angle(first, second)
% Quaternion rows use [x y z w]. Handles q/-q equivalence, excludes invalid rows.
n1 = sqrt(sum(first.^2, 2));
n2 = sqrt(sum(second.^2, 2));
valid = isfinite(n1) & isfinite(n2) & n1 > 0 & n2 > 0;
angle = nan(size(n1));
a = first(valid, :) ./ n1(valid);
b = second(valid, :) ./ n2(valid);
angle(valid) = 2 * acos(min(1, max(0, abs(sum(a .* b, 2)))));
end
