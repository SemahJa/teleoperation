"""Independent sampled trajectory processors for mapped pose and velocity.

The four-pole filter has a C3 step response in continuous time. Sampled outputs
are evaluated at Quest location timestamps. Interpolation is causal and uses a
fixed transition duration; it never extrapolates a future Quest sample.
"""

import math


class TrajectoryProcessor:
    def __init__(self, filter_mode="none", interpolation="none", cutoff_hz=10.0,
                 duration_ms=40.0):
        if filter_mode not in ("none", "low_pass", "four_pole"):
            raise ValueError("filter must be none, low_pass or four_pole")
        if interpolation not in ("none", "ptp_polynomial", "spline"):
            raise ValueError("interpolation must be none, ptp_polynomial or spline")
        if not math.isfinite(cutoff_hz) or cutoff_hz <= 0:
            raise ValueError("filter cutoff must be positive")
        if not math.isfinite(duration_ms) or duration_ms <= 0:
            raise ValueError("interpolation duration must be positive")
        self.filter_mode = filter_mode
        self.interpolation = interpolation
        self.cutoff_hz = cutoff_hz
        self.duration_ns = int(duration_ms * 1e6)
        self.reset()

    def reset(self):
        self.stages = None
        self.last_time = None
        self.segment_start = None
        self.segment_target = None
        self.segment_time = None
        self.output = None
        self.pending = None

    def update(self, values, timestamp_ns):
        values = tuple(float(x) for x in values)
        if not all(math.isfinite(x) for x in values):
            raise ValueError("nonfinite trajectory sample")
        if self.last_time is None:
            self.stages = [values for _ in range(4)]
            self.output = values
            self.segment_start = values
            self.segment_target = values
            self.segment_time = timestamp_ns
            self.last_time = timestamp_ns
            return values
        dt_ns = timestamp_ns - self.last_time
        if dt_ns <= 0:
            raise ValueError("trajectory timestamps must increase")
        dt = dt_ns * 1e-9
        if self.filter_mode != "none":
            x = 2 * math.pi * self.cutoff_hz * dt
            decay = math.exp(-x)
            old = self.stages[:]
            poles = 1 if self.filter_mode == "low_pass" else 4
            for index in range(poles):
                self.stages[index] = tuple(
                    sample + decay * sum(
                        x**power / math.factorial(power) *
                        (old[index - power][axis] - sample)
                        for power in range(index + 1)
                    )
                    for axis, sample in enumerate(values)
                )
            values = self.stages[poles - 1]
        if self.interpolation == "none":
            self.output = values
        else:
            # Coalesce incoming waypoints while a segment runs. Completed knots
            # have zero derivatives, preserving C3 (septic) or C2 (quintic).
            self.pending = values
            u = min(1.0, max(0.0, (timestamp_ns - self.segment_time) / self.duration_ns))
            if self.interpolation == "ptp_polynomial":
                blend = u**4 * (35 - 84*u + 70*u*u - 20*u**3)
            else:
                # Quintic zero-end-slope spline segment (C2 at completed knots).
                blend = u**3 * (10 - 15*u + 6*u*u)
            self.output = tuple(a + blend * (b - a) for a, b in
                                zip(self.segment_start, self.segment_target))
            if u >= 1.0 or self.segment_start == self.segment_target:
                self.segment_start = self.output
                self.segment_target = self.pending
                self.segment_time = timestamp_ns
        self.last_time = timestamp_ns
        return self.output
