"""Delivery diagnostics, not capture-clock synchronization certification."""
from bisect import bisect_left
import statistics


def summary(values):
    values=list(values)
    if not values: return {'count':0}
    return {'count':len(values),'min':min(values),'median':statistics.median(values),'max':max(values)}


def nearest_differences(left,right,tolerance_ns):
    """Each left stamp -> nearest right, signed right-left; bounded, reusable right."""
    right=sorted(right); out=[]
    for t in left:
        j=bisect_left(right,t)
        candidates=right[max(0,j-1):j+1]
        if candidates:
            dt=min(candidates,key=lambda x:abs(x-t))-t
            if abs(dt)<=tolerance_ns: out.append(dt)
    return out


def stream_metrics(rows, expected_fps=None):
    stamps=[r['header_ros_ns'] for r in rows]
    deltas=[b-a for a,b in zip(stamps,stamps[1:])]
    expected=round(1e9/expected_fps) if expected_fps else None
    missing=sum(max(0,round(d/expected)-1) for d in deltas if d>0) if expected else None
    counters=[r.get('frame_number') for r in rows]
    steps=[b-a for a,b in zip(counters,counters[1:]) if a is not None and b is not None]
    hardware=[r['metadata'].get('hw_timestamp') for r in rows if 'metadata' in r]
    hardware_steps=[b-a for a,b in zip(hardware,hardware[1:]) if a is not None and b is not None]
    native=[r['native_frame_ns'] for r in rows if 'native_frame_ns' in r]
    return {'hardware_timestamp_decreases':sum(x<0 for x in hardware_steps),
            'native_timestamp_non_monotonic':sum(b<=a for a,b in zip(native,native[1:])),
            'native_frame_gap_ns':summary(b-a for a,b in zip(native,native[1:])),
            'samples':len(rows),'header_gap_ns':summary(deltas),
            'non_monotonic':sum(x<0 for x in deltas),'duplicated_stamps':sum(x==0 for x in deltas),
            'estimated_missing_from_header_gaps':missing,'expected_fps':expected_fps,
            'frame_numbers_available':sum(x is not None for x in counters),
            'counter_gaps':sum(x-1 for x in steps if x>1),
            'counter_duplicates':sum(x==0 for x in steps),'counter_resets_or_reorder':sum(x<0 for x in steps),
            'arrival_ros_minus_header_ns':summary(r['arrival_ros_ns']-r['header_ros_ns'] for r in rows),
            'warning':'Arrival-minus-header includes clock error and transport/scheduling; not capture latency alone.'}


class SegmentTracker:
    def __init__(self, max_gap_ns=1_000_000_000):
        self.last={}; self.count={}; self.max_gap_ns=max_gap_ns

    def observe(self, stream, stamp, identity, valid=True):
        old=self.last.get(stream); reason=None
        if old:
            if identity!=old[1]: reason='identity_or_domain_change'
            elif not valid or not old[2]: reason='invalid_boundary'
            elif stamp==old[0]: reason='duplicate'
            elif stamp<old[0]: reason='timestamp_reset_or_reorder'
            elif stamp-old[0]>self.max_gap_ns: reason='gap'
        if reason: self.count[stream]=self.count.get(stream,0)+1
        self.last[stream]=(stamp,identity,valid)
        return str(self.count.get(stream,0)),reason
