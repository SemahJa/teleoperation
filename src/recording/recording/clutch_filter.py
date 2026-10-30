"""Conservative offline selection; no ROS or hardware publishers."""
import numpy as np


def latched_axis(values, valid, identities, times, engage=.4, neutral=.2, max_gap_ns=100_000_000, initial_neutral_required=True):
    """Return -1/0/+1 with a fresh neutral required after unavailable/reset input."""
    result=np.zeros(len(values),dtype=np.int8);direction=0;need_neutral=True
    for i,(value,ok) in enumerate(zip(values,valid)):
        reset=(i==0 or identities[i]!=identities[i-1] or not 0<times[i]-times[i-1]<=max_gap_ns)
        if reset:direction=0;need_neutral=initial_neutral_required if i==0 else True
        if not ok or not np.isfinite(value):direction=0;need_neutral=True;continue
        if abs(value)<=neutral:direction=0;need_neutral=False
        elif not need_neutral:
            if value>=engage or (direction==1 and value>neutral):direction=1
            elif value<=-engage or (direction==-1 and value<-neutral):direction=-1
            else:direction=0
        result[i]=direction
    return result


class TemporalIndex:
    """Split arrival-order discontinuities; reject overlapping clock/identity epochs."""
    def __init__(self,rows,time_key='t',identity_key='identity'):
        self.groups=[];current=[];last=None
        for row in rows:
            if row[time_key]<=0:continue
            if last is not None and (row[time_key]<=last[time_key] or row.get(identity_key)!=last.get(identity_key)):
                if current:self.groups.append((np.array([r[time_key] for r in current],dtype=np.int64),current))
                current=[]
            row=dict(row,epoch=len(self.groups));current.append(row);last=row
        if current:self.groups.append((np.array([r[time_key] for r in current],dtype=np.int64),current))

    def pick(self,t,tolerance_ns,causal=False):
        candidates=[]
        for times,rows in self.groups:
            i=int(np.searchsorted(times,t,side='right'))
            choices=[i-1] if causal else [i-1,i]
            choices=[j for j in choices if 0<=j<len(times) and abs(int(times[j])-t)<=tolerance_ns]
            if choices:
                j=min(choices,key=lambda j:abs(int(times[j])-t));candidates.append(rows[j])
        return candidates[0] if len(candidates)==1 else None


def context_mask(times,active,pre_ns,post_ns):
    result=np.zeros(len(times),dtype=bool)
    for t in np.asarray(times)[active]:
        a=np.searchsorted(times,t-pre_ns,side='left');b=np.searchsorted(times,t+post_ns,side='right')
        result[a:b]=True
    return result


def continuous_segments(keep,keys,min_frames=2):
    segments=[];start=None
    for i,ok in enumerate(keep):
        if start is not None and (not ok or keys[i]!=keys[i-1]):
            if i-start>=min_frames:segments.append((start,i))
            start=None
        if ok and start is None:start=i
    if start is not None and len(keep)-start>=min_frames:segments.append((start,len(keep)))
    return segments


def recorded_arm_gate(motion_enabled, trigger_value, trigger_available, release=.25):
    """Recorded motion gate carries controller hysteresis across bag start.

    A bag boundary is not a controller reset. Reject a released/unavailable
    current trigger even if an older in-tolerance control sample is enabled.
    """
    return bool(motion_enabled and trigger_available and np.isfinite(trigger_value) and trigger_value>release)
