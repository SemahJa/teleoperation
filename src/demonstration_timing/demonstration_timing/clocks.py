"""Explicit, bounded clock maps. Integer origins avoid epoch-float cancellation."""
from dataclasses import dataclass, asdict
import math


class ClockError(ValueError):
    pass


@dataclass(frozen=True)
class ClockModel:
    source: str
    reference: str
    segment: str
    source_origin_ns: int
    reference_origin_ns: int
    rate: float
    uncertainty_ns: int
    valid_from_ns: int
    valid_to_ns: int
    method: str

    def __post_init__(self):
        if not self.source or not self.reference or not isinstance(self.segment,str):
            raise ClockError('explicit clock identities and string segment required')
        if not math.isfinite(self.rate) or self.rate<=0:
            raise ClockError('clock rate must be finite and positive')
        if type(self.uncertainty_ns) is not int or self.uncertainty_ns<0:
            raise ClockError('model uncertainty must be known nonnegative integer ns')
        for value in (self.source_origin_ns,self.reference_origin_ns,self.valid_from_ns,self.valid_to_ns):
            if type(value) is not int: raise ClockError('model origins and interval must be integer ns')
        if not self.valid_from_ns<=self.source_origin_ns<=self.valid_to_ns:
            raise ClockError('model origin outside observed interval')

    def convert(self, timestamp_ns, clock_id, segment):
        if clock_id != self.source or segment != self.segment:
            raise ClockError('clock or segment mismatch')
        if not self.valid_from_ns <= timestamp_ns <= self.valid_to_ns:
            raise ClockError('extrapolation refused')
        if not math.isfinite(self.rate) or self.rate <= 0 or self.uncertainty_ns < 0:
            raise ClockError('invalid model')
        return self.reference_origin_ns + round((timestamp_ns-self.source_origin_ns)*self.rate)


def fit_clock(pairs, source, reference, segment, measurement_uncertainty_ns, max_residual_ns=2_000_000):
    """Pairs MUST measure simultaneous clocks, never device capture vs callback arrival.

    Returned uncertainty is supplied measurement uncertainty plus max fit residual.
    It is an empirical in-window envelope, not an absolute statistical guarantee.
    """
    if len(pairs) < 3 or measurement_uncertainty_ns is None or measurement_uncertainty_ns < 0:
        raise ClockError('need >=3 simultaneous pairs and known measurement uncertainty')
    if any(b[0] <= a[0] or b[1] <= a[1] for a,b in zip(pairs,pairs[1:])):
        raise ClockError('clock reset/non-monotonic anchors; split segments')
    x0,y0=pairs[0]
    xs=[x-x0 for x,y in pairs]; ys=[y-y0 for x,y in pairs]
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    denom=sum((x-mx)**2 for x in xs)
    if denom == 0: raise ClockError('unobservable drift')
    rate=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/denom
    origin=y0+round(my-rate*mx)
    residual=max(abs(origin+round(x*rate)-(y0+y)) for x,y in zip(xs,ys))
    if residual > max_residual_ns: raise ClockError('discontinuity or excessive residual')
    return ClockModel(source,reference,segment,x0,origin,rate,
                      math.ceil(residual+measurement_uncertainty_ns),pairs[0][0],pairs[-1][0],
                      'simultaneous_pairs_affine_empirical_envelope')


class HostClockBridge:
    """Bracketed ROS reads against local CLOCK_MONOTONIC; no capture-time inference."""
    def __init__(self, discontinuity_ns=2_000_000):
        self.segment=0; self.last=None; self.discontinuity_ns=discontinuity_ns

    def observe(self, before_ns, ros_ns, after_ns):
        if after_ns < before_ns: raise ClockError('invalid bracket')
        mid=(before_ns+after_ns)//2
        offset=mid-ros_ns
        reset=self.last is not None and (ros_ns <= self.last[0] or mid <= self.last[1]
                  or abs(offset-self.last[2]) > self.discontinuity_ns+(after_ns-before_ns))
        if reset: self.segment+=1
        self.last=(ros_ns,mid,offset)
        return {'ros_ns':ros_ns,'monotonic_ns':mid,'bracket_ns':after_ns-before_ns,
                'uncertainty_ns':(after_ns-before_ns+1)//2,'offset_monotonic_minus_ros_ns':offset,
                'segment':str(self.segment),'discontinuity':reset}

    @staticmethod
    def models(anchors, source, reference, max_residual_ns=2_000_000):
        models=[]
        for segment in sorted({a['segment'] for a in anchors}):
            group=[a for a in anchors if a['segment']==segment]
            if len(group)>=3:
                models.append(fit_clock([(a['ros_ns'],a['monotonic_ns']) for a in group],
                    source,reference,segment,max(a['uncertainty_ns'] for a in group),max_residual_ns))
        return models


def map_record(record, models, reference):
    """No fallback from source stamp to callback reception."""
    if type(record.get('timestamp_ns')) is not int or record['timestamp_ns'] < 0:
        raise ClockError('timestamp must be nonnegative integer nanoseconds')
    if record.get('uncertainty_ns') is not None and (type(record['uncertainty_ns']) is not int or record['uncertainty_ns'] < 0):
        raise ClockError('uncertainty must be nonnegative integer nanoseconds or null')
    if record['clock_id']==reference:
        return dict(record, common_ns=record['timestamp_ns'], clock_mapping_uncertainty_ns=0)
    candidates=[]
    for model in models:
        if model.reference != reference: continue
        try:
            t=model.convert(record['timestamp_ns'],record['clock_id'],record['clock_segment'])
            candidates.append((t,model.uncertainty_ns))
        except ClockError: pass
    if len(candidates)!=1: raise ClockError('missing or ambiguous measured clock relationship')
    t,u=candidates[0]
    return dict(record,common_ns=t,clock_mapping_uncertainty_ns=u)
