"""Read Image layout without copying pixel buffers; used by observer and validator."""
import struct
from demonstration_timing.diagnostics import image_header


def image_view(raw):
    stamp,frame=image_header(raw);view=memoryview(raw)
    endian='<' if view[1]==1 else '>'
    length=struct.unpack_from(endian+'I',view,12)[0]
    offset=(16+length+3)&~3
    height,width,n=struct.unpack_from(endian+'III',view,offset);offset+=12
    if n<1 or offset+n+1>len(view):raise ValueError('invalid Image encoding string')
    encoding=bytes(view[offset:offset+n-1]).decode();offset+=n
    bigendian=bool(view[offset]);offset=(offset+1+3)&~3
    step,size=struct.unpack_from(endian+'II',view,offset);offset+=8
    if offset+size>len(view) or size!=step*height:raise ValueError('invalid Image payload length')
    return dict(stamp_ns=stamp,frame_id=frame,height=height,width=width,encoding=encoding,
                is_bigendian=bigendian,step=step),view[offset:offset+size]
