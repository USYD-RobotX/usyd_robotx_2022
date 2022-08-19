"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class os_compass_t(object):
    __slots__ = ["utime", "rph", "T", "depth", "p_volts", "p_meas", "p_gage", "p_o", "Mxyz", "Gxyz"]

    def __init__(self):
        self.utime = 0
        self.rph = [ 0.0 for dim0 in range(3) ]
        self.T = 0.0
        self.depth = 0.0
        self.p_volts = 0.0
        self.p_meas = 0.0
        self.p_gage = 0.0
        self.p_o = 0.0
        self.Mxyz = [ 0.0 for dim0 in range(3) ]
        self.Gxyz = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(os_compass_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.utime))
        buf.write(struct.pack('>3d', *self.rph[:3]))
        buf.write(struct.pack(">dddddd", self.T, self.depth, self.p_volts, self.p_meas, self.p_gage, self.p_o))
        buf.write(struct.pack('>3d', *self.Mxyz[:3]))
        buf.write(struct.pack('>3d', *self.Gxyz[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != os_compass_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return os_compass_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = os_compass_t()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.rph = struct.unpack('>3d', buf.read(24))
        self.T, self.depth, self.p_volts, self.p_meas, self.p_gage, self.p_o = struct.unpack(">dddddd", buf.read(48))
        self.Mxyz = struct.unpack('>3d', buf.read(24))
        self.Gxyz = struct.unpack('>3d', buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if os_compass_t in parents: return 0
        tmphash = (0xa1708942b089a7e8) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if os_compass_t._packed_fingerprint is None:
            os_compass_t._packed_fingerprint = struct.pack(">Q", os_compass_t._get_hash_recursive([]))
        return os_compass_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

