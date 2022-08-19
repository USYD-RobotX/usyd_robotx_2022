"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class auv_control_goal_t(object):
    __slots__ = ["utime", "pitch_goal"]

    def __init__(self):
        self.utime = 0
        self.pitch_goal = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(auv_control_goal_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qd", self.utime, self.pitch_goal))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != auv_control_goal_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return auv_control_goal_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = auv_control_goal_t()
        self.utime, self.pitch_goal = struct.unpack(">qd", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if auv_control_goal_t in parents: return 0
        tmphash = (0x6a7fcbd10e534b77) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if auv_control_goal_t._packed_fingerprint is None:
            auv_control_goal_t._packed_fingerprint = struct.pack(">Q", auv_control_goal_t._get_hash_recursive([]))
        return auv_control_goal_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

