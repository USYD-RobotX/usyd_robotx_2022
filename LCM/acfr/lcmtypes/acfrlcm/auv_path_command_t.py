"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class auv_path_command_t(object):
    __slots__ = ["utime", "goal_id", "depth_mode", "run_mode", "waypoint"]

    DEPTH = 0
    ALTITUDE = 1
    NORMAL = 0
    HOLD = 1

    def __init__(self):
        self.utime = 0
        self.goal_id = 0
        self.depth_mode = 0
        self.run_mode = 0
        self.waypoint = [ 0.0 for dim0 in range(7) ]

    def encode(self):
        buf = BytesIO()
        buf.write(auv_path_command_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qibb", self.utime, self.goal_id, self.depth_mode, self.run_mode))
        buf.write(struct.pack('>7d', *self.waypoint[:7]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != auv_path_command_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return auv_path_command_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = auv_path_command_t()
        self.utime, self.goal_id, self.depth_mode, self.run_mode = struct.unpack(">qibb", buf.read(14))
        self.waypoint = struct.unpack('>7d', buf.read(56))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if auv_path_command_t in parents: return 0
        tmphash = (0x2d0ebf3fda1571c7) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if auv_path_command_t._packed_fingerprint is None:
            auv_path_command_t._packed_fingerprint = struct.pack(">Q", auv_path_command_t._get_hash_recursive([]))
        return auv_path_command_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

