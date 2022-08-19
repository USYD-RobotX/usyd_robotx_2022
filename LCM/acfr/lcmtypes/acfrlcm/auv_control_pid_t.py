"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import acfrlcm.auv_base_pid_t

class auv_control_pid_t(object):
    __slots__ = ["utime", "velocity", "roll", "pitch", "pitch_r", "depth", "altitude", "heading", "tunnel_depth", "tunnel_descent", "tunnel_pitch", "tunnel_heading"]

    def __init__(self):
        self.utime = 0
        self.velocity = acfrlcm.auv_base_pid_t()
        self.roll = acfrlcm.auv_base_pid_t()
        self.pitch = acfrlcm.auv_base_pid_t()
        self.pitch_r = acfrlcm.auv_base_pid_t()
        self.depth = acfrlcm.auv_base_pid_t()
        self.altitude = acfrlcm.auv_base_pid_t()
        self.heading = acfrlcm.auv_base_pid_t()
        self.tunnel_depth = acfrlcm.auv_base_pid_t()
        self.tunnel_descent = acfrlcm.auv_base_pid_t()
        self.tunnel_pitch = acfrlcm.auv_base_pid_t()
        self.tunnel_heading = acfrlcm.auv_base_pid_t()

    def encode(self):
        buf = BytesIO()
        buf.write(auv_control_pid_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.utime))
        assert self.velocity._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.velocity._encode_one(buf)
        assert self.roll._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.roll._encode_one(buf)
        assert self.pitch._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.pitch._encode_one(buf)
        assert self.pitch_r._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.pitch_r._encode_one(buf)
        assert self.depth._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.depth._encode_one(buf)
        assert self.altitude._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.altitude._encode_one(buf)
        assert self.heading._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.heading._encode_one(buf)
        assert self.tunnel_depth._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.tunnel_depth._encode_one(buf)
        assert self.tunnel_descent._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.tunnel_descent._encode_one(buf)
        assert self.tunnel_pitch._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.tunnel_pitch._encode_one(buf)
        assert self.tunnel_heading._get_packed_fingerprint() == acfrlcm.auv_base_pid_t._get_packed_fingerprint()
        self.tunnel_heading._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != auv_control_pid_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return auv_control_pid_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = auv_control_pid_t()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.velocity = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.roll = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.pitch = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.pitch_r = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.depth = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.altitude = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.heading = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.tunnel_depth = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.tunnel_descent = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.tunnel_pitch = acfrlcm.auv_base_pid_t._decode_one(buf)
        self.tunnel_heading = acfrlcm.auv_base_pid_t._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if auv_control_pid_t in parents: return 0
        newparents = parents + [auv_control_pid_t]
        tmphash = (0xe46e1d4ed2cdc914+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)+ acfrlcm.auv_base_pid_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if auv_control_pid_t._packed_fingerprint is None:
            auv_control_pid_t._packed_fingerprint = struct.pack(">Q", auv_control_pid_t._get_hash_recursive([]))
        return auv_control_pid_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

