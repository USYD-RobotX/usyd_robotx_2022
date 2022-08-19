"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import perllcm.position_t

class auv_iver_state_t(object):
    __slots__ = ["utime", "position", "altitude", "orglat", "orglon", "abort_state"]

    def __init__(self):
        self.utime = 0
        self.position = perllcm.position_t()
        self.altitude = 0.0
        self.orglat = 0.0
        self.orglon = 0.0
        self.abort_state = False

    def encode(self):
        buf = BytesIO()
        buf.write(auv_iver_state_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.utime))
        assert self.position._get_packed_fingerprint() == perllcm.position_t._get_packed_fingerprint()
        self.position._encode_one(buf)
        buf.write(struct.pack(">dddb", self.altitude, self.orglat, self.orglon, self.abort_state))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != auv_iver_state_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return auv_iver_state_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = auv_iver_state_t()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.position = perllcm.position_t._decode_one(buf)
        self.altitude, self.orglat, self.orglon = struct.unpack(">ddd", buf.read(24))
        self.abort_state = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if auv_iver_state_t in parents: return 0
        newparents = parents + [auv_iver_state_t]
        tmphash = (0xafbe1e43a6092be3+ perllcm.position_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if auv_iver_state_t._packed_fingerprint is None:
            auv_iver_state_t._packed_fingerprint = struct.pack(">Q", auv_iver_state_t._get_hash_recursive([]))
        return auv_iver_state_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

