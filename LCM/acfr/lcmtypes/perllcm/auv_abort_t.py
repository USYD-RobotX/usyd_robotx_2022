"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class auv_abort_t(object):
    __slots__ = ["utime", "dest", "abort"]

    ABORT_FALSE = 0
    ABORT_HARD = 1
    ABORT_TO_SURFACE = 2
    ABORT_TO_WAYPOINT = 3
    ABORT_TO_POS = 4

    def __init__(self):
        self.utime = 0
        self.dest = 0
        self.abort = 0

    def encode(self):
        buf = BytesIO()
        buf.write(auv_abort_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qii", self.utime, self.dest, self.abort))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != auv_abort_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return auv_abort_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = auv_abort_t()
        self.utime, self.dest, self.abort = struct.unpack(">qii", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if auv_abort_t in parents: return 0
        tmphash = (0x7058af0d95a14f58) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if auv_abort_t._packed_fingerprint is None:
            auv_abort_t._packed_fingerprint = struct.pack(">Q", auv_abort_t._get_hash_recursive([]))
        return auv_abort_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

