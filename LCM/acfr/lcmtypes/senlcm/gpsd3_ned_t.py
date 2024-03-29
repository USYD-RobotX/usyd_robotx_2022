"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class gpsd3_ned_t(object):
    __slots__ = ["relPosN", "relPosE", "relPosD", "relPosL", "relPosH", "velN", "velE", "velD"]

    def __init__(self):
        self.relPosN = 0.0
        self.relPosE = 0.0
        self.relPosD = 0.0
        self.relPosL = 0.0
        self.relPosH = 0.0
        self.velN = 0.0
        self.velE = 0.0
        self.velD = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(gpsd3_ned_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">dddddddd", self.relPosN, self.relPosE, self.relPosD, self.relPosL, self.relPosH, self.velN, self.velE, self.velD))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != gpsd3_ned_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return gpsd3_ned_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = gpsd3_ned_t()
        self.relPosN, self.relPosE, self.relPosD, self.relPosL, self.relPosH, self.velN, self.velE, self.velD = struct.unpack(">dddddddd", buf.read(64))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if gpsd3_ned_t in parents: return 0
        tmphash = (0xdcad442ade2f4f4d) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if gpsd3_ned_t._packed_fingerprint is None:
            gpsd3_ned_t._packed_fingerprint = struct.pack(">Q", gpsd3_ned_t._get_hash_recursive([]))
        return gpsd3_ned_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

