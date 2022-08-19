"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class micron_sounder_t(object):
    __slots__ = ["utime", "altitude"]

    def __init__(self):
        self.utime = 0
        self.altitude = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(micron_sounder_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qd", self.utime, self.altitude))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != micron_sounder_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return micron_sounder_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = micron_sounder_t()
        self.utime, self.altitude = struct.unpack(">qd", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if micron_sounder_t in parents: return 0
        tmphash = (0xaeb92c853f0bfa23) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if micron_sounder_t._packed_fingerprint is None:
            micron_sounder_t._packed_fingerprint = struct.pack(">Q", micron_sounder_t._get_hash_recursive([]))
        return micron_sounder_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

