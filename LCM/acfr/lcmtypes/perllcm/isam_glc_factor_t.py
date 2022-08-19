"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class isam_glc_factor_t(object):
    __slots__ = ["utime", "np", "ids", "n", "x", "m", "U", "is_root_shifted"]

    def __init__(self):
        self.utime = 0
        self.np = 0
        self.ids = []
        self.n = 0
        self.x = []
        self.m = 0
        self.U = []
        self.is_root_shifted = 0

    def encode(self):
        buf = BytesIO()
        buf.write(isam_glc_factor_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.utime, self.np))
        buf.write(struct.pack('>%dq' % self.np, *self.ids[:self.np]))
        buf.write(struct.pack(">i", self.n))
        buf.write(struct.pack('>%dd' % self.n, *self.x[:self.n]))
        buf.write(struct.pack(">i", self.m))
        buf.write(struct.pack('>%dd' % self.m, *self.U[:self.m]))
        buf.write(struct.pack(">b", self.is_root_shifted))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != isam_glc_factor_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return isam_glc_factor_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = isam_glc_factor_t()
        self.utime, self.np = struct.unpack(">qi", buf.read(12))
        self.ids = struct.unpack('>%dq' % self.np, buf.read(self.np * 8))
        self.n = struct.unpack(">i", buf.read(4))[0]
        self.x = struct.unpack('>%dd' % self.n, buf.read(self.n * 8))
        self.m = struct.unpack(">i", buf.read(4))[0]
        self.U = struct.unpack('>%dd' % self.m, buf.read(self.m * 8))
        self.is_root_shifted = struct.unpack(">b", buf.read(1))[0]
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if isam_glc_factor_t in parents: return 0
        tmphash = (0xe2d0deb0a20c1b5b) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if isam_glc_factor_t._packed_fingerprint is None:
            isam_glc_factor_t._packed_fingerprint = struct.pack(">Q", isam_glc_factor_t._get_hash_recursive([]))
        return isam_glc_factor_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

