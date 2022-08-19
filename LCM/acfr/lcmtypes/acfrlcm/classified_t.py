"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class classified_t(object):
    __slots__ = ["utime", "sqid", "pct_cover"]

    def __init__(self):
        self.utime = 0
        self.sqid = [ 0 for dim0 in range(8) ]
        self.pct_cover = [ 0 for dim0 in range(8) ]

    def encode(self):
        buf = BytesIO()
        buf.write(classified_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.utime))
        buf.write(struct.pack('>8b', *self.sqid[:8]))
        buf.write(struct.pack('>8b', *self.pct_cover[:8]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != classified_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return classified_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = classified_t()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.sqid = struct.unpack('>8b', buf.read(8))
        self.pct_cover = struct.unpack('>8b', buf.read(8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if classified_t in parents: return 0
        tmphash = (0x57e35895c77246c6) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if classified_t._packed_fingerprint is None:
            classified_t._packed_fingerprint = struct.pack(">Q", classified_t._get_hash_recursive([]))
        return classified_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

