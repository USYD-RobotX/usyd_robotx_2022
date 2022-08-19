"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import acfrlcm.auv_occmap_element_t

class auv_occmap_t(object):
    __slots__ = ["utime", "num_elements", "elements"]

    def __init__(self):
        self.utime = 0
        self.num_elements = 0
        self.elements = []

    def encode(self):
        buf = BytesIO()
        buf.write(auv_occmap_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.utime, self.num_elements))
        for i0 in range(self.num_elements):
            assert self.elements[i0]._get_packed_fingerprint() == acfrlcm.auv_occmap_element_t._get_packed_fingerprint()
            self.elements[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != auv_occmap_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return auv_occmap_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = auv_occmap_t()
        self.utime, self.num_elements = struct.unpack(">qi", buf.read(12))
        self.elements = []
        for i0 in range(self.num_elements):
            self.elements.append(acfrlcm.auv_occmap_element_t._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if auv_occmap_t in parents: return 0
        newparents = parents + [auv_occmap_t]
        tmphash = (0x93ccee2b1caea57d+ acfrlcm.auv_occmap_element_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if auv_occmap_t._packed_fingerprint is None:
            auv_occmap_t._packed_fingerprint = struct.pack(">Q", auv_occmap_t._get_hash_recursive([]))
        return auv_occmap_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

