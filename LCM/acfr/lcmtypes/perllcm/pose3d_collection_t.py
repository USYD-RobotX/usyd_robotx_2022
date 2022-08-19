"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import perllcm.pose3d_t

class pose3d_collection_t(object):
    __slots__ = ["utime", "npose", "pose"]

    def __init__(self):
        self.utime = 0
        self.npose = 0
        self.pose = []

    def encode(self):
        buf = BytesIO()
        buf.write(pose3d_collection_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.utime, self.npose))
        for i0 in range(self.npose):
            assert self.pose[i0]._get_packed_fingerprint() == perllcm.pose3d_t._get_packed_fingerprint()
            self.pose[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != pose3d_collection_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return pose3d_collection_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = pose3d_collection_t()
        self.utime, self.npose = struct.unpack(">qi", buf.read(12))
        self.pose = []
        for i0 in range(self.npose):
            self.pose.append(perllcm.pose3d_t._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if pose3d_collection_t in parents: return 0
        newparents = parents + [pose3d_collection_t]
        tmphash = (0x4c95ac83a5b66668+ perllcm.pose3d_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if pose3d_collection_t._packed_fingerprint is None:
            pose3d_collection_t._packed_fingerprint = struct.pack(">Q", pose3d_collection_t._get_hash_recursive([]))
        return pose3d_collection_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

