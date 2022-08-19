"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import perllcm.velodyne_laser_return_t

import perllcm.velodyne_laser_return_lite_t

class velodyne_laser_return_collection_t(object):
    __slots__ = ["num_lr", "laser_returns", "num_lrl", "laser_returns_lite", "utime", "x_vs", "pose", "has_pose"]

    def __init__(self):
        self.num_lr = 0
        self.laser_returns = []
        self.num_lrl = 0
        self.laser_returns_lite = []
        self.utime = 0
        self.x_vs = [ 0.0 for dim0 in range(6) ]
        self.pose = [ 0.0 for dim0 in range(6) ]
        self.has_pose = 0

    def encode(self):
        buf = BytesIO()
        buf.write(velodyne_laser_return_collection_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.num_lr))
        for i0 in range(self.num_lr):
            assert self.laser_returns[i0]._get_packed_fingerprint() == perllcm.velodyne_laser_return_t._get_packed_fingerprint()
            self.laser_returns[i0]._encode_one(buf)
        buf.write(struct.pack(">i", self.num_lrl))
        for i0 in range(self.num_lrl):
            assert self.laser_returns_lite[i0]._get_packed_fingerprint() == perllcm.velodyne_laser_return_lite_t._get_packed_fingerprint()
            self.laser_returns_lite[i0]._encode_one(buf)
        buf.write(struct.pack(">q", self.utime))
        buf.write(struct.pack('>6d', *self.x_vs[:6]))
        buf.write(struct.pack('>6d', *self.pose[:6]))
        buf.write(struct.pack(">b", self.has_pose))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != velodyne_laser_return_collection_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return velodyne_laser_return_collection_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = velodyne_laser_return_collection_t()
        self.num_lr = struct.unpack(">i", buf.read(4))[0]
        self.laser_returns = []
        for i0 in range(self.num_lr):
            self.laser_returns.append(perllcm.velodyne_laser_return_t._decode_one(buf))
        self.num_lrl = struct.unpack(">i", buf.read(4))[0]
        self.laser_returns_lite = []
        for i0 in range(self.num_lrl):
            self.laser_returns_lite.append(perllcm.velodyne_laser_return_lite_t._decode_one(buf))
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.x_vs = struct.unpack('>6d', buf.read(48))
        self.pose = struct.unpack('>6d', buf.read(48))
        self.has_pose = struct.unpack(">b", buf.read(1))[0]
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if velodyne_laser_return_collection_t in parents: return 0
        newparents = parents + [velodyne_laser_return_collection_t]
        tmphash = (0xb866667dad3f61ef+ perllcm.velodyne_laser_return_t._get_hash_recursive(newparents)+ perllcm.velodyne_laser_return_lite_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if velodyne_laser_return_collection_t._packed_fingerprint is None:
            velodyne_laser_return_collection_t._packed_fingerprint = struct.pack(">Q", velodyne_laser_return_collection_t._get_hash_recursive([]))
        return velodyne_laser_return_collection_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

