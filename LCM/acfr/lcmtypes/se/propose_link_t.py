"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class propose_link_t(object):
    __slots__ = ["sensor_id", "link_id", "utime1", "utime2", "x12", "sigma", "x1s", "x2s", "plink_max"]

    SENSOR_ID_ODO = 1
    SENSOR_ID_SONAR = 2
    SENSOR_ID_CAMERA = 4

    def __init__(self):
        self.sensor_id = 0
        self.link_id = 0
        self.utime1 = 0
        self.utime2 = 0
        self.x12 = [ 0.0 for dim0 in range(6) ]
        self.sigma = [ 0.0 for dim0 in range(36) ]
        self.x1s = [ 0.0 for dim0 in range(6) ]
        self.x2s = [ 0.0 for dim0 in range(6) ]
        self.plink_max = 0

    def encode(self):
        buf = BytesIO()
        buf.write(propose_link_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">iiqq", self.sensor_id, self.link_id, self.utime1, self.utime2))
        buf.write(struct.pack('>6d', *self.x12[:6]))
        buf.write(struct.pack('>36d', *self.sigma[:36]))
        buf.write(struct.pack('>6d', *self.x1s[:6]))
        buf.write(struct.pack('>6d', *self.x2s[:6]))
        buf.write(struct.pack(">i", self.plink_max))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != propose_link_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return propose_link_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = propose_link_t()
        self.sensor_id, self.link_id, self.utime1, self.utime2 = struct.unpack(">iiqq", buf.read(24))
        self.x12 = struct.unpack('>6d', buf.read(48))
        self.sigma = struct.unpack('>36d', buf.read(288))
        self.x1s = struct.unpack('>6d', buf.read(48))
        self.x2s = struct.unpack('>6d', buf.read(48))
        self.plink_max = struct.unpack(">i", buf.read(4))[0]
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if propose_link_t in parents: return 0
        tmphash = (0x5f4644e2d0db98e7) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if propose_link_t._packed_fingerprint is None:
            propose_link_t._packed_fingerprint = struct.pack(">Q", propose_link_t._get_hash_recursive([]))
        return propose_link_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

