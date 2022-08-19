"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class bs_nvg_t(object):
    __slots__ = ["time_received", "time", "latitude", "hemisphere_ns", "longitude", "hemisphere_ew", "quality", "altitude", "depth", "heading", "roll", "pitch", "time_compute"]

    def __init__(self):
        self.time_received = 0
        self.time = 0
        self.latitude = 0.0
        self.hemisphere_ns = ""
        self.longitude = 0.0
        self.hemisphere_ew = ""
        self.quality = 0
        self.altitude = 0.0
        self.depth = 0.0
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.time_compute = 0

    def encode(self):
        buf = BytesIO()
        buf.write(bs_nvg_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qqd", self.time_received, self.time, self.latitude))
        __hemisphere_ns_encoded = self.hemisphere_ns.encode('utf-8')
        buf.write(struct.pack('>I', len(__hemisphere_ns_encoded)+1))
        buf.write(__hemisphere_ns_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">d", self.longitude))
        __hemisphere_ew_encoded = self.hemisphere_ew.encode('utf-8')
        buf.write(struct.pack('>I', len(__hemisphere_ew_encoded)+1))
        buf.write(__hemisphere_ew_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">bdddddq", self.quality, self.altitude, self.depth, self.heading, self.roll, self.pitch, self.time_compute))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != bs_nvg_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return bs_nvg_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = bs_nvg_t()
        self.time_received, self.time, self.latitude = struct.unpack(">qqd", buf.read(24))
        __hemisphere_ns_len = struct.unpack('>I', buf.read(4))[0]
        self.hemisphere_ns = buf.read(__hemisphere_ns_len)[:-1].decode('utf-8', 'replace')
        self.longitude = struct.unpack(">d", buf.read(8))[0]
        __hemisphere_ew_len = struct.unpack('>I', buf.read(4))[0]
        self.hemisphere_ew = buf.read(__hemisphere_ew_len)[:-1].decode('utf-8', 'replace')
        self.quality, self.altitude, self.depth, self.heading, self.roll, self.pitch, self.time_compute = struct.unpack(">bdddddq", buf.read(49))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if bs_nvg_t in parents: return 0
        tmphash = (0x6efd82426393146) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if bs_nvg_t._packed_fingerprint is None:
            bs_nvg_t._packed_fingerprint = struct.pack(">Q", bs_nvg_t._get_hash_recursive([]))
        return bs_nvg_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

