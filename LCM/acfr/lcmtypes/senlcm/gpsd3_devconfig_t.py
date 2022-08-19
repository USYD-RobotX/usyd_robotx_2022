"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class gpsd3_devconfig_t(object):
    __slots__ = ["path", "flags", "driver", "subtype", "activated", "baudrate", "stopbits", "cycle", "mincycle", "driver_mode"]

    FLAGS_SEEN_GPS = 1
    FLAGS_SEEN_RTCM2 = 2
    FLAGS_SEEN_RTCM3 = 4
    FLAGS_SEEN_AIS = 8

    def __init__(self):
        self.path = ""
        self.flags = 0
        self.driver = ""
        self.subtype = ""
        self.activated = 0.0
        self.baudrate = 0
        self.stopbits = 0
        self.cycle = 0.0
        self.mincycle = 0.0
        self.driver_mode = 0

    def encode(self):
        buf = BytesIO()
        buf.write(gpsd3_devconfig_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        __path_encoded = self.path.encode('utf-8')
        buf.write(struct.pack('>I', len(__path_encoded)+1))
        buf.write(__path_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">h", self.flags))
        __driver_encoded = self.driver.encode('utf-8')
        buf.write(struct.pack('>I', len(__driver_encoded)+1))
        buf.write(__driver_encoded)
        buf.write(b"\0")
        __subtype_encoded = self.subtype.encode('utf-8')
        buf.write(struct.pack('>I', len(__subtype_encoded)+1))
        buf.write(__subtype_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">dibddh", self.activated, self.baudrate, self.stopbits, self.cycle, self.mincycle, self.driver_mode))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != gpsd3_devconfig_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return gpsd3_devconfig_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = gpsd3_devconfig_t()
        __path_len = struct.unpack('>I', buf.read(4))[0]
        self.path = buf.read(__path_len)[:-1].decode('utf-8', 'replace')
        self.flags = struct.unpack(">h", buf.read(2))[0]
        __driver_len = struct.unpack('>I', buf.read(4))[0]
        self.driver = buf.read(__driver_len)[:-1].decode('utf-8', 'replace')
        __subtype_len = struct.unpack('>I', buf.read(4))[0]
        self.subtype = buf.read(__subtype_len)[:-1].decode('utf-8', 'replace')
        self.activated, self.baudrate, self.stopbits, self.cycle, self.mincycle, self.driver_mode = struct.unpack(">dibddh", buf.read(31))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if gpsd3_devconfig_t in parents: return 0
        tmphash = (0xaf6ced509cc9000d) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if gpsd3_devconfig_t._packed_fingerprint is None:
            gpsd3_devconfig_t._packed_fingerprint = struct.pack(">Q", gpsd3_devconfig_t._get_hash_recursive([]))
        return gpsd3_devconfig_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

