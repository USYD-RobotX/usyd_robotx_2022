"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class auv_camera_control_t(object):
    __slots__ = ["utime", "command", "path"]

    LOG_START = 1
    LOG_STOP = 2
    SET_PATH = 3

    def __init__(self):
        self.utime = 0
        self.command = 0
        self.path = ""

    def encode(self):
        buf = BytesIO()
        buf.write(auv_camera_control_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.utime, self.command))
        __path_encoded = self.path.encode('utf-8')
        buf.write(struct.pack('>I', len(__path_encoded)+1))
        buf.write(__path_encoded)
        buf.write(b"\0")

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != auv_camera_control_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return auv_camera_control_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = auv_camera_control_t()
        self.utime, self.command = struct.unpack(">qi", buf.read(12))
        __path_len = struct.unpack('>I', buf.read(4))[0]
        self.path = buf.read(__path_len)[:-1].decode('utf-8', 'replace')
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if auv_camera_control_t in parents: return 0
        tmphash = (0x4877abc26d809574) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if auv_camera_control_t._packed_fingerprint is None:
            auv_camera_control_t._packed_fingerprint = struct.pack(">Q", auv_camera_control_t._get_hash_recursive([]))
        return auv_camera_control_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

