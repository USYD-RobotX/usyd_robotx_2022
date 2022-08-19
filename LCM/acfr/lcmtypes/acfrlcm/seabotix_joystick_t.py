"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class seabotix_joystick_t(object):
    __slots__ = ["utime", "x", "y", "z", "v"]

    def __init__(self):
        self.utime = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.v = 0

    def encode(self):
        buf = BytesIO()
        buf.write(seabotix_joystick_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qhhhh", self.utime, self.x, self.y, self.z, self.v))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != seabotix_joystick_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return seabotix_joystick_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = seabotix_joystick_t()
        self.utime, self.x, self.y, self.z, self.v = struct.unpack(">qhhhh", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if seabotix_joystick_t in parents: return 0
        tmphash = (0x34b3fefc9a05b995) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if seabotix_joystick_t._packed_fingerprint is None:
            seabotix_joystick_t._packed_fingerprint = struct.pack(">Q", seabotix_joystick_t._get_hash_recursive([]))
        return seabotix_joystick_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

