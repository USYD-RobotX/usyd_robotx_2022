"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class ardrone_pid_t(object):
    __slots__ = ["utime", "x_pTerm", "x_iTerm", "x_dTerm", "x_pid", "y_pTerm", "y_iTerm", "y_dTerm", "y_pid", "z_pTerm", "z_iTerm", "z_dTerm", "z_pid", "h_pTerm", "h_iTerm", "h_dTerm", "h_pid"]

    def __init__(self):
        self.utime = 0
        self.x_pTerm = 0.0
        self.x_iTerm = 0.0
        self.x_dTerm = 0.0
        self.x_pid = 0.0
        self.y_pTerm = 0.0
        self.y_iTerm = 0.0
        self.y_dTerm = 0.0
        self.y_pid = 0.0
        self.z_pTerm = 0.0
        self.z_iTerm = 0.0
        self.z_dTerm = 0.0
        self.z_pid = 0.0
        self.h_pTerm = 0.0
        self.h_iTerm = 0.0
        self.h_dTerm = 0.0
        self.h_pid = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(ardrone_pid_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qffffffffffffffff", self.utime, self.x_pTerm, self.x_iTerm, self.x_dTerm, self.x_pid, self.y_pTerm, self.y_iTerm, self.y_dTerm, self.y_pid, self.z_pTerm, self.z_iTerm, self.z_dTerm, self.z_pid, self.h_pTerm, self.h_iTerm, self.h_dTerm, self.h_pid))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != ardrone_pid_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return ardrone_pid_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = ardrone_pid_t()
        self.utime, self.x_pTerm, self.x_iTerm, self.x_dTerm, self.x_pid, self.y_pTerm, self.y_iTerm, self.y_dTerm, self.y_pid, self.z_pTerm, self.z_iTerm, self.z_dTerm, self.z_pid, self.h_pTerm, self.h_iTerm, self.h_dTerm, self.h_pid = struct.unpack(">qffffffffffffffff", buf.read(72))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if ardrone_pid_t in parents: return 0
        tmphash = (0x275e7b5b4cc6f45b) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if ardrone_pid_t._packed_fingerprint is None:
            ardrone_pid_t._packed_fingerprint = struct.pack(">Q", ardrone_pid_t._get_hash_recursive([]))
        return ardrone_pid_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

