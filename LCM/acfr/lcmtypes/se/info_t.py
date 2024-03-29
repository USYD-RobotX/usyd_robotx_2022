"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class info_t(object):
    __slots__ = ["key", "data"]

    def __init__(self):
        self.key = ""
        self.data = ""

    def encode(self):
        buf = BytesIO()
        buf.write(info_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        __key_encoded = self.key.encode('utf-8')
        buf.write(struct.pack('>I', len(__key_encoded)+1))
        buf.write(__key_encoded)
        buf.write(b"\0")
        __data_encoded = self.data.encode('utf-8')
        buf.write(struct.pack('>I', len(__data_encoded)+1))
        buf.write(__data_encoded)
        buf.write(b"\0")

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != info_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return info_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = info_t()
        __key_len = struct.unpack('>I', buf.read(4))[0]
        self.key = buf.read(__key_len)[:-1].decode('utf-8', 'replace')
        __data_len = struct.unpack('>I', buf.read(4))[0]
        self.data = buf.read(__data_len)[:-1].decode('utf-8', 'replace')
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if info_t in parents: return 0
        tmphash = (0x90d5a7bfcf2fd983) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if info_t._packed_fingerprint is None:
            info_t._packed_fingerprint = struct.pack(">Q", info_t._get_hash_recursive([]))
        return info_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

