"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class acfr_psu_type_2_t(object):
    __slots__ = ["utime", "address", "voltage_in", "voltage_out", "current_out", "temperature"]

    def __init__(self):
        self.utime = 0
        self.address = 0
        self.voltage_in = [ 0.0 for dim0 in range(2) ]
        self.voltage_out = [ 0.0 for dim0 in range(2) ]
        self.current_out = [ 0.0 for dim0 in range(2) ]
        self.temperature = [ 0.0 for dim0 in range(2) ]

    def encode(self):
        buf = BytesIO()
        buf.write(acfr_psu_type_2_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qb", self.utime, self.address))
        buf.write(struct.pack('>2d', *self.voltage_in[:2]))
        buf.write(struct.pack('>2d', *self.voltage_out[:2]))
        buf.write(struct.pack('>2d', *self.current_out[:2]))
        buf.write(struct.pack('>2d', *self.temperature[:2]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != acfr_psu_type_2_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return acfr_psu_type_2_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = acfr_psu_type_2_t()
        self.utime, self.address = struct.unpack(">qb", buf.read(9))
        self.voltage_in = struct.unpack('>2d', buf.read(16))
        self.voltage_out = struct.unpack('>2d', buf.read(16))
        self.current_out = struct.unpack('>2d', buf.read(16))
        self.temperature = struct.unpack('>2d', buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if acfr_psu_type_2_t in parents: return 0
        tmphash = (0x8998735ff639f6c6) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if acfr_psu_type_2_t._packed_fingerprint is None:
            acfr_psu_type_2_t._packed_fingerprint = struct.pack(">Q", acfr_psu_type_2_t._get_hash_recursive([]))
        return acfr_psu_type_2_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

