"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class command2_t(object):
    __slots__ = ["exec_str", "command_name", "group", "auto_respawn", "stop_signal", "stop_time_allowed", "num_options", "option_names", "option_values"]

    def __init__(self):
        self.exec_str = ""
        self.command_name = ""
        self.group = ""
        self.auto_respawn = False
        self.stop_signal = 0
        self.stop_time_allowed = 0.0
        self.num_options = 0
        self.option_names = []
        self.option_values = []

    def encode(self):
        buf = BytesIO()
        buf.write(command2_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        __exec_str_encoded = self.exec_str.encode('utf-8')
        buf.write(struct.pack('>I', len(__exec_str_encoded)+1))
        buf.write(__exec_str_encoded)
        buf.write(b"\0")
        __command_name_encoded = self.command_name.encode('utf-8')
        buf.write(struct.pack('>I', len(__command_name_encoded)+1))
        buf.write(__command_name_encoded)
        buf.write(b"\0")
        __group_encoded = self.group.encode('utf-8')
        buf.write(struct.pack('>I', len(__group_encoded)+1))
        buf.write(__group_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">bbfi", self.auto_respawn, self.stop_signal, self.stop_time_allowed, self.num_options))
        for i0 in range(self.num_options):
            __option_names_encoded = self.option_names[i0].encode('utf-8')
            buf.write(struct.pack('>I', len(__option_names_encoded)+1))
            buf.write(__option_names_encoded)
            buf.write(b"\0")
        for i0 in range(self.num_options):
            __option_values_encoded = self.option_values[i0].encode('utf-8')
            buf.write(struct.pack('>I', len(__option_values_encoded)+1))
            buf.write(__option_values_encoded)
            buf.write(b"\0")

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != command2_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return command2_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = command2_t()
        __exec_str_len = struct.unpack('>I', buf.read(4))[0]
        self.exec_str = buf.read(__exec_str_len)[:-1].decode('utf-8', 'replace')
        __command_name_len = struct.unpack('>I', buf.read(4))[0]
        self.command_name = buf.read(__command_name_len)[:-1].decode('utf-8', 'replace')
        __group_len = struct.unpack('>I', buf.read(4))[0]
        self.group = buf.read(__group_len)[:-1].decode('utf-8', 'replace')
        self.auto_respawn = bool(struct.unpack('b', buf.read(1))[0])
        self.stop_signal, self.stop_time_allowed, self.num_options = struct.unpack(">bfi", buf.read(9))
        self.option_names = []
        for i0 in range(self.num_options):
            __option_names_len = struct.unpack('>I', buf.read(4))[0]
            self.option_names.append(buf.read(__option_names_len)[:-1].decode('utf-8', 'replace'))
        self.option_values = []
        for i0 in range(self.num_options):
            __option_values_len = struct.unpack('>I', buf.read(4))[0]
            self.option_values.append(buf.read(__option_values_len)[:-1].decode('utf-8', 'replace'))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if command2_t in parents: return 0
        tmphash = (0xfc82ea7fd014c086) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if command2_t._packed_fingerprint is None:
            command2_t._packed_fingerprint = struct.pack(">Q", command2_t._get_hash_recursive([]))
        return command2_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

