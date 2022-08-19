"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class ms_gx3_45_t(object):
    __slots__ = ["utime", "filter_state", "dynamics_mode", "status_flags", "gps_timestamp_valid", "gps_timeofweek_seconds", "gps_week_number", "lat_lon_alt_valid", "lat_lon_alt", "lat_lon_alt_std_valid", "lat_lon_alt_std", "ned_vel_valid", "ned_vel", "ned_vel_std_valid", "ned_vel_std", "euler_valid", "euler", "euler_std_valid", "euler_std", "ang_rate_valid", "ang_rate"]

    FILTER_STATE_STARTUP = 0
    FILTER_STATE_INIT = 1
    FILTER_STATE_RUN_VALID = 2
    FILTER_STATE_RUN_ERROR = 3
    FILTER_DYNM_PORTABLE = 1
    FILTER_DYNM_AUTOMOTIVE = 2
    FILTER_DYNM_AIRBORNE = 3
    FILTER_STAT_ATT_NOT_INIT = 4096
    FILTER_STAT_PV_NOT_INIT = 8192
    FILTER_STAT_NO_IMU = 1
    FILTER_STAT_NO_GPS = 2
    FILTER_STAT_MAT_SING = 8
    FILTER_STAT_P_HIGH_COV = 16
    FILTER_STAT_V_HIGH_COV = 32
    FILTER_STAT_A_HIGH_COV = 64
    FILTER_STAT_NAN_SOL = 128

    def __init__(self):
        self.utime = 0
        self.filter_state = 0
        self.dynamics_mode = 0
        self.status_flags = 0
        self.gps_timestamp_valid = 0
        self.gps_timeofweek_seconds = 0.0
        self.gps_week_number = 0
        self.lat_lon_alt_valid = 0
        self.lat_lon_alt = [ 0.0 for dim0 in range(3) ]
        self.lat_lon_alt_std_valid = 0
        self.lat_lon_alt_std = [ 0.0 for dim0 in range(3) ]
        self.ned_vel_valid = 0
        self.ned_vel = [ 0.0 for dim0 in range(3) ]
        self.ned_vel_std_valid = 0
        self.ned_vel_std = [ 0.0 for dim0 in range(3) ]
        self.euler_valid = 0
        self.euler = [ 0.0 for dim0 in range(3) ]
        self.euler_std_valid = 0
        self.euler_std = [ 0.0 for dim0 in range(3) ]
        self.ang_rate_valid = 0
        self.ang_rate = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(ms_gx3_45_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qhhhbdhb", self.utime, self.filter_state, self.dynamics_mode, self.status_flags, self.gps_timestamp_valid, self.gps_timeofweek_seconds, self.gps_week_number, self.lat_lon_alt_valid))
        buf.write(struct.pack('>3d', *self.lat_lon_alt[:3]))
        buf.write(struct.pack(">b", self.lat_lon_alt_std_valid))
        buf.write(struct.pack('>3f', *self.lat_lon_alt_std[:3]))
        buf.write(struct.pack(">b", self.ned_vel_valid))
        buf.write(struct.pack('>3f', *self.ned_vel[:3]))
        buf.write(struct.pack(">b", self.ned_vel_std_valid))
        buf.write(struct.pack('>3f', *self.ned_vel_std[:3]))
        buf.write(struct.pack(">b", self.euler_valid))
        buf.write(struct.pack('>3f', *self.euler[:3]))
        buf.write(struct.pack(">b", self.euler_std_valid))
        buf.write(struct.pack('>3f', *self.euler_std[:3]))
        buf.write(struct.pack(">b", self.ang_rate_valid))
        buf.write(struct.pack('>3f', *self.ang_rate[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != ms_gx3_45_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return ms_gx3_45_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = ms_gx3_45_t()
        self.utime, self.filter_state, self.dynamics_mode, self.status_flags, self.gps_timestamp_valid, self.gps_timeofweek_seconds, self.gps_week_number, self.lat_lon_alt_valid = struct.unpack(">qhhhbdhb", buf.read(26))
        self.lat_lon_alt = struct.unpack('>3d', buf.read(24))
        self.lat_lon_alt_std_valid = struct.unpack(">b", buf.read(1))[0]
        self.lat_lon_alt_std = struct.unpack('>3f', buf.read(12))
        self.ned_vel_valid = struct.unpack(">b", buf.read(1))[0]
        self.ned_vel = struct.unpack('>3f', buf.read(12))
        self.ned_vel_std_valid = struct.unpack(">b", buf.read(1))[0]
        self.ned_vel_std = struct.unpack('>3f', buf.read(12))
        self.euler_valid = struct.unpack(">b", buf.read(1))[0]
        self.euler = struct.unpack('>3f', buf.read(12))
        self.euler_std_valid = struct.unpack(">b", buf.read(1))[0]
        self.euler_std = struct.unpack('>3f', buf.read(12))
        self.ang_rate_valid = struct.unpack(">b", buf.read(1))[0]
        self.ang_rate = struct.unpack('>3f', buf.read(12))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if ms_gx3_45_t in parents: return 0
        tmphash = (0xeae5faaa7bf153f8) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if ms_gx3_45_t._packed_fingerprint is None:
            ms_gx3_45_t._packed_fingerprint = struct.pack(">Q", ms_gx3_45_t._get_hash_recursive([]))
        return ms_gx3_45_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

