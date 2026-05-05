"""
Minimal Python wrapper for libdm_device.so (DM_DeviceSDK).
Talks to USB-CANFD adapter in **classic CAN 2.0B** mode at 1Mbps,
suited to a J4310 motor on default 1Mbps baud (CAN_BR=4).
"""
import ctypes
from ctypes import (
    CDLL, CFUNCTYPE, POINTER, Structure,
    c_bool, c_char_p, c_float, c_int, c_int32, c_size_t,
    c_uint8, c_uint16, c_uint32, c_uint64, c_void_p, byref,
)
import sys
import threading

SDK_PATH = "/home/ubuntu/damiao/8.工具和上位机/dm-tools/DM_DeviceSDK/C&C++/lib/linux/libdm_device.so"
LIBUSB_PATH = "/home/ubuntu/miniforge3/lib/libusb-1.0.so.0"  # newer libusb with libusb_init_context

# Preload libusb so libdm_device's unresolved symbols (libusb_open, etc.) bind at load time.
ctypes.CDLL(LIBUSB_PATH, mode=ctypes.RTLD_GLOBAL)
_sdk = CDLL(SDK_PATH)

DEV_USB2CANFD = 0
DEV_USB2CANFD_DUAL = 1
DEV_TYPE = DEV_USB2CANFD


class usb_rx_frame_head_t(Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", c_uint32, 29),
        ("esi", c_uint32, 1),
        ("ext", c_uint32, 1),
        ("rtr", c_uint32, 1),
        ("time_stamp", c_uint64),
        ("channel", c_uint8),
        ("canfd", c_uint8, 1),
        ("dir", c_uint8, 1),
        ("brs", c_uint8, 1),
        ("ack", c_uint8, 1),
        ("dlc", c_uint8, 4),
        ("reserved", c_uint16),
    ]


class usb_rx_frame_t(Structure):
    _pack_ = 1
    _fields_ = [
        ("head", usb_rx_frame_head_t),
        ("payload", c_uint8 * 64),
    ]


REC_CALLBACK = CFUNCTYPE(None, POINTER(usb_rx_frame_t))


_sdk.damiao_handle_create.argtypes = [c_int]
_sdk.damiao_handle_create.restype = c_void_p

_sdk.damiao_handle_destroy.argtypes = [c_void_p]
_sdk.damiao_handle_destroy.restype = None

_sdk.damiao_handle_find_devices.argtypes = [c_void_p]
_sdk.damiao_handle_find_devices.restype = c_int

_sdk.damiao_handle_get_devices.argtypes = [c_void_p, POINTER(c_void_p), POINTER(c_int)]
_sdk.damiao_handle_get_devices.restype = None

_sdk.device_get_serial_number.argtypes = [c_void_p, c_char_p, c_size_t]
_sdk.device_get_serial_number.restype = None

_sdk.device_open.argtypes = [c_void_p]
_sdk.device_open.restype = c_bool

_sdk.device_close.argtypes = [c_void_p]
_sdk.device_close.restype = c_bool

_sdk.device_open_channel.argtypes = [c_void_p, c_uint8]
_sdk.device_open_channel.restype = c_bool

_sdk.device_close_channel.argtypes = [c_void_p, c_uint8]
_sdk.device_close_channel.restype = c_bool

_sdk.device_channel_set_baud_with_sp.argtypes = [
    c_void_p, c_uint8, c_bool, c_int, c_int, c_float, c_float
]
_sdk.device_channel_set_baud_with_sp.restype = c_bool

_sdk.device_hook_to_rec.argtypes = [c_void_p, REC_CALLBACK]
_sdk.device_hook_to_rec.restype = None

_sdk.device_hook_to_sent.argtypes = [c_void_p, REC_CALLBACK]
_sdk.device_hook_to_sent.restype = None

_sdk.device_hook_to_err.argtypes = [c_void_p, REC_CALLBACK]
_sdk.device_hook_to_err.restype = None

_sdk.damiao_print_version.argtypes = [c_void_p]
_sdk.damiao_print_version.restype = None

_sdk.device_get_version.argtypes = [c_void_p, c_char_p, c_size_t]
_sdk.device_get_version.restype = None

_sdk.device_channel_get_baudrate.argtypes = [c_void_p, c_uint8, c_void_p]
_sdk.device_channel_get_baudrate.restype = c_bool

_sdk.device_save_config.argtypes = [c_void_p]
_sdk.device_save_config.restype = c_bool


class device_baud_t(Structure):
    _fields_ = [
        ("can_baudrate", c_int),
        ("canfd_baudrate", c_int),
        ("can_sp", c_float),
        ("canfd_sp", c_float),
    ]

_sdk.device_channel_send_fast.argtypes = [
    c_void_p, c_uint8, c_uint32, c_int32, c_bool, c_bool, c_bool,
    c_uint8, POINTER(c_uint8)
]
_sdk.device_channel_send_fast.restype = None


# ---------- DM motor MIT-mode protocol ----------

# J4310 limits (from Python u2canfd example, limit_param[1])
PMAX = 12.5     # rad
VMAX = 30.0     # rad/s
TMAX = 10.0     # Nm


def _f2u(x: float, xmin: float, xmax: float, bits: int) -> int:
    span = xmax - xmin
    return max(0, min((1 << bits) - 1, int((x - xmin) / span * ((1 << bits) - 1))))


def _u2f(u: int, xmin: float, xmax: float, bits: int) -> float:
    return (u / ((1 << bits) - 1)) * (xmax - xmin) + xmin


def encode_mit(kp: float, kd: float, q: float, dq: float, tau: float) -> bytes:
    kp_u = _f2u(kp, 0, 500, 12)
    kd_u = _f2u(kd, 0, 5, 12)
    q_u = _f2u(q, -PMAX, PMAX, 16)
    dq_u = _f2u(dq, -VMAX, VMAX, 12)
    tau_u = _f2u(tau, -TMAX, TMAX, 12)
    return bytes([
        (q_u >> 8) & 0xff,
        q_u & 0xff,
        dq_u >> 4,
        ((dq_u & 0xf) << 4) | ((kp_u >> 8) & 0xf),
        kp_u & 0xff,
        kd_u >> 4,
        ((kd_u & 0xf) << 4) | ((tau_u >> 8) & 0xf),
        tau_u & 0xff,
    ])


def decode_feedback(payload: bytes):
    # payload[0] = id_low(4) | err(4)
    err = (payload[0] >> 4) & 0xf
    q_u = (payload[1] << 8) | payload[2]
    dq_u = (payload[3] << 4) | (payload[4] >> 4)
    tau_u = ((payload[4] & 0xf) << 8) | payload[5]
    q = _u2f(q_u, -PMAX, PMAX, 16)
    dq = _u2f(dq_u, -VMAX, VMAX, 12)
    tau = _u2f(tau_u, -TMAX, TMAX, 12)
    t_mos = payload[6]
    t_rotor = payload[7]
    return dict(err=err, q=q, dq=dq, tau=tau, t_mos=t_mos, t_rotor=t_rotor)


# ---------- High-level wrapper ----------

class DmDevice:
    def __init__(self):
        self.handle = None
        self.dev = None
        self._latest = {}            # mst_id -> last decoded feedback
        self._lock = threading.Lock()
        self._rec_cb = REC_CALLBACK(self._on_rx)
        self._sent_cb = REC_CALLBACK(self._on_sent)
        self._err_cb = REC_CALLBACK(self._on_err)
        self.sent_count = 0
        self.err_count = 0

    def _on_sent(self, frame_ptr):
        self.sent_count += 1

    def _on_err(self, frame_ptr):
        self.err_count += 1
        if self.err_count <= 3:
            f = frame_ptr.contents
            payload = bytes(f.payload[:max(8, f.head.dlc)])
            print(f"[DM ERR #{self.err_count}] id=0x{f.head.can_id:X} "
                  f"dlc={f.head.dlc} canfd={f.head.canfd} brs={f.head.brs} "
                  f"data={[hex(b) for b in payload]}",
                  file=sys.stderr)

    def _on_rx(self, frame_ptr):
        f = frame_ptr.contents
        cid = f.head.can_id
        dlc = f.head.dlc
        # DM feedback frames have dlc=8
        if dlc != 8:
            return
        payload = bytes(f.payload[:dlc])
        try:
            fb = decode_feedback(payload)
        except Exception:
            return
        with self._lock:
            self._latest[cid] = fb

    def open(self, nom_baud_hz: int = 1_000_000, dat_baud_hz: int = 1_000_000,
             canfd: bool = False, brs: bool = None,
             nom_sp: float = 0.75, dat_sp: float = 0.75):
        """Open the USB-CANFD device.
        canfd=False: Classic CAN 2.0B at nom_baud_hz (dat ignored)
        canfd=True : CAN-FD frames; brs controls whether data segment uses dat_baud_hz
        nom_sp/dat_sp: arbitration / data sample point (0.50-1.0, e.g. 0.75 or 0.80)
        """
        self.canfd = canfd
        self.brs_default = brs if brs is not None else canfd
        self.handle = _sdk.damiao_handle_create(DEV_TYPE)
        if not self.handle:
            raise RuntimeError("damiao_handle_create failed")
        n = _sdk.damiao_handle_find_devices(self.handle)
        if n <= 0:
            raise RuntimeError("no USB-CANFD device found")
        dev_list = (c_void_p * 16)()
        count = c_int(0)
        _sdk.damiao_handle_get_devices(self.handle, dev_list, byref(count))
        if count.value == 0:
            raise RuntimeError("get_devices returned 0")
        self.dev = dev_list[0]

        # mimic C++ example: print_version then get_version, get_serial
        _sdk.damiao_print_version(self.handle)
        ver_buf = ctypes.create_string_buffer(255)
        _sdk.device_get_version(self.dev, ver_buf, 255)
        print(f"[DM] device version: {ver_buf.value.decode(errors='replace')}",
              file=sys.stderr)

        sn_buf = ctypes.create_string_buffer(64)
        _sdk.device_get_serial_number(self.dev, sn_buf, 64)
        print(f"[DM] device SN: {sn_buf.value.decode(errors='replace')}", file=sys.stderr)

        if not _sdk.device_open(self.dev):
            raise RuntimeError("device_open failed")

        ok = _sdk.device_channel_set_baud_with_sp(
            self.dev, 0, canfd, nom_baud_hz, dat_baud_hz, nom_sp, dat_sp
        )
        if not ok:
            raise RuntimeError("set_baud failed")

        if not _sdk.device_open_channel(self.dev, 0):
            raise RuntimeError("open_channel failed")

        # C++ example registers hooks AFTER open_channel
        _sdk.device_hook_to_rec(self.dev, self._rec_cb)
        _sdk.device_hook_to_sent(self.dev, self._sent_cb)
        _sdk.device_hook_to_err(self.dev, self._err_cb)
        mode_str = f"CAN-FD {nom_baud_hz//1000}K/{dat_baud_hz//1000}K BRS" if canfd \
                   else f"Classic CAN {nom_baud_hz//1000}K"
        print(f"[DM] channel 0 opened, {mode_str}", file=sys.stderr)

    def close(self):
        if self.dev:
            _sdk.device_close_channel(self.dev, 0)
            _sdk.device_close(self.dev)
            self.dev = None
        if self.handle:
            _sdk.damiao_handle_destroy(self.handle)
            self.handle = None

    # default BRS: True when canfd, False otherwise. Override via brs= kwarg.
    brs_default = None  # set in open()

    # --- frame send ---
    def send(self, can_id: int, payload: bytes, brs: bool = None):
        """Send a frame. canfd inherited from open(); brs default = self.brs_default."""
        if brs is None:
            brs = self.brs_default if self.brs_default is not None else self.canfd
        buf = (c_uint8 * len(payload)).from_buffer_copy(payload)
        _sdk.device_channel_send_fast(
            self.dev, 0, can_id, 1,
            False,        # ext (standard 11-bit)
            self.canfd,   # canfd
            brs,
            len(payload),
            buf
        )

    # back-compat alias
    send_classic = send

    # --- DM motor mode offsets (added to CAN_ID per command) ---
    MIT_OFFSET = 0x000
    POS_VEL_OFFSET = 0x100
    VEL_OFFSET = 0x200
    POS_FORCE_OFFSET = 0x300

    # --- DM motor enable/disable (mode-aware) ---
    def enable(self, can_id: int, mode_offset: int = MIT_OFFSET):
        """Send enable (0xFC). mode_offset must match motor's CTRL_MODE register."""
        self.send(can_id + mode_offset, bytes([0xff]*7 + [0xfc]))

    def disable(self, can_id: int, mode_offset: int = MIT_OFFSET):
        self.send(can_id + mode_offset, bytes([0xff]*7 + [0xfd]))

    # --- MIT mode (CTRL_MODE=1) ---
    def control_mit(self, can_id: int, kp: float, kd: float,
                    q: float, dq: float, tau: float):
        self.send(can_id + self.MIT_OFFSET,
                  encode_mit(kp, kd, q, dq, tau))

    # --- Speed/VEL mode (CTRL_MODE=3) ---
    def control_vel(self, can_id: int, vel_radps: float):
        import struct
        self.send(can_id + self.VEL_OFFSET,
                  struct.pack('<f', vel_radps))

    def get_feedback(self, mst_id: int):
        with self._lock:
            return self._latest.get(mst_id)

    # --- DM motor parameter read/write/save ---
    def write_param(self, can_id: int, rid: int, value_u32: int):
        """Write a uint32 parameter to motor's RID register."""
        idl = can_id & 0xff
        idh = (can_id >> 8) & 0xff
        v = value_u32 & 0xffffffff
        payload = bytes([
            idl, idh, 0x55, rid,
            v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, (v >> 24) & 0xff,
        ])
        self.send(0x7FF, payload)

    def read_param(self, can_id: int, rid: int):
        """Trigger a read; reply lands in the rx callback (handled separately)."""
        idl = can_id & 0xff
        idh = (can_id >> 8) & 0xff
        payload = bytes([idl, idh, 0x33, rid, 0, 0, 0, 0])
        self.send(0x7FF, payload)

    def save_params(self, can_id: int):
        """Persist current RAM parameters to motor's flash."""
        idl = can_id & 0xff
        idh = (can_id >> 8) & 0xff
        payload = bytes([idl, idh, 0xAA, 0x01, 0, 0, 0, 0])
        self.send(0x7FF, payload)
