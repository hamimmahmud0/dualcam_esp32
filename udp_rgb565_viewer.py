#!/usr/bin/env python3
import argparse
import math
import signal
import socket
import struct
import sys

try:
    import numpy as np
except Exception as exc:  # pragma: no cover
    print(f"numpy is required: {exc}")
    sys.exit(2)

try:
    import cv2
except Exception as exc:  # pragma: no cover
    print(f"opencv-python is required: {exc}")
    sys.exit(2)

UDP_PAYLOAD_MAX = 1472
HEADER_STRUCT = struct.Struct("<IHHHH")
HEADER_SIZE = HEADER_STRUCT.size
CHUNK_SIZE = UDP_PAYLOAD_MAX - HEADER_SIZE

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480


class FrameAssembler:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.frame_size = width * height * 2
        self.expected_packets = math.ceil(self.frame_size / CHUNK_SIZE)
        self.current_frame_id = None
        self.last_complete_id = None
        self.buffer = bytearray(self.frame_size)
        self.received = set()
        self.packet_count = self.expected_packets

    def reset(self, frame_id: int, packet_count: int):
        self.current_frame_id = frame_id
        self.packet_count = packet_count or self.expected_packets
        self.received.clear()
        self.buffer[:] = b"\x00" * self.frame_size

    def add_packet(self, frame_id: int, packet_index: int, packet_count: int, payload: bytes):
        if self.last_complete_id is not None and frame_id <= self.last_complete_id:
            return None

        finished = None
        if self.current_frame_id is None or frame_id != self.current_frame_id:
            finished = self.finalize()
            self.reset(frame_id, packet_count)

        if packet_index in self.received:
            return finished

        offset = packet_index * CHUNK_SIZE
        if offset >= self.frame_size:
            return finished

        end = min(offset + len(payload), self.frame_size)
        self.buffer[offset:end] = payload[: end - offset]
        self.received.add(packet_index)

        if len(self.received) >= self.packet_count:
            return self.finalize()
        return finished

    def finalize(self):
        if self.current_frame_id is None:
            return None
        loss_packets = max(self.packet_count - len(self.received), 0)
        loss_pct = (loss_packets / self.packet_count) * 100.0 if self.packet_count else 0.0
        frame_id = self.current_frame_id
        self.last_complete_id = frame_id
        self.current_frame_id = None
        return frame_id, loss_pct, bytes(self.buffer)


def rgb565_to_bgr(frame_bytes: bytes, width: int, height: int) -> np.ndarray:
    arr = np.frombuffer(frame_bytes, dtype="<u2", count=width * height)
    r = (arr >> 11) & 0x1F
    g = (arr >> 5) & 0x3F
    b = arr & 0x1F
    r = (r * 255) // 31
    g = (g * 255) // 63
    b = (b * 255) // 31
    bgr = np.stack([b, g, r], axis=-1).astype(np.uint8)
    return bgr.reshape((height, width, 3))


def send_command(sock: socket.socket, target: tuple, cmd: str) -> None:
    sock.sendto(cmd.encode("ascii"), target)


def main() -> int:
    parser = argparse.ArgumentParser(description="View RGB565 VGA frames streamed over UDP.")
    parser.add_argument("--host", default="cam-calib.local", help="Device hostname/IP")
    parser.add_argument("--cmd-port", type=int, default=12500, help="Command port (default: 55)")
    parser.add_argument("--stream-port", type=int, default=12501, help="Stream port (default: 81)")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    args = parser.parse_args()

    device_ip = socket.gethostbyname(args.host)
    target = (device_ip, args.cmd_port)

    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl_sock.settimeout(1.0)

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind(("", args.stream_port))
    recv_sock.settimeout(0.5)

    running = True

    def handle_signal(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    send_command(ctrl_sock, target, "START")
    try:
        data, _ = ctrl_sock.recvfrom(64)
        if data.strip() != b"OK":
            print("Unexpected response:", data)
    except socket.timeout:
        pass

    assembler = FrameAssembler(args.width, args.height)
    window_name = "RGB565 UDP"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while running:
        try:
            packet, _ = recv_sock.recvfrom(2048)
        except socket.timeout:
            continue
        if len(packet) < HEADER_SIZE:
            continue

        frame_id, packet_index, packet_count, payload_len, _reserved = HEADER_STRUCT.unpack_from(packet)
        if payload_len == 0:
            continue
        payload = packet[HEADER_SIZE:HEADER_SIZE + payload_len]
        if len(payload) < payload_len:
            continue

        result = assembler.add_packet(frame_id, packet_index, packet_count, payload)
        if result is None:
            continue

        frame_id, loss_pct, frame_bytes = result
        bgr = rgb565_to_bgr(frame_bytes, args.width, args.height)

        text = f"frame {frame_id} loss {loss_pct:.1f}%"
        cv2.putText(bgr, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow(window_name, bgr)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    send_command(ctrl_sock, target, "STOP")
    recv_sock.close()
    ctrl_sock.close()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
