#!/usr/bin/env python3
"""
Upgraded D200 LiDAR live map (Pygame) for Windows (COM6).
- Serial: COM6 @ 230400 (change SERIAL_PORT if needed)
- Parses 47-byte frames from Waveshare D200 (header 0x54, ver_len 0x2C)
- Background thread reads and parses frames, UI thread displays points
- Controls:
    +/- : zoom in/out
    arrow keys : pan
    c : clear trail
    q or ESC or window close : quit

Improvements:
- Reduced trail age & enforced point limit for smooth, near-real-time updates
- Faster serial reading loop (larger read + very small sleep)
- Latest points highlighted for immediate feedback
- Optional simulation mode if serial port can't be opened
Dependencies: pyserial, pygame, numpy (numpy only used for potential future use)
Install: pip install pyserial pygame numpy
"""
import serial
import struct
import threading
import time
import math
import collections
import pygame
import sys
import random

# ----------------- CONFIG -----------------
SERIAL_PORT = "COM6"     # <- set to your COM port
BAUDRATE = 230400
SERIAL_TIMEOUT = 0.01    # seconds for serial read timeout
FRAME_LEN = 47
HEADER = 0x54
POINTS_PER_PACK = 12

WINDOW_W, WINDOW_H = 1000, 800
CENTER_X, CENTER_Y = WINDOW_W // 2, WINDOW_H // 2
INIT_SCALE = 0.18       # pixels per mm (tune to fit)
TRAIL_MAX_AGE = 1.0     # seconds to keep historical points (smaller -> more realtime)
POINT_LIMIT = 7000      # cap number of stored points (avoid huge memory/draw)
LATEST_AGE_BRIGHT = 0.15  # points younger than this are drawn extra bright/large

SIMULATE_ON_FAIL = True  # if serial open fails, generate fake sweeps for testing
# ------------------------------------------

# CRC table from Waveshare / D200 documentation
CRC_TABLE = [
0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
]

def calc_crc8(data_bytes: bytes) -> int:
    crc = 0
    for b in data_bytes:
        crc = CRC_TABLE[(crc ^ b) & 0xff]
    return crc

# Thread-safe storage for parsed points
# Each entry: (timestamp, angle_deg, dist_mm, intensity)
points_deque = collections.deque()
points_lock = threading.Lock()

# Reader thread stops when this is set
reader_stop = threading.Event()

def parse_frame_bytes(frame: bytes):
    """
    frame: full FRAME_LEN bytes (47)
    Returns list of (timestamp, angle_deg, dist_mm, intensity)
    """
    try:
        ver_len = frame[1]
        speed, start_angle_raw = struct.unpack_from("<HH", frame, 2)
        offset = 6
        raw_points = []
        for i in range(POINTS_PER_PACK):
            dist, inten = struct.unpack_from("<HB", frame, offset)
            raw_points.append((dist, inten))
            offset += 3
        end_angle_raw, timestamp = struct.unpack_from("<HH", frame, offset)
    except Exception:
        return []

    start_deg = start_angle_raw / 100.0
    end_deg = end_angle_raw / 100.0
    if end_deg < start_deg:
        end_deg += 360.0
    step = (end_deg - start_deg) / (POINTS_PER_PACK - 1) if POINTS_PER_PACK > 1 else 0.0

    pts = []
    now = time.time()
    for i, (dist, inten) in enumerate(raw_points):
        angle = (start_deg + step * i) % 360.0
        pts.append((now, angle, dist, inten))
    return pts

def serial_reader_thread(serial_port: str, baudrate: int):
    """
    Background thread that synchronizes frames from the serial port and
    appends parsed points to points_deque.
    If serial port open fails and SIMULATE_ON_FAIL is True, starts a simulator thread.
    """
    try:
        ser = serial.Serial(serial_port, baudrate, timeout=SERIAL_TIMEOUT)
    except Exception as e:
        print(f"[reader] Failed to open serial port {serial_port}: {e}")
        if SIMULATE_ON_FAIL:
            print("[reader] Starting simulator instead (SIMULATE_ON_FAIL=True).")
            serial_simulator_thread()  # produces data in same global deque
        else:
            reader_stop.set()
        return

    print(f"[reader] Opened serial {serial_port} @ {baudrate}")
    buffer = bytearray()
    try:
        while not reader_stop.is_set():
            # read a bunch of bytes (faster than tiny reads)
            chunk = ser.read(4096)
            if chunk:
                buffer.extend(chunk)

            # Attempt to find header and complete frame
            while True:
                idx = buffer.find(bytes([HEADER]))
                if idx == -1:
                    # no header, keep last few bytes to handle partial header
                    if len(buffer) > FRAME_LEN * 2:
                        del buffer[:-FRAME_LEN]
                    break
                # Wait until full frame available
                if len(buffer) - idx < FRAME_LEN:
                    # keep partial from idx
                    if idx > 0:
                        del buffer[:idx]
                    break
                # candidate frame
                frame = bytes(buffer[idx: idx + FRAME_LEN])
                crc_calc = calc_crc8(frame[:-1])
                crc_recv = frame[-1]
                if crc_calc == crc_recv:
                    parsed = parse_frame_bytes(frame)
                    if parsed:
                        with points_lock:
                            for p in parsed:
                                points_deque.append(p)
                            # enforce point cap
                            while len(points_deque) > POINT_LIMIT:
                                points_deque.popleft()
                    # remove consumed bytes
                    del buffer[:idx + FRAME_LEN]
                    continue
                else:
                    # CRC mismatch -> drop this header byte and re-search (realign)
                    del buffer[idx]
                    continue

            # tiny sleep so thread yields; fast enough to be responsive
            time.sleep(0.0005)
    except Exception as e:
        print("[reader] Exception:", e)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[reader] stopped")

def serial_simulator_thread():
    """
    Simple simulator that injects rotating points similar to expected D200 frames
    Useful when hardware not connected.
    """
    print("[sim] Simulator started")
    def sim_loop():
        angle_base = 0.0
        while not reader_stop.is_set():
            now = time.time()
            # produce one "frame" with 12 points sweeping ~10 degrees
            start = angle_base
            end = angle_base + 10.0
            step = (end - start) / (POINTS_PER_PACK - 1)
            for i in range(POINTS_PER_PACK):
                angle = (start + step * i) % 360.0
                # simple radial field with noise
                dist = int(1500 + 800 * math.sin(math.radians(angle*3)) + random.uniform(-40, 40))
                inten = int(100 + 80 * math.cos(math.radians(angle*5)) + random.uniform(-10, 10))
                with points_lock:
                    points_deque.append((now, angle, max(20, dist), max(0, min(255, inten))))
                    while len(points_deque) > POINT_LIMIT:
                        points_deque.popleft()
            angle_base = (angle_base + 7.0) % 360.0
            time.sleep(0.02)  # ~50 frames/sec simulation
        print("[sim] Simulator stopped")

    t = threading.Thread(target=sim_loop, daemon=True)
    t.start()

# ---------- Pygame visualization ----------
def run_visualizer():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("D200 LiDAR - live map")
    font = pygame.font.SysFont("Arial", 16)
    clock = pygame.time.Clock()

    scale = INIT_SCALE
    offset_x, offset_y = 0, 0  # pan offsets in pixels
    running = True

    while running and not reader_stop.is_set():
        # --- Input events
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
                reader_stop.set()
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                    reader_stop.set()
                elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    scale *= 1.12
                elif ev.key in (pygame.K_MINUS, pygame.K_UNDERSCORE):
                    scale /= 1.12
                elif ev.key == pygame.K_c:
                    with points_lock:
                        points_deque.clear()
                elif ev.key == pygame.K_UP:
                    offset_y += 40
                elif ev.key == pygame.K_DOWN:
                    offset_y -= 40
                elif ev.key == pygame.K_LEFT:
                    offset_x += 40
                elif ev.key == pygame.K_RIGHT:
                    offset_x -= 40

        # --- Draw background
        screen.fill((10, 10, 10))
        max_radius_px = min(WINDOW_W, WINDOW_H) // 2
        # concentric rings every 500 mm
        r_mm = 500
        while True:
            r_px = int(r_mm * scale)
            if r_px > max_radius_px:
                break
            pygame.draw.circle(screen, (25, 25, 25), (CENTER_X + offset_x, CENTER_Y + offset_y), r_px, 1)
            r_mm += 500

        # axes
        pygame.draw.line(screen, (40,40,40), (CENTER_X + offset_x - max_radius_px, CENTER_Y + offset_y),
                         (CENTER_X + offset_x + max_radius_px, CENTER_Y + offset_y), 1)
        pygame.draw.line(screen, (40,40,40), (CENTER_X + offset_x, CENTER_Y + offset_y - max_radius_px),
                         (CENTER_X + offset_x, CENTER_Y + offset_y + max_radius_px), 1)

        # --- Gather points for drawing (copy under lock)
        now = time.time()
        with points_lock:
            # purge truly old points (older than TRAIL_MAX_AGE * 2) to prevent unlimited growth
            while points_deque and (now - points_deque[0][0]) > (TRAIL_MAX_AGE * 2):
                points_deque.popleft()
            # enforce cap again
            while len(points_deque) > POINT_LIMIT:
                points_deque.popleft()
            pts_to_draw = list(points_deque)

        # --- Draw points
        # draw older/faded points first, then latest points on top
        for ts, angle_deg, dist_mm, inten in pts_to_draw:
            age = now - ts
            if age > TRAIL_MAX_AGE:
                continue
            if dist_mm == 0 or dist_mm > 12000:
                continue
            angle_rad = math.radians(angle_deg)
            x = CENTER_X + offset_x + int(math.cos(angle_rad) * dist_mm * scale)
            y = CENTER_Y + offset_y - int(math.sin(angle_rad) * dist_mm * scale)

            I = max(0, min(255, int(inten)))
            # alpha scale 1.0..0.0
            alpha = 1.0 - (age / TRAIL_MAX_AGE)
            alpha = max(0.12, alpha)
            # base color mapping
            r = min(255, int(I * 1.1))
            g = min(255, int(200 - I * 0.35))
            b = min(255, int(60 + I * 0.12))
            # apply alpha
            col = (max(0, min(255, int(r * alpha))),
                   max(0, min(255, int(g * alpha))),
                   max(0, min(255, int(b * alpha))))
            # larger and brighter for newest points
            if (now - ts) <= LATEST_AGE_BRIGHT:
                pygame.draw.circle(screen, col, (x, y), 3)
            else:
                pygame.draw.circle(screen, col, (x, y), 1)

        # --- HUD
        fps = int(clock.get_fps() or 0)
        with points_lock:
            pt_count = len(points_deque)
        hud_lines = [
            f"Port: {SERIAL_PORT} @ {BAUDRATE}",
            f"FPS: {fps}  Points stored: {pt_count}",
            f"Zoom: {scale:.3f} px/mm   Pan: ({offset_x},{offset_y})",
            "Controls: +/- zoom  arrows pan  c clear  q/ESC quit"
        ]
        y = 6
        for line in hud_lines:
            surf = font.render(line, True, (220,220,220))
            screen.blit(surf, (8, y))
            y += 18

        pygame.display.flip()
        # attempt high refresh, but not to starve CPU
        clock.tick(120)

    pygame.quit()

def main():
    # start reader thread
    reader = threading.Thread(target=serial_reader_thread, args=(SERIAL_PORT, BAUDRATE), daemon=True)
    reader.start()

    # start visualizer (main thread)
    try:
        run_visualizer()
    except KeyboardInterrupt:
        reader_stop.set()
    finally:
        reader_stop.set()
        print("Shutting down, waiting a moment for threads...")
        time.sleep(0.25)

if __name__ == "__main__":
    main()



##chnge the direction of lidar its upside down 
##everything is blur show points in real time .
