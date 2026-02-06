#!/usr/bin/env python3
import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional

import serial


@dataclass
class Telemetry:
    r_sign: str = "p"
    r_vel: float = 0.0
    l_sign: str = "p"
    l_vel: float = 0.0
    battery: Optional[float] = None
    bumper: Optional[int] = None


def parse_telemetry(line: str) -> Optional[Telemetry]:
    # Expected format: r[p/n]X.XX,l[p/n]X.XX,vpYY.YY,bpZ,
    parts = [p for p in line.strip().split(",") if p]
    if not parts:
        return None
    t = Telemetry()
    try:
        for p in parts:
            if p.startswith("r") and len(p) > 2:
                t.r_sign = p[1]
                t.r_vel = float(p[2:])
            elif p.startswith("l") and len(p) > 2:
                t.l_sign = p[1]
                t.l_vel = float(p[2:])
            elif p.startswith("v") and len(p) > 2:
                t.battery = float(p[2:])
            elif p.startswith("b") and len(p) > 2:
                t.bumper = int(float(p[2:]))
    except ValueError:
        return None
    return t


def send_cmd(ser: serial.Serial, cmd: str) -> None:
    if not cmd.endswith(","):
        cmd += ","
    ser.write(cmd.encode("ascii"))


def main() -> int:
    parser = argparse.ArgumentParser(description="AlphaBot PID tuning CLI")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--rate", type=float, default=5.0, help="Telemetry display rate (Hz)")
    parser.add_argument(
        "--velocity",
        default="",
        help="Initial velocity command(s) to send, e.g. 'rp2.00,lp2.00,'",
    )
    parser.add_argument(
        "--repeat-rate",
        type=float,
        default=0.0,
        help="Resend --velocity at this rate (Hz). 0 disables keepalive.",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for telemetry before sending velocity (0 disables wait).",
    )
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Failed to open port: {exc}", file=sys.stderr)
        return 1

    print("Connected. Commands:")
    print("  r<vel>, l<vel>, k<Kp>, i<Ki>, d<Kd>, m<min_pwm>, f<ff_pwm>, s<low_speed_thresh>")
    print("  Example: k70, i20.5, d0.1, m30, f15, s0.06")
    print("  Type 'quit' to exit.")

    last_send = 0.0
    if args.velocity and args.connect_timeout > 0:
        print("Waiting for telemetry...")
        start_wait = time.time()
        while time.time() - start_wait < args.connect_timeout:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if parse_telemetry(line):
                break

    if args.velocity:
        send_cmd(ser, args.velocity)
        last_send = time.time()

    last_print = 0.0
    try:
        while True:
            # Read incoming telemetry
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line:
                t = parse_telemetry(line)
                if t and (time.time() - last_print) >= (1.0 / args.rate):
                    last_print = time.time()
                    r = f"{t.r_sign}{t.r_vel:.2f}"
                    l = f"{t.l_sign}{t.l_vel:.2f}"
                    v = f"{t.battery:.1f}%" if t.battery is not None else "?"
                    b = f"{t.bumper}" if t.bumper is not None else "?"
                    print(f"R:{r}  L:{l}  V:{v}  B:{b}")

            # Optional keepalive: resend velocity at a fixed rate
            if args.velocity and args.repeat_rate > 0:
                now = time.time()
                if now - last_send >= (1.0 / args.repeat_rate):
                    send_cmd(ser, args.velocity)
                    last_send = now

            # Non-blocking user input
            if sys.stdin in select_inputs():
                cmd = sys.stdin.readline().strip()
                if not cmd:
                    continue
                if cmd.lower() in {"quit", "exit"}:
                    break
                send_cmd(ser, cmd)
    finally:
        ser.close()

    return 0


def select_inputs():
    import select
    return select.select([sys.stdin], [], [], 0)[0]


if __name__ == "__main__":
    raise SystemExit(main())
