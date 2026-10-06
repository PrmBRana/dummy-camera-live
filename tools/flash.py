#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flash prebuilt STM32WL55 binaries. Works on Linux / Windows / macOS.
Does not build NuttX. Does not use /home/prem paths.

  python3 tools/flash.py satellite   # M4 nuttx.bin + M0+ satellite.bin (JC1)
  python3 tools/flash.py gs          # M0+ gs_m0plus.bin (JC2)

Install STM32CubeProgrammer (STM32_Programmer_CLI) first.
Plug only the board you are flashing. Two Nucleos at once will flash the wrong one.
"""

from __future__ import print_function

import argparse
import os
import shutil
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

NUTTX_BIN = os.path.join(ROOT, "nuttx", "nuttx.bin")
SAT_BIN = os.path.join(ROOT, "Satellite_M0+", "Com_sat", "satellite.bin")
GS_BIN = os.path.join(ROOT, "Ground_Station", "gs_m0plus.bin")

M4_ADDR = "0x08000000"
M0_ADDR = "0x08032000"
C2BOOT_REG = "0x5800040C"
C2BOOT_VAL = "0x00008000"


def die(msg):
    print("ERROR:", msg, file=sys.stderr)
    sys.exit(1)


def find_programmer():
    env = os.environ.get("STM32_PROG")
    if env and os.path.isfile(env):
        return env
    which = shutil.which("STM32_Programmer_CLI") or shutil.which(
        "STM32_Programmer_CLI.exe")
    if which:
        return which

    home = os.path.expanduser("~")
    candidates = []
    if sys.platform == "win32":
        pf = os.environ.get("ProgramFiles", r"C:\Program Files")
        pf86 = os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")
        candidates += [
            os.path.join(pf, "STMicroelectronics", "STM32Cube",
                         "STM32CubeProgrammer", "bin", "STM32_Programmer_CLI.exe"),
            os.path.join(pf86, "STMicroelectronics", "STM32Cube",
                         "STM32CubeProgrammer", "bin", "STM32_Programmer_CLI.exe"),
        ]
    else:
        candidates += [
            os.path.join(home, "STMicroelectronics", "STM32Cube",
                         "STM32CubeProgrammer", "bin", "STM32_Programmer_CLI"),
        ]
        try:
            import glob
            candidates += glob.glob(
                "/home/*/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI")
            candidates += glob.glob(
                "/opt/st/stm32cubeide_*/plugins/"
                "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin/STM32_Programmer_CLI")
            candidates += glob.glob(
                "/opt/st/stm32cubeprogrammer_*/bin/STM32_Programmer_CLI")
        except Exception:
            pass

    for path in candidates:
        if path and os.path.isfile(path) and os.access(path, os.X_OK if sys.platform != "win32" else os.F_OK):
            return path
    return None


def run_prog(prog, args):
    cmd = [prog] + args
    print("  $", " ".join(cmd))
    p = subprocess.run(cmd)
    return p.returncode == 0


def flash_bin(prog, path, addr):
    if not os.path.isfile(path):
        die("missing binary: %s\nClone -b working-firmware (prebuilt images are on that branch)." % path)
    print("Flashing %s -> %s" % (path, addr))
    for extra in (
        ["-c", "port=SWD", "mode=UR", "-w", path, addr, "-v", "-hardRst"],
        ["-c", "port=SWD", "mode=HOTPLUG", "-w", path, addr, "-v", "-hardRst"],
        ["-c", "port=SWD", "-w", path, addr, "-v", "-hardRst"],
    ):
        if run_prog(prog, extra):
            return
    die("STM32_Programmer_CLI failed to flash %s. Unplug the other Nucleo and retry." % path)


def poke_c2boot(prog):
    print("Starting CPU2 (C2BOOT)...")
    run_prog(prog, ["-c", "port=SWD", "-w32", C2BOOT_REG, C2BOOT_VAL])


def confirm(board):
    print("")
    print("=" * 64)
    print(" Plug ONLY the %s Nucleo ST-LINK USB cable." % board)
    print(" Unplug the other board. Two ST-LINKs will flash the wrong MCU.")
    print("=" * 64)
    if sys.stdin.isatty():
        try:
            input("Press Enter to flash %s..." % board)
        except EOFError:
            pass


def flash_satellite(prog):
    confirm("SATELLITE (JC1)")
    flash_bin(prog, NUTTX_BIN, M4_ADDR)
    flash_bin(prog, SAT_BIN, M0_ADDR)
    poke_c2boot(prog)
    print("")
    print("Satellite flashed.")
    print("  Open SAT UART 115200 and confirm:")
    print("    FW-ID: SAT-G3RUH-UPLINK7")
    print("    [BOOT] obc_main auto-started ... ADC1+ADC2+IMU")
    print("    CW telemetry source: LIVE (M4 ADC1/ADC2/IMU)")
    print("  If you see DEFAULT (no M4 data yet), M4 did not start OBC — reflash satellite.")


def flash_gs(prog):
    confirm("GROUND STATION (JC2)")
    flash_bin(prog, GS_BIN, M0_ADDR)
    poke_c2boot(prog)
    print("")
    print("Ground Station flashed.")
    print("  Open GS UART 115200 and confirm:")
    print("    FW-ID: GS-G3RUH-20261006 | RFO_HP +22 dBm")
    print("  Then:  python3 Ground_Station/gs_communicator.py")
    print("  Connect that same GS COM port (not the satellite COM).")


def main():
    ap = argparse.ArgumentParser(
        description="Flash prebuilt S2S-2 satellite / ground-station images")
    ap.add_argument(
        "target",
        choices=("satellite", "sat", "gs", "ground"),
        help="satellite = M4+M0+ on JC1,  gs = GS M0+ on JC2")
    args = ap.parse_args()

    prog = find_programmer()
    if not prog:
        die(
            "STM32_Programmer_CLI not found.\n"
            "Install STM32CubeProgrammer and add it to PATH, or set STM32_PROG.\n"
            "https://www.st.com/en/development-tools/stm32cubeprog.html")
    print("Using programmer:", prog)

    if args.target in ("satellite", "sat"):
        flash_satellite(prog)
    else:
        flash_gs(prog)


if __name__ == "__main__":
    main()
