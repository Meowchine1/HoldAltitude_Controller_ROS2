#!/usr/bin/env python3

import subprocess
import signal
import os
import sys
import time

processes = []
  
def terminate_processes(signal_received=None, frame=None):
    print("\n[INFO] Завершаем все процессы...")
    for p in processes:
        try:
            p.terminate()
            time.sleep(1)
            p.kill()
        except Exception:
            pass
 
    subprocess.run(["pkill", "-f", "px4"])
    subprocess.run(["pkill", "-9", "ruby"]) 
    print("[INFO] Все процессы завершены.")
    sys.exit(0)

signal.signal(signal.SIGINT, terminate_processes)

p1 = subprocess.Popen(
    ["MicroXRCEAgent", "udp4", "-p", "8888"],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)
processes.append(p1)

p2 = subprocess.Popen(
    ["gnome-terminal", "--", "bash", "-c", "cd ~/PX4-Autopilot && make px4_sitl gz_x500; exec bash"],
    preexec_fn=os.setsid
)
processes.append(p2)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    terminate_processes()
