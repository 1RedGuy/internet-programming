#!/usr/bin/env python3
"""Final-render queue: renders scenes one after another, each as N parallel
shard processes (every N-th frame, T threads each) so one process's serial
per-frame overhead overlaps another's path tracing; then composites the
overlay and encodes video/<scene>_final.mp4.  Fully resumable (re-run the
same command after a crash or reboot).

    setsid nohup python3 tools/render_finals.py s02 s03 --shards 2 --threads 2 --nice 10 \
        > out/render_finals.log 2>&1 &
"""
import argparse
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def log(msg):
    print(time.strftime("%Y-%m-%d %H:%M:%S"), msg, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", nargs="+")
    ap.add_argument("--shards", type=int, default=2)
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--nice", type=int, default=10)
    ap.add_argument("--quality", default="final")
    ap.add_argument("--device", default="cpu", help="cpu | metal (Apple Silicon) | optix | cuda | hip | oneapi")
    a = ap.parse_args()
    env = dict(os.environ, EGL_PLATFORM="surfaceless")
    for sc in a.scenes:
        t0 = time.time()
        log(f"== {sc}: rendering with {a.shards} shards x {a.threads} threads")
        procs = []
        for i in range(a.shards):
            cmd = ["nice", "-n", str(a.nice), sys.executable, os.path.join(ROOT, "tools/render.py"), sc,
                   "--quality", a.quality, "--shard", f"{i}/{a.shards}", "--threads", str(a.threads),
                   "--device", a.device]
            if i > 0:
                cmd.append("--no-meta")
                time.sleep(20)  # let shard 0 write the label/HUD metadata first
            lf = open(os.path.join(ROOT, "out", f"render_{sc}_{a.quality}_shard{i}.log"), "a")
            procs.append(subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=lf, stderr=subprocess.STDOUT))
        codes = [p.wait() for p in procs]
        log(f"   {sc}: shards finished with {codes} in {(time.time() - t0) / 3600:.2f} h")
        if any(codes):
            log(f"   {sc}: a shard failed - re-run to resume; skipping overlay/encode")
            continue
        cmd = ["nice", "-n", str(a.nice), sys.executable, os.path.join(ROOT, "tools/render.py"), sc,
               "--quality", a.quality, "--no-render", "--workers", "3"]
        r = subprocess.run(cmd, cwd=ROOT, env=env)
        log(f"   {sc}: overlay+encode exit {r.returncode}; total {(time.time() - t0) / 3600:.2f} h")
    log("queue done")


if __name__ == "__main__":
    main()
