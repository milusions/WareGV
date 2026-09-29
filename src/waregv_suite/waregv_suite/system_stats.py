"""Lightweight CPU + RAM sampler.

Uses psutil if available (fast, accurate). Falls back to /proc parsing
on Linux so the Pi 4 doesn't need an extra dependency.
"""
import os
import threading
import time
from typing import Any, Dict, Optional

try:
    import psutil
    _HAVE_PSUTIL = True
except ImportError:
    _HAVE_PSUTIL = False


class SystemStatsSampler:
    """Samples CPU and RAM every `interval` seconds in a daemon thread."""

    _last_cpu: Optional[tuple] = None

    def __init__(self, interval: float = 1.0):
        self.interval = interval
        self._latest: Dict[str, Any] = {
            "ok": False,
            "cpu_percent": 0.0,
            "cpu_count": os.cpu_count() or 1,
            "ram_percent": 0.0,
            "ram_used_mb": 0.0,
            "ram_total_mb": 0.0,
            "load_avg": [0.0, 0.0, 0.0],
            "temperature_c": None,
            "uptime_s": 0.0,
            "source": "psutil" if _HAVE_PSUTIL else "proc",
            "timestamp": time.time(),
        }
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # sampling backends
    # ------------------------------------------------------------------
    def _sample_psutil(self) -> Dict[str, Any]:
        vm = psutil.virtual_memory()
        try:
            load = list(os.getloadavg())
        except (OSError, AttributeError):
            load = [0.0, 0.0, 0.0]

        temp = None
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                for key in ("cpu_thermal", "coretemp", "k10temp", "acpitz"):
                    if key in temps and temps[key]:
                        temp = float(temps[key][0].current)
                        break
                if temp is None:
                    first = next(iter(temps.values()))
                    if first:
                        temp = float(first[0].current)
        except Exception:
            temp = None

        return {
            "cpu_percent": float(psutil.cpu_percent(interval=None)),
            "cpu_count": psutil.cpu_count(logical=True) or 1,
            "ram_percent": float(vm.percent),
            "ram_used_mb": round(vm.used / (1024 * 1024), 1),
            "ram_total_mb": round(vm.total / (1024 * 1024), 1),
            "load_avg": load,
            "temperature_c": temp,
        }

    def _proc_cpu_percent(self) -> float:
        try:
            with open("/proc/stat", "r") as f:
                first = f.readline()
            parts = first.split()
            values = [int(x) for x in parts[1:9]]
            idle = values[3] + values[4]
            total = sum(values)
        except (OSError, ValueError, IndexError):
            return 0.0

        if self._last_cpu is None:
            self._last_cpu = (total, idle)
            return 0.0

        last_total, last_idle = self._last_cpu
        self._last_cpu = (total, idle)

        d_total = total - last_total
        d_idle = idle - last_idle
        if d_total <= 0:
            return 0.0
        return round((1.0 - d_idle / d_total) * 100.0, 1)

    def _sample_proc(self) -> Dict[str, Any]:
        total_kb = available_kb = 0
        try:
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    if line.startswith("MemTotal:"):
                        total_kb = int(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        available_kb = int(line.split()[1])
                    if total_kb and available_kb:
                        break
        except OSError:
            pass

        used_kb = max(total_kb - available_kb, 0)
        ram_percent = (used_kb / total_kb * 100.0) if total_kb else 0.0

        cpu_percent = self._proc_cpu_percent()

        try:
            load = list(os.getloadavg())
        except (OSError, AttributeError):
            load = [0.0, 0.0, 0.0]

        temp = None
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            temp = None

        return {
            "cpu_percent": cpu_percent,
            "cpu_count": os.cpu_count() or 1,
            "ram_percent": round(ram_percent, 1),
            "ram_used_mb": round(used_kb / 1024.0, 1),
            "ram_total_mb": round(total_kb / 1024.0, 1),
            "load_avg": load,
            "temperature_c": temp,
        }

    # ------------------------------------------------------------------
    # thread
    # ------------------------------------------------------------------
    def _run(self):
        if _HAVE_PSUTIL:
            psutil.cpu_percent(interval=None)
        else:
            self._proc_cpu_percent()

        while not self._stop.is_set():
            try:
                if _HAVE_PSUTIL:
                    data = self._sample_psutil()
                else:
                    data = self._sample_proc()
                data["ok"] = True
                data["timestamp"] = time.time()
                data["source"] = "psutil" if _HAVE_PSUTIL else "proc"
                try:
                    data["uptime_s"] = round(time.monotonic(), 1)
                except Exception:
                    data["uptime_s"] = 0.0
                with self._lock:
                    self._latest = data
            except Exception as exc:
                with self._lock:
                    self._latest = {
                        **self._latest,
                        "ok": False,
                        "error": str(exc),
                        "timestamp": time.time(),
                    }
            self._stop.wait(self.interval)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)