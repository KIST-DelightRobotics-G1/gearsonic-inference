#!/usr/bin/env python3
"""Read logs/latest.trace (StateTrace) into numpy.

    from read_trace import load
    t = load("logs/latest.trace")
    t["q_3"]              # one column as a 1-D array
    t.cols("q")           # all q_* columns as an (N, 29) array
    t.time                # seconds since the trace started

CLI:
    python3 tools/read_trace.py logs/latest.trace                  # summary
    python3 tools/read_trace.py logs/latest.trace --plot q_3 q_target_3
    python3 tools/read_trace.py logs/latest.trace --csv out.csv --cols q_26 dq_26 fault_26 --from 120 --to 135
        # --cols: column names, or a group prefix (q, dq, token, smpl_j, ...) for all of it;
        #         omit to export every column. --from/--to: seconds since trace start.
"""
import argparse
import re
import sys

import numpy as np


class Trace:
    def __init__(self, names, data):
        self.names = names
        self.index = {n: i for i, n in enumerate(names)}
        self.data = data

    def __getitem__(self, name):
        return self.data[:, self.index[name]]

    def cols(self, prefix):
        """All columns named <prefix>_<k>, ordered by k, as (N, K)."""
        pat = re.compile(rf"^{re.escape(prefix)}_(\d+)$")
        ks = sorted((int(m.group(1)), n) for n in self.names if (m := pat.match(n)))
        return self.data[:, [self.index[n] for _, n in ks]]

    @property
    def time(self):
        return self["t_s"]

    def __len__(self):
        return self.data.shape[0]

    def select(self, cols=None, t_from=None, t_to=None):
        """(names, rows) for the given columns (names or group prefixes) and
        time window in seconds. Always includes t_s first."""
        names = []
        for c in cols or self.names:
            if c in self.index:
                names.append(c)
            else:
                grp = [n for n in self.names if re.match(rf"^{re.escape(c)}_\d+$", n)]
                if not grp:
                    raise KeyError(f"unknown column or group: {c}")
                names.extend(sorted(grp, key=lambda n: int(n.rsplit("_", 1)[1])))
        if "t_s" in names:
            names.remove("t_s")
        names.insert(0, "t_s")
        mask = np.ones(len(self), dtype=bool)
        if t_from is not None:
            mask &= self.time >= t_from
        if t_to is not None:
            mask &= self.time <= t_to
        return names, self.data[mask][:, [self.index[n] for n in names]]

    def to_csv(self, path, cols=None, t_from=None, t_to=None):
        names, rows = self.select(cols, t_from, t_to)
        np.savetxt(path, rows, delimiter=",", fmt="%.6g", header=",".join(names), comments="")
        return len(rows), len(names)


def load(path):
    with open(path, "rb") as f:
        head = b""
        while not head.endswith(b"END\n"):
            chunk = f.readline()
            if not chunk:
                raise ValueError("no END marker in header")
            head += chunk
        lines = head.decode().splitlines()
        if lines[0] != "STATETRACE v1":
            raise ValueError(f"unexpected magic: {lines[0]}")
        ncol = int(lines[1].split("=")[1])
        names = lines[3].split(",")
        if len(names) != ncol:
            raise ValueError(f"header says {ncol} columns, lists {len(names)}")
        raw = np.frombuffer(f.read(), dtype="<f4")
    rows = raw.size // ncol
    return Trace(names, raw[: rows * ncol].reshape(rows, ncol))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="logs/latest.trace")
    ap.add_argument("--plot", nargs="*", help="column names to plot against time")
    ap.add_argument("--csv", help="write the selected columns/window as CSV to this file")
    ap.add_argument("--cols", nargs="*", help="columns or group prefixes for --csv (default: all)")
    ap.add_argument("--from", dest="t_from", type=float, help="window start, seconds since trace start")
    ap.add_argument("--to", dest="t_to", type=float, help="window end, seconds since trace start")
    a = ap.parse_args()
    t = load(a.path)
    if a.csv:
        n, k = t.to_csv(a.csv, a.cols, a.t_from, a.t_to)
        print(f"wrote {a.csv}: {n} rows x {k} columns")
        return
    dur = t.time[-1] - t.time[0] if len(t) else 0.0
    print(f"{a.path}: {len(t)} ticks, {dur:.1f} s, {len(t.names)} columns, "
          f"dropped {int(t['trace_dropped'][-1]) if len(t) else 0}")
    if len(t):
        print(f"tick timing us  enc/dec/total (max): {t['timing_enc_us'].max():.0f} / "
              f"{t['timing_dec_us'].max():.0f} / {t['timing_total_us'].max():.0f}")
        fl = t.cols("health")
        if fl.any():
            hit = np.argwhere(fl != 0)
            print(f"health flags set on {len(set(hit[:, 1]))} motor(s), first at t={t.time[hit[0, 0]]:.2f}s")
    if a.plot:
        import matplotlib.pyplot as plt
        for name in a.plot:
            plt.plot(t.time, t[name], label=name)
        plt.xlabel("s"); plt.legend(); plt.grid(True); plt.show()


if __name__ == "__main__":
    sys.exit(main())
