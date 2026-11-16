#!/usr/bin/env python3
"""Camera setup window: every connected camera with its live picture and a name drop-down (wrist_left, wrist_right,
head, knife, or "(not used)"). Save overwrites config/real_cameras.json, the file the recorder reads
(observation.images.<name>); the previous file is kept as config/real_cameras.json.bak.

  recording/run_camera_setup.sh [--config PATH] [--all-interfaces]

- Picking a name that another camera already has swaps the two names (one click fixes swapped wrist cameras).
- A camera is stored the way real_cameras.py finds it again: Orbbec by serial + colour interface (any port), the ARC
  wrist cameras by USB port (no serial) - so after Save keep each wrist camera in the same USB port.
- Picture settings (MJPG / size / fps / rotate) follow the physical camera: they are taken from the config entry that
  pointed at it before, else from an entry for the same camera model, else the first format that gives a picture.
- Names in the config whose camera is not connected now are kept unchanged (listed at the bottom of the window).
- Only one colour stream per USB camera is opened (the configured one, else the lowest interface): opening the
  Gemini 305's second colour interface (6) once left its main stream (4) without frames until replugged.
  --all-interfaces shows every colour node anyway.
- Close the recorder first: a camera that is in use cannot show a picture.
"""
import argparse
import base64
import json
import os
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import cv2

import real_cameras as rc

NOT_USED = "(not used)"
DEFAULT_NAMES = ["wrist_left", "wrist_right", "head", "knife"]
# picture settings that belong to the physical camera: they move with it when names are swapped, and the preview
# uses them ("controls" = exposure / anti-flicker etc., see real_cameras.py)
FORMAT_KEYS = ("fourcc", "width", "height", "fps", "decode_fps", "rotate", "controls")
FALLBACK_FORMATS = [  # tried in order when no config entry knows this camera model
    {"fourcc": "MJPG", "width": 640, "height": 480, "fps": 30},
    {"fourcc": "MJPG", "width": 640, "height": 480, "fps": 120, "decode_fps": 60},  # ARC: 640x480 MJPG only at 120
    {"fourcc": "YUYV", "width": 640, "height": 480, "fps": 30},
]
PREVIEW_W, PREVIEW_H = 320, 240
COMMENT = ("Real rig USB cameras -> observation.images.<key> (recording/real_cameras.py). Orbbecs by serial + colour "
           "interface (any port); the ARC wrist cameras have no serial, so by USB port: keep them in the same ports. "
           "ARC 640x480 MJPG exists only at 120 fps: decoded at <= 60 fps. 'controls' = camera settings applied on "
           "every open (run_cameras.sh --controls lists them). Set with recording/run_camera_setup.sh")


# ---------------------------------------------------------------- logic (no Tk: unit-tested)
def entry_matches(spec, node) -> bool:
    """Does config entry `spec` point at video node `node` (same rules as real_cameras.resolve_device)?"""
    if spec.get("device"):
        return spec["device"] == node["dev"]
    if spec.get("serial"):
        return spec["serial"] == node["serial"] and int(spec.get("interface", 0)) == node["interface"]
    if spec.get("usb_port"):
        return (spec["usb_port"] == node["port"] and int(spec.get("interface", 0)) == node["interface"]
                and (not spec.get("usb_id") or spec["usb_id"] == node["id"]))
    return False


def current_names(cfg, nodes) -> dict:
    """{dev: name} of the config entries that point at exactly one of the connected nodes."""
    out = {}
    for name, spec in cfg.items():
        hit = [n for n in nodes if entry_matches(spec, n)]
        if len(hit) == 1 and hit[0]["dev"] not in out:
            out[hit[0]["dev"]] = name
    return out


def find_cameras(cfg, all_interfaces=False, nodes=None) -> list:
    """Colour nodes to show: one per USB camera (the configured interface, else the lowest), unless all_interfaces."""
    nodes = rc.colour_nodes() if nodes is None else nodes
    if all_interfaces:
        return list(nodes)
    configured = {n["dev"] for n in nodes if any(entry_matches(s, n) for s in cfg.values())}
    by_device = {}
    for n in nodes:
        by_device.setdefault(n["port"], []).append(n)
    out = []
    for group in by_device.values():
        pick = [n for n in group if n["dev"] in configured] or sorted(group, key=lambda n: n["interface"])
        out.append(pick[0])
    return sorted(out, key=lambda n: (n["port"], n["interface"]))


def _fmt(spec) -> dict:
    return {k: spec[k] for k in FORMAT_KEYS if k in spec}


def format_candidates(cfg, node) -> list:
    """Picture settings to try for `node`: the entry that points at it, same camera model, then generic ones."""
    out = []
    for name, spec in cfg.items():
        if entry_matches(spec, node):
            out.append(_fmt(spec))
    for spec in cfg.values():
        if (node["serial"] and spec.get("serial") == node["serial"]) or (spec.get("usb_id") == node["id"]):
            out.append(_fmt(spec))
    out += FALLBACK_FORMATS
    unique = []
    for f in out:
        if f and f not in unique:
            unique.append(f)
    return unique


def label_for(name, node, old_spec) -> str:
    """Keep the old label when the name stays on the same camera model, else describe the new camera."""
    if old_spec and old_spec.get("label") and (
            (node["serial"] and old_spec.get("serial") == node["serial"]) or old_spec.get("usb_id") == node["id"]):
        return old_spec["label"]
    return f"{name} ({node['product'] or node['id']})"


def build_config(old_cfg, nodes, chosen, formats) -> dict:
    """New config. chosen = {dev: name or NOT_USED}; formats = {dev: picture settings that worked (or None)}.
    Raises ValueError when one name is given to two cameras."""
    used = [n for n in chosen.values() if n != NOT_USED]
    dup = sorted({n for n in used if used.count(n) > 1})
    if dup:
        raise ValueError(f"name used for more than one camera: {', '.join(dup)}")
    new = {}
    for node in nodes:
        name = chosen.get(node["dev"], NOT_USED)
        if name == NOT_USED:
            continue
        fmt = formats.get(node["dev"]) or format_candidates(old_cfg, node)[0]
        new[name] = {"label": label_for(name, node, old_cfg.get(name)), **rc.node_spec(node), **fmt}
    # names whose camera is not connected now: keep them as they are (a name the user took away is dropped)
    for name, spec in old_cfg.items():
        if name not in new and not any(entry_matches(spec, n) for n in nodes):
            new[name] = spec
    return new


def config_text(cfg) -> str:
    lines = [f'  "_comment": {json.dumps(COMMENT)}']
    lines += [f"  {json.dumps(k)}: {json.dumps(v)}" for k, v in cfg.items()]
    return "{\n" + ",\n".join(lines) + "\n}\n"


def save_config(path, cfg) -> Path:
    """Write cfg to path (old file -> path.bak); atomic replace. Returns the backup path (or None)."""
    path = Path(path)
    backup = None
    if path.exists():
        backup = path.with_name(path.name + ".bak")
        backup.write_bytes(path.read_bytes())
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(config_text(cfg))
    os.replace(tmp, path)
    return backup


# ---------------------------------------------------------------- window
class Tile:
    """One camera: live picture, info, name drop-down."""

    def __init__(self, app, parent, node, name, candidates):
        self.node, self.candidates = node, candidates
        self.cam = self.fmt = self.error = None
        self.photo = None
        self.prev_name = name
        title = f"{node['product'] or node['id']}   port {node['port']}  interface {node['interface']}"
        self.frame = ttk.LabelFrame(parent, text=title, padding=6)
        self.blank = tk.PhotoImage(width=PREVIEW_W, height=PREVIEW_H)  # with an image, the size is in pixels
        self.picture = tk.Label(self.frame, text="opening ...", image=self.blank, bg="#222", fg="#ddd",
                                compound="center")
        self.picture.pack()
        bits = [node["dev"]]
        if node["serial"]:
            bits.append(f"serial {node['serial']}")
        if node.get("speed"):
            bits.append(f"{node['speed']} Mbit/s")
        self.info = ttk.Label(self.frame, text="   ".join(bits) + f"\nin the config now: {name}", foreground="#555",
                              wraplength=PREVIEW_W + 40, justify="left")
        self.info.pack(anchor="w", pady=(4, 2))
        row = ttk.Frame(self.frame)
        row.pack(fill="x")
        ttk.Label(row, text="Name:").pack(side="left")
        self.var = tk.StringVar(value=name)
        self.box = ttk.Combobox(row, textvariable=self.var, values=app.names, state="readonly", width=18)
        self.box.pack(side="left", padx=6)
        self.box.bind("<<ComboboxSelected>>", lambda e: app.on_pick(self))
        self.fmt_label = ttk.Label(row, text="", foreground="#555")
        self.fmt_label.pack(side="left")

    def open(self, stop):
        """Background thread: try the picture settings in order; keep the first that gives frames."""
        err = None
        for fmt in self.candidates:
            if stop.is_set():
                return
            try:
                cam = rc.RealCamera(self.node["port"], self.node["dev"], fmt.get("width", 640),
                                    fmt.get("height", 480), fmt.get("fps", 30), fmt.get("fourcc", "MJPG"),
                                    fmt.get("decode_fps"), fmt.get("rotate", 0), open_timeout=4.0,
                                    controls=fmt.get("controls")).open()
                if stop.is_set():  # the window closed / rescanned meanwhile
                    cam.close()
                    return
                self.fmt = fmt
                self.cam = cam  # last: refresh() uses fmt once cam is set
                return
            except Exception as exc:  # noqa: BLE001 - shown in the tile
                err = exc
        self.error = str(err) if err else "no format to try"

    def refresh(self):
        if self.cam is None:
            if self.error and self.picture.cget("text") != "no picture":
                self.picture.configure(text="no picture", image=self.blank)
                self.info.configure(text=self.info.cget("text") + "\n" + self.error[:160], foreground="#a00")
            return
        rgb, _ = self.cam.latest()
        if rgb is None:
            return
        h, w = rgb.shape[:2]
        s = min(PREVIEW_W / w, PREVIEW_H / h)
        small = cv2.resize(rgb, (max(1, int(w * s)), max(1, int(h * s))), interpolation=cv2.INTER_AREA)
        ok, png = cv2.imencode(".png", cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
        if not ok:
            return
        self.photo = tk.PhotoImage(data=base64.b64encode(png.tobytes()))
        self.picture.configure(image=self.photo, text="")
        f = self.fmt
        probs = self.cam.control_problems
        self.fmt_label.configure(text=f"{f.get('fourcc')} {w}x{h} @ {f.get('fps')}"
                                      + (f" (rotate {f['rotate']})" if f.get("rotate") else "")
                                      + (f" + {len(self.cam.controls)} settings" if self.cam.controls else "")
                                      + (f", {len(probs)} refused" if probs else ""),
                                 foreground="#a00" if probs else "#555")
        if probs and "refused:" not in self.info.cget("text"):
            self.info.configure(text=self.info.cget("text") + "\nrefused: " + "; ".join(probs)[:200])

    def close(self):
        if self.cam is not None:
            self.cam.close()
            self.cam = None


class CameraSetup:
    def __init__(self, root, config_path, all_interfaces=False, find=find_cameras):
        self.root, self.config_path, self.all_interfaces, self.find = root, Path(config_path), all_interfaces, find
        self.tiles, self.stop, self.opener = [], threading.Event(), None
        self.dirty = False
        root.title("Camera setup - real_cameras.json")
        top = ttk.Frame(root, padding=(10, 8))
        top.pack(fill="x")
        ttk.Label(top, text=f"Config: {self.config_path}").pack(side="left")
        ttk.Button(top, text="Close", command=self.close).pack(side="right")
        ttk.Button(top, text="Save", command=self.save).pack(side="right", padx=6)
        ttk.Button(top, text="Rescan", command=self.rescan).pack(side="right")
        self.status = ttk.Label(root, text="", padding=(10, 0))
        self.status.pack(fill="x")
        self.grid = ttk.Frame(root, padding=10)
        self.grid.pack(fill="both", expand=True)
        self.kept = ttk.Label(root, text="", padding=(10, 0, 10, 10), foreground="#555", wraplength=680, justify="left")
        self.kept.pack(fill="x")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.load()
        self.tick()

    def load(self):
        self.cfg = rc.load_config(self.config_path) if self.config_path.exists() else {}
        self.nodes = self.find(self.cfg, self.all_interfaces)
        names_now = current_names(self.cfg, self.nodes)
        self.names = [NOT_USED] + list(dict.fromkeys(DEFAULT_NAMES + list(self.cfg)))
        for w in self.grid.winfo_children():
            w.destroy()
        self.tiles = []
        for i, node in enumerate(self.nodes):
            t = Tile(self, self.grid, node, names_now.get(node["dev"], NOT_USED), format_candidates(self.cfg, node))
            t.frame.grid(row=i // 2, column=i % 2, padx=6, pady=6, sticky="n")
            self.tiles.append(t)
        if not self.nodes:
            ttk.Label(self.grid, text="No cameras found. Plug them in (and close the recorder), then press Rescan.",
                      padding=20).pack()
        away = [k for k, s in self.cfg.items() if not any(entry_matches(s, n) for n in self.nodes)]
        self.kept.configure(text=("Not connected now, kept unchanged on Save: " + ", ".join(away)) if away else "")
        self.dirty = False
        self.set_status(f"{len(self.nodes)} camera(s) found. Pick a name under each picture, then Save.")
        self.stop = threading.Event()
        self.opener = threading.Thread(target=self._open_all, args=(list(self.tiles), self.stop), daemon=True)
        self.opener.start()

    def _open_all(self, tiles, stop):
        for t in tiles:  # one after another: opening them all at once can trip over the USB bandwidth
            if stop.is_set():
                return
            t.open(stop)

    def set_status(self, text, error=False):
        self.status.configure(text=text, foreground="#a00" if error else "#063")

    def tick(self):
        for t in self.tiles:
            t.refresh()
        self.after_id = self.root.after(100, self.tick)

    def on_pick(self, tile):
        new = tile.var.get()
        if new != NOT_USED:
            for other in self.tiles:
                if other is not tile and other.var.get() == new:
                    other.var.set(tile.prev_name)  # swap: the other camera takes this one's old name
                    other.prev_name = tile.prev_name
        tile.prev_name = new
        self.dirty = True
        self.set_status("Unsaved changes - press Save to write the config.")

    def chosen(self):
        return {t.node["dev"]: t.var.get() for t in self.tiles}

    def save(self):
        try:
            new = build_config(self.cfg, self.nodes, self.chosen(), {t.node["dev"]: t.fmt for t in self.tiles})
        except ValueError as exc:
            messagebox.showerror("Camera setup", str(exc), parent=self.root)
            return False
        backup = save_config(self.config_path, new)
        self.cfg = new
        self.dirty = False
        summary = ", ".join(f"{k} = {v.get('usb_port') or v.get('serial') or v.get('device')}" for k, v in new.items())
        self.set_status(f"Saved {self.config_path.name} ({summary})" + (f"; previous file: {backup.name}" if backup else ""))
        print(f"[cam setup] saved {self.config_path}: {summary}", flush=True)
        return True

    def close_cameras(self):
        self.stop.set()
        if self.opener is not None:
            self.opener.join(timeout=6.0)  # a camera being opened right now is closed by the opener itself
        for t in self.tiles:
            t.close()

    def rescan(self):
        if self.dirty and not messagebox.askyesno("Camera setup", "Discard the unsaved names and rescan?",
                                                  parent=self.root):
            return
        self.close_cameras()
        self.load()

    def close(self):
        if self.dirty and not messagebox.askyesno("Camera setup", "Close without saving the new names?",
                                                  parent=self.root):
            return
        self.root.after_cancel(self.after_id)
        self.close_cameras()
        self.root.destroy()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(rc.CONFIG), help="camera config to edit (default config/real_cameras.json)")
    p.add_argument("--all-interfaces", action="store_true",
                   help="show every colour node, also a camera's second colour interface (see the caution above)")
    args = p.parse_args()
    root = tk.Tk()
    CameraSetup(root, args.config, args.all_interfaces)
    root.mainloop()


if __name__ == "__main__":
    main()
