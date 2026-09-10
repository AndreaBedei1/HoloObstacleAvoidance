"""COCKPIT VIDEO + TELEMETRY EXPLORER.

Standalone Windows viewer for BlueROV2 Cockpit recordings.  A recording is a
pair of files with the same basename: one MKV video and one ASS telemetry
subtitle file.  Original files are opened read-only; screenshots and CSV
exports are written only to user-selected new paths.

The video backend is optional.  With python-vlc and the VLC desktop runtime
installed, the MKV is embedded in the Tk GUI and the original ASS can be
loaded as VLC's subtitle track.  Without VLC the pairing, ASS inspection,
telemetry timeline, metadata and CSV export remain available with a clear
installation message instead of a crash.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
import subprocess
import sys
import tkinter as tk
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import pysubs2
    PYSUBS2_IMPORT_ERROR = None
except ImportError as exc:  # Keep the GUI importable for a useful error message.
    pysubs2 = None
    PYSUBS2_IMPORT_ERROR = exc

try:
    import vlc
    VLC_IMPORT_ERROR = None
except Exception as exc:  # Missing python-vlc or the VLC desktop runtime.
    vlc = None
    VLC_IMPORT_ERROR = exc


DEFAULT_VIDEO_ROOT = Path(__file__).resolve().parents[2] / "records" / "video"
VIDEO_EXTENSIONS = {".mkv", ".mp4", ".mov", ".avi", ".wmv"}
MAX_TELEMETRY_REFRESH_MS = 75


def format_ms(value: int) -> str:
    value = max(0, int(value))
    seconds = value // 1000
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}" if hours else f"{minutes:02d}:{seconds:02d}"


def format_bytes(value: int) -> str:
    size = float(value)
    for unit in ("B", "KB", "MB", "GB"):
        if size < 1024.0 or unit == "GB":
            return f"{size:.1f} {unit}" if unit != "B" else f"{int(size)} B"
        size /= 1024.0
    return "--"


def ass_time(value: int) -> str:
    """ASS-style timestamp for CSV/raw display."""
    total_cs = max(0, int(round(value / 10.0)))
    cs = total_cs % 100
    total_seconds = total_cs // 100
    seconds = total_seconds % 60
    minutes = (total_seconds // 60) % 60
    hours = total_seconds // 3600
    return f"{hours}:{minutes:02d}:{seconds:02d}.{cs:02d}"


def strip_ass_markup(text: str) -> str:
    """Remove ASS override tags while preserving readable text and line breaks."""
    readable = re.sub(r"\{[^}]*\}", "", str(text or ""))
    readable = readable.replace("\\N", "\n").replace("\\n", "\n").replace("\\h", " ")
    return readable


@dataclass
class RecordingPair:
    """One basename-matched Cockpit recording, possibly incomplete."""

    stem: str
    video: Optional[Path] = None
    ass: Optional[Path] = None

    @property
    def status(self) -> str:
        if self.video and self.ass:
            return "VIDEO OK · ASS OK"
        if self.video:
            return "VIDEO WITHOUT ASS"
        return "ASS WITHOUT VIDEO"

    @property
    def display_name(self) -> str:
        return self.video.name if self.video else self.ass.name if self.ass else self.stem


def pair_recordings(folder: Path) -> List[RecordingPair]:
    """Find MKV/ASS pairs recursively without changing any files."""
    if not folder.exists():
        return []
    grouped: Dict[Tuple[str, str], RecordingPair] = {}
    for path in sorted(folder.rglob("*"), key=lambda item: str(item).casefold()):
        if not path.is_file():
            continue
        suffix = path.suffix.casefold()
        if suffix not in VIDEO_EXTENSIONS and suffix != ".ass":
            continue
        key = (str(path.parent.resolve()).casefold(), path.stem.casefold())
        pair = grouped.setdefault(key, RecordingPair(stem=path.stem))
        if suffix == ".ass":
            pair.ass = path
        else:
            pair.video = path
    return sorted(grouped.values(), key=lambda item: item.display_name.casefold())


@dataclass
class AssEvent:
    index: int
    start_ms: int
    end_ms: int
    raw_text: str
    readable_text: str
    raw_dialogue: str
    metadata: Dict[str, str] = field(default_factory=dict)
    fields: List[Tuple[str, str]] = field(default_factory=list)
    other_lines: List[str] = field(default_factory=list)

    def is_active(self, timestamp_ms: int) -> bool:
        return self.start_ms <= int(timestamp_ms) < self.end_ms


class AssIndex:
    """In-memory ASS event index; no file reads are needed during playback."""

    def __init__(self, path: Path, events: Sequence[AssEvent]):
        self.path = path
        self.events = sorted(events, key=lambda event: (event.start_ms, event.end_ms, event.index))
        self.starts = [event.start_ms for event in self.events]

    def active_at(self, timestamp_ms: int) -> List[AssEvent]:
        cutoff = bisect.bisect_right(self.starts, int(timestamp_ms))
        return [event for event in self.events[:cutoff] if event.is_active(timestamp_ms)]

    @property
    def start_ms(self) -> int:
        return min((event.start_ms for event in self.events), default=0)

    @property
    def end_ms(self) -> int:
        return max((event.end_ms for event in self.events), default=0)


def _event_raw_line(event, index: int) -> str:
    """Preserve a useful raw Dialogue representation across pysubs2 versions."""
    try:
        value = event.to_string()
        if value:
            return str(value)
    except Exception:
        pass
    layer = getattr(event, "layer", 0)
    style = getattr(event, "style", "Default")
    name = getattr(event, "name", "")
    effect = getattr(event, "effect", "")
    return f"Dialogue: {layer},{ass_time(int(event.start))},{ass_time(int(event.end))},{style},{name},0,0,0,{effect},{event.text}"


def _parse_key_values(readable_text: str) -> Tuple[List[Tuple[str, str]], List[str]]:
    """Parse generic key:value/key=value fields and retain every other line."""
    fields: List[Tuple[str, str]] = []
    other: List[str] = []
    for line in readable_text.splitlines() or [readable_text]:
        line = line.strip()
        if not line:
            continue
        # Cockpit overlays often put several fields on one line separated by |.
        fragments = [part.strip() for part in re.split(r"\s*\|\s*|\t+", line) if part.strip()]
        if not fragments:
            fragments = [line]
        for fragment in fragments:
            match = re.match(r"^([^:=]{1,80}?)\s*[:=]\s*(.+?)\s*$", fragment)
            if match:
                fields.append((match.group(1).strip(), match.group(2).strip()))
            else:
                other.append(fragment)
    return fields, other


def parse_ass_file(path: Path) -> AssIndex:
    """Load and index all ASS events once using pysubs2."""
    if pysubs2 is None:
        raise RuntimeError("pysubs2 non è installato. Esegui: python -m pip install -r requirements_cockpit_video.txt")
    try:
        subtitles = pysubs2.load(str(path), encoding="utf-8")
    except UnicodeDecodeError:
        subtitles = pysubs2.load(str(path), encoding="utf-8-sig")
    events = []
    for index, event in enumerate(subtitles):
        raw_text = str(getattr(event, "text", "") or "")
        readable_text = strip_ass_markup(raw_text)
        fields, other_lines = _parse_key_values(readable_text)
        metadata = {
            "Layer": str(getattr(event, "layer", "")),
            "Style": str(getattr(event, "style", "")),
            "Name": str(getattr(event, "name", "")),
            "Effect": str(getattr(event, "effect", "")),
        }
        events.append(AssEvent(
            index=index,
            start_ms=int(getattr(event, "start", 0)),
            end_ms=int(getattr(event, "end", 0)),
            raw_text=raw_text,
            readable_text=readable_text,
            raw_dialogue=_event_raw_line(event, index),
            metadata=metadata,
            fields=fields,
            other_lines=other_lines,
        ))
    return AssIndex(path, events)


def recording_date_from_name(path: Path) -> str:
    match = re.search(r"\(([^)]*GMT[+-]\d+)\)", path.name)
    return match.group(1) if match else "Non rilevata dal nome"


def ffprobe_metadata(path: Path) -> Dict[str, str]:
    """Read optional media metadata; absence of ffprobe is non-fatal."""
    try:
        result = subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration:stream=codec_name,width,height,avg_frame_rate", "-of", "json", str(path)],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=8,
        )
        document = json.loads(result.stdout)
    except (OSError, subprocess.SubprocessError, ValueError, json.JSONDecodeError):
        return {}
    metadata: Dict[str, str] = {}
    fmt = document.get("format", {})
    if fmt.get("duration"):
        try:
            metadata["duration"] = format_ms(int(float(fmt["duration"]) * 1000))
        except (TypeError, ValueError):
            pass
    streams = document.get("streams", [])
    video = next((stream for stream in streams if stream.get("codec_name") or stream.get("width")), {})
    if video.get("width") and video.get("height"):
        metadata["resolution"] = f"{video['width']}×{video['height']}"
    if video.get("avg_frame_rate") and video["avg_frame_rate"] != "0/0":
        try:
            num, den = video["avg_frame_rate"].split("/")
            metadata["fps"] = f"{float(num) / float(den):.2f}"
        except (ValueError, ZeroDivisionError):
            pass
    if video.get("codec_name"):
        metadata["codec"] = str(video["codec_name"])
    return metadata


def export_telemetry_csv(index: AssIndex, destination: Path) -> None:
    """Export a tabular copy of indexed ASS events; source remains untouched."""
    with destination.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(["event_index", "start_ms", "end_ms", "start_ass", "end_ass", "field", "value", "readable_text", "raw_ass_text"])
        for event in index.events:
            fields = event.fields or [("Other / Raw", "\n".join(event.other_lines) or event.readable_text)]
            for field_name, value in fields:
                writer.writerow([event.index, event.start_ms, event.end_ms, ass_time(event.start_ms), ass_time(event.end_ms), field_name, value, event.readable_text, event.raw_text])


class CockpitVideoViewer(tk.Tk):
    """Standalone Tk/VLC Cockpit explorer."""

    BG = "#07111f"
    PANEL = "#0d1c2d"
    PANEL_2 = "#12263b"
    TEXT = "#e6f0f7"
    MUTED = "#93a9ba"
    ACCENT = "#43d5dc"
    ORANGE = "#ffbf57"

    def __init__(self, folder: Path = DEFAULT_VIDEO_ROOT, use_vlc: bool = True):
        super().__init__()
        self.title("COCKPIT VIDEO + TELEMETRY EXPLORER")
        self.geometry("1560x920")
        self.minsize(1180, 720)
        self.configure(bg=self.BG)
        self.folder = folder
        self.pairs: List[RecordingPair] = []
        self.selected_pair: Optional[RecordingPair] = None
        self.ass_index: Optional[AssIndex] = None
        self.current_ms = 0
        self.duration_ms = 0
        self.player = None
        self.vlc_instance = None
        self.vlc_error: Optional[str] = None
        self.ass_spu_id: Optional[int] = None
        self._timeline_guard = False
        self._pending_play = False
        self._build_style()
        self._build_ui()
        if use_vlc:
            self._init_vlc()
        else:
            self.vlc_error = "Playback VLC disabilitato per il test offline."
        self._scan_folder(folder)
        self.after(MAX_TELEMETRY_REFRESH_MS, self._poll_player)

    def _build_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background=self.BG)
        style.configure("Panel.TFrame", background=self.PANEL)
        style.configure("TLabel", background=self.BG, foreground=self.TEXT, font=("Segoe UI", 10))
        style.configure("Title.TLabel", background=self.BG, foreground=self.TEXT, font=("Segoe UI", 20, "bold"))
        style.configure("Subtitle.TLabel", background=self.BG, foreground=self.MUTED, font=("Segoe UI", 10))
        style.configure("Panel.TLabel", background=self.PANEL, foreground=self.TEXT, font=("Segoe UI", 10))
        style.configure("PanelTitle.TLabel", background=self.PANEL, foreground=self.TEXT, font=("Segoe UI", 12, "bold"))
        style.configure("Accent.TButton", background=self.ACCENT, foreground="#001116", font=("Segoe UI", 10, "bold"), padding=(10, 6))
        style.map("Accent.TButton", background=[("active", "#7cf3ed")])
        style.configure("Treeview", background="#0a1727", fieldbackground="#0a1727", foreground=self.TEXT, rowheight=24)
        style.configure("Treeview.Heading", background=self.PANEL_2, foreground=self.TEXT, font=("Segoe UI", 9, "bold"))
        style.map("Treeview", background=[("selected", "#1d5965")], foreground=[("selected", "#ffffff")])
        style.configure("TScale", background=self.BG, troughcolor=self.PANEL_2)

    def _build_ui(self) -> None:
        header = ttk.Frame(self)
        header.pack(fill="x", padx=20, pady=(16, 8))
        ttk.Label(header, text="COCKPIT VIDEO + TELEMETRY EXPLORER", style="Title.TLabel").pack(anchor="w")
        ttk.Label(header, text="BlueROV2 · coppie MKV + ASS · sorgenti originali sempre in sola lettura", style="Subtitle.TLabel").pack(anchor="w", pady=(2, 0))

        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", padx=20, pady=(0, 10))
        ttk.Button(toolbar, text="Open folder", command=self._choose_folder).pack(side="left")
        ttk.Button(toolbar, text="Previous recording", command=self._previous_recording).pack(side="left", padx=(6, 0))
        ttk.Button(toolbar, text="Next recording", command=self._next_recording).pack(side="left", padx=(6, 0))
        ttk.Button(toolbar, text="Export telemetry CSV", command=self._export_csv).pack(side="left", padx=(18, 0))
        ttk.Button(toolbar, text="Screenshot", command=self._screenshot).pack(side="left", padx=(6, 0))
        self.folder_label = ttk.Label(toolbar, text=str(self.folder), style="Subtitle.TLabel")
        self.folder_label.pack(side="right")

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=20, pady=(0, 16))
        body.columnconfigure(0, weight=0, minsize=340)
        body.columnconfigure(1, weight=1)
        body.columnconfigure(2, weight=0, minsize=390)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body, style="Panel.TFrame", padding=12)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        ttk.Label(left, text="RECORDINGS", style="PanelTitle.TLabel").pack(anchor="w")
        self.list_hint = ttk.Label(left, text="", style="Panel.TLabel", wraplength=300)
        self.list_hint.pack(anchor="w", pady=(3, 8))
        list_frame = ttk.Frame(left, style="Panel.TFrame")
        list_frame.pack(fill="both", expand=True)
        self.recording_list = tk.Listbox(list_frame, bg="#0a1727", fg=self.TEXT, selectbackground="#1d5965", selectforeground="#ffffff", relief="flat", highlightthickness=0, activestyle="none", font=("Segoe UI", 9), width=42)
        list_scroll = ttk.Scrollbar(list_frame, orient="vertical", command=self.recording_list.yview)
        self.recording_list.configure(yscrollcommand=list_scroll.set)
        self.recording_list.pack(side="left", fill="both", expand=True)
        list_scroll.pack(side="right", fill="y")
        self.recording_list.bind("<<ListboxSelect>>", self._recording_selected)

        center = ttk.Frame(body)
        center.grid(row=0, column=1, sticky="nsew", padx=(0, 12))
        center.columnconfigure(0, weight=1)
        center.rowconfigure(0, weight=1)
        center.rowconfigure(2, weight=0)
        self.video_panel = tk.Frame(center, bg="#000000", highlightthickness=1, highlightbackground="#23485f")
        self.video_panel.grid(row=0, column=0, sticky="nsew")
        self.video_message = tk.Label(self.video_panel, text="Seleziona una registrazione", bg="#000000", fg="#ffcf72", font=("Segoe UI", 12), wraplength=700)
        self.video_message.place(relx=0.5, rely=0.5, anchor="center")

        controls = ttk.Frame(center, style="Panel.TFrame", padding=10)
        controls.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        controls.columnconfigure(4, weight=1)
        self.play_button = ttk.Button(controls, text="▶ Play", style="Accent.TButton", command=self._play)
        self.play_button.grid(row=0, column=0, padx=(0, 5))
        ttk.Button(controls, text="❚❚ Pause", command=self._pause).grid(row=0, column=1, padx=5)
        ttk.Button(controls, text="■ Stop", command=self._stop).grid(row=0, column=2, padx=5)
        ttk.Button(controls, text="−5 s", command=lambda: self._jump(-5000)).grid(row=0, column=3, padx=(12, 4))
        ttk.Button(controls, text="+5 s", command=lambda: self._jump(5000)).grid(row=0, column=4, padx=4, sticky="w")
        self.time_label = ttk.Label(controls, text="00:00 / 00:00", style="Panel.TLabel")
        self.time_label.grid(row=0, column=5, padx=(14, 8))
        self.timeline = ttk.Scale(controls, from_=0, to=1, orient="horizontal", command=self._timeline_changed)
        self.timeline.grid(row=1, column=0, columnspan=6, sticky="ew", pady=(10, 4))
        self.volume = ttk.Scale(controls, from_=0, to=100, orient="horizontal", length=110, command=self._volume_changed)
        self.volume.set(80)
        self.volume.grid(row=0, column=6, padx=(12, 6))
        ttk.Label(controls, text="Volume", style="Panel.TLabel").grid(row=0, column=7)
        self.speed = ttk.Combobox(controls, state="readonly", width=6, values=("0.5x", "1x", "1.5x", "2x"))
        self.speed.set("1x")
        self.speed.bind("<<ComboboxSelected>>", self._speed_changed)
        self.speed.grid(row=0, column=8, padx=(12, 0))

        ass_bar = ttk.Frame(center, style="Panel.TFrame", padding=(10, 8))
        ass_bar.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        self.show_ass = tk.BooleanVar(value=True)
        self.ass_check = ttk.Checkbutton(ass_bar, text="Show original ASS overlay", variable=self.show_ass, command=self._ass_toggled)
        self.ass_check.pack(side="left")
        self.player_status = ttk.Label(ass_bar, text="VLC: verifica dipendenza…", style="Panel.TLabel")
        self.player_status.pack(side="right")

        right = ttk.Frame(body, style="Panel.TFrame", padding=12)
        right.grid(row=0, column=2, sticky="nsew")
        right.rowconfigure(3, weight=1)
        right.rowconfigure(5, weight=1)
        right.columnconfigure(0, weight=1)
        ttk.Label(right, text="CURRENT TELEMETRY", style="PanelTitle.TLabel").grid(row=0, column=0, sticky="w")
        self.telemetry_time = ttk.Label(right, text="Video time 00:00", style="Panel.TLabel")
        self.telemetry_time.grid(row=1, column=0, sticky="w", pady=(3, 8))
        self.field_tree = ttk.Treeview(right, columns=("field", "value"), show="headings", height=9)
        self.field_tree.heading("field", text="FIELD")
        self.field_tree.heading("value", text="VALUE")
        self.field_tree.column("field", width=130, anchor="w")
        self.field_tree.column("value", width=210, anchor="w")
        self.field_tree.grid(row=2, column=0, sticky="nsew")
        field_scroll = ttk.Scrollbar(right, orient="vertical", command=self.field_tree.yview)
        field_scroll.grid(row=2, column=1, sticky="ns")
        self.field_tree.configure(yscrollcommand=field_scroll.set)
        ttk.Label(right, text="RAW ASS EVENTS AT CURRENT TIME", style="PanelTitle.TLabel").grid(row=4, column=0, sticky="w", pady=(12, 6))
        raw_frame = ttk.Frame(right, style="Panel.TFrame")
        raw_frame.grid(row=5, column=0, sticky="nsew")
        raw_frame.rowconfigure(0, weight=1)
        raw_frame.columnconfigure(0, weight=1)
        self.raw_text = tk.Text(raw_frame, bg="#07111f", fg="#d9e8f0", insertbackground="#ffffff", wrap="word", relief="flat", font=("Consolas", 9), state="disabled")
        raw_scroll = ttk.Scrollbar(raw_frame, orient="vertical", command=self.raw_text.yview)
        self.raw_text.configure(yscrollcommand=raw_scroll.set)
        self.raw_text.grid(row=0, column=0, sticky="nsew")
        raw_scroll.grid(row=0, column=1, sticky="ns")
        ttk.Label(right, text="FILE INFORMATION", style="PanelTitle.TLabel").grid(row=6, column=0, sticky="w", pady=(12, 6))
        self.file_info = ttk.Label(right, text="--", style="Panel.TLabel", justify="left", wraplength=340)
        self.file_info.grid(row=7, column=0, sticky="ew")

    def _init_vlc(self) -> None:
        if vlc is None:
            self.vlc_error = "python-vlc non è installato. Installa il requirements file e VLC desktop 64-bit."
            self._set_player_status(self.vlc_error)
            return
        try:
            self.vlc_instance = vlc.Instance()
            self.player = self.vlc_instance.media_player_new()
            self.player.set_hwnd(self.video_panel.winfo_id())
            self._set_player_status("VLC pronto")
        except Exception as exc:
            self.vlc_error = f"VLC non disponibile: {exc}. Installa VLC desktop 64-bit."
            self.player = None
            self.vlc_instance = None
            self._set_player_status(self.vlc_error)

    def _set_player_status(self, text: str) -> None:
        if hasattr(self, "player_status"):
            self.player_status.configure(text=text)
        if hasattr(self, "video_message") and self.vlc_error:
            self.video_message.configure(text=self.vlc_error)

    def _scan_folder(self, folder: Path) -> None:
        self.folder = Path(folder)
        self.folder_label.configure(text=str(self.folder))
        self.pairs = pair_recordings(self.folder)
        self.recording_list.delete(0, "end")
        for pair in self.pairs:
            self.recording_list.insert("end", f"{pair.display_name}\n  {pair.status}")
        self.list_hint.configure(text=f"{len(self.pairs)} registrazioni trovate\nLe coppie sono abbinate solo per basename identico.")
        if self.pairs:
            self.recording_list.selection_set(0)
            self._load_pair(0)
        else:
            self._clear_recording("Nessuna coppia MKV/ASS trovata nella cartella selezionata.")

    def _choose_folder(self) -> None:
        selected = filedialog.askdirectory(initialdir=str(self.folder))
        if selected:
            self._scan_folder(Path(selected))

    def _recording_selected(self, _event=None) -> None:
        selection = self.recording_list.curselection()
        if selection:
            self._load_pair(selection[0])

    def _clear_recording(self, message: str) -> None:
        self.selected_pair = None
        self.ass_index = None
        self.current_ms = 0
        self.duration_ms = 0
        self._set_timeline(0, 1)
        self.video_message.configure(text=message)
        self._update_telemetry(0)
        self.file_info.configure(text="--")

    def _load_pair(self, index: int) -> None:
        if not (0 <= index < len(self.pairs)):
            return
        self.selected_pair = self.pairs[index]
        self.current_ms = 0
        self.duration_ms = 0
        self.ass_index = None
        self.ass_spu_id = None
        if self.selected_pair.ass:
            try:
                self.ass_index = parse_ass_file(self.selected_pair.ass)
            except Exception as exc:
                self.video_message.configure(text=f"ASS non caricabile: {exc}")
        self._update_file_info()
        self._update_telemetry(0)
        if not self.selected_pair.video:
            self.video_message.configure(text="ASS presente senza video MKV.")
            self._set_player_status("Nessun video da riprodurre")
            return
        if not self.player:
            self.video_message.configure(text=self.vlc_error or "VLC non disponibile.")
            self._set_player_status(self.vlc_error or "VLC non disponibile")
            return
        try:
            media = self.vlc_instance.media_new(str(self.selected_pair.video))
            if self.selected_pair.ass and self.show_ass.get():
                media.add_option(f":sub-file={str(self.selected_pair.ass)}")
            self.player.set_media(media)
            self.player.set_hwnd(self.video_panel.winfo_id())
            self._pending_play = True
            self.player.play()
            self.video_message.configure(text="Caricamento MKV…")
            self.after(500, self._pause_after_load)
        except Exception as exc:
            self.vlc_error = f"Impossibile aprire il video con VLC: {exc}"
            self.video_message.configure(text=self.vlc_error)
            self._set_player_status(self.vlc_error)

    def _pause_after_load(self) -> None:
        if self.player and self._pending_play:
            self.player.pause()
            self._pending_play = False
            self._sync_spu()
            self.video_message.configure(text="")
            self._update_duration_from_player()

    def _update_file_info(self) -> None:
        pair = self.selected_pair
        if not pair:
            self.file_info.configure(text="--")
            return
        lines = [f"filename: {pair.display_name}", f"recording date: {recording_date_from_name(pair.video or pair.ass)}"]
        if pair.video:
            stat = pair.video.stat()
            meta = ffprobe_metadata(pair.video)
            lines.extend([
                f"video: {pair.video.name}",
                f"size: {format_bytes(stat.st_size)}",
                f"duration: {meta.get('duration', '--')}",
                f"resolution: {meta.get('resolution', '--')}",
                f"FPS: {meta.get('fps', '--')}",
                f"video codec: {meta.get('codec', '--')}",
            ])
        else:
            lines.append("video: VIDEO WITHOUT VIDEO")
        if pair.ass:
            lines.extend([
                f"ASS: {pair.ass.name}",
                f"ASS events: {len(self.ass_index.events) if self.ass_index else '--'}",
                f"ASS start/end: {format_ms(self.ass_index.start_ms) if self.ass_index else '--'} → {format_ms(self.ass_index.end_ms) if self.ass_index else '--'}",
            ])
        else:
            lines.append("ASS: ASS WITHOUT VIDEO")
        self.file_info.configure(text="\n".join(lines))

    def _previous_recording(self) -> None:
        self._move_recording(-1)

    def _next_recording(self) -> None:
        self._move_recording(1)

    def _move_recording(self, delta: int) -> None:
        if not self.pairs:
            return
        selected = self.recording_list.curselection()
        index = selected[0] if selected else 0
        index = (index + delta) % len(self.pairs)
        self.recording_list.selection_clear(0, "end")
        self.recording_list.selection_set(index)
        self.recording_list.see(index)
        self._load_pair(index)

    def _play(self) -> None:
        if self.player:
            self.player.play()
            self._sync_spu()

    def _pause(self) -> None:
        if self.player:
            self.player.pause()
            self._update_telemetry(self._player_time_ms())

    def _stop(self) -> None:
        if self.player:
            self.player.stop()
        self.current_ms = 0
        self._set_timeline(0, max(1, self.duration_ms))
        self._update_telemetry(0)

    def _jump(self, delta_ms: int) -> None:
        target = max(0, min(max(self.duration_ms, 0), self._player_time_ms() + delta_ms))
        self._seek_to(target)

    def _seek_to(self, value_ms: int) -> None:
        value_ms = max(0, min(max(self.duration_ms, 0), int(value_ms)))
        if self.player:
            try:
                self.player.set_time(value_ms)
            except Exception:
                pass
        self.current_ms = value_ms
        self._update_telemetry(value_ms)

    def _timeline_changed(self, value: str) -> None:
        if not self._timeline_guard:
            self._seek_to(int(float(value)))

    def _set_timeline(self, value_ms: int, duration_ms: int) -> None:
        self._timeline_guard = True
        self.timeline.configure(to=max(1, duration_ms))
        self.timeline.set(max(0, min(max(1, duration_ms), value_ms)))
        self._timeline_guard = False
        self.time_label.configure(text=f"{format_ms(value_ms)} / {format_ms(duration_ms)}")

    def _volume_changed(self, value: str) -> None:
        if self.player:
            try:
                self.player.audio_set_volume(int(float(value)))
            except Exception:
                pass

    def _speed_changed(self, _event=None) -> None:
        if self.player:
            try:
                self.player.set_rate(float(self.speed.get().rstrip("x")))
            except Exception:
                pass

    def _ass_toggled(self) -> None:
        self._sync_spu()

    def _sync_spu(self) -> None:
        if not self.player or not self.selected_pair or not self.selected_pair.ass:
            return
        if not self.show_ass.get():
            try:
                self.player.video_set_spu(-1)
            except Exception:
                pass
            return
        try:
            descriptions = self.player.video_get_spu_description() or []
            candidates = [(int(track_id), str(name)) for track_id, name in descriptions if int(track_id) >= 0]
            if candidates:
                self.ass_spu_id = candidates[0][0]
                self.player.video_set_spu(self.ass_spu_id)
        except Exception:
            # VLC may expose the subtitle track only after the first decoded frame.
            pass

    def _player_time_ms(self) -> int:
        if not self.player:
            return self.current_ms
        try:
            value = int(self.player.get_time())
            return max(0, value)
        except Exception:
            return self.current_ms

    def _update_duration_from_player(self) -> None:
        if not self.player:
            return
        try:
            value = int(self.player.get_length())
            if value > 0:
                self.duration_ms = value
                self._set_timeline(self.current_ms, self.duration_ms)
                self._update_file_info()
        except Exception:
            pass

    def _poll_player(self) -> None:
        if self.player:
            self._update_duration_from_player()
            now = self._player_time_ms()
            self.current_ms = now
            self._set_timeline(now, max(1, self.duration_ms))
            self._update_telemetry(now)
            self._sync_spu()
        else:
            self._update_telemetry(self.current_ms)
        self.after(MAX_TELEMETRY_REFRESH_MS, self._poll_player)

    def _update_telemetry(self, timestamp_ms: int) -> None:
        timestamp_ms = max(0, int(timestamp_ms))
        self.telemetry_time.configure(text=f"Video time {format_ms(timestamp_ms)}")
        for item in self.field_tree.get_children():
            self.field_tree.delete(item)
        raw_lines: List[str] = []
        active = self.ass_index.active_at(timestamp_ms) if self.ass_index else []
        for event in active:
            for field_name, value in event.fields:
                self.field_tree.insert("", "end", values=(field_name, value))
            for line in event.other_lines:
                self.field_tree.insert("", "end", values=("Other / Raw", line))
            raw_lines.append(f"[{ass_time(event.start_ms)} → {ass_time(event.end_ms)}] EVENT {event.index}")
            raw_lines.append(f"  {event.raw_dialogue}")
            raw_lines.append(f"  readable: {event.readable_text}")
        if not active:
            self.field_tree.insert("", "end", values=("--", "Nessun evento ASS attivo"))
            raw_lines.append("Nessun evento ASS attivo a questo timestamp.")
        self.raw_text.configure(state="normal")
        self.raw_text.delete("1.0", "end")
        self.raw_text.insert("1.0", "\n".join(raw_lines))
        self.raw_text.configure(state="disabled")

    def _export_csv(self) -> None:
        if not self.ass_index:
            messagebox.showinfo("ASS", "La registrazione selezionata non contiene un ASS leggibile.")
            return
        destination = filedialog.asksaveasfilename(initialdir=str(self.folder), defaultextension=".csv", filetypes=[("CSV", "*.csv")], title="Export telemetry CSV")
        if destination:
            try:
                export_telemetry_csv(self.ass_index, Path(destination))
                self.player_status.configure(text=f"CSV esportato: {Path(destination).name}")
            except OSError as exc:
                messagebox.showerror("Export CSV", str(exc))

    def _screenshot(self) -> None:
        if not self.player:
            messagebox.showinfo("Screenshot", self.vlc_error or "VLC non disponibile.")
            return
        destination = filedialog.asksaveasfilename(initialdir=str(self.folder), defaultextension=".png", filetypes=[("PNG", "*.png")], title="Screenshot current frame")
        if destination:
            try:
                result = self.player.video_take_snapshot(0, destination, 0, 0)
                if result != 0:
                    messagebox.showerror("Screenshot", "VLC non ha potuto salvare il frame corrente.")
            except Exception as exc:
                messagebox.showerror("Screenshot", str(exc))


def print_summary(folder: Path) -> None:
    pairs = pair_recordings(folder)
    print(f"Folder: {folder}")
    for pair in pairs:
        events = "--"
        if pair.ass:
            try:
                events = str(len(parse_ass_file(pair.ass).events))
            except Exception as exc:
                events = f"ERROR: {exc}"
        print(f"{pair.display_name}: {pair.status}; ASS events={events}")


def main() -> int:
    # Cockpit exports can contain the U+A789 modifier letter used in the
    # timestamp ("꞉"). Windows consoles using cp1252 cannot print it.
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    parser = argparse.ArgumentParser(description="COCKPIT VIDEO + TELEMETRY EXPLORER")
    parser.add_argument("--folder", type=Path, default=DEFAULT_VIDEO_ROOT, help="cartella iniziale delle registrazioni")
    parser.add_argument("--summary", action="store_true", help="stampa pairing/eventi senza aprire la GUI")
    parser.add_argument("--no-vlc", action="store_true", help="disabilita il backend VLC, utile per test offline")
    args = parser.parse_args()
    if args.summary:
        print_summary(args.folder)
        return 0
    app = CockpitVideoViewer(args.folder, use_vlc=not args.no_vlc)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
