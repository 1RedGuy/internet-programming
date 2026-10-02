#!/usr/bin/env python3
"""Generate script.md, narration.srt and storyboard.md from carviz/timeline.py.

The timeline is the single source of truth for timing, so the documents can
never drift from the rendered scenes.  Run after editing the timeline:
    python3 tools/make_docs.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from carviz import timeline as TL  # noqa: E402

LINE = 42          # max characters per subtitle line
MAX_LINES = 2


def ts(t, srt=False):
    ms = int(round(t * 1000))
    h, ms = divmod(ms, 3600_000)
    m, ms = divmod(ms, 60_000)
    s, ms = divmod(ms, 1000)
    if srt:
        return f"{h:02d}:{m:02d}:{s:02d},{ms:03d}"
    return f"{m}:{s:02d}.{ms // 100}"


def wrap(text, width=LINE):
    words, lines, cur = text.split(), [], ""
    for w in words:
        if cur and len(cur) + 1 + len(w) > width:
            lines.append(cur)
            cur = w
        else:
            cur = (cur + " " + w).strip()
    if cur:
        lines.append(cur)
    return lines


def chunks(text):
    """Split narration into subtitle cues of <= 2 lines x 42 chars."""
    sentences = re.split(r"(?<=[.!?])\s+", text.strip())
    cues = []
    for s in sentences:
        if len(wrap(s)) <= MAX_LINES:
            cues.append(s)
            continue
        # split long sentences at clause punctuation, then by length
        parts = re.split(r"(?<=[,:;—])\s+", s)
        cur = ""
        for p in parts:
            cand = (cur + " " + p).strip()
            if len(wrap(cand)) <= MAX_LINES:
                cur = cand
            else:
                if cur:
                    cues.append(cur)
                cur = p
                while len(wrap(cur)) > MAX_LINES:
                    ws = cur.split()
                    k = len(ws) // 2
                    cues.append(" ".join(ws[:k]))
                    cur = " ".join(ws[k:])
        if cur:
            cues.append(cur)
    return cues


def nwords(s):
    return len(s.replace("—", " ").split())


def make_srt():
    out, n = [], 0
    rate = TL.WPM / 60.0
    for sc in TL.SCENES:
        for b in sc.beats:
            t = b.gstart + TL.LEAD
            cues = chunks(b.text)
            for j, c in enumerate(cues):
                dur = nwords(c) / rate
                start = t
                end = t + dur
                nxt_limit = b.gend - 0.05 if j == len(cues) - 1 else end + 0.0
                end = min(max(end + 0.25, start + 1.0), nxt_limit if j == len(cues) - 1 else end + 0.25)
                if j < len(cues) - 1:
                    end = min(end, t + dur)  # no overlap with the next cue
                n += 1
                out.append(f"{n}\n{ts(start, True)} --> {ts(end, True)}\n" + "\n".join(wrap(c)) + "\n")
                t += dur
    return "\n".join(out)


def make_script():
    total_words = sum(b.words for s in TL.SCENES for b in s.beats)
    L = ["# Narration script — How a Manual Car Works", "",
         f"Total running time **{ts(TL.TOTAL)}** ({TL.TOTAL:.1f} s, {int(round(TL.TOTAL * TL.FPS))} frames at "
         f"{TL.FPS} fps). {total_words} words at {TL.WPM:.0f} words/min "
         f"({total_words / TL.WPM:.2f} min of speech).", "",
         "Tone: clear, confident explainer. Each line starts "
         f"{TL.LEAD:.2f} s after its timestamp and fits inside its slot at {TL.WPM:.0f} wpm; the picture is cut to "
         "these exact timings, so read each line at its timestamp and let the pauses breathe.",
         "Every claim is backed by an entry in FACTS.md (see the narration-to-facts table there).", ""]
    for i, sc in enumerate(TL.SCENES, 1):
        L.append(f"## {i}. {sc.title}  ({ts(sc.start)} – {ts(sc.start + sc.dur)}, {sc.dur:.1f} s)")
        L.append("")
        L.append("| Time | Line | Words | Slot / speech |")
        L.append("|---|---|---|---|")
        for b in sc.beats:
            L.append(f"| {ts(b.gstart)}–{ts(b.gend)} | {b.text} | {b.words} | {b.dur:.1f} s / {b.speak_time:.1f} s |")
        L.append("")
    return "\n".join(L)


def make_storyboard():
    L = ["# Storyboard — How a Manual Car Works", "",
         "Generated from `carviz/timeline.py` (timing + narration + visual intent). Timestamps are global "
         "(video) time; scene-local times are in brackets. Every mechanical motion is computed from the "
         "drivetrain state (`carviz/state.py`); cameras, labels and HUD are code-driven (`scenes/`).", "",
         "**Look:** realistic studio cutaway style — PBR cast iron/aluminium, machined steel, brass synchro rings, "
         "friction material; soft studio lighting; subtle depth of field; red-painted section faces like real "
         "training cutaways; a single warm accent glow for the power path. **Overlay:** minimal part labels with "
         "leader lines, gear indicator, rpm, km/h, slow-motion badge; bottom-centre kept free for subtitles.", ""]
    for i, sc in enumerate(TL.SCENES, 1):
        L.append(f"## {i}. {sc.title} — {ts(sc.start)}–{ts(sc.start + sc.dur)} ({sc.dur:.1f} s, {sc.frames} frames)")
        L.append("")
        L.append(f"_{sc.summary}_")
        L.append("")
        for b in sc.beats:
            L.append(f"**{ts(b.gstart)}–{ts(b.gend)}** [{b.start:.1f}–{b.end:.1f} s] `{sc.id}.{b.id}`  ")
            L.append(f"*Narration:* “{b.text}”  ")
            L.append(f"*Picture:* {b.visual}")
            L.append("")
    return "\n".join(L)


def main():
    TL.check(verbose=False)
    with open(os.path.join(ROOT, "script.md"), "w") as fh:
        fh.write(make_script() + "\n")
    with open(os.path.join(ROOT, "narration.srt"), "w") as fh:
        fh.write(make_srt())
    sb_path = os.path.join(ROOT, "storyboard.md")
    with open(sb_path, "w") as fh:
        fh.write(make_storyboard() + "\n")
    print("wrote script.md, narration.srt, storyboard.md")


if __name__ == "__main__":
    main()
