"""Still images from the GIFs in output/, for documents and slides that cannot play an animation.

    python gif_stills.py                                   # every GIF in output/, 6 key frames each
    python gif_stills.py output/seed1_partition.gif -n 8   # one GIF, 8 key frames
    python gif_stills.py output/seed1_game.gif --at 0 12.5 30   # the frames showing these times [s]

For each GIF it writes, in output/stills/:

    <name>/frame_<k>_t<time>s.png   each key frame, at the GIF's own size (a copy of that frame, not redrawn)
    <name>_sheet.png                all key frames on one page, each labelled with its time in the GIF

Frames that only repeat the one before (the pauses) are skipped before the key frames are chosen, so every
key frame shows something new; the first and the last frame are always included. Earlier frame files of the
same GIF are removed first, so the folder always matches the last run.
"""
import argparse
import glob
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageSequence

from plot import INK, QUIET, SURFACE


def read_gif(path):
    """Every frame as an RGB array, with the time [s] at which it appears."""
    frames, times, t = [], [], 0.0
    with Image.open(path) as im:
        for frame in ImageSequence.Iterator(im):
            frames.append(np.asarray(frame.convert("RGB")))
            times.append(t)
            t += frame.info.get("duration", 100) / 1000.0
    return frames, np.array(times), t


def key_frames(frames, times, n=6, at=None):
    """Indices of the key frames: the frames showing the times in `at`, or n frames evenly spread over the
    distinct ones (a frame equal to the one before it is a pause and is skipped)."""
    if at is not None:
        return sorted({int(np.searchsorted(times, a, side="right") - 1) for a in at})
    distinct = [0] + [i for i in range(1, len(frames)) if not np.array_equal(frames[i], frames[i - 1])]
    pick = np.unique(np.round(np.linspace(0, len(distinct) - 1, min(n, len(distinct)))).astype(int))
    return [distinct[q] for q in pick]


def stills(path, folder, n=6, at=None):
    """Write the key frames of one GIF and its contact sheet; return the paths written."""
    frames, times, total = read_gif(path)
    keys = key_frames(frames, times, n, at)
    name = os.path.splitext(os.path.basename(path))[0]
    out_dir = os.path.join(folder, name)
    os.makedirs(out_dir, exist_ok=True)
    for old in glob.glob(os.path.join(out_dir, "frame_*.png")):
        os.remove(old)
    written = []
    for k, i in enumerate(keys):
        p = os.path.join(out_dir, f"frame_{k + 1:02d}_t{times[i]:05.1f}s.png")
        Image.fromarray(frames[i]).save(p)
        written.append(p)

    h, w = frames[0].shape[:2]
    ncol = 2 if w > 1.3 * h else 3
    nrow = int(np.ceil(len(keys) / ncol))
    dpi = 100
    pad_top, cap = 0.55, 0.3                                     # inches: page title, caption per tile
    fig = plt.figure(figsize=(ncol * w / dpi, nrow * (h / dpi + cap) + pad_top), dpi=dpi, facecolor=SURFACE)
    H = nrow * (h / dpi + cap) + pad_top
    fig.text(0.01, 1 - 0.12 / H, f"{name}.gif: {len(keys)} key frames of {len(frames)} ({total:.1f} s in all)",
             fontsize=13, fontweight="bold", color=INK, va="top")
    for k, i in enumerate(keys):
        r, c = divmod(k, ncol)
        top = pad_top + r * (h / dpi + cap)
        ax = fig.add_axes([c * w / dpi / (ncol * w / dpi), 1 - (top + cap + h / dpi) / H, 1 / ncol, (h / dpi) / H])
        ax.imshow(frames[i], interpolation="nearest")
        ax.axis("off")
        fig.text((c + 0.01) / ncol, 1 - (top + 0.05) / H, f"({k + 1}) t = {times[i]:.1f} s, frame {i + 1} of "
                 f"{len(frames)}", fontsize=10, color=QUIET, va="top")
    sheet = os.path.join(folder, f"{name}_sheet.png")
    fig.savefig(sheet, facecolor=SURFACE)
    plt.close(fig)
    return written + [sheet]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("gifs", nargs="*", help="GIF files (default: every GIF in output/)")
    ap.add_argument("-n", type=int, default=6, help="number of key frames per GIF")
    ap.add_argument("--at", type=float, nargs="+", default=None, help="take the frames showing these times [s]")
    args = ap.parse_args()
    gifs = args.gifs or sorted(glob.glob(os.path.join(here, "output", "*.gif")))
    folder = os.path.join(here, "output", "stills")
    os.makedirs(folder, exist_ok=True)
    for g in gifs:
        written = stills(g, folder, args.n, args.at)
        print(f"{os.path.basename(g)}: {len(written) - 1} frames and {os.path.relpath(written[-1], here)}")


if __name__ == "__main__":
    main()
