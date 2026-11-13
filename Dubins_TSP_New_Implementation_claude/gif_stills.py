# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# A GIF is a little film made of many pictures ("frames") shown one after
# another. Paper documents cannot play films, so this file picks a few
# important frames of each GIF and saves them as ordinary pictures (PNG files),
# plus one "contact sheet": a page with all the chosen frames side by side,
# each labelled with the moment of the film it comes from.
# ==============================================================================
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
# "import" borrows a toolbox. argparse reads options typed on the command line.
import argparse
# glob finds files whose names match a pattern, like "every file ending in .gif".
import glob
# os: folders and file names.
import os

# matplotlib draws pictures.
import matplotlib

# "Agg" tells matplotlib to draw into files only, without opening a window.
matplotlib.use("Agg")
# pyplot: matplotlib's drawing commands.
import matplotlib.pyplot as plt
# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np
# PIL ("Pillow") reads and writes picture files. Image opens a picture;
# ImageSequence walks through the frames of a film.
from PIL import Image, ImageSequence

# Colours used by all the project's pictures: dark ink, quieter grey, and the
# page colour.
from plot import INK, QUIET, SURFACE


# "def" starts a recipe (a "function"). This one reads every frame of a GIF.
def read_gif(path):
    """Every frame as an RGB array, with the time [s] at which it appears."""
    # frames: the pictures; times: when each one appears; t: the clock, from 0.
    frames, times, t = [], [], 0.0
    # Open the file. "with" makes sure it is closed again at the end.
    with Image.open(path) as im:
        # For every frame of the film:
        for frame in ImageSequence.Iterator(im):
            # Keep the picture as an array of pixels in red-green-blue ("RGB") colour.
            frames.append(np.asarray(frame.convert("RGB")))
            # Note the moment it appears.
            times.append(t)
            # Move the clock on by how long this frame is shown (stored in
            # milliseconds, so divide by 1000; 100 ms if the file does not say).
            t += frame.info.get("duration", 100) / 1000.0
    # Hand back the frames, their times, and the film's total length.
    return frames, np.array(times), t


# Chooses which frames to keep: n of them spread evenly, or the frames shown
# at the times listed in "at".
def key_frames(frames, times, n=6, at=None):
    """Indices of the key frames: the frames showing the times in `at`, or n frames evenly spread over the
    distinct ones (a frame equal to the one before it is a pause and is skipped)."""
    # If times were asked for: for each, the last frame that starts at or
    # before it (np.searchsorted finds where a time fits in the sorted list).
    # A set { } removes doubles, and sorted puts them in order.
    if at is not None:
        # Hand back those frame numbers.
        return sorted({int(np.searchsorted(times, a, side="right") - 1) for a in at})
    # The frames that differ from the one before (a frame equal to the one
    # before it is just a pause). The first frame always counts.
    distinct = [0] + [i for i in range(1, len(frames)) if not np.array_equal(frames[i], frames[i - 1])]
    # n positions spread evenly from the first to the last distinct frame
    # (np.linspace), rounded to whole numbers, with doubles removed (np.unique).
    pick = np.unique(np.round(np.linspace(0, len(distinct) - 1, min(n, len(distinct)))).astype(int))
    # Hand back the frame numbers at those positions.
    return [distinct[q] for q in pick]


# Saves the chosen frames of one GIF, and its contact sheet, into "folder".
def stills(path, folder, n=6, at=None):
    """Write the key frames of one GIF and its contact sheet; return the paths written."""
    # Read the film and choose the frames.
    frames, times, total = read_gif(path)
    # Choose the frames to keep.
    keys = key_frames(frames, times, n, at)
    # The GIF's name without its ending, e.g. "seed1_game".
    name = os.path.splitext(os.path.basename(path))[0]
    # A folder of its own for the frames; make it if it is missing.
    out_dir = os.path.join(folder, name)
    # Make the folder if it is missing (exist_ok=True: no error if it is there).
    os.makedirs(out_dir, exist_ok=True)
    # Delete frames saved by an earlier run, so the folder matches this run.
    for old in glob.glob(os.path.join(out_dir, "frame_*.png")):
        # Delete each old frame file.
        os.remove(old)
    # written: the files saved.
    written = []
    # For each chosen frame (k counts 0, 1, 2 ...; i is the frame's number):
    for k, i in enumerate(keys):
        # Its file name, e.g. "frame_01_t012.5s.png": the order, then the time.
        # {k + 1:02d} writes the number with 2 digits (01, 02 ...).
        p = os.path.join(out_dir, f"frame_{k + 1:02d}_t{times[i]:05.1f}s.png")
        # Save the frame exactly as it is in the GIF.
        Image.fromarray(frames[i]).save(p)
        # Note the file.
        written.append(p)

    # The contact sheet. h, w: the height and width of a frame in pixels.
    h, w = frames[0].shape[:2]
    # 2 columns for wide frames, 3 for squarer ones.
    ncol = 2 if w > 1.3 * h else 3
    # Enough rows to hold every frame (np.ceil rounds up).
    nrow = int(np.ceil(len(keys) / ncol))
    # 100 pixels per inch, so each frame keeps its own size on the sheet.
    dpi = 100
    pad_top, cap = 0.55, 0.3                                     # inches: page title, caption per tile
    # The page: wide enough for the columns, tall enough for the rows,
    # their captions and the title.
    fig = plt.figure(figsize=(ncol * w / dpi, nrow * (h / dpi + cap) + pad_top), dpi=dpi, facecolor=SURFACE)
    # H: the page height in inches.
    H = nrow * (h / dpi + cap) + pad_top
    # The page title: the GIF's name, how many frames were kept, and its length.
    fig.text(0.01, 1 - 0.12 / H, f"{name}.gif: {len(keys)} key frames of {len(frames)} ({total:.1f} s in all)",
             fontsize=13, fontweight="bold", color=INK, va="top")
    # Place every chosen frame on the page.
    for k, i in enumerate(keys):
        # Its row and column ("divmod" gives the whole part and the remainder of k / ncol).
        r, c = divmod(k, ncol)
        # How far down the page its row starts, in inches.
        top = pad_top + r * (h / dpi + cap)
        # A drawing area ("axes") for it. Positions on a matplotlib page go
        # from 0 to 1 across and up, so inches are divided by the page size.
        ax = fig.add_axes([c * w / dpi / (ncol * w / dpi), 1 - (top + cap + h / dpi) / H, 1 / ncol, (h / dpi) / H])
        # Show the frame, pixel for pixel ("nearest": no blurring).
        ax.imshow(frames[i], interpolation="nearest")
        # Hide the axes' scales and border.
        ax.axis("off")
        # Its caption: number, time in the film, and frame number.
        fig.text((c + 0.01) / ncol, 1 - (top + 0.05) / H, f"({k + 1}) t = {times[i]:.1f} s, frame {i + 1} of "
                 f"{len(frames)}", fontsize=10, color=QUIET, va="top")
    # Save the sheet next to the frames' folder, and close the page.
    sheet = os.path.join(folder, f"{name}_sheet.png")
    # Save the sheet ...
    fig.savefig(sheet, facecolor=SURFACE)
    # ... and close the page, to free the memory.
    plt.close(fig)
    # Hand back every file saved, the sheet last.
    return written + [sheet]


# What happens when the file is started from the command line.
def main():
    # The folder this file is in.
    here = os.path.dirname(os.path.abspath(__file__))
    # Describe the options the user may type.
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # The GIF files to use (any number; "*" means zero or more).
    ap.add_argument("gifs", nargs="*", help="GIF files (default: every GIF in output/)")
    # -n: how many frames to keep from each.
    ap.add_argument("-n", type=int, default=6, help="number of key frames per GIF")
    # --at: keep the frames shown at these times instead.
    ap.add_argument("--at", type=float, nargs="+", default=None, help="take the frames showing these times [s]")
    # Read what the user typed.
    args = ap.parse_args()
    # The GIFs named by the user, or else ("or") every GIF in the output folder.
    gifs = args.gifs or sorted(glob.glob(os.path.join(here, "output", "*.gif")))
    # Where the stills go: output/stills; make it if it is missing.
    folder = os.path.join(here, "output", "stills")
    # Make the folder if it is missing (exist_ok=True: no error if it is there).
    os.makedirs(folder, exist_ok=True)
    # For every GIF: save its stills, and print what was written.
    for g in gifs:
        # Save this GIF's stills.
        written = stills(g, folder, args.n, args.at)
        # Print what was written.
        print(f"{os.path.basename(g)}: {len(written) - 1} frames and {os.path.relpath(written[-1], here)}")


# True only when this file is started directly, not when it is imported.
if __name__ == "__main__":
    # Run the main recipe.
    main()
