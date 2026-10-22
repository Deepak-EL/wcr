#!/usr/bin/env python3
"""Combine recorded PPM frames (wide shot + close-up inset) into an MP4.
usage: python tools/make_video.py <video_dir with wide/ and close/> <out.mp4> [fps]
needs: pip install imageio imageio-ffmpeg pillow numpy"""
import sys
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw

src, out = Path(sys.argv[1]), Path(sys.argv[2])
fps = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
wide = sorted((src / 'wide').glob('*.ppm'))
close = sorted((src / 'close').glob('*.ppm'))
n = min(len(wide), len(close))
out.parent.mkdir(parents=True, exist_ok=True)
with imageio.get_writer(out, fps=fps, codec='libx264', quality=8, macro_block_size=8) as w:
    for i in range(n):
        frame = Image.open(wide[i]).convert('RGB')
        inset = Image.open(close[i]).convert('RGB').resize((448, 252), Image.LANCZOS)
        frame.paste(inset, (frame.width - 448 - 16, 16))
        d = ImageDraw.Draw(frame)
        d.rectangle([frame.width - 448 - 16, 16, frame.width - 16, 16 + 252], outline=(255, 200, 0), width=3)
        d.text((16, 16), f'WCR on USACE gravity dam (upstream face, 90 deg)   t = {i / 5.0:5.1f} s   x{fps / 5:.0f} speed',
               fill=(20, 20, 20))
        w.append_data(np.asarray(frame))
print(f'{n} frames -> {out} ({n / fps:.1f} s at {fps:.0f} fps)')
