"""Compress photos for Hugo life section.

Usage: python compress_photos.py <source_dir> <slug>

Example:
  python compress_photos.py "G:/Blog_Photos/Korea" korea
  -> compresss all images in G:/Blog_Photos/Korea
  -> save to content/life/korea/

Deletes existing compressed images before re-compressing.
"""

import os
import sys
from PIL import Image

MAX_PX = 1920
QUALITY = 80


def compress(source_dir: str, slug: str):
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    dest_dir = os.path.join(project_root, "content", "life", slug)

    # Clear old compressed images, keep index.md
    for f in os.listdir(dest_dir):
        if f.lower().endswith(('.jpg', '.jpeg', '.png')):
            os.remove(os.path.join(dest_dir, f))

    files = sorted(
        f for f in os.listdir(source_dir)
        if f.lower().endswith(('.jpg', '.jpeg', '.png'))
    )

    if not files:
        print(f"No images found in {source_dir}")
        return

    before_total = after_total = 0

    for i, f in enumerate(files):
        src = os.path.join(source_dir, f)
        dst = os.path.join(dest_dir, f)

        before_total += os.path.getsize(src)

        img = Image.open(src).convert("RGB")
        w, h = img.size

        if max(w, h) > MAX_PX:
            ratio = MAX_PX / max(w, h)
            img = img.resize((int(w * ratio), int(h * ratio)), Image.LANCZOS)

        img.save(dst, "JPEG", quality=QUALITY, optimize=True)
        after_total += os.path.getsize(dst)

        if (i + 1) % 10 == 0:
            print(f"  {i + 1}/{len(files)}")

    reduction = (1 - after_total / before_total) * 100
    print(f"\n[{slug}] {len(files)} photos")
    print(f"  {before_total/1024/1024:.1f} MB -> {after_total/1024/1024:.1f} MB ({reduction:.0f}% smaller)")
    print(f"  saved to: {dest_dir}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compress_photos.py <source_dir> <slug>")
        print('Example: python compress_photos.py "G:/Blog_Photos/Korea" korea')
        sys.exit(1)

    src_dir = sys.argv[1]
    slug = sys.argv[2]
    compress(src_dir, slug)
