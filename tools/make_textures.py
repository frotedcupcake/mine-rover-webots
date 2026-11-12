#!/usr/bin/env python3
"""Generate the procedural textures used by the mine worlds.

Webots R2025a does not ship its texture library on disk (it downloads assets from
GitHub on first use), so the mine uses its own small procedural textures instead.
No network access is needed at runtime.  Run:  .venv/bin/python tools/make_textures.py
"""
import os
import numpy as np
from PIL import Image, ImageFilter

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "protos", "textures")
N = 512


def tileable_noise(rng, n, octaves=6, persistence=0.55):
    """Sum of value-noise octaves, range 0..1."""
    out = np.zeros((n, n))
    amp, total = 1.0, 0.0
    for o in range(octaves):
        cells = 4 * 2 ** o
        if cells > n:
            break
        g = rng.random((cells, cells))
        img = Image.fromarray((g * 255).astype(np.uint8)).resize((n, n), Image.BICUBIC)
        out += amp * np.asarray(img, dtype=float) / 255.0
        total += amp
        amp *= persistence
    out /= total
    return (out - out.min()) / (out.max() - out.min())


def cracks(rng, n, count=14):
    img = Image.new("L", (n, n), 0)
    px = img.load()
    for _ in range(count):
        x, y = rng.integers(0, n, 2)
        ang = rng.random() * 2 * np.pi
        for _ in range(int(rng.integers(60, 260))):
            ang += rng.normal(0, 0.35)
            x = (x + np.cos(ang)) % n
            y = (y + np.sin(ang)) % n
            px[int(x), int(y)] = 255
    img = img.filter(ImageFilter.GaussianBlur(0.8))
    return np.asarray(img, dtype=float) / 255.0


def colourise(v, dark, light):
    dark, light = np.array(dark, float), np.array(light, float)
    return (dark[None, None, :] + (light - dark)[None, None, :] * v[:, :, None]).clip(0, 255).astype(np.uint8)


def save(arr, name):
    os.makedirs(OUT, exist_ok=True)
    Image.fromarray(arr).save(os.path.join(OUT, name), quality=88)
    print("wrote", name)


def main():
    rng = np.random.default_rng(357)
    # rock wall: grey-brown, blotchy, cracked
    v = tileable_noise(rng, N) * 0.75 + tileable_noise(rng, N, octaves=3) * 0.25
    v = np.clip(v - 0.55 * cracks(rng, N), 0, 1)
    save(colourise(v, (38, 32, 27), (150, 132, 112)), "rock.jpg")
    # floor: darker packed dirt with gravel speckle
    v = tileable_noise(rng, N) * 0.8
    speck = (rng.random((N, N)) > 0.985).astype(float)
    speck = np.asarray(Image.fromarray((speck * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(1.2)), float) / 255
    v = np.clip(v + 1.6 * speck, 0, 1)
    save(colourise(v, (30, 25, 20), (128, 108, 86)), "floor.jpg")
    # rubble: lighter, freshly broken grey rock
    v = tileable_noise(rng, N, octaves=7, persistence=0.65)
    v = np.clip(v - 0.4 * cracks(rng, N, 20), 0, 1)
    save(colourise(v, (60, 56, 52), (176, 166, 152)), "rubble.jpg")
    # wood: stretched grain
    g = tileable_noise(rng, N, octaves=5)
    grain = np.asarray(Image.fromarray((g * 255).astype(np.uint8)).resize((N, N // 16)).resize((N, N), Image.BICUBIC), float) / 255
    rings = 0.5 + 0.5 * np.sin(grain * 40.0)
    v = np.clip(0.55 * grain + 0.3 * rings + 0.15 * tileable_noise(rng, N, 3), 0, 1)
    save(colourise(v, (52, 33, 17), (140, 98, 56)), "wood.jpg")


if __name__ == "__main__":
    main()
