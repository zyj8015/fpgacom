"""Prepare 12 numbered, border-free test inputs and FPGA-coordinate previews.

Offline crop/resize prepares different inputs. FPGA still performs the final
center crop and scale to 1280x720. This tool does not modify original PNGs.
"""
import argparse
import hashlib
import json
import struct
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw

PLAN = [(896, 504), (960, 540), (1280, 720), (1366, 768),
        (1024, 576), (1440, 810), (1152, 648), (1536, 864),
        (1600, 900), (1200, 675), (1280, 768), (1600, 800)]

# Current supplied page10 is sideways. Correct only its generated copy when
# portrait; a later, already-landscape replacement must not be rotated again.
PORTRAIT_CLOCKWISE_PAGES = {10}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def crop_geometry(w, h, ow, oh, bottom_up=False):
    cw, ch = (h * ow // oh, h) if w * oh >= h * ow else (w, w * oh // ow)
    left, bottom = (w-cw)//2, (h-ch)//2
    top = h-ch-bottom if bottom_up else bottom
    return (left, top, left+cw, top+ch)


def fpga_preview(image):
    w, h = image.size
    box = crop_geometry(w, h, 1280, 720, bottom_up=True)
    cw, ch = box[2]-box[0], box[3]-box[1]
    bottom = h-box[3]
    # Output stream is bottom-up; frame writer reverses rows for display.
    sx = box[0] + np.arange(1280) * cw // 1280
    sy = h-1-(bottom + np.arange(720)[::-1] * ch // 720)
    return Image.fromarray(np.asarray(image)[sy[:, None], sx[None, :]]), box


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path, help="New flat folder containing only the 12 BMP files")
    parser.add_argument("--previews", type=Path, help="Optional separate preview folder")
    parser.add_argument("--manifest", type=Path, help="Optional separate JSON metadata file")
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if output.exists():
        raise SystemExit("Choose a new package directory; no existing package is overwritten.")
    inputs = [source / f"{i}.png" for i in range(1, 13)]
    hashes = {p.name: digest(p) for p in inputs}
    sd = output
    previews = args.previews.resolve() if args.previews else None
    if previews is not None and (previews == sd or previews.exists()):
        raise SystemExit("Choose a new, separate preview folder.")
    sd.mkdir(parents=True)
    if previews is not None:
        previews.mkdir(parents=True)
    records = []
    contact = Image.new("RGB", (960, 6*205), (220, 220, 220)) if previews is not None else None
    draw = ImageDraw.Draw(contact) if contact is not None else None
    for n, (path, target) in enumerate(zip(inputs, PLAN), 1):
        with Image.open(path) as original:
            original = original.convert("RGB")
            source_size = original.size
            rotation = 0
            if n in PORTRAIT_CLOCKWISE_PAGES and original.height > original.width:
                original = original.transpose(Image.Transpose.ROTATE_270)
                rotation = 90
            preparation_box = crop_geometry(*original.size, *target)
            image = original.crop(preparation_box).resize(target, Image.Resampling.LANCZOS)
            dest = sd / f"{n}.BMP"
            image.save(dest, format="BMP")
            raw = dest.read_bytes()
            w, h, planes, bits, compression = struct.unpack_from("<iiHHI", raw, 18)
            stride = (w*3+3) & ~3
            assert raw[:2] == b"BM" and struct.unpack_from("<II", raw, 10) == (54, 40)
            assert (w,h) == target and (planes,bits,compression) == (1,24,0)
            assert len(raw) == 54+stride*h
            with Image.open(dest) as check:
                assert check.convert("RGB").tobytes() == image.tobytes()
            hardware_box = crop_geometry(w, h, 1280, 720, bottom_up=True)
            cw,ch = hardware_box[2]-hardware_box[0],hardware_box[3]-hardware_box[1]
            if previews is not None:
                expected, _ = fpga_preview(image)
                expected.save(previews / f"{n}_expected.png")
                region = image.copy()
                ImageDraw.Draw(region).rectangle(
                    (hardware_box[0],hardware_box[1],hardware_box[2]-1,hardware_box[3]-1),
                    outline="red",width=3)
                region.save(previews / f"{n}_crop_region.png")
                thumb = expected.copy()
                thumb.thumbnail((470,180))
                x,y = ((n-1)%2)*480, ((n-1)//2)*205
                contact.paste(thumb,(x,y+20))
                draw.text((x+3,y+3),f'{n:02d}: {w}x{h} -> crop {cw}x{ch} -> 1280x720',fill="black")
            # Which ORIGINAL area survives both preparation and hardware crop?
            pw,ph=preparation_box[2]-preparation_box[0],preparation_box[3]-preparation_box[1]
            surviving=[preparation_box[0]+hardware_box[0]*pw/w,
                       preparation_box[1]+hardware_box[1]*ph/h,
                       preparation_box[0]+hardware_box[2]*pw/w,
                       preparation_box[1]+hardware_box[3]*ph/h]
            records.append(dict(page=n,source=path.name,source_size=source_size,
                clockwise_rotation=rotation,oriented_size=original.size,
                preparation_crop=list(preparation_box),input_size=target,
                bmp=dest.name,wav=f"{n}.WAV",stride=stride,padding=stride-w*3,
                fpga_crop=list(hardware_box),crop_size=[cw,ch],
                surviving_original_box=surviving,added_border=False,sha256=digest(dest)))
    if previews is not None:
        contact.save(previews / "preview_contact.png")
    assert hashes == {p.name:digest(p) for p in inputs}
    manifest = dict(strategy="FPGA center crop + nearest fill",output_hdmi=[1280,720],
        source_hashes=hashes,added_borders=False,
        preview_note="Python coordinate reference, not RTL simulation or hardware evidence",
        pages=records)
    if args.manifest:
        args.manifest.write_text(json.dumps(manifest,ensure_ascii=True,indent=2),encoding="utf-8")
    print(json.dumps(manifest,ensure_ascii=True,indent=2))


if __name__ == "__main__":
    main()
