"""Optional user-run FAT32/album and internal test-audio simulations."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
from io import BytesIO
from PIL import Image, ImageOps

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "sim"
HDL = ROOT / "src" / "user_source" / "hdl_source"


def run(command, log):
    result = subprocess.run(command, cwd=BUILD, capture_output=True, text=True, errors="replace")
    output = result.stdout + result.stderr
    (BUILD / log).write_text(output, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"{command[0]} failed; see {BUILD / log}\n{output[-3000:]}")
    return output


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    disk = bytearray([255]) * (5417 * 512)
    for index, sector in enumerate((8, 1812, 3616), 1):
        # Recreate legacy fixtures in memory; no obsolete BMP folders needed.
        source = ROOT.parents[1] / "电子相册" / f"img{index:03d}.bmp"
        with Image.open(source) as opened:
            photo = ImageOps.exif_transpose(opened).convert("RGB")
            fitted = ImageOps.contain(photo, (640, 480), Image.Resampling.LANCZOS)
            canvas = Image.new("RGB", (640, 480), (0, 0, 0))
            canvas.paste(fitted, ((640-fitted.width)//2, (480-fitted.height)//2))
            stream = BytesIO()
            canvas.save(stream, format="BMP")
            data = stream.getvalue()
        disk[sector * 512:sector * 512 + len(data)] = data
    (BUILD / "photos.hex").write_text("\n".join(f"{b:02x}" for b in disk) + "\n", encoding="ascii")
    for tool in ("vlib", "vlog", "vsim"):
        if shutil.which(tool) is None:
            raise RuntimeError(f"ModelSim tool not found: {tool}")
    if not (BUILD / "work").exists():
        run(["vlib", "work"], "vlib.log")
    sources = [HDL / "SD" / "bmp_read.v", HDL / "SD" / "sd_card_bmp.v"]
    # hdmi_test_audio lives beside top, so TD's cached file list cannot omit it.
    sources += [HDL / "top_tf_hdmi_audio.v"]
    sources += [ROOT / "verification" / n for n in (
        "fat32_sector_model.v", "tb_photos.v", "tb_reference_audio.v", "tb_reference_album.v")]
    run(["vlog", "-quiet", "-sv", *map(str, sources)], "compile.log")
    results = {}
    for bench, marker in (("tb_bmp_sector_boundaries", "BMP_SECTOR_BOUNDARIES_PASS"),
                          ("tb_photos", "PHOTOS_PASS"), ("tb_reference_audio", "TEST_AUDIO_PASS"),
                          ("tb_reference_album", "REFERENCE_ALBUM_PASS")):
        output = run(["vsim", "-c", "-quiet", "-onfinish", "stop", "work." + bench,
                      "-do", "onerror {quit -code 1}; onbreak {quit -code 1}; run -all; quit -code 0"], bench + ".log")
        if marker not in output:
            raise RuntimeError(f"Missing PASS marker for {bench}; see its log")
        results[bench] = {"passed": True, "log": f"sim/{bench}.log"}
        print("\n".join(line for line in output.splitlines() if "PASS" in line))
    (ROOT / "build" / "test_results.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    source_hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (ROOT / "build" / "test_source_sha256.json").write_text(json.dumps(source_hashes, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
