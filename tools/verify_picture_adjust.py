"""Optional USER-RUN buttons/pixel/overlay tests; no TD build or bit generation."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/picture_adjust_user"


def run(name, args):
    result = subprocess.run(args, cwd=OUT, capture_output=True, text=True, errors="replace")
    log = result.stdout + result.stderr
    (OUT / f"{name}.log").write_text(log, encoding="utf-8")
    print(log)
    if result.returncode or "** Error:" in log or "** Fatal:" in log:
        raise SystemExit(f"{name} failed")
    return log


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if not (OUT / "work").exists():
        run("vlib", ["vlib", "work"])
    hdl = ROOT / "src/user_source/hdl_source"
    run("compile", ["vlog", "-sv", "-lint", str(hdl / "top_tf_hdmi_audio.v"),
                    str(hdl / "SD/sd_card_bmp.v"), str(ROOT / "verification/tb_picture_adjust.v")])
    for bench, marker in [("tb_picture_buttons", "PICTURE_BUTTONS_PASS:"),
                          ("tb_picture_pixels", "PICTURE_PIXELS_PASS:")]:
        log = run(bench, ["vsim", "-c", "work." + bench, "-do", "run -all; quit -f"])
        if marker not in log:
            raise SystemExit("Missing completion marker")


if __name__ == "__main__":
    main()
