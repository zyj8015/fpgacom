"""Optional user-run 720p timing/OSD simulation; no TD build is launched."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/video_720p_user"


def run(name, args):
    result = subprocess.run(args, cwd=OUT, capture_output=True, text=True, errors="replace")
    log = result.stdout + result.stderr
    (OUT / f"{name}.log").write_text(log, encoding="utf-8")
    print(log)
    if result.returncode or "** Fatal:" in log or "** Error:" in log:
        raise SystemExit(f"{name} failed")
    return log


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if not (OUT / "work").exists():
        run("vlib", ["vlib", "work"])
    hdl = ROOT / "src/user_source/hdl_source"
    run("compile", ["vlog", "-sv", "-lint", "-work", "work", f"+incdir+{hdl / 'SD'}",
                    str(hdl / "top_tf_hdmi_audio.v"),
                    str(hdl / "SD/sd_card_bmp.v"),
                    str(hdl / "video_rgb_to_axis_640x480.v"),
                    str(hdl / "SD/video_timing_data.v"), str(hdl / "SD/color_bar.v"),
                    str(ROOT / "verification/tb_video_720p_osd.v")])
    log = run("simulate", ["vsim", "-c", "-lib", "work", "tb_video_720p_osd",
                           "-do", "run -all; quit -f"])
    if "VIDEO_720P_OSD_PASS:" not in log:
        raise SystemExit("No completion marker")


if __name__ == "__main__":
    main()
