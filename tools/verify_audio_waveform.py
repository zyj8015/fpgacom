"""PCM waveform/720p pixel regression; no TD build, bit or board access."""
from pathlib import Path
import os
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/audio_waveform_filled_user"


def run(name, args):
    env = dict(os.environ, TMPDIR=str(OUT / "temp"))
    result = subprocess.run(args, cwd=OUT, env=env, capture_output=True, text=True, errors="replace")
    log = result.stdout + result.stderr
    (OUT / (name + ".log")).write_text(log, encoding="utf-8")
    print(log)
    if result.returncode or "** Error:" in log or "** Fatal:" in log:
        raise SystemExit(name + " failed")
    return log


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "temp").mkdir(exist_ok=True)
    if not (OUT / "work").exists():
        run("vlib", ["vlib", "work"])
    run("compile", ["vlog", "-sv", "-lint",
                    str(ROOT / "src/user_source/hdl_source/top_tf_hdmi_audio.v"),
                    str(ROOT / "src/user_source/hdl_source/SD/sd_card_bmp.v"),
                    str(ROOT / "verification/tb_wav_player.v"),
                    str(ROOT / "verification/tb_visual_label_rom.v"),
                    str(ROOT / "verification/tb_audio_waveform.v")])
    log = run("title_rom", ["vsim", "-c", "-quiet", "-onfinish", "stop",
                            "work.tb_visual_label_rom", "-do", "run -all; quit -code 0"])
    if "VISUAL_LABEL_ROM_PASS:" not in log:
        raise SystemExit("Title glyph regression did not complete")
    log = run("simulate", ["vsim", "-c", "-quiet", "-onfinish", "stop",
                           "work.tb_audio_waveform", "-do", "run -all; quit -code 0"])
    if "AUDIO_WAVEFORM_PASS:" not in log:
        raise SystemExit("Waveform simulation did not complete")
    log = run("playback_720p", ["vsim", "-c", "-quiet", "-onfinish", "stop",
                               "-gVIDEO_HZ=74250000", "-gENABLE_WAVEFORM=1",
                               "work.tb_wav_player", "-do", "run -all; quit -code 0"])
    if "WAV_PLAYER_PASS:" not in log:
        raise SystemExit("Actual PCM-bridge tap regression did not complete")


if __name__ == "__main__":
    main()
