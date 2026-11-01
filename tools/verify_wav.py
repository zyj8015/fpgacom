"""Optional USER-RUN WAV functional test. Never invoked by a normal TD build."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build" / "wav_unit_user"


def run(name, args):
    completed = subprocess.run(args, cwd=OUT, capture_output=True, text=True, errors="replace")
    text = completed.stdout + completed.stderr
    (OUT / (name + ".log")).write_text(text, encoding="utf-8")
    print(text)
    if completed.returncode or "** Fatal:" in text or "** Error:" in text:
        raise SystemExit(f"{name} failed; inspect {OUT / (name + '.log')}")
    return text


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if not (OUT / "work").exists():
        run("vlib", ["vlib", "work"])
    run("compile", ["vlog", "-sv", "-lint", "-work", "work",
                    str(ROOT / "src/user_source/hdl_source/SD/sd_card_bmp.v"),
                    str(ROOT / "verification/tb_wav_player.v")])
    for name,number in [("1       ",1),("YL000012",12),("12345678",12345678)]:
        encoded=name.encode("ascii").hex()
        result = run(f"simulate_{number}", ["vsim", "-c", "-lib", "work",
                     f"-gNAME_TEXT=64'h{encoded}",f"-gNAME_NUMBER={number}","tb_wav_player",
                     "-do", "run -all; quit -f"])
        if "WAV_PLAYER_PASS:" not in result:
            raise SystemExit("Simulation did not report WAV_PLAYER_PASS; inspect simulation log")
    print("WAV unit test passed. This does not validate full-board HDMI/SD timing.")


if __name__ == "__main__":
    main()
