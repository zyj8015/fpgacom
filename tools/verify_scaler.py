"""Optional user-run scaler simulation. Does not run as part of a TD build."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/scaler_unit_user"


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
    run("compile", ["vlog", "-sv", "-lint", "-work", "work",
                    str(ROOT / "src/user_source/hdl_source/SD/bmp_read.v"),
                    str(ROOT / "verification/tb_bmp_scaler.v")])
    for width,height in [(640,480),(1280,720)]:
        for fill in (0,1):
            log = run(f"simulate_{width}x{height}_fill{fill}", ["vsim", "-c", "-lib", "work",
                      f"-gOUTPUT_WIDTH={width}",f"-gOUTPUT_HEIGHT={height}",f"-gFILL_MODE={fill}",
                      "tb_bmp_scaler", "-do", "run -all; quit -f"])
            if "PASS all scaler cases" not in log:
                raise SystemExit("No completion marker; inspect simulation log")
    print("Scaler unit simulation passed; full SD/SDRAM/HDMI and physical timing remain separate checks.")


if __name__ == "__main__":
    main()
