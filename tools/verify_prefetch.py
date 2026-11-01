"""Optional USER-RUN scheduling unit test. Does not run TD or generate a bit."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/prefetch_unit_user"


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
    sd = ROOT / "src/user_source/hdl_source/SD"
    sources = [sd / name for name in ("sd_card_bmp.v", "bmp_read.v", "sd_card_top.v",
               "sd_card_sec_read_write.v", "sd_card_cmd.v", "spi_master.v",
               "frame_read_write.v", "frame_fifo_write.v", "frame_fifo_read.v")]
    ip=ROOT/"src/user_source/hdl_source/IP"
    sources += [ip/"afifo_16_32_256.v",ip/"afifo_32_16_256.v"]
    run("compile", ["vlog", "-sv", "-lint", *map(str, sources),
                    str(ROOT / "verification/tb_album_prefetch.v"),
                    str(ROOT / "verification/tb_manual_frame_integrity.v")])
    for bench, marker in [("tb_album_prefetch", "ALBUM_PREFETCH_PASS:"),
                          ("tb_sector_cancel", "SECTOR_CANCEL_PASS:")]:
        log = run(bench, ["vsim", "-c", "work." + bench,
                          "-do", "run -all; quit -f"])
        if marker not in log:
            raise SystemExit("No completion marker; inspect simulation log")
    for name,bench,extra,marker in (
        ("memory_cancel","tb_album_prefetch",["-gMEMORY_CANCEL=1"],"ALBUM_PREFETCH_PASS:"),
        ("frame_integrity","tb_manual_frame_integrity",[],"MANUAL_FRAME_INTEGRITY_PASS:")):
        log=run(name,["vsim","-c","work."+bench,*extra,"-do","run -all; quit -f"])
        if marker not in log:raise SystemExit("No completion marker: "+name)


if __name__ == "__main__":
    main()
