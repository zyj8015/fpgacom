"""Optional USER-RUN scene simulation; no TD synthesis or bit generation."""
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"build/teaching_scene_user"


def run(name,args):
    p=subprocess.run(args,cwd=OUT,capture_output=True,text=True,errors="replace")
    log=p.stdout+p.stderr
    (OUT/(name+".log")).write_text(log,encoding="utf-8")
    print(log)
    if p.returncode or "** Error:" in log or "** Fatal:" in log:
        raise SystemExit(name+" failed")
    return log


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    if not (OUT/"work").exists():run("vlib",["vlib","work"])
    sd=ROOT/"src/user_source/hdl_source/SD"
    sources=[ROOT/"src/user_source/hdl_source/top_tf_hdmi_audio.v"]+[
        sd/name for name in ("sd_card_bmp.v","bmp_read.v","sd_card_top.v",
                             "sd_card_sec_read_write.v","sd_card_cmd.v","spi_master.v")]
    run("compile",["vlog","-sv","-lint",*map(str,sources),
                   str(ROOT/"verification/tb_teaching_scene.v"),str(ROOT/"verification/tb_ppt_extensions.v")])
    for bench,marker in (("tb_teaching_scene","TEACHING_SCENE_PASS:"),
                         ("tb_slide_transitions","SLIDE_TRANSITIONS_PASS:"),
                         ("tb_album_transition_control","ALBUM_TRANSITION_CONTROL_PASS:")):
        log=run(bench,["vsim","-c","work."+bench,"-do","run -all; quit -f"])
        if marker not in log:raise SystemExit("No completion marker: "+bench)


if __name__=="__main__":main()
