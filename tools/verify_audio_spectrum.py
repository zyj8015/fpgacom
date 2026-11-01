"""Goertzel fixed-point regression plus independent floating DFT checks."""
from pathlib import Path
import cmath
import json
import math
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/audio_spectrum_filled_user"
BINS = list(range(1, 33))
LOG_BINS = []
for i in range(96):
    target = round(33*(213/33)**(i/95))
    lower = LOG_BINS[-1]+1 if LOG_BINS else 33
    upper = 213-(95-i)
    LOG_BINS.append(max(lower, min(upper, target)))
BINS += LOG_BINS
assert len(BINS) == 128 and len(set(BINS)) == 128
CASE_COUNT = len(BINS) + 5


def height(magnitude):
    if magnitude < 16:
        return 0
    bit = magnitude.bit_length() - 1
    if bit >= 19:
        return 63
    return min(63, (bit - 4) * 4 + ((magnitude >> (bit - 2)) & 3) + 1)


def analyze(samples):
    x = [min(511, (value + 32) >> 6) for value in samples]
    magnitudes, heights = [], []
    for k in BINS:
        angle = 2 * math.pi * k / 512
        coeff = round(32768 * math.cos(angle))
        cosine, sine = round(16384 * math.cos(angle)), round(16384 * math.sin(angle))
        a = b = 0
        for value in x:
            a, b = ((coeff * a + 8192) >> 14) - b + value, a
            assert -(1 << 25) <= a < (1 << 25), "State width insufficient"
        real, imag = ((cosine * a + 8192) >> 14) - b, (sine * a + 8192) >> 14
        high, low = sorted((abs(real), abs(imag)), reverse=True)
        magnitude = high + low // 2
        magnitudes.append(magnitude)
        heights.append(height(magnitude))
    dft = [abs(sum(value * cmath.exp(-2j * math.pi * k * n / 512)
                   for n, value in enumerate(x))) for k in BINS]
    return magnitudes, heights, dft


def vectors():
    cases = [[round(12000 * math.sin(2 * math.pi * k * n / 512)) for n in range(512)] for k in BINS]
    cases += [[round(a * math.sin(2 * math.pi * BINS[5] * n / 512)) for n in range(512)]
              for a in (2048, 12288)]
    cases += [[0] * 512, [32767] * 512,
              [round(16000 * math.sin(2 * math.pi * BINS[3] * n / 512) +
                     8000 * math.sin(2 * math.pi * BINS[12] * n / 512)) for n in range(512)]]
    reports, words = [], []
    for index, values in enumerate(cases):
        magnitudes, levels, dft = analyze(values)
        if index < 128:
            assert max(range(128), key=lambda i: dft[i]) == index
            assert max(range(128), key=lambda i: magnitudes[i]) == index
            assert levels[index] >= max(levels[:index] + levels[index + 1:]) + 8
            assert abs(magnitudes[index] / dft[index] - 1) < 0.3
        if index == 130:
            assert levels == [0] * 128
        if index == 132:
            assert set(sorted(range(128), key=lambda i: magnitudes[i], reverse=True)[:2]) == {3, 12}
        words.append(sum(value << (6 * i) for i, value in enumerate(levels)))
        reports.append({"case": index, "levels": levels, "magnitudes": magnitudes, "floating_dft": dft,
                        "input_rms": math.sqrt(sum(v*v for v in values)/len(values)),
                        "dominant_frequency_hz": BINS[max(range(128), key=lambda i: magnitudes[i])]*48000/512})
    assert reports[129]["levels"][5] >= reports[128]["levels"][5] + 8
    # Same RMS but different frequencies: validates frequency selectivity,
    # separately from the quiet/loud test at one fixed frequency.
    tone_rms = [report["input_rms"] for report in reports[:128]]
    assert max(tone_rms)/min(tone_rms) < 1.001
    print("FREQUENCY_SELECTIVITY_REFERENCE_PASS: 128 tones with equal RMS within 0.1%, distinct dominant frequency columns")
    (OUT / "spectrum_samples.hex").write_text("\n".join(f"{value & 65535:04x}" for case in cases for value in case) + "\n")
    (OUT / "spectrum_expected.hex").write_text("\n".join(f"{word:0192x}" for word in words) + "\n")
    (OUT / "reference.json").write_text(json.dumps({"frequencies_hz": [k * 48000 / 512 for k in BINS],
                                                  "cases": reports}, indent=2))


def run(name, args):
    p = subprocess.run(args, cwd=OUT, capture_output=True, text=True, errors="replace")
    log = p.stdout + p.stderr
    (OUT / (name + ".log")).write_text(log, encoding="utf-8")
    print(log)
    if p.returncode or "** Fatal:" in log or "** Error:" in log:
        raise SystemExit(name + " failed")
    return log


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    vectors()
    if not (OUT / "work").exists():
        run("vlib", ["vlib", "work"])
    run("compile", ["vlog", "-sv", "-lint", str(ROOT / "src/user_source/hdl_source/top_tf_hdmi_audio.v"),
                    str(ROOT / "verification/tb_audio_spectrum.v"),
                    str(ROOT / "verification/tb_audio_spectrum_ui.v")])
    for name, args in (("spectral_vectors", []), ("actual_sample_rate", ["-gCASE_COUNT=3", "-gSAMPLE_CLOCKS=1547"])):
        log = run(name, ["vsim", "-c", "-quiet", "-onfinish", "stop", *args, "work.tb_audio_spectrum",
                         "-do", "run -all; quit -code 0"])
        if "AUDIO_SPECTRUM_PASS:" not in log:
            raise SystemExit(name + " did not finish")
    log = run("spectrum_ui", ["vsim", "-c", "-quiet", "-onfinish", "stop", "work.tb_audio_spectrum_ui",
                              "-do", "run -all; quit -code 0"])
    if "AUDIO_SPECTRUM_UI_PASS:" not in log:
        raise SystemExit("Spectrum UI did not finish")


if __name__ == "__main__":
    main()
