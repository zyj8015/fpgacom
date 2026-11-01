"""Convert 48 kHz 24-bit stereo WAV files to the TF-card format.

The FPGA WAV player expects RIFF/WAVE PCM, 48 kHz, signed 16-bit little-endian,
mono. The default downmix is the arithmetic mean of the two source channels,
with rounding and no gain normalization or duration change.
"""
from pathlib import Path
import argparse
import hashlib
import json
import wave

TARGET_RATE = 48000


def decode_s24(data, offset):
    value = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16)
    return value - (1 << 24) if value & 0x800000 else value


def round_div(value, divisor):
    return (value + divisor // 2) // divisor if value >= 0 else -((-value + divisor // 2) // divisor)


def convert(src, dst, downmix):
    with wave.open(str(src), 'rb') as reader:
        channels = reader.getnchannels()
        width = reader.getsampwidth()
        rate = reader.getframerate()
        frames = reader.getnframes()
        if (channels, width, rate) != (2, 3, TARGET_RATE):
            raise ValueError(f'{src.name}: expected stereo 24-bit 48 kHz, got '
                             f'{channels}ch/{width * 8}bit/{rate}Hz')
        raw = reader.readframes(frames)

    left_sq = right_sq = 0
    left_peak = right_peak = 0
    values = []
    for pos in range(0, frames * 6, 6):
        left = decode_s24(raw, pos)
        right = decode_s24(raw, pos + 3)
        values.append((left, right))
        left_sq += left * left
        right_sq += right * right
        left_peak = max(left_peak, abs(left))
        right_peak = max(right_peak, abs(right))

    use_right = right_sq > left_sq
    mono = bytearray(frames * 2)
    peak = 0
    clamps = 0
    sum_sq = 0
    for index, (left, right) in enumerate(values):
        if downmix == 'average':
            mixed = round_div(left + right, 2)
        else:
            mixed = right if use_right else left
        sample = round_div(mixed, 256)
        if sample > 32767:
            sample = 32767
            clamps += 1
        elif sample < -32768:
            sample = -32768
            clamps += 1
        peak = max(peak, abs(sample))
        sum_sq += sample * sample
        offset = index * 2
        mono[offset] = sample & 0xff
        mono[offset + 1] = (sample >> 8) & 0xff

    with wave.open(str(dst), 'wb') as writer:
        writer.setnchannels(1)
        writer.setsampwidth(2)
        writer.setframerate(TARGET_RATE)
        writer.writeframes(mono)

    return {
        'source': src.name,
        'source_sha256': hashlib.sha256(src.read_bytes()).hexdigest(),
        'output': dst.name,
        'output_sha256': hashlib.sha256(dst.read_bytes()).hexdigest(),
        'downmix': downmix,
        'selected_channel': ('right' if use_right else 'left') if downmix == 'dominant' else None,
        'left_rms_24bit': round((left_sq / frames) ** 0.5, 2),
        'right_rms_24bit': round((right_sq / frames) ** 0.5, 2),
        'left_peak_24bit': left_peak,
        'right_peak_24bit': right_peak,
        'frames': frames,
        'duration_seconds': round(frames / TARGET_RATE, 3),
        'rms': round((sum_sq / frames) ** 0.5, 2),
        'peak_amplitude': peak,
        'quantization_upper_bound_clamps': clamps,
        'source_format': {'channels': 2, 'bits_per_sample': 24, 'sample_rate': TARGET_RATE},
        'target_format': {'channels': 1, 'bits_per_sample': 16, 'sample_rate': TARGET_RATE},
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source_dir', type=Path)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--downmix', choices=('average', 'dominant'), default='average')
    args = parser.parse_args()
    sources = [args.source_dir / f'{n}.wav' for n in range(1, 13)]
    missing = [str(p) for p in sources if not p.is_file()]
    if missing:
        raise SystemExit('Missing source files: ' + ', '.join(missing))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    files = []
    for source in sources:
        target = args.output_dir / f'{source.stem}.WAV'
        files.append(convert(source, target, args.downmix))

    method = (
        'Arithmetic stereo mean, round to nearest; 24-to-16-bit round to nearest; '
        'no normalization, limiting, trimming, resampling or duration change'
        if args.downmix == 'average' else
        'Select the higher-RMS stereo channel; round 24-bit samples to 16-bit; '
        'no normalization, limiting, trimming, resampling or gain change'
    )
    manifest = {
        'method': method,
        'downmix': args.downmix,
        'total_duration_seconds': round(sum(x['duration_seconds'] for x in files), 3),
        'files_verified': len(files),
        'files': files,
    }
    if args.manifest:
        args.manifest.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps({'converted': len(files), 'downmix': args.downmix,
                      'total_duration_seconds': manifest['total_duration_seconds']}))


if __name__ == '__main__':
    main()

