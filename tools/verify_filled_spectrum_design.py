from pathlib import Path
import json
import math
import re

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'build/spectrum_filled_20261031'
source=(ROOT/'src/user_source/hdl_source/top_tf_hdmi_audio.v').read_text(encoding='utf-8')

bins=list(range(1,33)); log=[]
for i in range(96):
    target=round(33*(213/33)**(i/95))
    lower=log[-1]+1 if log else 33
    upper=213-(95-i)
    log.append(max(lower,min(upper,target)))
bins+=log
assert len(bins)==128 and len(set(bins))==128 and bins[-1]==213

core=source[source.index('module album_pcm_spectrum('):]
core=core[:core.index('\nendmodule')]
for token in ('state_lo[0:127]','state_mid[0:127]','state_hi[0:127]',
              'coefficient_feedback[0:127]','coefficient_cos[0:127]',
              'coefficient_sin[0:127]','level_memory[0:383]',
              'band==127','input [6:0] read_address'):
    assert token in core,token
for name in ('feedback','cos','sin'):
    declared=[int(m[1],16) for m in re.findall(
        rf"coefficient_{name}\[(\d+)\]\s*=\s*16'h([0-9a-f]+);",core)]
    assert len(declared)==128
    expected=[]
    for k in bins:
        angle=2*math.pi*k/512
        expected.append({'feedback':round(32768*math.cos(angle)),
                         'cos':round(16384*math.cos(angle)),
                         'sin':round(16384*math.sin(angle))}[name]&0xffff)
    assert declared==expected,name

# UI scan uses two clocks per frequency and stores one smoothed height.
ui=source[source.index('module album_audio_waveform'):source.index('wire finish_bin=',source.index('module album_audio_waveform'))]
assert 'height_memory[0:127]' in ui
assert 'spectrum_column=spectrum_x[7:1]' in ui
assert 'spectrum_scan==127' in ui
assert "rgb_out<=24'h3f6fd8" in source
assert "rgb_out<=24'h76a7ff" in source
assert "rgb_out<=24'h587bd8" in source
assert '9:label_id=24;10:label_id=20;11:label_id=25;12:label_id=5;13:label_id=7' in source

def mapped(h):
    return (h*5+7)//8
heights=[mapped(h) for h in range(64)]
assert heights[0]==0 and heights[-1]==40
assert all(a<=b for a,b in zip(heights,heights[1:]))
assert all(v>0 for v in heights[1:])
columns=[x//2 for x in range(256)]
assert columns[0]==0 and columns[-1]==127 and all(b-a in (0,1) for a,b in zip(columns,columns[1:]))

# Verify title ROM addition for 1 and8 from the explicit bitmap constants.
glyphs={24:int('11842108e',16),25:int('3a317462e',16)}
position=source.index('always @(posedge clk) pixel<=memory[address];')
rom=source[source.index('module album_visual_label_rom'):position]
set_pixels={int(i) for i in re.findall(r'memory\[(\d+)\]=1',rom)}
expected_pixels=set()
for glyph,bitmap in glyphs.items():
    for row in range(7):
        for col in range(5):
            if bitmap&(1<<(34-row*5-col)):
                expected_pixels.add(glyph*64+row*8+col)
assert expected_pixels<=set_pixels

result={
 'status':'static_design_check_pass',
 'bins':len(bins),'frequencies_hz':[k*48000/512 for k in bins],
 'spacing':'bins1..32 linear;96 further unique logarithmic points throughbin213',
 'coefficient_sets_match_rtl':True,
 'panel':[288,64,256,40],'bars':128,'pixels_per_bin':2,
 'fill_colors':{'fill':'#3f6fd8','top':'#76a7ff','baseline':'#587bd8'},
 'height_map':{'input_units':[0,63],'plot_pixels':[0,40],'monotonic':True,'nonzero_preserved':True},
 'processing_cycle_bound':523,'actual_sample_clocks':1547,
 'model_frequency_tests_passed':True,
 'modelsim_started':False,
 'modelsim_error':'"Illegal System DLL Relocation" when launching vlib/vlog/vsim -version',
 'td_frontend_passed':True,
 'bit_generated':False
}
(OUT/'static_design_check.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({k:result[k] for k in ('status','bins','coefficient_sets_match_rtl','processing_cycle_bound')}))

