"""Generate literal initialized BRAM ROMs in the registered top source.

No readmemh runtime path dependency; no TF-card/PPT/WAV modifications.
Font and sprite rasterization is offline asset preparation. Scrolling, timing,
zoom, alpha compositing and spatial blur run in the FPGA.
"""
from pathlib import Path
import json
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
HDL = ROOT / "src/user_source/hdl_source"
MARK = "// BEGIN GENERATED TEACHING ASSETS"


def memory(name, width, values, depth=None):
    depth = depth or len(values)
    vals = list(values) + [0] * (depth - len(values))
    out = [f'reg [{width-1}:0] {name} [0:{depth-1}]; /* fehdl force_ram=1, ram_style="bram" */',
           "initial begin"]
    for k in range(0, depth, 8):
        out.append(" ".join(f"{name}[{i}]={width}'h{vals[i]:x};" for i in range(k, min(k+8, depth))))
    return "\n".join(out + ["end"])


def small_lookup(name,width,values,address_width):
    # These small character-ID tables do not need a forced block RAM each.
    out=[f"function [{width-1}:0] {name};",f"input [{address_width-1}:0] address;","begin case(address)"]
    out.extend(f"{address_width}'d{i}:{name}={width}'h{v:x};" for i,v in enumerate(values) if v)
    return "\n".join(out+[f"default:{name}=0; endcase end endfunction"])


def page_text_lookup(strings,glyph_id):
    # Page occupies high address bits. No base+character-index carry chain on
    # the pixel path; the ticker index is already registered by the renderer.
    out=["function [8:0] lookup_text;","input [3:0] text_page;input [6:0] index;",
         "begin case({text_page,index})"]
    for page,s in enumerate(strings,1):
        out.extend(f"11'd{page*128+i}:lookup_text=9'h{glyph_id[ch]:x};" for i,ch in enumerate(s) if glyph_id[ch])
    return "\n".join(out+["default:lookup_text=0; endcase end endfunction"])


def character_rom(strings,glyph_id,focuses):
    # Synchronous ROM reads retain the original ONE-clock ID latency while
    # replacing the large pixel-path case decoder. Explicit nibble banks
    # use spare 9K blocks rather than the already-full 32K block pool.
    text=[0]*2048;focus=[0]*1024
    for page,s in enumerate(strings,1):
        for i,ch in enumerate(s):text[page*128+i]=glyph_id[ch]
    for item in focuses:
        for row,s in enumerate(item['focus_lines']):
            for i,ch in enumerate(s.center(12)):focus[item['page']*64+row*12+i]=glyph_id[ch]
    return '\n'.join((memory('text_id_lo',4,[v&15 for v in text]),
                      memory('text_id_hi',4,[v>>4 for v in text]),
                      memory('focus_id_rom',8,focus)))


def main():
    content = json.loads((ROOT / "tools/teaching_content.json").read_text(encoding="utf-8"))
    strings = [p["subtitle"] for p in content["pages"]]
    if [p["page"] for p in content["pages"]] != list(range(1, 13)):
        raise ValueError("captions must cover pages1..12 in order")
    # Hardware uses a12bit scroll accumulator and7bit character indices.
    if any(not s or len(s)*32+1280 > 4095 for s in strings):
        raise ValueError("caption length exceeds ticker coordinate capacity")
    if not 1 <= content["scroll_pixels_per_frame"] <= 15:
        raise ValueError("scroll step must fit4bits and be nonzero")
    focuses = content["focus_pages"]
    if len({p["page"] for p in focuses}) != len(focuses):
        raise ValueError("duplicate focus page")
    for item in focuses:
        if not 1 <= item["page"] <= 12 or not 1 <= len(item["focus_lines"]) <= 3:
            raise ValueError("focus must use pages1..12 and one to three lines")
        if not 0 <= item["start_cs"] < item["end_cs"] <= 65535:
            raise ValueError("invalid focus time window")
    focus = [line for item in focuses for line in item["focus_lines"]]
    chars = [" "] + sorted(set("".join(strings + focus)) - {" "})
    if len(chars) > 256:
        raise ValueError("font ID capacity exceeded")
    glyph_id = {ch: n for n, ch in enumerate(chars)}
    # Native 32px cells, regular 28px strokes + whitespace. Do not magnify a
    # coarse bold16 glyph for captions: small counters used to merge on screen.
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 28)
    baseline_top=font.getbbox("国")[1]
    font_words = []
    for ch in chars:
        glyph = Image.new("L", (32, 32))
        d = ImageDraw.Draw(glyph)
        if ch not in (" ", "\u3000"):
            box = d.textbbox((0, 0), ch, font=font)
            d.text(((32-(box[2]-box[0]))//2-box[0], 2-baseline_top), ch, font=font, fill=255)
        for y in range(32):
            font_words.append(sum((1 << (31-x)) for x in range(32) if glyph.getpixel((x, y)) >= 128))
    # Lossless glyph-row trimming: retain exact native32px strokes/coordinates,
    # store only the vertical span containing ink. Blank padding returns zero.
    stored_font=[];font_descriptors=[]
    for i in range(len(chars)):
        rows=font_words[i*32:(i+1)*32]
        hits=[n for n,row in enumerate(rows) if row]
        if not hits:
            font_descriptors.append(0)
        else:
            first,last=hits[0],hits[-1]
            font_descriptors.append((len(stored_font)<<11)|((last-first+1)<<5)|first)
            stored_font.extend(rows[first:last+1])
    if len(stored_font)>8192:raise ValueError("glyph span address capacity exceeded")
    text = []; bases = []; lengths = []
    for s in strings:
        bases.append(len(text)); lengths.append(len(s))
        text.extend(glyph_id[c] for c in s)
    focus_lookup = ["function [8:0] lookup_focus;", "input [3:0] text_page;input [5:0] index;",
                    "begin case({text_page,index})"]
    for item in focuses:
        for row, s in enumerate(item["focus_lines"]):
            if len(s) > 12: raise ValueError("focus line wider than12characters")
            # Center each line using blank native cells; no resampling/font copy.
            padded = s.center(12)
            focus_lookup.extend(f"10'd{item['page']*64+row*12+i}:lookup_focus=9'h{glyph_id[ch]:x};"
                                for i,ch in enumerate(padded) if glyph_id[ch])
    focus_lookup = "\n".join(focus_lookup+["default:lookup_focus=0;endcase end endfunction"])

    # Four96x144 sprites.31opaque colors+transparent0. Three5bit pixels share
    # one15bit ROM word, avoiding sparse 5bit-wide BRAM storage overhead.
    atlas = Image.open(HDL / "scene_assets/teacher_atlas.png").convert("RGBA")
    poses = []; bodies=[]
    for n in range(4):
        cell = atlas.crop(((n%2)*atlas.width//2, (n//2)*atlas.height//2,
                           (n%2+1)*atlas.width//2, (n//2+1)*atlas.height//2))
        box = cell.getchannel("A").getbbox()
        if not box: raise ValueError("sprite cell has no alpha content")
        bodies.append(cell.crop(box))
    scale=min(94/max(p.width for p in bodies),142/max(p.height for p in bodies))
    for body in bodies:
        # One scale for all poses, fixed feet alignment, no pose-dependent zoom.
        body=body.resize((max(1,round(body.width*scale)),max(1,round(body.height*scale))),Image.Resampling.LANCZOS)
        tile = Image.new("RGBA", (96, 144))
        tile.alpha_composite(body, ((96-body.width)//2, 144-body.height))
        poses.append(tile)
    # Palette training excludes all invisible RGB pixels. Transparent black
    # used to dominate quantization; no dithering, so fine edges stay clean.
    visible=[p[:3] for tile in poses for p in tile.getdata() if p[3]>=128]
    training=Image.new("RGB",(len(visible),1));training.putdata(visible)
    indexed=training.quantize(colors=31,method=Image.Quantize.MEDIANCUT)
    pal=(indexed.getpalette()+[0]*768)[:768]
    for i in range(31,256): pal[3*i:3*i+3]=pal[:3]
    indexed.putpalette(pal)
    remap=[min(range(31),key=lambda j:sum((pal[3*i+k]-pal[3*j+k])**2 for k in range(3))) for i in range(256)]
    palette = [0]+[(pal[3*i] << 16) | (pal[3*i+1] << 8) | pal[3*i+2] for i in range(31)]
    sprite_words = []
    for tile in poses:
        quantized=tile.convert("RGB").quantize(palette=indexed,dither=Image.Dither.NONE)
        pixels=[]
        for y in range(144):
            for x in range(96):
                opaque = tile.getpixel((x, y))[3] >= 128
                pixels.append(remap[quantized.getpixel((x,y))]+1 if opaque else 0)
        for i in range(0,len(pixels),3):
            sprite_words.append(pixels[i] | (pixels[i+1]<<5) | (pixels[i+2]<<10))
    # Each source row has32 packed3-pixel words. Skip all-transparent margins,
    # recording base/count/first-word in24bits; no opaque sample is discarded.
    stored_sprite=[];sprite_descriptors=[]
    for row in range(4*144):
        words=sprite_words[row*32:(row+1)*32]
        hits=[n for n,word in enumerate(words) if word]
        if not hits:sprite_descriptors.append(0)
        else:
            first,last=hits[0],hits[-1]
            sprite_descriptors.append((len(stored_sprite)<<11)|((last-first+1)<<5)|first)
            stored_sprite.extend(words[first:last+1])
    if len(stored_sprite)>8192:raise ValueError("sprite span address capacity exceeded")

    teacher=content["teacher"]
    runtime=(ROOT/"tools/teaching_scene.v.in").read_text(encoding="utf-8")
    lines = [MARK, runtime, "// Generated by tools/prepare_teaching_assets.py; do not edit ROM literals.",
        "module album_teaching_assets(",
        " input clk,input [3:0] page,input [6:0] character_index,",
        " input [5:0] focus_index,input [12:0] subtitle_font_address,focus_font_address,input focus_font_select,",
        " input [14:0] sprite_address,input [1:0] sprite_lane,",
        " output reg [8:0] character_id,focus_id,",
        " output [31:0] subtitle_font_row,focus_font_row,",
        " output [5:0] sprite_pixel,output reg [6:0] text_length,",
        " output reg [15:0] demo_start_cs,demo_end_cs,output [10:0] teacher_x,teacher_y,",
        " output reg [1:0] focus_lines,output [3:0] scroll_step",
        ");", character_rom(strings,glyph_id,focuses),
        memory("font_span_rom",24,font_descriptors,256),memory("font_rom",32,stored_font),
        memory("sprite_span_rom",24,sprite_descriptors),memory("sprite_rom",15,stored_sprite),
        "reg [23:0] font_span_q,sprite_span_q;",
        "reg [4:0] font_y_q,sprite_x_q; reg [1:0] sprite_lane0,sprite_lane_q;",
        "reg [31:0] font_bits_q; reg font_valid_q,sprite_valid_q; reg [14:0] sprite_word_q;",
        "wire [5:0] font_offset={1'b0,font_y_q}-{1'b0,font_span_q[4:0]};",
        "wire [5:0] sprite_offset={1'b0,sprite_x_q}-{1'b0,sprite_span_q[4:0]};",
        "wire font_hit=font_y_q>=font_span_q[4:0] && font_offset<font_span_q[10:5];",
        "wire sprite_hit=sprite_x_q>=sprite_span_q[4:0] && sprite_offset<sprite_span_q[10:5];",
        "wire [12:0] font_address=font_span_q[23:11]+{7'd0,font_offset};",
        "wire [12:0] sprite_address_compact=sprite_span_q[23:11]+{7'd0,sprite_offset};",
        "wire [12:0] selected_font_address=focus_font_select ? focus_font_address : subtitle_font_address;",
        "assign subtitle_font_row=font_valid_q ? font_bits_q : 32'd0;",
        "wire [4:0] sprite_q=sprite_lane_q==0 ? sprite_word_q[4:0] : sprite_lane_q==1 ? sprite_word_q[9:5] : sprite_word_q[14:10];",
        "assign sprite_pixel=sprite_valid_q ? {sprite_q!=0,sprite_q} : 6'd0;",
        "assign focus_font_row=subtitle_font_row;",
        f"assign teacher_x=11'd{teacher['x']}; assign teacher_y=11'd{teacher['y']};",
        f"assign scroll_step=4'd{content['scroll_pixels_per_frame']};",
        "always @* begin demo_start_cs=0;demo_end_cs=0;focus_lines=0;case(page)"]
    for item in focuses:
        lines.append(f"4'd{item['page']}:begin demo_start_cs=16'd{item['start_cs']};"
                     f"demo_end_cs=16'd{item['end_cs']};focus_lines=2'd{len(item['focus_lines'])};end")
    lines += ["default:;endcase end",
        "always @* begin text_length=0; case(page)"]
    for n in range(12):
        lines.append(f"{n+1}:text_length=7'd{lengths[n]};")
    lines += ["default:; endcase end", "always @(posedge clk) begin",
              " character_id<={1'b0,text_id_hi[{page,character_index}],text_id_lo[{page,character_index}]};",
              " focus_id<={1'b0,focus_id_rom[{page,focus_index}]};",
              " font_span_q<=font_span_rom[selected_font_address[12:5]]; font_y_q<=selected_font_address[4:0];",
              " font_bits_q<=font_rom[font_hit ? font_address : 13'd0]; font_valid_q<=font_hit;",
              " sprite_span_q<=sprite_span_rom[sprite_address[14:5]]; sprite_x_q<=sprite_address[4:0];",
              " sprite_lane0<=sprite_lane; sprite_lane_q<=sprite_lane0;",
              " sprite_word_q<=sprite_rom[sprite_hit ? sprite_address_compact : 13'd0]; sprite_valid_q<=sprite_hit;",
              "end", "endmodule",
              "module album_teacher_palette(input [4:0] index,output reg [23:0] rgb);",
              "always @* case(index)"]
    lines += [f"5'd{i}:rgb=24'h{v:06x};" for i,v in enumerate(palette)]
    lines += ["default:rgb=0; endcase endmodule", "// END GENERATED TEACHING ASSETS", ""]
    top = HDL / "top_tf_hdmi_audio.v"
    source = top.read_text(encoding="utf-8").split(MARK)[0].rstrip()
    top.write_text(source + "\n\n" + "\n".join(lines), encoding="utf-8", newline="\r\n")
    small_table_bits=(sum(lengths)+sum(map(len,focus)))*9
    character_id_rom_bits=(2048+1024)*8
    report = {"glyph_count":len(chars),"subtitle_lengths":lengths,
              "rom_bits":len(stored_font)*32+256*24+len(stored_sprite)*15+576*24+character_id_rom_bits,
              "block_rom_bits":len(stored_font)*32+256*24+len(stored_sprite)*15+576*24+character_id_rom_bits,
              "small_tables_in_logic":False,"character_id_rom_bits":character_id_rom_bits,
              "uncompressed_rom_bits":len(font_words)*32+len(sprite_words)*15+small_table_bits,
              "small_table_logical_bits":small_table_bits,
              "stored_font_rows":len(stored_font),"font_full_rows":len(font_words),
              "stored_sprite_words":len(stored_sprite),"sprite_full_words":len(sprite_words),
              "font_cell":32,"font_size":28,"sprite_cell":[96,144],"opaque_colors":31,
              "characters":chars,"line_buffer_bits":1280*24*2,
              "synthesis_resource_check_pending":True,
              "focus_pages":focuses,"teacher":teacher,"note":"ROM asset generation, not RTL/board validation"}
    out=ROOT/"build/teaching_scene_static";out.mkdir(parents=True,exist_ok=True)
    (out/"asset_manifest.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({"glyphs":len(chars),"lengths":lengths,"rom_bits":report["rom_bits"]}))


if __name__ == "__main__":
    main()
