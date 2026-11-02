import math

rows = []
for i in range(256):
    v = round(math.sin(2 * math.pi * i / 256) * 32767)
    if v < 0:
        lit = f"-16'sd{-v}"
    else:
        lit = f"16'sd{v}"
    rows.append(f"        8'd{i}: val = {lit};")

text = f"""// 256-entry sine LUT, sin(2*pi*i/256), Q15 signed
module sine_lut (
    input  wire [7:0] idx,
    output reg signed [15:0] val
);
    always @* begin
        case (idx)
{chr(10).join(rows)}
        endcase
    end
endmodule
"""
open(r"D:\DeepSeek\start_move\src\sine_lut.v", "w", encoding="ascii").write(text)
