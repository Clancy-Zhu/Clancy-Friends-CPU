module timer (
    input wire clk,
    input wire reset,
    input wire mtime_e,
    input wire mtimecmp_e,
    input wire [63:0] mtime_i,
    input wire [63:0] mtimecmp_i,
    output reg [63:0] mtime_o,
    output reg [63:0] mtimecmp_o,
    output wire timer_interrupt_o
);

  assign timer_interrupt_o = mtime_o >= mtimecmp_o;

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      mtime_o <= 64'h0;
      mtimecmp_o <= 64'hFFFFFFFFFFFFFFFF;
    end else begin
      mtime_o <= mtime_o + 1'b1;
      if (mtime_e) begin
        mtime_o <= mtime_i;
      end
      if (mtimecmp_e) begin
        mtimecmp_o <= mtimecmp_i;
      end
    end
  end

endmodule
