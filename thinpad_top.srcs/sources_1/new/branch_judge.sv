`default_nettype none `timescale 1ns / 1ps
module branch_judge (
    input  wire [31:0] A,
    input  wire [31:0] B,
    input  wire [ 3:0] op,
    output wire        Y
);

  typedef enum logic [3:0] {
    BEQ,
    BNE,
    BLT,
    BGE,
    BLTU,
    BGEU,
    JAL
  } state_t;
  logic out;

  always_comb begin
    case (op)
      BEQ: begin
        out = (A == B);
      end
      BNE: begin
        out = (A != B);
      end
      BLT: begin
        out = ($signed(A) < $signed(B));
      end
      BGE: begin
        out = ($signed(A) >= $signed(B));
      end
      BLTU: begin
        out = ($unsigned(A) < $unsigned(B));
      end
      BGEU: begin
        out = ($unsigned(A) >= $unsigned(B));
      end
      JAL: begin
        out = 1;
      end
      default: begin
        out = 0;
      end
    endcase
  end

  assign Y = out;
endmodule
