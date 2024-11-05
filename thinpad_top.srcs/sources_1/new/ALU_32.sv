`timescale 1ns / 1ps `default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2023/04/24 14:32:13
// Design Name:
// Module Name: ALU_32
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module ALU_32 (
    input  wire [31:0] A,
    input  wire [31:0] B,
    input  wire [ 4:0] op,
    output wire [31:0] Y
);

  typedef enum logic [4:0] {
    NULL,
    ADD,
    SUB,
    AND,
    OR,
    XOR,
    NOT,
    SLL,
    SRL,
    SRA,
    ROL,
    SLT,
    SLTU,
    ANDN,
    SBSET,
    PCNT,
    CRAS16
  } state_t;
  logic [31:0] out;
  logic [1:0] out0[0:15];
  logic [2:0] out1[0:7];
  logic [3:0] out2[0:3];
  logic [4:0] out3[0:1];
  always_comb begin
    case (op)
      NULL: begin
        out = 32'b0;
      end
      ADD: begin
        out = A + B;
      end
      SUB: begin
        out = A - B;
      end
      AND: begin
        out = A & B;
      end
      OR: begin
        out = A | B;
      end
      XOR: begin
        out = A ^ B;
      end
      NOT: begin
        out = ~A;
      end
      SLL: begin
        out = A << (B[4:0]);
      end
      SRL: begin
        out = A >> (B[4:0]);
      end
      SRA: begin
        out = $signed(A) >>> (B[4:0]);
      end
      ROL: begin
        out = {A << (B[4:0]), A >> (32 - B[4:0])};
      end
      SLT: begin
        out = $signed(A) < $signed(B) ? 32'b1 : 32'b0;
      end
      SLTU: begin
        out = $unsigned(A) < $unsigned(B) ? 32'b1 : 32'b0;
      end
      ANDN: begin
        out = A & ~B;
      end
      SBSET: begin
        out = A | (1 << B[4:0]);
      end
      PCNT: begin
        for(int i = 0; i < 16; i = i + 1) begin
          out0[i] = A[i * 2] + A[i * 2 + 1];
        end
        for(int i = 0; i < 8; i = i + 1) begin
          out1[i] = out0[i * 2] + out0[i * 2 + 1];
        end
        for(int i = 0; i < 4; i = i + 1) begin
          out2[i] = out1[i * 2] + out1[i * 2 + 1];
        end
        for(int i = 0; i < 2; i = i + 1) begin
          out3[i] = out2[i * 2] + out2[i * 2 + 1];
        end
        out = out3[0] + out3[1];
      end
      CRAS16: begin
        out[31:16] = A[31:16] + B[15:0];
        out[15:0] = A[15:0] - B[31:16];
      end
      default: begin
        out = 32'b0;
      end
    endcase
  end

  assign Y = out;
endmodule
