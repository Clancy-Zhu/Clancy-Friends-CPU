`timescale 1ns / 1ps `default_nettype none

module btb (
    input wire clk,
    input wire reset,

    input wire rst_cache_i,    // reset all entries
    input wire [31:0] pc_i,    // pc input
    output reg [31:0] pc_o,    // pc output

    input wire write_en_i,           // enable write
    input wire [31:0] adr_i,   // address input
    input wire [31:0] target_i,// target input
    input wire is_branch_i,    // is a branch instruction
    input wire is_jump_i       // is jump in this time
  );

  logic [29:0] entry[8];
  logic [29:0] target[8];
  logic [7:0] entry_valid;
  logic [2:0] lru_counter[8]; // replace LRU
  logic [1:0] counter[8]; // we use 2-bit counter

  logic [31:0] pc_out;
  logic [31:0] pc_next;

  assign pc_o = pc_out;
  assign pc_next = pc_i + 4;

  always_comb
  begin
    pc_out = pc_next;
    for(int i=0;i<8;i=i+1)
    begin
      if(entry_valid[i] && entry[i]==pc_i[31:2] && counter[i][1])
      begin
        pc_out = {target[i],2'b00};
      end
    end
  end

  logic write_hit;              // hit when has a same entry
  logic [2:0] write_hit_index;  // when hit: the index of hit entry
  logic [2:0] replace_index;    // when not hit: the index of LRU entry
  always_comb
  begin
    write_hit = 0;
    write_hit_index = 0;
    for(int i=0;i<8;i=i+1)
    begin
      if(entry_valid[i] && entry[i]==adr_i[31:2])
      begin
        write_hit = 1;
        write_hit_index = i;
      end
      if(lru_counter[i]==3'b000)
      begin
        replace_index = i;
      end
    end
  end


  always_ff @(posedge clk or posedge reset)
  begin
    if(reset || rst_cache_i)
    begin
      for(int i=0;i<8;i=i+1)
      begin
        entry[i] <= 30'b0;
        target[i] <= 30'b0;
        entry_valid[i] <= 1'b0;
        lru_counter[i] <= 8'b0;
        counter[i] <= 2'b0;
      end
    end
    else
      if(write_en_i)
      begin
        for(int i=0;i<8;i=i+1)
        begin
          lru_counter[i] <= (lru_counter[i] == 3'b000) ? 3'b000 : lru_counter[i] - 1; // reduce all counter which is not hit
          if(write_hit && write_hit_index==i) // hit
          begin
            entry_valid[i] <= 1'b1;
            entry[i] <= adr_i[31:2];
            target[i] <= target_i[31:2];
            lru_counter[i] <= 3'b111; // hit entry counter = 7
            counter[i] <= is_jump_i ? (counter[i] == 2'b11 ? 2'b11 : counter[i] + 1) : (counter[i] == 2'b00 ? 2'b00 : counter[i] - 1); // jump: counter + 1; not jump: counter - 1
          end
          else if(!write_hit && replace_index==i) // not hit
          begin
            entry_valid[i] <= 1'b1;
            entry[i] <= adr_i[31:2];
            target[i] <= target_i[31:2];
            lru_counter[i] <= 3'b111; // hit entry counter = 7
            counter[i] <= is_branch_i ? (is_jump_i ? 2'b10 : 2'b01) : 2'b11; // branch jump: 10; branch not jump: 01; not branch: 11
          end
        end
      end
  end

endmodule

