//cache.sv
//author: Gaoxu Guo
//date: 2023/11/24
//description: cache,using ip core block memory generator
//TODO: cache now simply connect to wishbone bus, need to be modified ,considering using mmu interface instead of wishbone,after mmu is finished
//TODO: cache rst is not implemented
//COMMENT: using bram needs 2 clock cycles to read if cache hits,but if using native regsiter, it only needs 1 clock cycle
/*interface:
    input:
        clk: clock
        rst: reset
        rst_cache_i: reset cache ------------------->
        en_i: enable ------------------->
        adr_i: address input ------------------->
    output:
        dat_o: data output ------------------->
        cache_wait_o: cache wait ------------------->

    default wishbone interface:
        wb_cyc_o: wishbone cycle
        wb_stb_o: wishbone strobe
        wb_ack_i: wishbone ack
        wb_adr_o: wishbone address
        wb_dat_o: wishbone data
        wb_dat_i: wishbone data
        wb_sel_o: wishbone select
        wb_we_o:  wishbone write enable
*/
`default_nettype none
module cache (
    input wire clk,
    input wire rst,

    input wire rst_cache_i,  // reset cache
    input wire en_i,  //enable

    input wire [31:0] adr_i,  // address input
    output reg [31:0] dat_o,  // data output
    output reg cache_wait_o,  // cache wait
    output reg cache_ack_o,  // cache ack

    // wishbone interface
    output reg         wb_cyc_o,
    output reg         wb_stb_o,
    input  wire        wb_ack_i,
    output reg  [31:0] wb_adr_o,
    output reg  [31:0] wb_dat_o,
    input  wire [31:0] wb_dat_i,
    output reg  [ 3:0] wb_sel_o,
    output reg         wb_we_o
  );
  logic [56:0] cache_regs[64];

  logic [23:0] cache_tag;
  logic [5:0] cache_index;
  logic [56:0] cache_entry_write;
  logic [56:0] cache_entry_read;

  logic cache_hit;
  logic cache_valid;  //cache_entry_read[56]
  assign cache_entry_read = cache_regs[cache_index];
  assign cache_valid = cache_entry_read[56];
  assign cache_index = adr_i[7:2];
  assign cache_tag = adr_i[31:8];
  assign cache_hit = (cache_valid&cache_tag==cache_entry_read[55:32]);// cache hit if cache valid and cache tag match address

  //memory interface
  logic mem_read_req;
  logic mem_ack;

  enum logic [1:0] {
         CACHE_IDLE,
         CACHE_CHECK_HIT,
         CACHE_MEM_READ,
         CACHE_WRITE
       } cache_state;

  always_ff @(posedge clk)
  begin
    if (rst_cache_i | rst)
    begin
      for (integer i = 0; i < 64; i = i + 1)
        cache_regs[i][56] <= 1'b0;
      cache_state <= CACHE_IDLE;
    end
    else
    begin
      case (cache_state)
        CACHE_IDLE:
        begin
          if (en_i)
          begin
            if (cache_hit)
            begin
              cache_state <= CACHE_IDLE;
            end
            else
            begin
              wb_adr_o <= adr_i;
              mem_read_req <= 1'b1;
              cache_state <= CACHE_MEM_READ;
            end
          end
        end
        CACHE_MEM_READ:
        begin
          if (mem_ack)
          begin
            mem_read_req <= 1'b0;
            cache_regs[cache_index] <= {1'b1, cache_tag, wb_dat_i};
            cache_state <= CACHE_IDLE;
          end
        end
        default:
        begin
          cache_state <= CACHE_IDLE;
        end
      endcase
    end
  end

  always_comb
  begin
    cache_wait_o = 1'b1;
    cache_ack_o = 1'b0;
    cache_entry_write = 55'b0;
    dat_o = 32'b0;
    case (cache_state)
      CACHE_IDLE:
      begin
        if (en_i)
        begin
          if (cache_hit)
          begin
            dat_o = cache_entry_read[31:0];
            cache_ack_o = 1'b1;
            cache_wait_o = 1'b0;
          end
          else
          begin
            cache_wait_o = 1'b1;
          end
        end
        else
        begin
          cache_wait_o = 1'b0;
        end
      end
      CACHE_CHECK_HIT:
      begin

      end
      CACHE_MEM_READ:
      begin
        if (mem_ack)
        begin

          dat_o = wb_dat_i;
          cache_ack_o = 1'b1;
          cache_wait_o = 1'b0;
        end
      end
    endcase
  end

  // wishbone interface
  always_comb
  begin
    wb_stb_o = mem_read_req;
    wb_cyc_o = mem_read_req;
    wb_dat_o = 32'b0;
    wb_sel_o = 4'b1111;
    wb_we_o  = 1'b0;
    mem_ack  = wb_ack_i;
  end



endmodule
