//mem_master.sv
//author: Gaoxu Guo
//date: 2023/11/24
//description: memory master module , a wrapper of memory maniplation
//simplify the memory access process
/*
    memory master module
    input:
        clk: clock
        rst: reset
        syn_i: sync signal (memory access challenge)
        we_i: write enable
        adr_i: address
        dat_i: data input
        sel_i: select
    output:
        ack_o: acknowledge
        dat_o: data output

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

module mem_master(
    input wire clk,
    input wire rst,
    //input signal
    input wire syn_i,
    input wire we_i,
    input wire [31:0] adr_i,
    input wire [31:0] dat_i,
    input wire [3:0] sel_i,
    //output signal
    output reg ack_o,
    output reg [31:0] dat_o,

    // wishbone interface
    output reg        wb_cyc_o,
    output reg        wb_stb_o,
    input wire        wb_ack_i,
    output reg [31:0] wb_adr_o,
    output reg [31:0] wb_dat_o,
    input wire [31:0] wb_dat_i,
    output reg [3:0]  wb_sel_o,
    output reg        wb_we_o

);
    typedef enum logic [1:0]{
        IDLE,
        WAIT,
        DONE
    }state_t;
    state_t mem_state;

    assign ack_o = wb_ack_i;
    assign wb_stb_o = syn_i;
    assign wb_cyc_o = syn_i;
    assign wb_adr_o = adr_i;
    assign wb_dat_o = dat_i;
    assign wb_we_o = we_i;
    assign wb_sel_o = sel_i;
    assign dat_o = wb_dat_i;

endmodule