`timescale 1ns / 1ps `default_nettype none

module cpu #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
  ) (
    input wire clk_i,
    input wire rst_i,
    input wire [31:0] dip_sw,
    // wishbone master
    output reg wb_cyc_o,
    output reg wb_stb_o,
    input wire wb_ack_i,
    output reg [ADDR_WIDTH-1:0] wb_adr_o,
    output reg [DATA_WIDTH-1:0] wb_dat_o,
    input wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg [DATA_WIDTH/8-1:0] wb_sel_o,
    output reg wb_we_o
  );
  /* =========== Wishbone Master Arbiter begin ============ */

  // cpu_mem_master
  logic [31:0] cpu_mem_master_addr_o;
  logic [31:0] cpu_mem_master_data_o;
  logic [31:0] cpu_mem_master_data_i;
  logic [3:0] cpu_mem_master_sel_o;
  logic cpu_mem_master_cyc_o;
  logic cpu_mem_master_stb_o;
  logic cpu_mem_master_we_o;
  logic cpu_mem_master_ack_i;

  wb_arbiter_4 #(
                 .DATA_WIDTH(ADDR_WIDTH),
                 .ADDR_WIDTH(DATA_WIDTH),
                 .ARB_TYPE_ROUND_ROBIN(0),
                 .ARB_LSB_HIGH_PRIORITY(1)
               ) u_wb_arbiter_4 (
                 .clk(clk_i),
                 .rst(rst_i),
                 .wbm0_adr_i(icache_wb_adr_o),
                 .wbm0_dat_i(icache_wb_dat_o),
                 .wbm0_dat_o(icache_wb_dat_i),
                 .wbm0_we_i(icache_wb_we_o),
                 .wbm0_sel_i(icache_wb_sel_o),
                 .wbm0_stb_i(icache_wb_stb_o),
                 .wbm0_ack_o(icache_wb_ack_i),
                 .wbm0_err_o(),
                 .wbm0_rty_o(),
                 .wbm0_cyc_i(icache_wb_cyc_o),
                 .wbm1_adr_i(cpu_mem_master_addr_o),
                 .wbm1_dat_i(cpu_mem_master_data_o),
                 .wbm1_dat_o(cpu_mem_master_data_i),
                 .wbm1_we_i(cpu_mem_master_we_o),
                 .wbm1_sel_i(cpu_mem_master_sel_o),
                 .wbm1_stb_i(cpu_mem_master_stb_o),
                 .wbm1_ack_o(cpu_mem_master_ack_i),
                 .wbm1_err_o(),
                 .wbm1_rty_o(),
                 .wbm1_cyc_i(cpu_mem_master_cyc_o),
                 .wbm2_adr_i(immu_wb_adr_o),
                 .wbm2_dat_i(immu_wb_dat_o),
                 .wbm2_dat_o(immu_wb_dat_i),
                 .wbm2_we_i (immu_wb_we_o),
                 .wbm2_sel_i(immu_wb_sel_o),
                 .wbm2_stb_i(immu_wb_stb_o),
                 .wbm2_ack_o(immu_wb_ack_i),
                 .wbm2_err_o(),
                 .wbm2_rty_o(),
                 .wbm2_cyc_i(immu_wb_cyc_o),
                 .wbm3_adr_i(dmmu_wb_adr_o),
                 .wbm3_dat_i(dmmu_wb_dat_o),
                 .wbm3_dat_o(dmmu_wb_dat_i),
                 .wbm3_we_i (dmmu_wb_we_o),
                 .wbm3_sel_i(dmmu_wb_sel_o),
                 .wbm3_stb_i(dmmu_wb_stb_o),
                 .wbm3_ack_o(dmmu_wb_ack_i),
                 .wbm3_err_o(),
                 .wbm3_rty_o(),
                 .wbm3_cyc_i(dmmu_wb_cyc_o),
                 // .wbm3_adr_i(),
                 // .wbm3_dat_i(),
                 // .wbm3_dat_o(),
                 // .wbm3_we_i (),
                 // .wbm3_sel_i(4'b0000),
                 // .wbm3_stb_i(),
                 // .wbm3_ack_o(),
                 // .wbm3_err_o(),
                 // .wbm3_rty_o(),
                 // .wbm3_cyc_i(),
                 .wbs_adr_o(wb_adr_o),
                 .wbs_dat_i(wb_dat_i),
                 .wbs_dat_o(wb_dat_o),
                 .wbs_we_o(wb_we_o),
                 .wbs_sel_o(wb_sel_o),
                 .wbs_stb_o(wb_stb_o),
                 .wbs_ack_i(wb_ack_i),
                 .wbs_err_i(0),
                 .wbs_rty_i(0),
                 .wbs_cyc_o(wb_cyc_o)
               );
  /* =========== Wishbone Master Arbiter end ============ */

  /* =========== Hazard Controller begin ============ */
  logic immu_flush_o;
  logic if_flush_o;
  logic id_flush_o;
  logic ex_flush_o;
  logic dmmu_flush_o;
  logic mem_flush_o;
  logic wb_flush_o;
  logic immu_stall_o;
  logic if_stall_o;
  logic id_stall_o;
  logic ex_stall_o;
  logic dmmu_stall_o;
  logic mem_stall_o;
  logic wb_stall_o;
  wire  _if_flush_i;
  wire  immu_if_flush_i;
  wire  if_id_flush_i;
  wire  id_ex_flush_i;
  wire  ex_dmmu_flush_i;
  wire  dmmu_mem_flush_i;
  wire  mem_wb_flush_i;
  wire  _if_stall_i;
  wire  immu_if_stall_i;
  wire  if_id_stall_i;
  wire  id_ex_stall_i;
  wire  ex_dmmu_stall_i;
  wire  dmmu_mem_stall_i;
  wire  mem_wb_stall_i;
  assign immu_flush_o = 0;
  assign if_flush_o  = 0;
  assign id_flush_o  = 0;
  assign ex_flush_o = 0;
  assign dmmu_flush_o = 0;
  assign wb_flush_o  = 0;
  assign immu_stall_o = 0;
  assign if_stall_o  = 0;
  assign ex_stall_o  = 0;
  assign dmmu_stall_o = 0;
  assign mem_stall_o = 0;
  assign wb_stall_o  = 0;
  hazard_controller u_hazard_controller (
                      .immu_flush_i   (immu_flush_o),
                      .if_flush_i    (if_flush_o),
                      .id_flush_i    (id_flush_o),
                      .ex_flush_i    (ex_flush_o),
                      .dmmu_flush_i  (dmmu_flush_o),
                      .mem_flush_i   (mem_flush_o),
                      .wb_flush_i    (wb_flush_o),
                      .immu_stall_i  (immu_stall_o),
                      .if_stall_i    (if_stall_o),
                      .id_stall_i    (id_stall_o),
                      .ex_stall_i    (ex_stall_o),
                      .dmmu_stall_i  (dmmu_stall_o),
                      .mem_stall_i   (mem_stall_o),
                      .wb_stall_i    (wb_stall_o),
                      ._if_flush_o   (_if_flush_i),
                      .immu_if_flush_o(immu_if_flush_i),
                      .if_id_flush_o (if_id_flush_i),
                      .id_ex_flush_o (id_ex_flush_i),
                      .ex_dmmu_flush_o(ex_dmmu_flush_i),
                      .dmmu_mem_flush_o(dmmu_mem_flush_i),
                      .mem_wb_flush_o(mem_wb_flush_i),
                      ._if_stall_o   (_if_stall_i),
                      .immu_if_stall_o(immu_if_stall_i),
                      .if_id_stall_o (if_id_stall_i),
                      .id_ex_stall_o (id_ex_stall_i),
                      .ex_dmmu_stall_o(ex_dmmu_stall_i),
                      .dmmu_mem_stall_o(dmmu_mem_stall_i),
                      .mem_wb_stall_o(mem_wb_stall_i)
                    );
  /* =========== Hazard Controller end ============ */
  logic [ 4:0] rf_raddr_a;
  logic [31:0] rf_rdata_a;
  logic [ 4:0] rf_raddr_b;
  logic [31:0] rf_rdata_b;
  logic [ 4:0] rf_waddr;
  logic [31:0] rf_wdata;
  logic        rf_we;

  RegFile_32 u_reg_file (
               .clk(clk_i),
               .reset(rst_i),
               .waddr(rf_waddr),
               .wdata(rf_wdata),
               .we(rf_we),
               .raddr_a(rf_raddr_a),
               .rdata_a(rf_rdata_a),
               .raddr_b(rf_raddr_b),
               .rdata_b(rf_rdata_b)
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
          } opcode_t;

  typedef enum logic [3:0] {
            BEQ,
            BNE,
            BLT,
            BGE,
            BLTU,
            BGEU,
            JAL
          } branch_t;

  logic    [31:0] alu_a;
  logic    [31:0] alu_b;
  logic    [31:0] alu_y;
  opcode_t        alu_op;
  logic           branch_y;
  ALU_32 u_alu (
           .A (alu_a),
           .B (alu_b),
           .Y (alu_y),
           .op(alu_op)
         );
  branch_judge u_branch_judge (
                 .A (id_ex_src1_rf_data),
                 .B (id_ex_src2_rf_data),
                 .Y (branch_y),
                 .op(id_ex_branch_op)
               );
  typedef enum logic [1:0] {
            STATE_READ_PREV,
            STATE_READ_ACTION,
            STATE_READ_DONE,
            STATE_READ_IDLE
          } mem_read_state_t;

  logic [63:0] timer_mtime_i;
  logic [63:0] timer_mtimecmp_i;
  logic        timer_mtime_e;
  logic        timer_mtimecmp_e;
  logic [63:0] timer_mtime_o;
  logic [63:0] timer_mtimecmp_o;
  logic        timer_interrupt;

  timer u_timer (
          .clk(clk_i),
          .reset(rst_i),
          .mtime_i(timer_mtime_o),
          .mtime_e(timer_mtime_e),
          .mtimecmp_e(timer_mtimecmp_e),
          .mtimecmp_i(timer_mtimecmp_o),
          .mtime_o(timer_mtime_i),
          .mtimecmp_o(timer_mtimecmp_i),
          .timer_interrupt_o(timer_interrupt)
        );

  logic [31:0] csr_addr_o;
  logic [31:0] csr_wdata_o;
  logic [ 1:0] csr_type_o;
  logic [31:0] csr_rdata_i;
  logic        csr_pc_jump;
  logic [31:0] csr_pc_out;
  logic [ 1:0] csr_mode;
  logic [31:0] csr_satp;
  logic        csr_mstatus_sum;

  (* MARK_DEBUG = "TRUE" *)reg [1:0] csr_mode_reg;

  logic interrupt_sel;

  CSR u_csr (
        .clk(clk_i),
        .reset(rst_i),
        .pipeline_stall(pipeline_stall),
        .instr(dmmu_mem_instr),
        .csr_addr(csr_addr_o),
        .csr_wdata(csr_wdata_o),
        .csr_type(csr_type_o),
        .csr_rdata(csr_rdata_i),
        .csr_mode_o(csr_mode),
        .pc_in(dmmu_mem_pc),
        .pc_jump(csr_pc_jump),
        .pc_out(csr_pc_out),
        .ecall_i(dmmu_mem_is_ecall),
        .ebreak_i(dmmu_mem_is_ebreak),
        .mret_i(dmmu_mem_is_mret),
        .sret_i(dmmu_mem_is_sret),
        .timer_interrupt(timer_interrupt),
        .interrupt_sel(interrupt_sel),
        .undefined_instr_i(dmmu_mem_undefined_instr_i),
        /* THESE ARE NOT IMPLEMENTED YET
        .page_fault_r(mem_page_fault_r),
        .page_fault_w(mem_page_fault_w),
        .access_fault_i(ex_mem_access_fault_i),
        .access_fault_r(mem_access_fault_r),
        .access_fault_w(mem_access_fault_w),
         THESE ARE NOT IMPLEMENTED YET */
        .page_fault_i(dmmu_mem_page_fault_i),
        .page_fault_r(dmmu_mem_page_fault_r),
        .page_fault_w(dmmu_mem_page_fault_w),
        .access_fault_i(0),
        .access_fault_r(0),
        .access_fault_w(0),
        .addr_misaligned_i(dmmu_mem_instr_misaligned),
        .addr_misaligned_r(mem_data_misaligned && dmmu_mem_memread),
        .addr_misaligned_w(mem_data_misaligned && dmmu_mem_memwrite),
        .instr_fault_addr(dmmu_mem_pc),
        .data_fault_addr(dmmu_mem_alu_res),
        .mtime(timer_mtime_i),
        .satp_o(csr_satp),
        .mstatus_sum_o(csr_mstatus_sum)
      );

  cache instr_cache(
          .clk(clk_i),
          .rst(rst_i),
          .rst_cache_i(0),
          .en_i(if_icache_en),
          .adr_i(if_icache_addr),
          .dat_o(if_icache_data),
          .cache_wait_o(if_icache_wait),
          .cache_ack_o(if_icache_ack_i),

          .wb_cyc_o(icache_wb_cyc_o),
          .wb_stb_o(icache_wb_stb_o),
          .wb_ack_i(icache_wb_ack_i),
          .wb_adr_o(icache_wb_adr_o),
          .wb_dat_o(icache_wb_dat_o),
          .wb_dat_i(icache_wb_dat_i),
          .wb_sel_o(icache_wb_sel_o),
          .wb_we_o(icache_wb_we_o)
        );

  mmu i_mmu(
        .clk(clk_i),
        .rst(rst_i),

        .rst_tlb_i(id_is_sfence),
        .en_i(1'b1),

        .va_i(immu_vpc),
        .satp_i(csr_satp),
        .priv_mode_i(csr_mode),
        .permission_i(3'b100),
        .sum_i(csr_mstatus_sum),
        .pa_o(immu_ppc),
        .mmu_ack_o(immu_ack_o),
        .page_fault_o(immu_page_fault_i),

        .wb_cyc_o(immu_wb_cyc_o),
        .wb_stb_o(immu_wb_stb_o),
        .wb_ack_i(immu_wb_ack_i),
        .wb_adr_o(immu_wb_adr_o),
        .wb_dat_o(immu_wb_dat_o),
        .wb_dat_i(immu_wb_dat_i),
        .wb_sel_o(immu_wb_sel_o),
        .wb_we_o(immu_wb_we_o)
      );

  mmu d_mmu(
        .clk(clk_i),
        .rst(rst_i),

        .rst_tlb_i(ex_dmmu_is_sfence),
        .en_i((ex_dmmu_memread|ex_dmmu_memwrite)& !ex_dmmu_flush_i),

        .va_i(dmmu_va_o),
        .satp_i(csr_satp),
        .priv_mode_i(csr_mode),
        .permission_i({1'b0, ex_dmmu_memwrite, ex_dmmu_memread}),
        .sum_i(csr_mstatus_sum),
        .pa_o(dmmu_pa_i),
        .mmu_ack_o(dmmu_ack_o),
        .page_fault_o(dmmu_page_fault_i),

        .wb_cyc_o(dmmu_wb_cyc_o),
        .wb_stb_o(dmmu_wb_stb_o),
        .wb_ack_i(dmmu_wb_ack_i),
        .wb_adr_o(dmmu_wb_adr_o),
        .wb_dat_o(dmmu_wb_dat_o),
        .wb_dat_i(dmmu_wb_dat_i),
        .wb_sel_o(dmmu_wb_sel_o),
        .wb_we_o(dmmu_wb_we_o)
      );

  /* =========== Pipeline begin ============ */

  // Immu stage
  logic [31:0] immu_vpc;
  logic [31:0] immu_ppc;
  logic [31:0] immu_page_fault_i_reg;
  logic [31:0] immu_page_fault;
  logic immu_wb_cyc_o;
  logic immu_wb_stb_o;
  logic immu_wb_ack_i;
  logic [31:0] immu_wb_adr_o;
  logic [31:0] immu_wb_dat_o;
  logic [31:0] immu_wb_dat_i;
  logic [3:0] immu_wb_sel_o;
  logic immu_wb_we_o;
  logic immu_ack_o;
  logic immu_page_fault_i;

  // Immu/IF pipeline register
  logic [31:0] immu_if_vpc;
  logic [31:0] immu_if_ppc;
  logic immu_if_page_fault_i;

  // IF stage
  logic [31:0] if_ppc;
  logic [31:0] if_instr;
  logic if_instr_misaligned;
  logic if_access_fault_i;

  //-------------------------for icache-----------------------------------
  logic if_rst_icache;
  logic if_icache_en;
  logic if_icache_wait;
  logic if_icache_ack_i;
  logic [31:0] if_icache_addr;
  logic [31:0] if_icache_data;

  logic icache_wb_cyc_o;
  logic icache_wb_stb_o;
  logic icache_wb_ack_i;
  logic [31:0] icache_wb_adr_o;
  logic [31:0] icache_wb_dat_o;
  logic [31:0] icache_wb_dat_i;
  logic [3:0] icache_wb_sel_o;
  logic icache_wb_we_o;

  //-------------------------icache end-----------------------------------

  // IF/ID pipeline register
  logic [31:0] if_id_pc, if_id_instr;
  logic if_id_instr_misaligned;
  logic if_id_page_fault_i, if_id_access_fault_i;

  // ID stage
  logic [31:0] id_src1_rf_data, id_src2_rf_data;
  logic [31:0] id_imm;
  opcode_t id_opcode;
  logic id_branch, id_use_reg2, id_memread, id_memwrite, id_rfwrite;
  logic [4:0] id_rf_waddr;
  logic [3:0] id_op_len;
  branch_t    id_branch_op;
  logic id_is_jal;
  logic id_load_unsigned;
  logic [1:0] id_csr_type;
  logic [11:0] id_csr_addr;
  logic id_is_csr, id_is_ecall, id_is_ebreak, id_is_mret, id_is_sret, id_undefined_instr_i, id_is_sfence, id_is_fence;

  // ID/EX pipeline register
  logic [31:0] id_ex_pc, id_ex_instr, id_ex_src1_rf_data, id_ex_src2_rf_data, id_ex_imm;
  opcode_t id_ex_opcode;
  logic id_ex_branch, id_ex_usereg2, id_ex_memread, id_ex_memwrite, id_ex_rfwrite;
  logic [4:0] id_ex_rf_waddr;
  logic [3:0] id_ex_op_len;
  branch_t    id_ex_branch_op;
  logic id_ex_is_jal;
  logic id_ex_load_unsigned;
  logic [1:0] id_ex_csr_type;
  logic [11:0] id_ex_csr_addr;
  logic id_ex_is_csr, id_ex_is_ecall, id_ex_is_ebreak, id_ex_is_mret, id_ex_is_sret, id_ex_undefined_instr_i, id_ex_is_sfence;
  logic id_ex_instr_misaligned;
  logic id_ex_page_fault_i, id_ex_access_fault_i;
  logic id_ex_is_read, id_ex_is_write;

  // EX stage
  logic ex_beq;
  logic ex_branch_res;
  logic [31:0] ex_alu_res;

  // EX/DMMU pipeline register
  logic ex_dmmu_beq;
  logic [31:0] ex_dmmu_pc;
  logic [31:0] ex_dmmu_instr, ex_dmmu_alu_res, ex_dmmu_mem_wdata;
  logic ex_dmmu_memread, ex_dmmu_memwrite, ex_dmmu_rfwrite;
  logic [4:0] ex_dmmu_rf_waddr;
  logic [3:0] ex_dmmu_op_len;
  logic ex_dmmu_is_jal;
  logic ex_dmmu_load_unsigned;
  logic [1:0] ex_dmmu_csr_type;
  logic [11:0] ex_dmmu_csr_addr;
  logic [31:0] ex_dmmu_csr_data;
  logic ex_dmmu_is_csr, ex_dmmu_is_ecall, ex_dmmu_is_ebreak, ex_dmmu_is_mret, ex_dmmu_is_sret, ex_dmmu_undefined_instr_i, ex_dmmu_is_sfence;
  logic ex_dmmu_instr_misaligned;
  logic ex_dmmu_page_fault_i, ex_dmmu_access_fault_i;
  logic ex_dmmu_is_read, ex_dmmu_is_write;

  // Dmmu stage
  logic [31:0] dmmu_va_o;
  logic [31:0] dmmu_pa_i;
  logic dmmu_is_read;
  logic dmmu_is_write;
  logic dmmu_wb_cyc_o;
  logic dmmu_wb_stb_o;
  logic dmmu_wb_ack_i;
  logic [31:0] dmmu_wb_adr_o;
  logic [31:0] dmmu_wb_dat_o;
  logic [31:0] dmmu_wb_dat_i;
  logic [3:0] dmmu_wb_sel_o;
  logic dmmu_wb_we_o;
  logic dmmu_ack_o;
  logic dmmu_page_fault;

  logic dmmu_access_fault_r, dmmu_access_fault_w;
  logic dmmu_page_fault_r, dmmu_page_fault_w;

  // Dmmu/MEM pipeline register
  logic [31:0] dmmu_mem_va_i;
  logic [31:0] dmmu_mem_pa_i;
  logic dmmu_mem_page_fault_r;
  logic dmmu_mem_page_fault_w;
  logic dmmu_mem_beq;
  logic [31:0] dmmu_mem_pc;
  logic [31:0] dmmu_mem_instr, dmmu_mem_alu_res, dmmu_mem_mem_wdata;
  logic dmmu_mem_memread, dmmu_mem_memwrite, dmmu_mem_rfwrite;
  logic [4:0] dmmu_mem_rf_waddr;
  logic [3:0] dmmu_mem_op_len;
  logic dmmu_mem_is_jal;
  logic dmmu_mem_load_unsigned;
  logic [1:0] dmmu_mem_csr_type;
  logic [11:0] dmmu_mem_csr_addr;
  logic [31:0] dmmu_mem_csr_data;
  logic dmmu_mem_is_csr, dmmu_mem_is_ecall, dmmu_mem_is_ebreak, dmmu_mem_is_mret, dmmu_mem_is_sret, dmmu_mem_undefined_instr_i;
  logic dmmu_mem_instr_misaligned;
  logic dmmu_mem_page_fault_i, dmmu_mem_access_fault_i;
  logic [31:0] dmmu_mem_pa;
  logic [31:0] dmmu_mem_va;

  // MEM stage
  logic [31:0] mem_mem_rdata, mem_rf_wdata;
  logic mem_access_fault_r, mem_access_fault_w;
  logic mem_page_fault_r, mem_page_fault_w;

  // MEM/WB pipeline register
  (* MARK_DEBUG = "TRUE" *)logic [31:0] mem_wb_pc, mem_wb_instr;
  (* MARK_DEBUG = "TRUE" *)logic [31:0] mem_wb_rf_wdata;
  (* MARK_DEBUG = "TRUE" *)logic [4:0] mem_wb_rf_waddr;
  (* MARK_DEBUG = "TRUE" *)logic mem_wb_rfwrite;
  (* MARK_DEBUG = "TRUE" *)logic [31:0] mem_wb_va;
  (* MARK_DEBUG = "TRUE" *)logic [31:0] mem_wb_mem_wdata;

  // Pipeline control signals
  logic pipeline_stall;
  assign pipeline_stall = if_icache_en | cpu_mem_master_stb_o|dmmu_wb_stb_o|(if_read_state==STATE_READ_PREV);

  // PC controller
  always_ff @(posedge clk_i)
  begin
    if (rst_i)
    begin
      immu_vpc <= 32'h80000000;
      if_id_pc <= 32'h80000000;
      id_ex_pc <= 32'h80000000;
      ex_dmmu_pc <= 32'h80000000;
      dmmu_mem_pc <= 32'h80000000;
      mem_wb_pc <= 32'h80000000;
      if_id_instr <= 32'h00000013;  // nop
      id_ex_instr <= 32'h00000013;  // nop
      ex_dmmu_instr <= 32'h00000013;  // nop
      dmmu_mem_instr <= 32'h00000013;  // nop
      mem_wb_instr <= 32'h00000013;  // nop

      id_ex_memread <= 0;
      id_ex_memwrite <= 0;
      id_ex_rfwrite <= 0;
      id_ex_branch <= 0;
      id_ex_op_len <= 4'hf;
      id_ex_is_jal <= 0;
      id_ex_usereg2 <= 0;
      id_ex_load_unsigned <= 0;
      id_ex_instr_misaligned <= 0;
      id_ex_is_csr <= 0;
      id_ex_is_ecall <= 0;
      id_ex_is_ebreak <= 0;
      id_ex_is_mret <= 0;
      id_ex_is_sret <= 0;
      id_ex_csr_addr <= 0;
      id_ex_csr_type <= 0;
      id_ex_undefined_instr_i <= 0;

      ex_dmmu_memread <= 0;
      ex_dmmu_memwrite <= 0;
      ex_dmmu_rfwrite <= 0;
      ex_dmmu_op_len <= 4'hf;
      ex_dmmu_is_jal <= 0;
      ex_dmmu_beq <= 0;
      ex_dmmu_load_unsigned <= 0;
      ex_dmmu_instr_misaligned <= 0;
      ex_dmmu_is_csr <= 0;
      ex_dmmu_is_ecall <= 0;
      ex_dmmu_is_ebreak <= 0;
      ex_dmmu_is_mret <= 0;
      ex_dmmu_is_sret <= 0;
      ex_dmmu_csr_addr <= 0;
      ex_dmmu_csr_type <= 0;
      ex_dmmu_undefined_instr_i <= 0;

      dmmu_mem_memread <= 0;
      dmmu_mem_memwrite <= 0;
      dmmu_mem_rfwrite <= 0;
      dmmu_mem_op_len <= 4'hf;
      dmmu_mem_is_jal <= 0;
      dmmu_mem_beq <= 0;
      dmmu_mem_load_unsigned <= 0;
      dmmu_mem_instr_misaligned <= 0;
      dmmu_mem_is_csr <= 0;
      dmmu_mem_is_ecall <= 0;
      dmmu_mem_is_ebreak <= 0;
      dmmu_mem_is_mret <= 0;
      dmmu_mem_is_sret <= 0;
      dmmu_mem_csr_addr <= 0;
      dmmu_mem_csr_type <= 0;
      dmmu_mem_undefined_instr_i <= 0;

      mem_wb_rfwrite <= 0;

      csr_mode_reg <= 0;
    end
    else
    begin
      if (!pipeline_stall)
      begin
        // PC update
        if (_if_stall_i)
        begin
          immu_vpc <= immu_vpc;
        end
        else if (_if_flush_i)
        begin
          immu_vpc <= csr_pc_jump ? csr_pc_out : dmmu_mem_alu_res;
        end
        else
        begin
          immu_vpc <= immu_vpc + 4;
        end
        // IMMU/IF pipeline register update
        if (immu_if_stall_i)
        begin
          // nop
        end
        if (immu_if_flush_i)
        begin
          immu_if_page_fault_i <= 0;
        end
        else
        begin
          immu_if_page_fault_i <= immu_page_fault;
        end
        // IF/ID pipeline register update
        if (if_id_stall_i)
        begin
          if_id_instr <= if_id_instr;
        end
        else if (if_id_flush_i)
        begin
          if_id_instr <= 32'h00000013;  // nop
        end
        else
        begin
          if_id_instr <= if_instr;
          if_id_pc <= immu_if_vpc;
          if_id_instr_misaligned <= if_instr_misaligned;
          if_id_page_fault_i <= immu_if_page_fault_i;
        end
        // ID/EX pipeline register update
        if (id_ex_stall_i)
        begin
          id_ex_instr <= id_ex_instr;
        end
        else if (id_ex_flush_i)
        begin
          id_ex_instr <= 32'h00000013;  // nop
          id_ex_branch <= 0;
          id_ex_memread <= 0;
          id_ex_memwrite <= 0;
          id_ex_rfwrite <= 0;
          id_ex_is_jal <= 0;
          id_ex_branch_op <= BEQ;
          id_ex_imm <= 0;
          id_ex_rf_waddr <= 5'h0;
          id_ex_op_len <= 4'b0000;
          id_ex_opcode <= NULL;
          id_ex_src1_rf_data <= 0;
          id_ex_src2_rf_data <= 0;
          id_ex_usereg2 <= 0;
          id_ex_load_unsigned <= 0;
          id_ex_instr_misaligned <= 0;
          id_ex_is_csr <= 0;
          id_ex_is_ecall <= 0;
          id_ex_is_ebreak <= 0;
          id_ex_is_mret <= 0;
          id_ex_is_sret <= 0;
          id_ex_csr_addr <= 0;
          id_ex_csr_type <= 0;
          id_ex_undefined_instr_i <= 0;
          id_ex_page_fault_i <= 0;
          id_ex_is_sfence <= 0;
        end
        else
        begin
          id_ex_instr <= if_id_instr;
          id_ex_pc <= if_id_pc;
          id_ex_src1_rf_data <= id_src1_rf_data;
          id_ex_src2_rf_data <= id_src2_rf_data;
          id_ex_imm <= id_imm;
          id_ex_opcode <= id_opcode;
          id_ex_branch <= id_branch;
          id_ex_usereg2 <= id_use_reg2;
          id_ex_memread <= id_memread;
          id_ex_memwrite <= id_memwrite;
          id_ex_rfwrite <= id_rfwrite;
          id_ex_rf_waddr <= id_rf_waddr;
          id_ex_op_len <= id_op_len;
          id_ex_is_jal <= id_is_jal;
          id_ex_branch_op <= id_branch_op;
          id_ex_load_unsigned <= id_load_unsigned;
          id_ex_instr_misaligned <= if_id_instr_misaligned;
          id_ex_is_csr <= id_is_csr;
          id_ex_is_ecall <= id_is_ecall;
          id_ex_is_ebreak <= id_is_ebreak;
          id_ex_is_mret <= id_is_mret;
          id_ex_is_sret <= id_is_sret;
          id_ex_csr_addr <= id_csr_addr;
          id_ex_csr_type <= id_csr_type;
          id_ex_undefined_instr_i <= id_undefined_instr_i;
          id_ex_page_fault_i <= if_id_page_fault_i;
          id_ex_is_sfence <= id_is_sfence;
        end
        // EX/DMMU pipeline register update
        if (ex_dmmu_stall_i)
        begin
          ex_dmmu_instr <= ex_dmmu_instr;
        end
        else if (ex_dmmu_flush_i)
        begin
          ex_dmmu_instr <= 32'h00000013;  // nop
          ex_dmmu_memread <= 0;
          ex_dmmu_memwrite <= 0;
          ex_dmmu_rfwrite <= 0;
          ex_dmmu_is_jal <= 0;
          ex_dmmu_alu_res <= 0;
          ex_dmmu_mem_wdata <= 0;
          ex_dmmu_csr_data <= 0;
          ex_dmmu_rf_waddr <= 5'h0;
          ex_dmmu_op_len <= 4'b0000;
          ex_dmmu_load_unsigned <= 0;
          ex_dmmu_beq <= 0;
          ex_dmmu_instr_misaligned <= 0;
          ex_dmmu_is_csr <= 0;
          ex_dmmu_is_ecall <= 0;
          ex_dmmu_is_ebreak <= 0;
          ex_dmmu_is_mret <= 0;
          ex_dmmu_is_sret <= 0;
          ex_dmmu_csr_addr <= 0;
          ex_dmmu_csr_type <= 0;
          ex_dmmu_undefined_instr_i <= 0;
          ex_dmmu_page_fault_i <= 0;
          ex_dmmu_is_sfence <= 0;
        end
        else
        begin
          ex_dmmu_instr <= id_ex_instr;
          ex_dmmu_pc <= id_ex_pc;
          ex_dmmu_alu_res <= ex_alu_res;
          ex_dmmu_mem_wdata <= id_ex_src2_rf_data;
          ex_dmmu_csr_data <= id_ex_src1_rf_data;
          ex_dmmu_memread <= id_ex_memread;
          ex_dmmu_memwrite <= id_ex_memwrite;
          ex_dmmu_rfwrite <= id_ex_rfwrite;
          ex_dmmu_rf_waddr <= id_ex_rf_waddr;
          ex_dmmu_op_len <= id_ex_op_len;
          ex_dmmu_is_jal <= id_ex_is_jal;
          ex_dmmu_load_unsigned <= id_ex_load_unsigned;
          ex_dmmu_beq <= ex_beq;
          ex_dmmu_instr_misaligned <= id_ex_instr_misaligned;
          ex_dmmu_is_csr <= id_ex_is_csr;
          ex_dmmu_is_ecall <= id_ex_is_ecall;
          ex_dmmu_is_ebreak <= id_ex_is_ebreak;
          ex_dmmu_is_mret <= id_ex_is_mret;
          ex_dmmu_is_sret <= id_ex_is_sret;
          ex_dmmu_csr_addr <= id_ex_csr_addr;
          ex_dmmu_csr_type <= id_ex_csr_type;
          ex_dmmu_undefined_instr_i <= id_ex_undefined_instr_i;
          ex_dmmu_page_fault_i <= id_ex_page_fault_i;
          ex_dmmu_is_sfence <= id_ex_is_sfence;
        end
        // DMMU/MEM pipeline register update
        if (dmmu_mem_stall_i)
        begin
          dmmu_mem_instr <= dmmu_mem_instr;
        end
        else if (dmmu_mem_flush_i)
        begin
          dmmu_mem_instr <= 32'h00000013;  // nop
          dmmu_mem_memread <= 0;
          dmmu_mem_memwrite <= 0;
          dmmu_mem_rfwrite <= 0;
          dmmu_mem_is_jal <= 0;
          dmmu_mem_alu_res <= 0;
          dmmu_mem_mem_wdata <= 0;
          dmmu_mem_csr_data <= 0;
          dmmu_mem_rf_waddr <= 5'h0;
          dmmu_mem_op_len <= 4'b0000;
          dmmu_mem_load_unsigned <= 0;
          dmmu_mem_beq <= 0;
          dmmu_mem_instr_misaligned <= 0;
          dmmu_mem_is_csr <= 0;
          dmmu_mem_is_ecall <= 0;
          dmmu_mem_is_ebreak <= 0;
          dmmu_mem_is_mret <= 0;
          dmmu_mem_is_sret <= 0;
          dmmu_mem_csr_addr <= 0;
          dmmu_mem_csr_type <= 0;
          dmmu_mem_undefined_instr_i <= 0;
          dmmu_mem_page_fault_i <= 0;
          dmmu_mem_page_fault_r <= 0;
          dmmu_mem_page_fault_w <= 0;
          dmmu_mem_pa <= 0;
          dmmu_mem_va <= 0;
        end
        else
        begin
          dmmu_mem_instr <= ex_dmmu_instr;
          dmmu_mem_pc <= ex_dmmu_pc;
          dmmu_mem_alu_res <= ex_dmmu_alu_res;
          dmmu_mem_mem_wdata <= ex_dmmu_mem_wdata;
          dmmu_mem_csr_data <= ex_dmmu_csr_data;
          dmmu_mem_memread <= ex_dmmu_memread;
          dmmu_mem_memwrite <= ex_dmmu_memwrite;
          dmmu_mem_rfwrite <= ex_dmmu_rfwrite;
          dmmu_mem_rf_waddr <= ex_dmmu_rf_waddr;
          dmmu_mem_op_len <= ex_dmmu_op_len;
          dmmu_mem_is_jal <= ex_dmmu_is_jal;
          dmmu_mem_load_unsigned <= ex_dmmu_load_unsigned;
          dmmu_mem_beq <= ex_dmmu_beq;
          dmmu_mem_instr_misaligned <= ex_dmmu_instr_misaligned;
          dmmu_mem_is_csr <= ex_dmmu_is_csr;
          dmmu_mem_is_ecall <= ex_dmmu_is_ecall;
          dmmu_mem_is_ebreak <= ex_dmmu_is_ebreak;
          dmmu_mem_is_mret <= ex_dmmu_is_mret;
          dmmu_mem_is_sret <= ex_dmmu_is_sret;
          dmmu_mem_csr_addr <= ex_dmmu_csr_addr;
          dmmu_mem_csr_type <= ex_dmmu_csr_type;
          dmmu_mem_undefined_instr_i <= ex_dmmu_undefined_instr_i;
          dmmu_mem_page_fault_i <= ex_dmmu_page_fault_i;
          dmmu_mem_page_fault_r <= dmmu_page_fault_r;
          dmmu_mem_page_fault_w <= dmmu_page_fault_w;
          dmmu_mem_pa <= dmmu_pa;
          dmmu_mem_va <= dmmu_va_o;
        end
        // MEM/WB pipeline register update
        if (csr_mode == 2'b00)
          csr_mode_reg <= 2'b01;
        if (mem_wb_stall_i)
        begin
          mem_wb_instr <= mem_wb_instr;
        end
        else if (mem_wb_flush_i || interrupt_sel)
        begin
          mem_wb_instr <= 32'h00000013;  // nop
          mem_wb_rfwrite <= 0;
          mem_wb_rf_waddr <= 5'h0;
          mem_wb_rf_wdata <= 0;
          mem_wb_va <= 0;
          mem_wb_mem_wdata <= 0;
        end
        else
        begin
          mem_wb_instr <= dmmu_mem_instr;
          mem_wb_pc <= dmmu_mem_pc;
          mem_wb_rfwrite <= dmmu_mem_rfwrite;
          mem_wb_rf_waddr <= dmmu_mem_rf_waddr;
          mem_wb_rf_wdata <= mem_rf_wdata;
          mem_wb_va <= dmmu_mem_va;
          mem_wb_mem_wdata <= dmmu_mem_mem_wdata;
        end
      end
    end
  end
  assign mem_flush_o = dmmu_mem_beq || csr_pc_jump;

  /* =============== Pipeline end =============== */

  /* =============== IF stage =============== */
  assign if_instr_misaligned = immu_vpc[1:0] != 2'b00;

  assign immu_if_vpc = immu_vpc;
  assign if_ppc = immu_ack_o ? immu_ppc : immu_if_ppc;
  assign immu_page_fault=immu_ack_o?immu_if_page_fault_i:immu_page_fault_i_reg;
  mem_read_state_t if_read_state;

  always_ff @(posedge clk_i)
  begin
    if (rst_i)
    begin
      if_read_state <= STATE_READ_PREV;
    end
    else
    begin
      case (if_read_state)
        STATE_READ_PREV:
        begin
          if(immu_if_page_fault_i)
            if_read_state <= STATE_READ_DONE;
          else
            if(immu_ack_o)
              if_read_state <= STATE_READ_ACTION;
            else
              if_read_state <= STATE_READ_PREV;
        end
        STATE_READ_ACTION:
        begin
          if (if_icache_ack_i)
          begin
            if_read_state <= STATE_READ_DONE;
          end
        end
        STATE_READ_DONE:
        begin
          if (!pipeline_stall && !if_instr_misaligned)
            if_read_state <= STATE_READ_PREV;
        end
      endcase
    end
  end

  always_ff @(posedge clk_i)
  begin
    if(immu_ack_o)
      immu_if_ppc <= immu_ppc;
    immu_page_fault_i_reg<=immu_page_fault_i;
    if(!pipeline_stall)
      immu_page_fault_i_reg<=0;
  end

  always_ff @(posedge clk_i)
  begin
    if((if_read_state==STATE_READ_ACTION)&&!if_icache_wait)
    begin
      if(!if_instr_misaligned)
        if_instr <= if_icache_data;
      else
        if_instr <= 32'h00000013;  // nop
    end
  end

  assign if_icache_addr = if_ppc;
  assign if_icache_en  = if_read_state == STATE_READ_ACTION;

  /* =============== ID stage =============== */
  typedef enum logic [7:0] {
            INST_ERR,
            INST_LUI,
            INST_AUIPC,
            INST_JAL,
            INST_JALR,
            INST_BEQ,
            INST_BNE,
            INST_BLT,
            INST_BGE,
            INST_BLTU,
            INST_BGEU,
            INST_LB,
            INST_LH,
            INST_LW,
            INST_LBU,
            INST_LHU,
            INST_SB,
            INST_SH,
            INST_SW,
            INST_ADDI,
            INST_SLTI,
            INST_SLTIU,
            INST_XORI,
            INST_ORI,
            INST_ANDI,
            INST_SLLI,
            INST_SRLI,
            INST_SRAI,
            INST_ADD,
            INST_SUB,
            INST_SLL,
            INST_SLT,
            INST_SLTU,
            INST_XOR,
            INST_SRL,
            INST_SRA,
            INST_OR,
            INST_AND,
            INST_ANDN,
            INST_SBSET,
            INST_PCNT,
            INST_CRAS16,
            INST_CSRRC,
            INST_CSRRCI,
            INST_CSRRS,
            INST_CSRRSI,
            INST_CSRRW,
            INST_CSRRWI,
            INST_ECALL,
            INST_EBREAK,
            INST_MRET,
            INST_SRET,
            INST_SFENCE,
            INST_FENCE
          } inst_type;  // instruction type, add more types when needed
  inst_type id_inst_type;
  logic [11:0] I_imm;
  logic [11:0] S_imm;
  logic [12:0] B_imm;
  logic [31:0] U_imm;
  logic [20:0] J_imm;
  logic [4:0] id_rs1_addr;
  logic [4:0] id_rs2_addr;
  logic [4:0] id_rd_addr;
  logic [2:0] funct3;
  logic [6:0] funct7;
  logic [6:0] opcode;

  assign rf_raddr_a = id_rs1_addr;
  assign rf_raddr_b = id_rs2_addr;
  assign I_imm = if_id_instr[31:20];
  assign S_imm = {if_id_instr[31:25], if_id_instr[11:7]};
  assign B_imm = {if_id_instr[31], if_id_instr[7], if_id_instr[30:25], if_id_instr[11:8], 1'b0};
  assign U_imm = {if_id_instr[31:12], 12'h0};
  assign J_imm = {if_id_instr[31], if_id_instr[19:12], if_id_instr[20], if_id_instr[30:21], 1'b0};

  always_comb
  begin
    id_rs1_addr = if_id_instr[19:15];
    id_rs2_addr = if_id_instr[24:20];
    id_rd_addr = if_id_instr[11:7];
    funct3 = if_id_instr[14:12];
    funct7 = if_id_instr[31:25];
    opcode = if_id_instr[6:0];
    id_src1_rf_data = rf_rdata_a;
    id_src2_rf_data = rf_rdata_b;
    id_branch = 0;
    id_is_jal = 0;
    id_memread = 0;
    id_memwrite = 0;
    id_rfwrite = 0;
    id_use_reg2 = 1;
    id_imm = 0;
    id_rf_waddr = 5'h0;
    id_op_len = 4'b0000;
    id_opcode = NULL;
    id_branch_op = BEQ;
    id_inst_type = INST_ERR;
    id_is_csr = 0;
    id_is_ecall = 0;
    id_is_ebreak = 0;
    id_is_mret = 0;
    id_is_sret = 0;
    id_undefined_instr_i = 0;
    id_csr_addr = 0;
    id_csr_type = 2'b00;
    id_is_sfence = 0;

    case (opcode)
      7'b0110111:
        id_inst_type = INST_LUI;
      7'b0010111:
        id_inst_type = INST_AUIPC;
      7'b1101111:
        id_inst_type = INST_JAL;
      7'b1100111:
        id_inst_type = INST_JALR;
      7'b0001111:
        id_inst_type = INST_FENCE;
      7'b1100011:
      begin
        case (funct3)
          3'b000:
            id_inst_type = INST_BEQ;
          3'b001:
            id_inst_type = INST_BNE;
          3'b100:
            id_inst_type = INST_BLT;
          3'b101:
            id_inst_type = INST_BGE;
          3'b110:
            id_inst_type = INST_BLTU;
          3'b111:
            id_inst_type = INST_BGEU;
          default:
            id_inst_type = INST_ERR;
        endcase
      end
      7'b0000011:
      begin
        case (funct3)
          3'b000:
            id_inst_type = INST_LB;
          3'b001:
            id_inst_type = INST_LH;
          3'b010:
            id_inst_type = INST_LW;
          3'b100:
            id_inst_type = INST_LBU;
          3'b101:
            id_inst_type = INST_LHU;
          default:
            id_inst_type = INST_ERR;
        endcase
      end
      7'b0100011:
      begin
        case (funct3)
          3'b000:
            id_inst_type = INST_SB;
          3'b001:
            id_inst_type = INST_SH;
          3'b010:
            id_inst_type = INST_SW;
          default:
            id_inst_type = INST_ERR;
        endcase
      end
      7'b0010011:
      begin
        case (funct3)
          3'b000:
            id_inst_type = INST_ADDI;
          3'b010:
            id_inst_type = INST_SLTI;
          3'b011:
            id_inst_type = INST_SLTIU;
          3'b100:
            id_inst_type = INST_XORI;
          3'b110:
            id_inst_type = INST_ORI;
          3'b111:
            id_inst_type = INST_ANDI;
          3'b001:
          begin
            case (funct7)
              7'b0000000:
                id_inst_type = INST_SLLI;
              7'b0110000:
                id_inst_type = INST_PCNT;
              default:
                id_inst_type = INST_ERR;
            endcase
          end
          3'b101:
          begin
            case (funct7)
              7'b0000000:
                id_inst_type = INST_SRLI;
              7'b0100000:
                id_inst_type = INST_SRAI;
              default:
                id_inst_type = INST_ERR;
            endcase
          end
          default:
            id_inst_type = INST_ERR;
        endcase
      end
      7'b0110011:
      begin
        case (funct3)
          3'b000:
          begin
            case (funct7)
              7'b0000000:
                id_inst_type = INST_ADD;
              7'b0100000:
                id_inst_type = INST_SUB;
              default:
                id_inst_type = INST_ERR;
            endcase
          end
          3'b001:
          begin
            case (funct7)
              7'b0000000:
                id_inst_type = INST_SLL;
              7'b0010100:
                id_inst_type = INST_SBSET;
              default:
                id_inst_type = INST_ERR;
            endcase
          end
          3'b010:
            id_inst_type = INST_SLT;
          3'b011:
            id_inst_type = INST_SLTU;
          3'b100:
            id_inst_type = INST_XOR;
          3'b101:
          begin
            case (funct7)
              7'b0000000:
                id_inst_type = INST_SRL;
              7'b0100000:
                id_inst_type = INST_SRA;
              default:
                id_inst_type = INST_ERR;
            endcase
          end
          3'b110:
            id_inst_type = INST_OR;
          3'b111:
          begin
            case (funct7)
              7'b0000000:
                id_inst_type = INST_AND;
              7'b0100000:
                id_inst_type = INST_ANDN;
              default:
                id_inst_type = INST_ERR;
            endcase
          end
        endcase
      end
      7'b1110011:
      begin
        case (funct3)
          3'b001:
            id_inst_type = INST_CSRRW;
          3'b101:
            id_inst_type = INST_CSRRWI;
          3'b010:
            id_inst_type = INST_CSRRS;
          3'b110:
            id_inst_type = INST_CSRRSI;
          3'b011:
            id_inst_type = INST_CSRRC;
          3'b111:
            id_inst_type = INST_CSRRCI;
          3'b000:
          begin
            case (funct7) // ecall, ebreak, mret, sret
              7'b0000000:
              begin
                if(if_id_instr[20])
                  id_inst_type = INST_EBREAK;
                else
                  id_inst_type = INST_ECALL;
              end
              7'b0011000:
                id_inst_type = INST_MRET;
              7'b0001000:
                id_inst_type = INST_SRET;
              7'b0001001:
                id_inst_type = INST_SFENCE;
              default:
                id_inst_type = INST_ERR;
            endcase
          end
          default:
            id_inst_type = INST_ERR;
        endcase
      end
      7'b1110111:
      begin
        case (funct3)
          3'b000:
          begin
            case (funct7)
              7'b0100010:
                id_inst_type = INST_CRAS16;
              default:
                id_inst_type = INST_ERR;
            endcase
          end
          default:
            id_inst_type = INST_ERR;
        endcase
      end
      default:
        id_inst_type = INST_ERR;
    endcase

    id_load_unsigned = 0;
    case (id_inst_type)
      INST_LUI:
      begin
        id_op_len = 4'b1111;
        id_rs1_addr = 5'h0;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = ADD;
        id_imm = U_imm;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
      end
      INST_AUIPC:
      begin
        id_op_len = 4'b1111;
        id_rs1_addr = 5'h0;
        id_src1_rf_data = if_id_pc;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = ADD;
        id_imm = U_imm;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
      end
      INST_JAL:
      begin
        id_op_len = 4'b1111;
        id_rs1_addr = 5'h0;
        id_src1_rf_data = if_id_pc;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = ADD;
        id_imm = {{11{J_imm[20]}}, J_imm};
        id_rf_waddr = id_rd_addr;
        id_branch = 1;
        id_branch_op = JAL;
        id_is_jal = 1;
      end
      INST_JALR:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = ADD;
        id_imm = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch = 1;
        id_branch_op = JAL;
        id_is_jal = 1;
      end
      INST_BEQ:
      begin
        id_op_len    = 4'b1111;
        id_memwrite  = 0;
        id_rfwrite   = 0;
        id_memread   = 0;
        id_use_reg2  = 0;
        id_opcode    = ADD;
        id_imm       = {{19{B_imm[12]}}, B_imm};
        id_rf_waddr  = 5'h0;
        id_branch    = 1;
        id_branch_op = BEQ;
      end
      INST_BNE:
      begin
        id_op_len    = 4'b1111;
        id_memwrite  = 0;
        id_rfwrite   = 0;
        id_memread   = 0;
        id_use_reg2  = 0;
        id_opcode    = ADD;
        id_imm       = {{19{B_imm[12]}}, B_imm};
        id_rf_waddr  = 5'h0;
        id_branch    = 1;
        id_branch_op = BNE;
      end
      INST_BLT:
      begin
        id_op_len    = 4'b1111;
        id_memwrite  = 0;
        id_rfwrite   = 0;
        id_memread   = 0;
        id_use_reg2  = 0;
        id_opcode    = ADD;
        id_imm       = {{19{B_imm[12]}}, B_imm};
        id_rf_waddr  = 5'h0;
        id_branch    = 1;
        id_branch_op = BLT;
      end
      INST_BGE:
      begin
        id_op_len    = 4'b1111;
        id_memwrite  = 0;
        id_rfwrite   = 0;
        id_memread   = 0;
        id_use_reg2  = 0;
        id_opcode    = ADD;
        id_imm       = {{19{B_imm[12]}}, B_imm};
        id_rf_waddr  = 5'h0;
        id_branch    = 1;
        id_branch_op = BGE;
      end
      INST_BLTU:
      begin
        id_op_len    = 4'b1111;
        id_memwrite  = 0;
        id_rfwrite   = 0;
        id_memread   = 0;
        id_use_reg2  = 0;
        id_opcode    = ADD;
        id_imm       = {{19{B_imm[12]}}, B_imm};
        id_rf_waddr  = 5'h0;
        id_branch    = 1;
        id_branch_op = BLTU;
      end
      INST_BGEU:
      begin
        id_op_len    = 4'b1111;
        id_memwrite  = 0;
        id_rfwrite   = 0;
        id_memread   = 0;
        id_use_reg2  = 0;
        id_opcode    = ADD;
        id_imm       = {{19{B_imm[12]}}, B_imm};
        id_rf_waddr  = 5'h0;
        id_branch    = 1;
        id_branch_op = BGEU;
      end
      INST_LB:
      begin
        id_op_len   = 4'b0001;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 1;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_LH:
      begin
        id_op_len   = 4'b0011;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 1;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_LW:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 1;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_LBU:
      begin
        id_op_len   = 4'b0001;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 1;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
        id_load_unsigned = 1;
      end
      INST_LHU:
      begin
        id_op_len   = 4'b0011;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 1;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
        id_load_unsigned = 1;
      end
      INST_SB:
      begin
        id_op_len   = 4'b0001;
        id_memwrite = 1;
        id_rfwrite  = 0;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{S_imm[11]}}, S_imm};
        id_rf_waddr = 5'h0;
        id_branch   = 0;
      end
      INST_SH:
      begin
        id_op_len   = 4'b0011;
        id_memwrite = 1;
        id_rfwrite  = 0;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{S_imm[11]}}, S_imm};
        id_rf_waddr = 5'h0;
        id_branch   = 0;
      end
      INST_SW:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 1;
        id_rfwrite  = 0;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{S_imm[11]}}, S_imm};
        id_rf_waddr = 5'h0;
        id_branch   = 0;
      end
      INST_ADDI:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = ADD;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SLTI:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = SLT;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SLTIU:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = SLTU;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_XORI:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = XOR;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_ORI:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = OR;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_ANDI:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = AND;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SLLI:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = SLL;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SRLI:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = SRL;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SRAI:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = SRA;
        id_imm      = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_ADD:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 1;
        id_opcode   = ADD;
        id_imm      = 0;
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SUB:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 1;
        id_opcode   = SUB;
        id_imm      = 0;
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SLL:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 1;
        id_opcode   = SLL;
        id_imm      = 0;
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SLT:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 1;
        id_opcode   = SLT;
        id_imm      = 0;
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SLTU:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 1;
        id_opcode   = SLTU;
        id_imm      = 0;
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_XOR:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 1;
        id_opcode   = XOR;
        id_imm      = 0;
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_SRL:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 1;
        id_opcode = SRL;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
      end
      INST_SRA:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 1;
        id_opcode = SRA;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
      end
      INST_OR:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 1;
        id_opcode   = OR;
        id_imm      = 0;
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_AND:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 1;
        id_memread  = 0;
        id_use_reg2 = 1;
        id_opcode   = AND;
        id_imm      = 0;
        id_rf_waddr = id_rd_addr;
        id_branch   = 0;
      end
      INST_ANDN:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 1;
        id_opcode = ANDN;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
      end
      INST_SBSET:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 1;
        id_opcode = SBSET;
        id_imm = {{20{I_imm[11]}}, I_imm};
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
      end
      INST_PCNT:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = PCNT;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
      end
      INST_CRAS16:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 1;
        id_opcode = CRAS16;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
      end
      INST_CSRRW:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
        id_is_csr = 1;
        id_csr_type = 2'b01;
        id_csr_addr = if_id_instr[31:20];
      end
      INST_CSRRWI:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_src1_rf_data = {27'h0, if_id_instr[19:15]};
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
        id_is_csr = 1;
        id_csr_type = 2'b01;
        id_csr_addr = if_id_instr[31:20];
      end
      INST_CSRRS:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
        id_is_csr = 1;
        id_csr_type = 2'b10;
        id_csr_addr = if_id_instr[31:20];
      end
      INST_CSRRSI:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_src1_rf_data = {27'h0, if_id_instr[19:15]};
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
        id_is_csr = 1;
        id_csr_type = 2'b10;
        id_csr_addr = if_id_instr[31:20];
      end
      INST_CSRRC:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
        id_is_csr = 1;
        id_csr_type = 2'b11;
        id_csr_addr = if_id_instr[31:20];
      end
      INST_CSRRCI:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_src1_rf_data = {27'h0, if_id_instr[19:15]};
        id_rfwrite = 1;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = id_rd_addr;
        id_branch = 0;
        id_is_csr = 1;
        id_csr_type = 2'b11;
        id_csr_addr = if_id_instr[31:20];
      end
      INST_ECALL:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 0;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = 5'h0;
        id_branch = 0;
        id_is_ecall = 1;
      end
      INST_EBREAK:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 0;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = 5'h0;
        id_branch = 0;
        id_is_ebreak = 1;
      end
      INST_MRET:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 0;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = 5'h0;
        id_branch = 0;
        id_is_mret = 1;
      end
      INST_SRET:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 0;
        id_memread = 0;
        id_use_reg2 = 0;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = 5'h0;
        id_branch = 0;
        id_is_sret = 1;
      end
      INST_SFENCE:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 0;
        id_memread = 0;
        id_use_reg2 = 1;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = 5'h0;
        id_branch = 0;
        id_is_sfence = 1;
      end
      INST_FENCE:
      begin
        id_op_len = 4'b1111;
        id_memwrite = 0;
        id_rfwrite = 0;
        id_memread = 0;
        id_use_reg2 = 1;
        id_opcode = NULL;
        id_imm = 0;
        id_rf_waddr = 5'h0;
        id_branch = 0;
        id_is_fence = 1;
      end
      default:
      begin
        id_op_len   = 4'b1111;
        id_memwrite = 0;
        id_rfwrite  = 0;
        id_memread  = 0;
        id_use_reg2 = 0;
        id_opcode   = NULL;
        id_imm      = 0;
        id_rf_waddr = 5'h0;
        id_branch   = 0;
        id_undefined_instr_i = 1;
      end
    endcase
  end

  always_comb
  begin
    if((id_ex_rfwrite && (id_rs1_addr == id_ex_rf_waddr || id_rs2_addr == id_ex_rf_waddr) && id_ex_rf_waddr != 5'b00000)
        || (ex_dmmu_rfwrite && (id_rs1_addr == ex_dmmu_rf_waddr || id_rs2_addr == ex_dmmu_rf_waddr) && ex_dmmu_rf_waddr != 5'b00000)
        || (dmmu_mem_rfwrite && (id_rs1_addr == dmmu_mem_rf_waddr || id_rs2_addr == dmmu_mem_rf_waddr) && dmmu_mem_rf_waddr != 5'b00000)
        || (mem_wb_rfwrite && (id_rs1_addr == mem_wb_rf_waddr || id_rs2_addr == mem_wb_rf_waddr) && mem_wb_rf_waddr != 5'b00000))
      id_stall_o = 1;
    else if ((id_ex_is_csr || ex_dmmu_is_csr || dmmu_mem_is_csr) && id_is_csr)
      id_stall_o = 1;
    else
      id_stall_o = 0;
  end

  /* =========== EX stage =========== */
  assign alu_op = id_ex_opcode;
  assign alu_a = (id_ex_branch && !id_ex_is_jal) ? id_ex_pc : id_ex_src1_rf_data;
  assign alu_b = id_ex_usereg2 ? id_ex_src2_rf_data : id_ex_imm;
  assign ex_alu_res = alu_y;
  assign ex_branch_res = branch_y;
  assign ex_beq = id_ex_branch && branch_y && ex_alu_res[1:0] == 2'b00;

  /* =========== MEM stage =========== */
  assign dmmu_va_o = ex_dmmu_alu_res;
  assign dmmu_is_read = ex_dmmu_memread;
  assign dmmu_is_write = ex_dmmu_memwrite;
  assign dmmu_page_fault_r = dmmu_page_fault && ex_dmmu_memread;
  assign dmmu_page_fault_w = dmmu_page_fault && ex_dmmu_memwrite;
  typedef enum logic [2:0] {
            IDLE,
            READ,
            WRITE,
            DONE
          } mem_read_write_state_t;
  mem_read_write_state_t mem_read_write_state;

  logic mem_data_misaligned;

  always_ff @(posedge clk_i)
  begin
    if (rst_i)
      mem_read_write_state <= IDLE;
    else
    case (mem_read_write_state)
      IDLE:
      begin
        if (dmmu_mem_memread && !mem_data_misaligned && dmmu_mem_pa[31:28] != 4'h0 && !dmmu_page_fault && !dmmu_mem_flush_i && !interrupt_sel)
          mem_read_write_state <= READ;
        else if (dmmu_mem_memwrite && !mem_data_misaligned && dmmu_mem_pa[31:28] != 4'h0 && !dmmu_page_fault && !dmmu_mem_flush_i && !interrupt_sel)
          mem_read_write_state <= WRITE;
        else
          mem_read_write_state <= DONE;
      end
      READ:
      begin
        if (cpu_mem_master_ack_i)
        begin
          mem_read_write_state <= DONE;
          mem_mem_rdata <= cpu_mem_master_data_i;
        end
      end
      WRITE:
      begin
        if (cpu_mem_master_ack_i)
        begin
          mem_read_write_state <= DONE;
        end
      end
      DONE:
      begin
        if (!pipeline_stall)
          mem_read_write_state <= IDLE;
      end
    endcase
  end

  logic [31:0] dmmu_pa_i_reg;
  logic [31:0] dmmu_pa;
  logic [31:0] dmmu_page_fault_i_reg;
  logic [31:0] dmmu_page_fault_i;
  assign dmmu_pa = dmmu_ack_o ? dmmu_pa_i : dmmu_pa_i_reg;
  assign dmmu_page_fault = dmmu_ack_o ? dmmu_page_fault_i : dmmu_page_fault_i_reg;

  always_ff @(posedge clk_i)
  begin
    if(dmmu_ack_o)
    begin
      dmmu_pa_i_reg <= dmmu_pa_i;
      dmmu_page_fault_i_reg <= dmmu_page_fault_i;
    end
    if(!pipeline_stall)
      dmmu_page_fault_i_reg <= dmmu_page_fault_i;
  end

  logic [31:0] mem_rf_wdata_origin;
  logic [31:0] mem_mem_outdata;
  always_comb
  begin
    mem_rf_wdata_origin = (dmmu_mem_memread && dmmu_mem_pa[31:28] != 4'h0) ? mem_mem_rdata : dmmu_mem_alu_res;
    mem_mem_outdata = dmmu_mem_is_jal ? dmmu_mem_pc + 4 : mem_rf_wdata_origin;
    cpu_mem_master_addr_o = dmmu_mem_pa;
    cpu_mem_master_we_o = (dmmu_mem_memwrite && dmmu_mem_pa[31:28] != 4'h0) && mem_read_write_state != DONE;
    cpu_mem_master_sel_o = 4'b1111;  // default store 4 bytes
    cpu_mem_master_data_o = dmmu_mem_mem_wdata;  // default store 4 bytes
    cpu_mem_master_cyc_o = (mem_read_write_state == READ || mem_read_write_state == WRITE);
    cpu_mem_master_stb_o = (mem_read_write_state == READ || mem_read_write_state == WRITE);

    timer_mtime_e = 1'b0;
    timer_mtimecmp_e = 1'b0;
    timer_mtime_o = timer_mtime_i;
    timer_mtimecmp_o = timer_mtimecmp_i;
    mem_data_misaligned = 1'b0;

    if ((dmmu_mem_memread || dmmu_mem_memwrite) && dmmu_mem_pa[31:28] != 4'h0)
    begin
      case (dmmu_mem_op_len)
        // read or write 1 byte
        4'b0001:
        begin
          case (dmmu_mem_pa[1:0])
            2'b00:
            begin
              cpu_mem_master_sel_o = 4'b0001;
              mem_mem_outdata = dmmu_mem_load_unsigned ? {24'b0, mem_rf_wdata_origin[7:0]} : {{24{mem_rf_wdata_origin[7]}}, mem_rf_wdata_origin[7:0]};
              cpu_mem_master_data_o = {24'b0, dmmu_mem_mem_wdata[7:0]};
            end
            2'b01:
            begin
              cpu_mem_master_sel_o = 4'b0010;
              mem_mem_outdata = dmmu_mem_load_unsigned ? {24'b0, mem_rf_wdata_origin[15:8]} : {{24{mem_rf_wdata_origin[15]}}, mem_rf_wdata_origin[15:8]};
              cpu_mem_master_data_o = {16'b0, dmmu_mem_mem_wdata[7:0], 8'b0};
            end
            2'b10:
            begin
              cpu_mem_master_sel_o = 4'b0100;
              mem_mem_outdata = dmmu_mem_load_unsigned ? {24'b0, mem_rf_wdata_origin[23:16]} : {{24{mem_rf_wdata_origin[23]}}, mem_rf_wdata_origin[23:16]};
              cpu_mem_master_data_o = {8'b0, dmmu_mem_mem_wdata[7:0], 16'b0};
            end
            2'b11:
            begin
              cpu_mem_master_sel_o = 4'b1000;
              mem_mem_outdata = dmmu_mem_load_unsigned ? {24'b0, mem_rf_wdata_origin[31:24]} : {{24{mem_rf_wdata_origin[31]}}, mem_rf_wdata_origin[31:24]};
              cpu_mem_master_data_o = {dmmu_mem_mem_wdata[7:0], 24'b0};
            end
          endcase
        end
        // read or write 2 bytes
        4'b0011:
        begin
          case (dmmu_mem_pa[1:0])
            2'b00:
            begin
              cpu_mem_master_sel_o = 4'b0011;
              mem_mem_outdata = dmmu_mem_load_unsigned ? {16'b0, mem_rf_wdata_origin[15:0]}: {{16{mem_rf_wdata_origin[15]}}, mem_rf_wdata_origin[15:0]};
              cpu_mem_master_data_o = {16'b0, dmmu_mem_mem_wdata[15:0]};
            end
            2'b10:
            begin
              cpu_mem_master_sel_o = 4'b1100;
              mem_mem_outdata = dmmu_mem_load_unsigned ? {16'b0, mem_rf_wdata_origin[31:16]}: {{16{mem_rf_wdata_origin[31]}}, mem_rf_wdata_origin[31:16]};
              cpu_mem_master_data_o = {dmmu_mem_mem_wdata[15:0], 16'b0};
            end
            default:
              // data misaligned
              mem_data_misaligned = 1'b1;
          endcase
        end
        // default: 32-bit read or write
        4'b1111:
          if (dmmu_mem_pa[1:0] != 2'b00)
            // data misaligned
            mem_data_misaligned = 1'b1;
      endcase
    end

    if (dmmu_mem_memread && dmmu_mem_pa[31:28] == 4'h0)
    begin
      // clock
      case(dmmu_mem_pa[15:0])
        16'hBFF8:
          mem_mem_outdata = timer_mtime_i[31:0];
        16'hBFFC:
          mem_mem_outdata = timer_mtime_i[63:32];
        16'h4000:
          mem_mem_outdata = timer_mtimecmp_i[31:0];
        16'h4004:
          mem_mem_outdata = timer_mtimecmp_i[63:32];
      endcase
    end

    if (dmmu_mem_memwrite && dmmu_mem_pa[31:28] == 4'h0)
    begin
      // clock
      case(dmmu_mem_pa[15:0])
        16'hBFF8:
        begin
          timer_mtime_e = 1;
          timer_mtime_o = {timer_mtime_i[63:32], dmmu_mem_mem_wdata};
        end
        16'hBFFC:
        begin
          timer_mtime_e = 1;
          timer_mtime_o = {dmmu_mem_mem_wdata, timer_mtime_i[31:0]};
        end
        16'h4000:
        begin
          timer_mtimecmp_e = 1;
          timer_mtimecmp_o = {timer_mtimecmp_i[63:32], dmmu_mem_mem_wdata};
        end
        16'h4004:
        begin
          timer_mtimecmp_e = 1;
          timer_mtimecmp_o = {dmmu_mem_mem_wdata, timer_mtimecmp_i[31:0]};
        end
      endcase
    end
  end



  always_comb
  begin
    csr_addr_o  = dmmu_mem_csr_addr;
    csr_wdata_o = dmmu_mem_csr_data;
    csr_type_o  = dmmu_mem_csr_type;
  end

  assign mem_rf_wdata = dmmu_mem_is_csr ? csr_rdata_i : mem_mem_outdata;

  /* =========== WB stage =========== */
  assign rf_we = mem_wb_rfwrite;
  assign rf_waddr = mem_wb_rf_waddr;
  assign rf_wdata = mem_wb_rf_wdata;
endmodule
