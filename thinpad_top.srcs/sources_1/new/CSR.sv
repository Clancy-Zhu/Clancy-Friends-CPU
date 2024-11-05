`timescale 1ns / 1ps `default_nettype none

module CSR (
    input wire clk,
    input wire reset,
    input wire pipeline_stall,
    input wire [31:0] instr,        // we only update CSR when instruction is not nop

    input  wire [11:0] csr_addr,
    input  wire [31:0] csr_wdata,
    input  wire [ 1:0] csr_type,
    output wire [31:0] csr_rdata,

    input  wire [31:0] pc_in,
    output wire pc_jump,
    output wire [31:0] pc_out,

    output wire [1:0] csr_mode_o,
    output reg interrupt_sel,

    input wire mret_i,
    input wire sret_i,

    input wire ecall_i,              // exception 8 + csr_mode
    input wire ebreak_i,             // exception 3

    input wire undefined_instr_i,    // exception 2
    input wire page_fault_i,         // exception 12
    input wire page_fault_r,         // exception 13
    input wire page_fault_w,         // exception 15
    input wire access_fault_i,       // exception 1
    input wire access_fault_r,       // exception 5
    input wire access_fault_w,       // exception 7
    input wire addr_misaligned_i,    // exception 0
    input wire addr_misaligned_r,    // exception 4
    input wire addr_misaligned_w,    // exception 6

    input wire [31:0] instr_fault_addr,
    input wire [31:0] data_fault_addr,

    input wire [63:0] mtime,
    input wire timer_interrupt,      // interrupt 7 (mtime>=mtimecmp)
    output wire [31:0] satp_o,
    output wire mstatus_sum_o
  );

  // type: w(01), s(10), c(11), r(00)
  // IMPORTANT: WE DECODE w(x0) to r since it has not actually writing

  /* ====== ALL NEEDED CSR REGISTERS ====== /

  * mhartid: Hart ID

  * mideleg: Interrupts
  * medeleg: Synchronous Exceptions
  * mstratch: mscratch
  * mstatus: MPP, SIE, SPIE, SPP, SUM
  * mepc: mepc
  * mcause: Interrupt, Exception Code
  * mtval: mtval
  * mtvec: BASE, MODE
  * mie: MTIE, STIE
  * mip: MTIP, STIP

  * satp: MODE, PPN
  * sstatus: SIE, SPIE, SPP, SUM
  * sepc: sepc
  * scause: Interrupt, Exception Code
  * stval: stval
  * stvec: BASE, MODE
  * sscratch: sscratch
  * sie: STIE
  * sip: STIP
  */


  // enum: CSR_MODE
  typedef enum reg [1:0] {
            USER,
            SUPERVISOR,
            ERROR,
            MACHINE
          } csr_mode_t;
  csr_mode_t csr_mode;


  logic instr_fault, data_fault;
  assign instr_fault = page_fault_i | access_fault_i | addr_misaligned_i;
  assign data_fault = page_fault_r | page_fault_w | access_fault_r | access_fault_w | addr_misaligned_r | addr_misaligned_w;

  logic [31:0] cause_exception_comb;
  logic [4:0] cause_idx;
  logic pc_jump_comb;
  logic [31:0] pc_out_comb;

  assign pc_jump = pc_jump_comb;
  assign pc_out = pc_out_comb;

  /* === THESE ARE ALL MRW REGISTERS === */
  reg [31:0] mscratch;
  // [12:11] MPP, [1] SIE, [5] SPIE, [8] SPP, [18] SUM
  reg [31:0] mstatus;
  reg [31:0] mepc;
  reg [31:0] mtval;
  // [31:2] base, [1:0] mode
  reg [31:0] mtvec;
  // [7] MTIE, [5] STIE
  reg [31:0] mie;
  // [7] MTIP, [5] STIP
  reg [31:0] mip;
  // [31], [30:0]
  reg [31:0] mcause;
  // [31:0]
  reg [31:0] mideleg;
  // [31:0]
  reg [31:0] medeleg;

  /* === THESE ARE ALL MRO REGISTERS === */
  // [31:0]
  reg [31:0] mhartid;

  /* === THESE ARE ALL SRW REGISTERS === */
  // [31] mode, [21:0] ppn
  reg [31:0] satp;
  // [1] SIE, [5] SPIE, [8] SPP, [18] SUM
  reg [31:0] sstatus;
  // [31:0]
  reg [31:0] sepc;
  // [31:0]
  reg [31:0] stval;
  // [31:2] base, [1:0] mode
  reg [31:0] stvec;
  // [31:0]
  reg [31:0] sscratch;
  // [5] STIE
  reg [31:0] sie;
  // [5] STIP
  reg [31:0] sip;
  // [31:0]
  reg [31:0] scause;

  logic valid;
  assign valid = instr != 32'h00000013; // nop

  assign satp_o = satp;
  assign csr_mode_o = csr_mode;
  assign mstatus_sum_o = mstatus[18];
  // These should be the same
  always_comb
  begin
    sie = 0;
    sie[5] = mie[5];
    sip = 0;
    sip[5] = mip[5];
    sstatus = 0;
    sstatus[1] = mstatus[1];
    sstatus[5] = mstatus[5];
    sstatus[8] = mstatus[8];
    sstatus[18] = mstatus[18];
  end

  logic [31:0] rdata;
  logic [31:0] wdata;
  assign csr_rdata = rdata;
  always_comb
  begin
    case(csr_type)
      2'b00, 2'b01:  // w
        wdata = csr_wdata;
      2'b10:  // s
        wdata = csr_wdata | rdata;
      2'b11:  // c
        wdata = ~csr_wdata & rdata;
    endcase
  end

  always_comb
  begin
    rdata = 0;
    /* === MRO REGISTERS === */
    if (csr_addr[11:8] == 4'hF && csr_mode == MACHINE)
    begin
      case (csr_addr[7:0])
        8'h14:  // mhartid
          rdata = mhartid;
        default:
          rdata = 0;
      endcase
    end

    /* === MRW REGISTERS === */
    if (csr_addr[11:8] == 4'h3 && csr_mode == MACHINE)
    begin
      case (csr_addr[7:0])
        8'h00:  // mstatus
          rdata = mstatus;
        8'h02:  // medeleg
          rdata = medeleg;
        8'h03:  // mideleg
          rdata = mideleg;
        8'h04:  // mie
          rdata = mie;
        8'h05:  // mtvec
          rdata = mtvec;
        8'h40:  // mscratch
          rdata = mscratch;
        8'h41:  // mepc
          rdata = mepc;
        8'h42:  // mcause
          rdata = mcause;
        8'h43:  // mtval
          rdata = mtval;
        8'h44:  // mip
          rdata = mip;
        default:
          rdata = 0;
      endcase
    end

    /* === SRW REGISTERS === */
    if (csr_addr[11:8] == 4'h1 && (csr_mode == SUPERVISOR || csr_mode == MACHINE))
    begin
      case (csr_addr[7:0])
        8'h00:  // sstatus
          rdata = sstatus;
        8'h04:  // sie
          rdata = sie;
        8'h05:  // stvec
          rdata = stvec;
        8'h40:  // sscratch
          rdata = sscratch;
        8'h41:  // sepc
          rdata = sepc;
        8'h42:  // scause
          rdata = scause;
        8'h43:  // stval
          rdata = stval;
        8'h44:  // sip
          rdata = sip;
        8'h80:  // satp
          rdata = satp;
        default:
          rdata = 0;
      endcase
    end

    /* === TIMERS === */
    if (csr_addr == 12'hC01)
      rdata = mtime[31:0];
    if (csr_addr == 12'hC81)
      rdata = mtime[63:32];
  end


  always_ff @(posedge clk or posedge reset)
  begin
    if (reset)
    begin
      csr_mode <= MACHINE;
      mscratch <= 0;
      mstatus <= 0;
      mepc <= 0;
      mtval <= 0;
      mtvec <= 0;
      mie <= 0;
      mip <= 0;
      mcause <= 0;
      mideleg <= 0;
      medeleg <= 0;
      mhartid <= 0;
      satp <= 0;
      sepc <= 0;
      stval <= 0;
      stvec <= 0;
      sscratch <= 0;
      scause <= 0;
    end
    else if (!pipeline_stall && valid)
    begin
      mip[7] <= timer_interrupt;
      /* === MRW REGISTERS === */
      if(csr_type != 2'b00)
      begin
        if (csr_addr[11:8] == 4'h3 && csr_mode == MACHINE)
        begin
          case (csr_addr[7:0])
            8'h00:  // mstatus
              mstatus   <= wdata;
            8'h02:  // medeleg
            begin
              medeleg   <= wdata;
              medeleg[11] <= 0;   // disable ecall from M mode
            end
            8'h03:  // mideleg
              mideleg   <= wdata;
            8'h04:  // mie
              mie       <= wdata;
            8'h05:  // mtvec
              mtvec     <= wdata;
            8'h40:  // mscratch
              mscratch  <= wdata;
            8'h41:  // mepc
              mepc[31:2] <= wdata[31:2];
            8'h42:  // mcause
              mcause    <= wdata;
            8'h43:  // mtval
              mtval     <= wdata;
            8'h44:  // mip
              mip[5] <= wdata[5]; // only STIP is writable
            default:
            begin
              // ERROR
            end
          endcase
        end
        /* === SRW REGISTERS === */
        else if (csr_addr[11:8] == 4'h1 && (csr_mode == SUPERVISOR || csr_mode == MACHINE))
        begin
          case (csr_addr[7:0])
            8'h00:  // sstatus
            begin
              mstatus   <= wdata;
              mstatus[12:11] <= mstatus[12:11]; // MPP is read only
            end
            8'h04:  // sie
            begin
              mie    <= wdata;
              mie[7] <= mie[7]; // MTIP is read only in S mode
            end
            8'h05:  // stvec
              stvec     <= wdata;
            8'h40:  // sscratch
              sscratch  <= wdata;
            8'h41:  // sepc
              sepc[31:2] <= wdata[31:2];
            8'h42:  // scause
              scause    <= wdata;
            8'h43:  // stval
              stval     <= wdata;
            8'h44:  // sip
            begin
              // sip is read only in S mode
            end
            8'h80:  // satp
              satp      <= wdata;
            default:
            begin
              // ERROR
            end
          endcase
        end
      end
      /* === EXCEPTIONS === */
      else
      begin
        if(cause_exception_comb != 0) // exception detected
        begin
          if (cause_exception_comb & medeleg) // medeleg. use SUPERVISOR mode
          begin
            csr_mode <= SUPERVISOR;
            sepc <= pc_in;
            scause <= cause_idx;
            if(instr_fault)
              stval <= instr_fault_addr;
            else if(data_fault)
              stval <= data_fault_addr;
            else if(undefined_instr_i)
              stval <= instr;
            else
              stval <= 0;
            mstatus[5] <= mstatus[1]; // set SPIE
            mstatus[1] <= 0; // disable SIE
            mstatus[8] <= csr_mode[0]; // set SPP
          end
          else // use MACHINE mode
          begin
            csr_mode <= MACHINE;
            mepc <= pc_in;
            mcause <= cause_idx;
            if(instr_fault)
              mtval <= instr_fault_addr;
            else if(data_fault)
              mtval <= data_fault_addr;
            else if(undefined_instr_i)
              mtval <= instr;
            else
              mtval <= 0;
            mstatus[12:11] <= csr_mode; // set MPP
          end
        end
        else if (mip[7] && mie[7] && (csr_mode != MACHINE))
        begin
          csr_mode <= MACHINE;
          mepc <= pc_in;
          mcause <= {1'b1, 31'h7}; // interrupt 7
          mip[7] <= 0; // clear MTIP
          mstatus[12:11] <= csr_mode; // set MPP
        end
        else if (sip[5] && sie[5] && (csr_mode == USER || (csr_mode == SUPERVISOR && mstatus[1])))
        begin
          if(32'h00000020 & mideleg) // mideleg. use SUPERVISOR mode
          begin
            csr_mode <= SUPERVISOR;
            sepc <= pc_in;
            scause <= {1'b1, 31'h5}; // interrupt 5
            mstatus[5] <= mstatus[1]; // set SPIE
            mstatus[1] <= 0; // disable SIE
            mstatus[8] <= csr_mode[0]; // set SPP
            mip[5] <= 0; // clear STIP
          end
          else
          begin
            csr_mode <= MACHINE;
            mepc <= pc_in;
            mcause <= {1'b1, 31'h5}; // interrupt 5
            mstatus[12:11] <= csr_mode; // set MPP
          end
        end
        else if (mret_i && csr_mode == MACHINE)
        begin
          csr_mode <= mstatus[12]
                   ? MACHINE : (mstatus[11] ? SUPERVISOR : USER);
        end
        else if (sret_i && csr_mode == SUPERVISOR)
        begin
          csr_mode <= mstatus[8]
                   ? SUPERVISOR : USER;
          mstatus[1] <= mstatus[5]; // set SIE
          mstatus[5] <= 1; // set SPIE
        end
      end
    end
  end
  /* === EXCEPTIONS === */
  always_comb
  begin
    pc_jump_comb = 0;
    pc_out_comb = 0;
    interrupt_sel = 0;
    if(valid && csr_type == 2'b00)
      if(cause_exception_comb != 0) // exception detected
      begin
        pc_jump_comb = 1;
        if (cause_exception_comb & medeleg) // medeleg. use SUPERVISOR mode
        begin
          pc_out_comb = {stvec[31:2], 2'b00};
        end
        else // use MACHINE mode
        begin
          pc_out_comb = {mtvec[31:2], 2'b00};
        end
      end
      else if (mip[7] && mie[7] && (csr_mode != MACHINE))
      begin
        pc_jump_comb = 1;
        interrupt_sel = 1;
        if(mideleg[7])
        begin
          if(stvec[0])
            pc_out_comb = {(stvec[31:2] + 7), 2'b00};
          else
            pc_out_comb = {stvec[31:2], 2'b00};
        end
        else
        begin
          if(mtvec[0])
            pc_out_comb = {(mtvec[31:2] + 7), 2'b00};
          else
            pc_out_comb = {mtvec[31:2], 2'b00};
        end
      end
      else if (sip[5] && sie[5] && (csr_mode == USER || (csr_mode == SUPERVISOR && mstatus[1])))
      begin
        pc_jump_comb = 1;
        interrupt_sel = 1;
        if(mideleg[5])
        begin
          if(stvec[0])
            pc_out_comb = {(stvec[31:2] + 5), 2'b00};
          else
            pc_out_comb = {stvec[31:2], 2'b00};
        end
        else
        begin
          if(mtvec[0])
            pc_out_comb = {(mtvec[31:2] + 5), 2'b00};
          else
            pc_out_comb = {mtvec[31:2], 2'b00};
        end
      end
      else if (mret_i && csr_mode == MACHINE)
      begin
        pc_jump_comb = 1;
        pc_out_comb = {mepc[31:2], 2'b00};
      end
      else if (sret_i && csr_mode == SUPERVISOR)
      begin
        pc_jump_comb = 1;
        pc_out_comb = {sepc[31:2], 2'b00};
      end
  end

  always_comb
  begin
    cause_exception_comb = 0;
    cause_idx = 0;
    if(valid)
    begin
      if(ebreak_i)
      begin
        cause_exception_comb[3] = 1;
        cause_idx = 3;
      end
      if(undefined_instr_i)
      begin
        cause_exception_comb[2] = 1;
        cause_idx = 2;
      end
      if(page_fault_i)
      begin
        cause_exception_comb[12] = 1;
        cause_idx = 12;
      end
      if(page_fault_r)
      begin
        cause_exception_comb[13] = 1;
        cause_idx = 13;
      end
      if(page_fault_w)
      begin
        cause_exception_comb[15] = 1;
        cause_idx = 15;
      end
      if(access_fault_i)
      begin
        cause_exception_comb[1] = 1;
        cause_idx = 1;
      end
      if(access_fault_r)
      begin
        cause_exception_comb[5] = 1;
        cause_idx = 5;
      end
      if(access_fault_w)
      begin
        cause_exception_comb[7] = 1;
        cause_idx = 7;
      end
      if(addr_misaligned_i)
      begin
        cause_exception_comb[0] = 1;
        cause_idx = 0;
      end
      if(addr_misaligned_r)
      begin
        cause_exception_comb[4] = 1;
        cause_idx = 4;
      end
      if(addr_misaligned_w)
      begin
        cause_exception_comb[6] = 1;
        cause_idx = 6;
      end
      if (ecall_i && csr_mode == USER)
      begin
        cause_exception_comb[8] = 1;
        cause_idx = 8;
      end
      if (ecall_i && csr_mode == SUPERVISOR)
      begin
        cause_exception_comb[9] = 1;
        cause_idx = 9;
      end
      if (ecall_i && csr_mode == MACHINE)
      begin
        cause_exception_comb[11] = 1;
        cause_idx = 11;
      end
    end
  end

endmodule
