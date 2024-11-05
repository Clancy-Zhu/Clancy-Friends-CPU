// Memory management unit, translates virtual addresses to physical addresses
// contains a TLB and a two-level page table

`default_nettype none
module mmu (
    input wire clk,
    input wire rst,

    input wire rst_tlb_i,// reset tlb
    input wire en_i,//enable

    input wire [31:0] va_i,// virtual address input
    input wire [31:0] satp_i,// satp register
    input wire [1:0] priv_mode_i,// user, supervisor, machine
    input wire [2:0]  permission_i, // x, w, r
    input wire sum_i,// csr.mstatus.sum

    output reg [31:0] pa_o,// physical address output
    output reg mmu_ack_o,// mmu ack
    output reg page_fault_o,// page fault

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
  enum logic [2:0]{
         mmu_init,
         mmu_lv1,
         mmu_lv1_done,
         mmu_lv2
       }mmu_state;

  // TLB
  logic tlb_valid;
  logic tlb_hit;// tlb hit if match tlb_tag and tlb_vpn
  logic [19:0] tlb_tag;
  logic [31:0] tlb_entry;

  // page table
  logic pt_enabled;
  logic [31:0] pt_lv1_addr;
  logic [31:0] pt_lv2_addr;
  logic [31:0] pt_lv1_entry;
  logic [31:0] pt_lv2_entry;

  assign pt_enabled = (satp_i[31]&(priv_mode_i!=2'b11));//enable page table if satp.mode==1 and priv_mode!=machine
  assign tlb_hit=(pt_enabled&tlb_valid&tlb_tag==va_i[31:12]);// tlb hit if tlb valid and tlb tag match va_vpn
  // assign tlb_hit = 0;

  //part of the virtual address
  logic [11:0]  va_offset;

  logic [9:0]   va_vpn_1;
  logic [9:0]   va_vpn_2;
  assign va_offset = va_i[11:0];
  assign va_vpn_2 = va_i[21:12];
  assign va_vpn_1 = va_i[31:22];

  //final part of the physical address
  logic [19:0] leaf_ppn;
  assign pa_o = {leaf_ppn, va_offset};

  //todo: access memory
  logic mem_read_req;// wb_syn
  logic mem_ack;// wb_ack

  always_comb
  begin
    leaf_ppn = 20'b0;
    page_fault_o = 1'b0;
    mem_read_req = 1'b0;
    wb_adr_o = 32'b0;
    mmu_ack_o = 1'b0;

    case(mmu_state)
      mmu_init:
      begin
        if(en_i)
        begin
          if(pt_enabled)
          begin
            if(tlb_hit)
            begin
              //check page fault
              if((priv_mode_i==2'b00&&!tlb_entry[4])||
                  ((permission_i & ~tlb_entry[3:1]) != 3'b0))//(user but not user page) or (supervisor but user page and not sum) or (permission not match)
              begin
                page_fault_o = 1'b1;
              end
              leaf_ppn = tlb_entry[29:10];//simply omit the highest 2 bits
              mmu_ack_o = 1'b1;
            end
          end
          else
          begin // no page table
            leaf_ppn = va_i[31:12];
            mmu_ack_o = 1'b1;
          end
        end
      end
      mmu_lv1:
      begin
        mem_read_req = 1'b1;
        wb_adr_o[31:12] = satp_i[19:0];
        wb_adr_o[11:2]=va_vpn_1;
        wb_adr_o[1:0] = 2'b0;
      end
      mmu_lv1_done:
      begin
        //If pte:v = 0, or if pte is not pointer, stop and raise a page-fault exception
        if(!(pt_lv1_entry[0])||(pt_lv1_entry[3:1]!=3'b0))
        begin
          page_fault_o = 1'b1;
          mmu_ack_o = 1'b1;
        end
      end
      mmu_lv2:
      begin
        mem_read_req = 1'b1;
        wb_adr_o[31:12] = pt_lv1_entry[29:10];
        wb_adr_o[11:2] = va_vpn_2;
        wb_adr_o[1:0] = 2'b0;
        if (mem_ack)
        begin
          //check page fault
          if(!wb_dat_i[0]
              ||wb_dat_i[2:1]==2'b10||
              (priv_mode_i==2'b00&&!wb_dat_i[4])||
              (permission_i & ~wb_dat_i[3:1]) != 3'b0
            )//(user but not user page) or (supervisor but user page and not sum) or (permission not match)
          begin
            page_fault_o = 1'b1;
          end
          else
          begin
            leaf_ppn = wb_dat_i[29:10];
          end
          mmu_ack_o = 1'b1;
        end
      end
    endcase
  end

  // TLB
  always_ff @ (posedge clk)
  begin
    if(rst|rst_tlb_i)
    begin
      tlb_valid <= 0;
      tlb_entry <= 0;
      tlb_tag <= 0;
    end
    else
    begin
      if(mmu_state==mmu_lv2&&!page_fault_o&&mem_ack)
      begin
        tlb_valid <= 1;
        tlb_entry <= wb_dat_i;
        tlb_tag <= va_i[31:12];
      end
    end
  end



  always_ff @ (posedge clk or posedge rst)
  begin//state machine
    if (rst)
    begin
      mmu_state <= mmu_init;
    end
    else
    begin
      case(mmu_state)
        mmu_init:
        begin
          if (en_i)
          begin
            if (tlb_hit || !pt_enabled)
            begin
              mmu_state <= mmu_init;
            end
            else
            begin
              mmu_state <= mmu_lv1;
            end
          end
        end
        mmu_lv1:
        begin
          if (mem_ack)
          begin
            pt_lv1_entry <= wb_dat_i;
            mmu_state <= mmu_lv1_done;
          end
        end
        mmu_lv1_done:
        begin
          if (page_fault_o)
          begin
            mmu_state <= mmu_init;
          end
          else
          begin
            mmu_state <= mmu_lv2;
          end
        end
        mmu_lv2:
        begin
          if (mem_ack)
          begin
            mmu_state <= mmu_init;
          end
        end
        default:
        begin
          mmu_state <= mmu_init;
        end
      endcase
    end
  end
  // wishbone interface
  always_comb
  begin
    wb_stb_o = mem_read_req || mmu_state==mmu_lv1_done;
    wb_cyc_o = mem_read_req;
    wb_dat_o = 32'b0;
    wb_sel_o = 4'b1111;
    wb_we_o = 1'b0;
    mem_ack = wb_ack_i;
  end



endmodule
