 /*                                                                      
 Copyright 2017 Silicon Integrated Microelectronics, Inc.                
                                                                         
 Licensed under the Apache License, Version 2.0 (the "License");         
 you may not use this file except in compliance with the License.        
 You may obtain a copy of the License at                                 
                                                                         
     http://www.apache.org/licenses/LICENSE-2.0                          
                                                                         
  Unless required by applicable law or agreed to in writing, software    
 distributed under the License is distributed on an "AS IS" BASIS,       
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and     
 limitations under the License.                                          
 */                                                                      
                                                                         
                                                                         
                                                                         
//=====================================================================
//--        _______   ___
//--       (   ____/ /__/
//--        \ \     __
//--     ____\ \   / /
//--    /_______\ /_/   MICROELECTRONICS
//--
//=====================================================================
// Designer   : Bob Hu
//
// Description:
//  The EXU module to implement entire Execution Stage
//
// ====================================================================

`include "e203_defines.v"

module e203_exu(
  output commit_mret,
  output commit_trap,
  output exu_active,
  output excp_active,

  output core_wfi,
  output tm_stop,
  output itcm_nohold,
  output core_cgstop,
  output tcm_cgstop,

  input  [`E203_HART_ID_W-1:0] core_mhartid,
  input  dbg_irq_r,
  input  [`E203_LIRQ_NUM-1:0] lcl_irq_r,
  input  [`E203_EVT_NUM-1:0] evt_r,
  input  ext_irq_r,
  input  sft_irq_r,
  input  tmr_irq_r,

  //////////////////////////////////////////////////////////////
  // From/To debug ctrl module
  output  [`E203_PC_SIZE-1:0] cmt_dpc,
  output  cmt_dpc_ena,
  output  [3-1:0] cmt_dcause,
  output  cmt_dcause_ena,

  output wr_dcsr_ena    ,
  output wr_dpc_ena     ,
  output wr_dscratch_ena,



  output [`E203_XLEN-1:0] wr_csr_nxt    ,

  input [`E203_XLEN-1:0] dcsr_r    ,
  input [`E203_PC_SIZE-1:0] dpc_r     ,
  input [`E203_XLEN-1:0] dscratch_r,

  input  dbg_mode,
  input  dbg_halt_r,
  input  dbg_step_r,
  input  dbg_ebreakm_r,
  input  dbg_stopcycle,


  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  // The IFU IR stage to EXU interface
  input  i_valid, // Handshake signals with EXU stage
  output i_ready,

  input  [`E203_INSTR_SIZE-1:0] i_ir,// The instruction register
  input  [`E203_PC_SIZE-1:0] i_pc,   // The PC register along with
  input  i_pc_vld,             
  
  //////////////////////////////////////////////////////////////
  // The Flush interface to IFU
  //
  //   To save the gatecount, when we need to flush pipeline with new PC, 
  //     we want to reuse the adder in IFU, so we will not pass flush-PC
  //     to IFU, instead, we pass the flush-pc-adder-op1/op2 to IFU
  //     and IFU will just use its adder to caculate the flush-pc-adder-result
  //
  input   pipe_flush_ack,
  output  pipe_flush_req,
  output  [`E203_PC_SIZE-1:0] pipe_flush_add_op1,  
  output  [`E203_PC_SIZE-1:0] pipe_flush_add_op2,  
  `ifdef E203_TIMING_BOOST//}
  output  [`E203_PC_SIZE-1:0] pipe_flush_pc,  
  `endif//}

  output wfi_halt_ifu_req,
  input  wfi_halt_ifu_ack,


  output dec_rs1x0,
  output dec_rs2x0,
  output dec_rs1en,
  output dec_rs2en,
  output dec_rdwen,
  output [`E203_RFIDX_WIDTH-1:0] dec_rs1idx,
  output [`E203_RFIDX_WIDTH-1:0] dec_rs2idx,
  output [`E203_RFIDX_WIDTH-1:0] dec_rdidx,
  output [`E203_DECINFO_WIDTH-1:0] dec_info,  
  output [`E203_XLEN-1:0] dec_imm,
  output [`E203_PC_SIZE-1:0] dec_pc,
  output dec_misalgn,
  output dec_buserr,
  output dec_ilegl,

  output oitf_empty,
    

  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  // The AGU ICB Interface to LSU-ctrl
  //    * Bus cmd channel
  output                         agu_icb_cmd_valid, // Handshake valid
  input                          agu_icb_cmd_ready, // Handshake ready
  output [`E203_ADDR_SIZE-1:0]   agu_icb_cmd_addr, // Bus transaction start addr 
  output                         agu_icb_cmd_read,   // Read or write
  output [`E203_XLEN-1:0]        agu_icb_cmd_wdata, 
  output [`E203_XLEN/8-1:0]      agu_icb_cmd_wmask, 
  output                         agu_icb_cmd_lock,
  output                         agu_icb_cmd_excl,
  output [1:0]                   agu_icb_cmd_size,
  // Several additional side channel signals
  //   Indicate LSU-ctrl module to
  //     return the ICB response channel back to AGU
  //     this is only used by AMO or unaligned load/store 1st uop
  //     to return the response
  output                         agu_icb_cmd_back2agu, 
           //   Sign extension or not
  output                         agu_icb_cmd_usign,
  output [`E203_ITAG_WIDTH -1:0] agu_icb_cmd_itag,

  //    * Bus RSP channel
  input                          agu_icb_rsp_valid, // Response valid 
  output                         agu_icb_rsp_ready, // Response ready
  input                          agu_icb_rsp_err  , // Response error
  input                          agu_icb_rsp_excl_ok,
  input  [`E203_XLEN-1:0]        agu_icb_rsp_rdata,

  `ifdef E203_HAS_CSR_EAI//{
  output         eai_csr_valid,
  input          eai_csr_ready,
  output  [31:0] eai_csr_addr,
  output         eai_csr_wr,
  output  [31:0] eai_csr_wdata,
  input   [31:0] eai_csr_rdata,
  `endif//}

  input [`E203_XLEN-1:0] rf_rs1,
  input [`E203_XLEN-1:0] rf_rs2,
 
  // output exu_o_valid,
  // input exu_o_ready,


  output alu_wbck_o_valid,
  input alu_wbck_o_ready,
  output [`E203_XLEN-1:0] alu_wbck_o_wdat,
  output [`E203_RFIDX_WIDTH-1:0] alu_wbck_o_rdidx,

  output alu_wbck_o_valid_pre,
  output [`E203_XLEN-1:0] alu_wbck_o_wdat_pre,
  output [`E203_RFIDX_WIDTH-1:0] alu_wbck_o_rdidx_pre,

  output                     longp_excp_i_ready,
  input                      longp_excp_i_valid,
  input                      longp_excp_i_ld,
  input                      longp_excp_i_st,
  input                      longp_excp_i_buserr , // The load/store bus-error exception generated
  input [`E203_ADDR_SIZE-1:0]longp_excp_i_badaddr,
  input                      longp_excp_i_insterr,
  input [`E203_PC_SIZE-1:0]  longp_excp_i_pc,

  input oitf_ret_ena,
  output [`E203_ITAG_WIDTH-1:0] oitf_ret_ptr,
  output [`E203_RFIDX_WIDTH-1:0] oitf_ret_rdidx,
  output [`E203_PC_SIZE-1:0] oitf_ret_pc,
  output oitf_ret_rdwen,
  output oitf_ret_rdfpu,

  input  clk_aon,
  input  clk,
  input  rst_n
  );

  

  //////////////////////////////////////////////////////////////
  // Instantiate the Dispatch
  wire disp_alu_valid; 
  wire disp_alu_ready; 
  wire disp_alu_longpipe;
  wire [`E203_ITAG_WIDTH-1:0] disp_alu_itag;
  wire [`E203_XLEN-1:0] disp_alu_rs1;
  wire [`E203_XLEN-1:0] disp_alu_rs2;
  wire [`E203_XLEN-1:0] disp_alu_imm;
  wire [`E203_DECINFO_WIDTH-1:0]  disp_alu_info;  
  wire [`E203_PC_SIZE-1:0] disp_alu_pc;
  wire [`E203_RFIDX_WIDTH-1:0] disp_alu_rdidx;
  wire disp_alu_rdwen;
  wire disp_alu_ilegl;
  wire disp_alu_misalgn;
  wire disp_alu_buserr;

  wire [`E203_ITAG_WIDTH-1:0] disp_oitf_ptr;
  wire disp_oitf_ready;

  wire  disp_oitf_rs1fpu;
  wire  disp_oitf_rs2fpu;
  wire  disp_oitf_rs3fpu;
  wire  disp_oitf_rdfpu;
  wire  [`E203_RFIDX_WIDTH-1:0] disp_oitf_rs1idx;
  wire  [`E203_RFIDX_WIDTH-1:0] disp_oitf_rs2idx;
  wire  [`E203_RFIDX_WIDTH-1:0] disp_oitf_rs3idx;
  wire  [`E203_RFIDX_WIDTH-1:0] disp_oitf_rdidx;
  wire  disp_oitf_rs1en;
  wire  disp_oitf_rs2en;
  wire  disp_oitf_rs3en;
  wire  disp_oitf_rdwen;
  wire  [`E203_PC_SIZE-1:0] disp_oitf_pc;

  wire oitfrd_match_disprs1;
  wire oitfrd_match_disprs2;
  wire oitfrd_match_disprs3;
  wire oitfrd_match_disprd ;

  wire disp_oitf_ena;

  wire wfi_halt_exu_req;
  wire wfi_halt_exu_ack;

  wire amo_wait;


  wire flag_rs1 = (dec_rs1idx == alu_wbck_o_rdidx) & alu_wbck_o_valid;  
  wire [`E203_XLEN-1:0] rf_rs1_disp = flag_rs1 ? alu_wbck_o_wdat : rf_rs1  ;
  
  wire flag_rs2 = (dec_rs2idx == alu_wbck_o_rdidx) & alu_wbck_o_valid;  
  wire [`E203_XLEN-1:0] rf_rs2_disp = flag_rs2 ? alu_wbck_o_wdat : rf_rs2  ;



  e203_exu_disp u_e203_exu_disp(
    .wfi_halt_exu_req    (wfi_halt_exu_req),
    .wfi_halt_exu_ack    (wfi_halt_exu_ack),
    .oitf_empty          (oitf_empty),

    .amo_wait            (amo_wait),

    .disp_i_valid        (i_valid         ),
    .disp_i_ready        (i_ready         ),
                                       
    .disp_i_rs1x0        (dec_rs1x0       ),
    .disp_i_rs2x0        (dec_rs2x0       ),
    .disp_i_rs1en        (dec_rs1en       ),
    .disp_i_rs2en        (dec_rs2en       ),
    .disp_i_rs1idx       (dec_rs1idx      ),
    .disp_i_rs2idx       (dec_rs2idx      ),
    .disp_i_rdwen        (dec_rdwen       ),
    .disp_i_rdidx        (dec_rdidx       ),
    .disp_i_info         (dec_info        ),
    .disp_i_rs1          (rf_rs1_disp          ),
    .disp_i_rs2          (rf_rs2_disp          ),
    .disp_i_imm          (dec_imm        ),
    .disp_i_pc           (dec_pc         ),
    .disp_i_misalgn      (dec_misalgn    ),
    .disp_i_buserr       (dec_buserr     ),
    .disp_i_ilegl        (dec_ilegl      ),


    .disp_o_alu_valid    (disp_alu_valid   ),
    .disp_o_alu_ready    (disp_alu_ready   ),
    .disp_o_alu_longpipe (disp_alu_longpipe),
    .disp_o_alu_itag     (disp_alu_itag    ),
    .disp_o_alu_rs1      (disp_alu_rs1     ),
    .disp_o_alu_rs2      (disp_alu_rs2     ),
    .disp_o_alu_rdwen    (disp_alu_rdwen    ),
    .disp_o_alu_rdidx    (disp_alu_rdidx   ),
    .disp_o_alu_info     (disp_alu_info    ),
    .disp_o_alu_pc       (disp_alu_pc      ),
    .disp_o_alu_imm      (disp_alu_imm     ),
    .disp_o_alu_misalgn  (disp_alu_misalgn    ),
    .disp_o_alu_buserr   (disp_alu_buserr     ),
    .disp_o_alu_ilegl    (disp_alu_ilegl      ),

    .disp_oitf_ena       (disp_oitf_ena    ),
    .disp_oitf_ptr       (disp_oitf_ptr    ),
    .disp_oitf_ready     (disp_oitf_ready  ),

    .disp_oitf_rs1en     (disp_oitf_rs1en),
    .disp_oitf_rs2en     (disp_oitf_rs2en),
    .disp_oitf_rs3en     (disp_oitf_rs3en),
    .disp_oitf_rdwen     (disp_oitf_rdwen ),
    .disp_oitf_rs1idx    (disp_oitf_rs1idx),
    .disp_oitf_rs2idx    (disp_oitf_rs2idx),
    .disp_oitf_rs3idx    (disp_oitf_rs3idx),
    .disp_oitf_rdidx     (disp_oitf_rdidx ),
    .disp_oitf_rs1fpu    (disp_oitf_rs1fpu),
    .disp_oitf_rs2fpu    (disp_oitf_rs2fpu),
    .disp_oitf_rs3fpu    (disp_oitf_rs3fpu),
    .disp_oitf_rdfpu     (disp_oitf_rdfpu),
    .disp_oitf_pc        (disp_oitf_pc),

  
    .oitfrd_match_disprs1(oitfrd_match_disprs1),
    .oitfrd_match_disprs2(oitfrd_match_disprs2),
    .oitfrd_match_disprs3(oitfrd_match_disprs3),
    .oitfrd_match_disprd (oitfrd_match_disprd ),
    
    .clk                 (clk  ),
    .rst_n               (rst_n) 
  );
 
  e203_exu_oitf u_e203_exu_oitf(
    .dis_ready            (disp_oitf_ready),
    .dis_ena              (disp_oitf_ena  ),
    .ret_ena              (oitf_ret_ena  ),

    .dis_ptr              (disp_oitf_ptr  ),

    .ret_ptr              (oitf_ret_ptr  ),
    .ret_rdidx            (oitf_ret_rdidx),
    .ret_rdwen            (oitf_ret_rdwen),
    .ret_rdfpu            (oitf_ret_rdfpu),
    .ret_pc               (oitf_ret_pc),

    .disp_i_rs1en         (disp_oitf_rs1en),
    .disp_i_rs2en         (disp_oitf_rs2en),
    .disp_i_rs3en         (disp_oitf_rs3en),
    .disp_i_rdwen         (disp_oitf_rdwen ),
    .disp_i_rs1idx        (disp_oitf_rs1idx),
    .disp_i_rs2idx        (disp_oitf_rs2idx),
    .disp_i_rs3idx        (disp_oitf_rs3idx),
    .disp_i_rdidx         (disp_oitf_rdidx ),
    .disp_i_rs1fpu        (disp_oitf_rs1fpu),
    .disp_i_rs2fpu        (disp_oitf_rs2fpu),
    .disp_i_rs3fpu        (disp_oitf_rs3fpu),
    .disp_i_rdfpu         (disp_oitf_rdfpu ),
    .disp_i_pc            (disp_oitf_pc ),

    .oitfrd_match_disprs1 (oitfrd_match_disprs1),
    .oitfrd_match_disprs2 (oitfrd_match_disprs2),
    .oitfrd_match_disprs3 (oitfrd_match_disprs3),
    .oitfrd_match_disprd  (oitfrd_match_disprd ),

    .oitf_empty           (oitf_empty    ),

    .clk                  (clk           ),
    .rst_n                (rst_n         ) 
  );

  //////////////////////////////////////////////////////////////
  // Instantiate the ALU
  wire  alu_wbck_o_valid_nxt;// Handshake valid  
  wire  [`E203_XLEN-1:0] alu_wbck_o_wdat_nxt;
  wire  [`E203_RFIDX_WIDTH-1:0] alu_wbck_o_rdidx_nxt;
   
  wire alu_cmt_valid;
  wire alu_cmt_ready;
  wire alu_cmt_pc_vld;
  wire [`E203_PC_SIZE-1:0] alu_cmt_pc;
  wire [`E203_INSTR_SIZE-1:0] alu_cmt_instr;
  wire [`E203_XLEN-1:0]    alu_cmt_imm;
  wire alu_cmt_rv32;
  wire alu_cmt_bjp;
  wire alu_cmt_mret;
  wire alu_cmt_dret;
  wire alu_cmt_ecall;
  wire alu_cmt_ebreak;
  wire alu_cmt_wfi;
  wire alu_cmt_fencei;
  wire alu_cmt_ifu_misalgn;
  wire alu_cmt_ifu_buserr;
  wire alu_cmt_ifu_ilegl;
  wire alu_cmt_bjp_prdt;
  wire alu_cmt_bjp_rslv;
  wire alu_cmt_misalgn;
  wire alu_cmt_ld;
  wire alu_cmt_stamo;
  wire alu_cmt_buserr;
  wire [`E203_ADDR_SIZE-1:0] alu_cmt_badaddr;


  wire csr_ena;
  wire csr_wr_en;
  wire csr_rd_en;
  wire [12-1:0] csr_idx;

  wire [`E203_XLEN-1:0] read_csr_dat;
  wire [`E203_XLEN-1:0] wbck_csr_dat;

  wire flush_pulse;
  wire flush_req;

  wire nonflush_cmt_ena;


  wire eai_xs_off;

  wire csr_access_ilgl;

  wire mdv_nob2b;


  wire                         agu_icb_cmd_valid_nxt; // Handshake valid
  // input                          agu_icb_cmd_ready, // Handshake ready
  wire [`E203_ADDR_SIZE-1:0]   agu_icb_cmd_addr_nxt; // Bus transaction start addr 
  wire                         agu_icb_cmd_read_nxt;   // Read or write
  wire [`E203_XLEN-1:0]        agu_icb_cmd_wdata_nxt; 
  wire [`E203_XLEN/8-1:0]      agu_icb_cmd_wmask_nxt; 
  wire                         agu_icb_cmd_lock_nxt;
  wire                         agu_icb_cmd_excl_nxt;
  wire [1:0]                   agu_icb_cmd_size_nxt;
  wire                         agu_icb_cmd_back2agu_nxt;
  wire                         agu_icb_cmd_usign_nxt;
  wire [`E203_ITAG_WIDTH -1:0] agu_icb_cmd_itag_nxt;



  e203_exu_alu u_e203_exu_alu(


  `ifdef E203_HAS_CSR_EAI//{
    .eai_csr_valid (eai_csr_valid),
    .eai_csr_ready (eai_csr_ready),
    .eai_csr_addr  (eai_csr_addr ),
    .eai_csr_wr    (eai_csr_wr   ),
    .eai_csr_wdata (eai_csr_wdata),
    .eai_csr_rdata (eai_csr_rdata),
  `endif//}
    .csr_access_ilgl     (csr_access_ilgl),
    .nonflush_cmt_ena    (nonflush_cmt_ena),

    .i_valid             (disp_alu_valid   ),
    .i_ready             (disp_alu_ready   ),
    .i_longpipe          (disp_alu_longpipe),
    .i_itag              (disp_alu_itag    ),
    .i_rs1               (disp_alu_rs1     ),
    .i_rs2               (disp_alu_rs2     ),
    .eai_xs_off          (eai_xs_off),
    .i_rdwen             (disp_alu_rdwen   ),
    .i_rdidx             (disp_alu_rdidx   ),
    .i_info              (disp_alu_info    ),
    .i_pc                (i_pc    ),
    .i_pc_vld            (i_pc_vld),
    .i_instr             (i_ir    ),
    .i_imm               (disp_alu_imm     ),
    .i_misalgn           (disp_alu_misalgn    ),
    .i_buserr            (disp_alu_buserr     ),
    .i_ilegl             (disp_alu_ilegl      ),

    .flush_pulse         (flush_pulse    ),
    .flush_req           (flush_req      ),

    .oitf_empty          (oitf_empty),
    .amo_wait            (amo_wait),

    .cmt_o_valid         (alu_cmt_valid      ),
    .cmt_o_ready         (alu_cmt_ready      ),
    .cmt_o_pc_vld        (alu_cmt_pc_vld     ),
    .cmt_o_pc            (alu_cmt_pc         ),
    .cmt_o_instr         (alu_cmt_instr      ),
    .cmt_o_imm           (alu_cmt_imm        ),
    .cmt_o_rv32          (alu_cmt_rv32       ),
    .cmt_o_bjp           (alu_cmt_bjp        ),
    .cmt_o_dret          (alu_cmt_dret        ),
    .cmt_o_mret          (alu_cmt_mret        ),
    .cmt_o_ecall         (alu_cmt_ecall      ),
    .cmt_o_ebreak        (alu_cmt_ebreak     ),
    .cmt_o_fencei        (alu_cmt_fencei     ),
    .cmt_o_wfi           (alu_cmt_wfi        ),
    .cmt_o_ifu_misalgn   (alu_cmt_ifu_misalgn),
    .cmt_o_ifu_buserr    (alu_cmt_ifu_buserr ),
    .cmt_o_ifu_ilegl     (alu_cmt_ifu_ilegl  ),
    .cmt_o_bjp_prdt      (alu_cmt_bjp_prdt   ),
    .cmt_o_bjp_rslv      (alu_cmt_bjp_rslv   ),
    .cmt_o_misalgn       (alu_cmt_misalgn),
    .cmt_o_ld            (alu_cmt_ld),
    .cmt_o_stamo         (alu_cmt_stamo),
    .cmt_o_buserr        (alu_cmt_buserr),
    .cmt_o_badaddr       (alu_cmt_badaddr),

    .wbck_o_valid        (alu_wbck_o_valid_nxt ), 
    .wbck_o_ready        (alu_wbck_o_ready ),
    .wbck_o_wdat         (alu_wbck_o_wdat_nxt  ),
    .wbck_o_rdidx        (alu_wbck_o_rdidx_nxt ),

    .csr_ena             (csr_ena),
    .csr_idx             (csr_idx),
    .csr_rd_en           (csr_rd_en),
    .csr_wr_en           (csr_wr_en),
    .read_csr_dat        (read_csr_dat),
    .wbck_csr_dat        (wbck_csr_dat),

    .agu_icb_cmd_valid   (agu_icb_cmd_valid_nxt ),
    .agu_icb_cmd_ready   (agu_icb_cmd_ready ),
    .agu_icb_cmd_addr    (agu_icb_cmd_addr_nxt ),
    .agu_icb_cmd_read    (agu_icb_cmd_read_nxt   ),
    .agu_icb_cmd_wdata   (agu_icb_cmd_wdata_nxt ),
    .agu_icb_cmd_wmask   (agu_icb_cmd_wmask_nxt ),
    .agu_icb_cmd_lock    (agu_icb_cmd_lock_nxt),
    .agu_icb_cmd_excl    (agu_icb_cmd_excl_nxt),
    .agu_icb_cmd_size    (agu_icb_cmd_size_nxt),
   
    .agu_icb_cmd_back2agu(agu_icb_cmd_back2agu_nxt ),
    .agu_icb_cmd_usign   (agu_icb_cmd_usign_nxt),
    .agu_icb_cmd_itag    (agu_icb_cmd_itag_nxt),
  
    .agu_icb_rsp_valid   (agu_icb_rsp_valid ),
    .agu_icb_rsp_ready   (agu_icb_rsp_ready ),
    .agu_icb_rsp_err     (agu_icb_rsp_err   ),
    .agu_icb_rsp_excl_ok (agu_icb_rsp_excl_ok),
    .agu_icb_rsp_rdata   (agu_icb_rsp_rdata),

    .mdv_nob2b         (mdv_nob2b),

    .clk                 (clk          ),
    .rst_n               (rst_n        ) 
  );


  //////////////////////////////////////////////////////////////
  // Instantiate the Commit
  wire [`E203_ADDR_SIZE-1:0] cmt_badaddr;
  wire cmt_badaddr_ena;
  wire [`E203_PC_SIZE-1:0] cmt_epc;
  wire cmt_epc_ena;
  wire [`E203_XLEN-1:0] cmt_cause;
  wire cmt_cause_ena;
  wire cmt_instret_ena;
  wire cmt_status_ena;

  wire                      cmt_mret_ena;

  wire [`E203_PC_SIZE-1:0]  csr_epc_r;
  wire [`E203_PC_SIZE-1:0]  csr_dpc_r;
  wire [`E203_XLEN-1:0]     csr_mtvec_r;

  wire u_mode;
  wire s_mode;
  wire h_mode;
  wire m_mode;

  wire status_mie_r;
  wire mtie_r;
  wire msie_r;
  wire meie_r;



  e203_exu_commit u_e203_exu_commit(
    .commit_mret         (commit_mret),
    .commit_trap         (commit_trap),
    .core_wfi            (core_wfi        ),
    .nonflush_cmt_ena    (nonflush_cmt_ena),

    .excp_active         (excp_active),

    .amo_wait            (amo_wait     ),

    .wfi_halt_exu_req    (wfi_halt_exu_req),
    .wfi_halt_exu_ack    (wfi_halt_exu_ack),
    .wfi_halt_ifu_req    (wfi_halt_ifu_req),
    .wfi_halt_ifu_ack    (wfi_halt_ifu_ack),

    .dbg_irq_r               (dbg_irq_r),
    .lcl_irq_r               (lcl_irq_r),
    .ext_irq_r               (ext_irq_r),
    .sft_irq_r               (sft_irq_r),
    .tmr_irq_r               (tmr_irq_r),
    .evt_r                   (evt_r    ),

    .status_mie_r            (status_mie_r),
    .mtie_r                  (mtie_r      ),
    .msie_r                  (msie_r      ),
    .meie_r                  (meie_r      ),

    .alu_cmt_i_valid         (alu_cmt_valid      ),
    .alu_cmt_i_ready         (alu_cmt_ready      ),
    .alu_cmt_i_pc            (alu_cmt_pc         ),
    .alu_cmt_i_instr         (alu_cmt_instr      ),
    .alu_cmt_i_pc_vld        (alu_cmt_pc_vld     ),
    .alu_cmt_i_imm           (alu_cmt_imm        ),
    .alu_cmt_i_rv32          (alu_cmt_rv32       ),
    .alu_cmt_i_bjp           (alu_cmt_bjp        ),
    .alu_cmt_i_mret          (alu_cmt_mret        ),
    .alu_cmt_i_dret          (alu_cmt_dret        ),
    .alu_cmt_i_ecall         (alu_cmt_ecall      ),
    .alu_cmt_i_ebreak        (alu_cmt_ebreak     ),
    .alu_cmt_i_fencei        (alu_cmt_fencei     ),
    .alu_cmt_i_wfi           (alu_cmt_wfi     ),
    .alu_cmt_i_ifu_misalgn   (alu_cmt_ifu_misalgn),
    .alu_cmt_i_ifu_buserr    (alu_cmt_ifu_buserr ),
    .alu_cmt_i_ifu_ilegl     (alu_cmt_ifu_ilegl  ),
    .alu_cmt_i_bjp_prdt      (alu_cmt_bjp_prdt   ),
    .alu_cmt_i_bjp_rslv      (alu_cmt_bjp_rslv   ),
    .alu_cmt_i_misalgn       (alu_cmt_misalgn),
    .alu_cmt_i_ld            (alu_cmt_ld),
    .alu_cmt_i_stamo         (alu_cmt_stamo),
    .alu_cmt_i_buserr        (alu_cmt_buserr),
    .alu_cmt_i_badaddr       (alu_cmt_badaddr),


    .longp_excp_i_ready    (longp_excp_i_ready  ),
    .longp_excp_i_valid    (longp_excp_i_valid  ),
    .longp_excp_i_ld       (longp_excp_i_ld     ),
    .longp_excp_i_st       (longp_excp_i_st     ),
    .longp_excp_i_buserr   (longp_excp_i_buserr ),
    .longp_excp_i_badaddr  (longp_excp_i_badaddr),
    .longp_excp_i_insterr  (longp_excp_i_insterr),
    .longp_excp_i_pc       (longp_excp_i_pc     ),

    .dbg_mode              (dbg_mode),
    .dbg_halt_r            (dbg_halt_r),
    .dbg_step_r            (dbg_step_r),
    .dbg_ebreakm_r         (dbg_ebreakm_r),


    .oitf_empty            (oitf_empty),
    .u_mode                (u_mode),
    .s_mode                (s_mode),
    .h_mode                (h_mode),
    .m_mode                (m_mode),

    .cmt_badaddr           (cmt_badaddr    ), 
    .cmt_badaddr_ena       (cmt_badaddr_ena),
    .cmt_epc               (cmt_epc        ),
    .cmt_epc_ena           (cmt_epc_ena    ),
    .cmt_cause             (cmt_cause      ),
    .cmt_cause_ena         (cmt_cause_ena  ),
    .cmt_instret_ena       (cmt_instret_ena  ),
    .cmt_status_ena        (cmt_status_ena  ),
                           
    .cmt_dpc               (cmt_dpc        ),
    .cmt_dpc_ena           (cmt_dpc_ena    ),
    .cmt_dcause            (cmt_dcause     ),
    .cmt_dcause_ena        (cmt_dcause_ena ),

    .cmt_mret_ena            (cmt_mret_ena     ),
    .csr_epc_r               (csr_epc_r       ),
    .csr_dpc_r               (csr_dpc_r       ),
    .csr_mtvec_r             (csr_mtvec_r     ),

    .flush_pulse             (flush_pulse    ),
    .flush_req           (flush_req      ),

    .pipe_flush_ack          (pipe_flush_ack    ),
    .pipe_flush_req          (pipe_flush_req    ),
    .pipe_flush_add_op1      (pipe_flush_add_op1),  
    .pipe_flush_add_op2      (pipe_flush_add_op2),  
  `ifdef E203_TIMING_BOOST//}
    .pipe_flush_pc           (pipe_flush_pc),  
  `endif//}

    .clk                     (clk          ),
    .rst_n                   (rst_n        ) 
  );

    
  e203_exu_csr u_e203_exu_csr(
    .csr_access_ilgl     (csr_access_ilgl),
    .eai_xs_off          (eai_xs_off),
    .nonflush_cmt_ena    (nonflush_cmt_ena),
    .tm_stop             (tm_stop),
    .itcm_nohold         (itcm_nohold),
    .mdv_nob2b           (mdv_nob2b),
    .core_cgstop         (core_cgstop),
    .tcm_cgstop          (tcm_cgstop ),
    .csr_ena             (csr_ena),
    .csr_idx             (csr_idx),
    .csr_rd_en           (csr_rd_en),
    .csr_wr_en           (csr_wr_en),
    .read_csr_dat        (read_csr_dat),
    .wbck_csr_dat        (wbck_csr_dat),
   
    .cmt_badaddr           (cmt_badaddr    ), 
    .cmt_badaddr_ena       (cmt_badaddr_ena),
    .cmt_epc               (cmt_epc        ),
    .cmt_epc_ena           (cmt_epc_ena    ),
    .cmt_cause             (cmt_cause      ),
    .cmt_cause_ena         (cmt_cause_ena  ),
    .cmt_instret_ena       (cmt_instret_ena  ),
    .cmt_status_ena        (cmt_status_ena ),

    .cmt_mret_ena  (cmt_mret_ena     ),
    .csr_epc_r     (csr_epc_r       ),
    .csr_dpc_r     (csr_dpc_r       ),
    .csr_mtvec_r   (csr_mtvec_r     ),

    .wr_dcsr_ena     (wr_dcsr_ena    ),
    .wr_dpc_ena      (wr_dpc_ena     ),
    .wr_dscratch_ena (wr_dscratch_ena),

                                     
    .wr_csr_nxt      (wr_csr_nxt    ),
                                     
    .dcsr_r          (dcsr_r         ),
    .dpc_r           (dpc_r          ),
    .dscratch_r      (dscratch_r     ),
                                    
    .dbg_mode       (dbg_mode       ),
    .dbg_stopcycle  (dbg_stopcycle),

    .u_mode        (u_mode),
    .s_mode        (s_mode),
    .h_mode        (h_mode),
    .m_mode        (m_mode),

    .core_mhartid  (core_mhartid),

    .status_mie_r  (status_mie_r),
    .mtie_r        (mtie_r      ),
    .msie_r        (msie_r      ),
    .meie_r        (meie_r      ),

    .ext_irq_r     (ext_irq_r),
    .sft_irq_r     (sft_irq_r),
    .tmr_irq_r     (tmr_irq_r),

    .clk_aon       (clk_aon      ),
    .clk           (clk          ),
    .rst_n         (rst_n        ) 
  );
 

  wire  alu_wbck_o_valid_ena = alu_wbck_o_ready ;
  wire  alu_wbck_o_wdat_ena = alu_wbck_o_ready ;
  wire  alu_wbck_o_rdidx_ena = alu_wbck_o_ready ; 

  wire  agu_icb_cmd_valid_ena = agu_icb_cmd_ready ; 
  wire  agu_icb_cmd_addr_ena = agu_icb_cmd_ready;  
  wire  agu_icb_cmd_read_ena = agu_icb_cmd_ready; 
  wire  agu_icb_cmd_wdata_ena = agu_icb_cmd_ready; 
  wire  agu_icb_cmd_wmask_ena = agu_icb_cmd_ready;  
  wire  agu_icb_cmd_lock_ena = agu_icb_cmd_ready; 
  wire  agu_icb_cmd_excl_ena = agu_icb_cmd_ready; 
  wire  agu_icb_cmd_size_ena = agu_icb_cmd_ready; 
  wire  agu_icb_cmd_back2agu_ena = agu_icb_cmd_ready; 
  wire  agu_icb_cmd_usign_ena = agu_icb_cmd_ready; 
  wire  agu_icb_cmd_itag_ena = agu_icb_cmd_ready; 

  wire  alu_wbck_o_valid_r;
  wire  [`E203_XLEN-1:0] alu_wbck_o_wdat_r;
  wire  [`E203_RFIDX_WIDTH-1:0] alu_wbck_o_rdidx_r;

  wire                         agu_icb_cmd_valid_r; 
  wire [`E203_ADDR_SIZE-1:0]   agu_icb_cmd_addr_r;  
  wire                         agu_icb_cmd_read_r; 
  wire [`E203_XLEN-1:0]        agu_icb_cmd_wdata_r; 
  wire [`E203_XLEN/8-1:0]      agu_icb_cmd_wmask_r; 
  wire                         agu_icb_cmd_lock_r;
  wire                         agu_icb_cmd_excl_r;
  wire [1:0]                   agu_icb_cmd_size_r;
  wire                         agu_icb_cmd_back2agu_r;
  wire                         agu_icb_cmd_usign_r;
  wire [`E203_ITAG_WIDTH -1:0] agu_icb_cmd_itag_r;


  sirv_gnrl_dfflr #(1) alu_wbck_o_valid_dfflr (alu_wbck_o_valid_ena, alu_wbck_o_valid_nxt, alu_wbck_o_valid_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_XLEN) alu_wbck_o_wdat_dfflr (alu_wbck_o_wdat_ena, alu_wbck_o_wdat_nxt, alu_wbck_o_wdat_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_RFIDX_WIDTH) alu_wbck_o_rdidx_dfflr (alu_wbck_o_rdidx_ena, alu_wbck_o_rdidx_nxt, alu_wbck_o_rdidx_r, clk, rst_n);

  sirv_gnrl_dfflr #(1) agu_icb_cmd_valid_dfflr (agu_icb_cmd_valid_ena, agu_icb_cmd_valid_nxt, agu_icb_cmd_valid_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_ADDR_SIZE) agu_icb_cmd_addr_dfflr (agu_icb_cmd_addr_ena, agu_icb_cmd_addr_nxt, agu_icb_cmd_addr_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) agu_icb_cmd_read_dfflr (agu_icb_cmd_read_ena, agu_icb_cmd_read_nxt, agu_icb_cmd_read_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_XLEN) agu_icb_cmd_wdata_dfflr (agu_icb_cmd_wdata_ena, agu_icb_cmd_wdata_nxt, agu_icb_cmd_wdata_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_XLEN/8) agu_icb_cmd_wmask_dfflr (agu_icb_cmd_wmask_ena,agu_icb_cmd_wmask_nxt, agu_icb_cmd_wmask_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) agu_icb_cmd_lock_dfflr (agu_icb_cmd_lock_ena, agu_icb_cmd_lock_nxt, agu_icb_cmd_lock_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) agu_icb_cmd_excl_dfflr (agu_icb_cmd_excl_ena, agu_icb_cmd_excl_nxt, agu_icb_cmd_excl_r, clk, rst_n);
  sirv_gnrl_dfflr #(2) agu_icb_cmd_size_dfflr (agu_icb_cmd_size_ena, agu_icb_cmd_size_nxt, agu_icb_cmd_size_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) agu_icb_cmd_back2agu_dfflr (agu_icb_cmd_back2agu_ena, agu_icb_cmd_back2agu_nxt, agu_icb_cmd_back2agu_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) agu_icb_cmd_usign_dfflr (agu_icb_cmd_usign_ena, agu_icb_cmd_usign_nxt, agu_icb_cmd_usign_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_ITAG_WIDTH) agu_icb_cmd_itag_dfflr (agu_icb_cmd_itag_ena, agu_icb_cmd_itag_nxt, agu_icb_cmd_itag_r, clk, rst_n);


  assign alu_wbck_o_valid =  alu_wbck_o_valid_r;
  assign alu_wbck_o_wdat = alu_wbck_o_wdat_r;
  assign alu_wbck_o_rdidx = alu_wbck_o_rdidx_r;

  assign agu_icb_cmd_valid   = agu_icb_cmd_valid_r;
  assign agu_icb_cmd_addr    = agu_icb_cmd_addr_r;
  assign agu_icb_cmd_read    = agu_icb_cmd_read_r;
  assign agu_icb_cmd_wdata   = agu_icb_cmd_wdata_r;
  assign agu_icb_cmd_wmask   = agu_icb_cmd_wmask_r;
  assign agu_icb_cmd_lock    = agu_icb_cmd_lock_r;
  assign agu_icb_cmd_excl    = agu_icb_cmd_excl_r;
  assign agu_icb_cmd_size    = agu_icb_cmd_size_r;
  assign agu_icb_cmd_back2agu = agu_icb_cmd_back2agu_r;
  assign agu_icb_cmd_usign   = agu_icb_cmd_usign_r;
  assign agu_icb_cmd_itag    = agu_icb_cmd_itag_r;

  assign alu_wbck_o_valid_pre = alu_wbck_o_valid_nxt; 
  assign alu_wbck_o_wdat_pre = alu_wbck_o_wdat_nxt;
  assign alu_wbck_o_rdidx_pre = alu_wbck_o_rdidx_nxt; 
  
  assign exu_active =  (~oitf_empty) | i_valid | excp_active  ;
 

endmodule                            
                                               
