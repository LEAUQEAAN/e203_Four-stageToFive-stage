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
//
// Designer   : Bob Hu
//
// Description:
//  The decode module to decode the instruction details
//
// ====================================================================
`include "e203_defines.v"

module e203_decode(

  input  i_valid,  
  output i_ready,
  
  //////////////////////////////////////////////////////////////
  // The IR stage to Decoder
  input  [`E203_INSTR_SIZE-1:0] i_ir,
  input  [`E203_PC_SIZE-1:0] i_pc,
  input  i_pc_vld,        
      

  input  i_prdt_taken, 
  input  i_misalgn,              // The fetch misalign
  input  i_buserr,               // The fetch bus error
  input  i_muldiv_b2b,           // The back2back case for mul/div
  input [`E203_RFIDX_WIDTH-1:0] i_rs1idx,
  input [`E203_RFIDX_WIDTH-1:0] i_rs2idx,
  input  dbg_mode,
  //////////////////////////////////////////////////////////////
  // The Decoded Info-Bus

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
  
  output dec2ifu_rden,
  output dec2ifu_rs1en,
  output [`E203_RFIDX_WIDTH-1:0] dec2ifu_rdidx,

  output dec2ifu_mulhsu,
  output dec2ifu_div   ,
  output dec2ifu_rem   ,
  output dec2ifu_divu  ,
  output dec2ifu_remu  ,

  output [`E203_XLEN-1:0] rf2ifu_x1,
  output [`E203_XLEN-1:0] rf2ifu_rs1,

  output [`E203_XLEN-1:0] rf_rs1,
  output [`E203_XLEN-1:0] rf_rs2,

  input rf_wbck_ena,
  input [`E203_XLEN-1:0] rf_wbck_wdat,
  input [`E203_RFIDX_WIDTH-1:0] rf_wbck_rdidx,
  

  input  rf_wbck_o_ena_pre,
  input  [`E203_XLEN-1:0] rf_wbck_o_wdat_pre,
  input  [`E203_RFIDX_WIDTH-1:0] rf_wbck_o_rdidx_pre,

  input alu_wbck_o_valid_pre,
  input [`E203_XLEN-1:0] alu_wbck_o_wdat_pre,
  input [`E203_RFIDX_WIDTH-1:0] alu_wbck_o_rdidx_pre,
 
  input  test_mode,
  input  clk,
  input  clk_exu,
  input  rst_n,
  output dec_active,

  output dec_o_valid,
  input dec_o_ready,

  output [`E203_INSTR_SIZE-1:0] dec_o_ir,
  output [`E203_PC_SIZE-1:0] dec_o_pc,
  output dec_o_pc_vld,
  input ifu_o_ir_pc_vld_set
  );


  // //////////////////////////////////////////////////////////////
  // // Instantiate the Decode
  wire [`E203_DECINFO_WIDTH-1:0]  dec_info_nxt;
  wire [`E203_XLEN-1:0] dec_imm_nxt;
  wire [`E203_PC_SIZE-1:0] dec_pc_nxt;
  wire dec_rs1x0_nxt;
  wire dec_rs2x0_nxt;
  wire dec_rdwen_nxt;
  wire [`E203_RFIDX_WIDTH-1:0] dec_rdidx_nxt;
  wire dec_misalgn_nxt;
  wire dec_buserr_nxt;
  wire dec_ilegl_nxt;
  wire dec_rs1en_nxt;
  wire dec_rs2en_nxt; 



  ////////////////////////////////////////////////////////////
  // The Decoded Info-Bus
  e203_dec_decode u_e203_dec_decode (
    .dbg_mode     (dbg_mode),

    .i_instr      (i_ir    ),
    .i_pc         (i_pc    ),
    .i_misalgn    (i_misalgn),
    .i_buserr     (i_buserr ),
    .i_prdt_taken (i_prdt_taken), 
    .i_muldiv_b2b (i_muldiv_b2b), 
      
    .dec_rv32  (),
    .dec_bjp   (),
    .dec_jal   (),
    .dec_jalr  (),
    .dec_bxx   (),
    .dec_jalr_rs1idx(),
    .dec_bjp_imm(),

    .dec_mulhsu  (dec2ifu_mulhsu),
    .dec_mul     (),
    .dec_div     (dec2ifu_div   ),
    .dec_rem     (dec2ifu_rem   ),
    .dec_divu    (dec2ifu_divu  ),
    .dec_remu    (dec2ifu_remu  ),

    .dec_info  (dec_info_nxt ),
    .dec_rs1x0 (dec_rs1x0_nxt),
    .dec_rs2x0 (dec_rs2x0_nxt),
    .dec_rs1en (dec_rs1en_nxt),
    .dec_rs2en (dec_rs2en_nxt),
    .dec_rdwen (dec_rdwen_nxt),
    .dec_rs1idx(),
    .dec_rs2idx(),
    .dec_misalgn(dec_misalgn_nxt),
    .dec_buserr (dec_buserr_nxt ),
    .dec_ilegl  (dec_ilegl_nxt),
    .dec_rdidx (dec_rdidx_nxt),
    .dec_pc    (dec_pc_nxt),
    .dec_imm   (dec_imm_nxt)

  );


  assign dec2ifu_rden  = dec_rdwen_nxt; 
  assign dec2ifu_rs1en = dec_rs1en_nxt;
  assign dec2ifu_rdidx = dec_rdidx_nxt; 



  wire [`E203_RFIDX_WIDTH-1:0] dec_rs1idx_nxt = i_rs1idx;
  wire [`E203_RFIDX_WIDTH-1:0] dec_rs2idx_nxt = i_rs2idx;


  wire dec_rs1idx_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_rs2idx_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_rdidx_ena = ifu_o_ir_pc_vld_set  ; 
  wire dec_rs1en_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_rs2en_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_rdwen_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_rs1x0_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_rs2x0_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_misalgn_ena =  ifu_o_ir_pc_vld_set  ;
  wire dec_buserr_ena =  ifu_o_ir_pc_vld_set  ;
  wire dec_ilegl_ena =  ifu_o_ir_pc_vld_set  ;
  wire dec_imm_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_info_ena = ifu_o_ir_pc_vld_set  ;
  wire dec_pc_ena = ifu_o_ir_pc_vld_set  ;

  wire [`E203_RFIDX_WIDTH-1:0] dec_rs1idx_r;
  wire [`E203_RFIDX_WIDTH-1:0] dec_rs2idx_r;
  wire [`E203_RFIDX_WIDTH-1:0] dec_rdidx_r;
  wire dec_rs1en_r;
  wire dec_rs2en_r;
  wire dec_rdwen_r;
  wire dec_rs1x0_r;
  wire dec_rs2x0_r;
  wire dec_misalgn_r;
  wire dec_buserr_r;
  wire dec_ilegl_r;
  wire [`E203_XLEN-1:0] dec_imm_r; 
  wire [`E203_DECINFO_WIDTH-1:0] dec_info_r;
  wire [`E203_PC_SIZE-1:0] dec_pc_r;

  
  sirv_gnrl_dfflr #(`E203_RFIDX_WIDTH) dec_rs1idx_dfflr (dec_rs1idx_ena, dec_rs1idx_nxt, dec_rs1idx_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_RFIDX_WIDTH) dec_rs2idx_dfflr (dec_rs2idx_ena, dec_rs2idx_nxt, dec_rs2idx_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_RFIDX_WIDTH) dec_rdidx_dfflr (dec_rdidx_ena, dec_rdidx_nxt, dec_rdidx_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_rs1en_dfflr (dec_rs1en_ena, dec_rs1en_nxt, dec_rs1en_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_rs2en_dfflr (dec_rs2en_ena, dec_rs2en_nxt, dec_rs2en_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_rdwen_dfflr (dec_rdwen_ena, dec_rdwen_nxt, dec_rdwen_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_rs1x0_dfflr (dec_rs1x0_ena, dec_rs1x0_nxt, dec_rs1x0_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_rs2x0_dfflr (dec_rs2x0_ena, dec_rs2x0_nxt, dec_rs2x0_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_misalgn_dfflr (dec_misalgn_ena, dec_misalgn_nxt, dec_misalgn_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_buserr_dfflr (dec_buserr_ena, dec_buserr_nxt, dec_buserr_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_ilegl_dfflr (dec_ilegl_ena, dec_ilegl_nxt, dec_ilegl_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_XLEN) dec_imm_dfflr (dec_imm_ena, dec_imm_nxt, dec_imm_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_DECINFO_WIDTH) dec_info_dfflr (dec_info_ena, dec_info_nxt, dec_info_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_PC_SIZE) dec_pc_dfflr (dec_pc_ena, dec_pc_nxt,  dec_pc_r, clk, rst_n);

 
  assign dec_rs1idx = dec_rs1idx_r;
  assign dec_rs2idx = dec_rs2idx_r;
  assign dec_rdidx =  dec_rdidx_r;
  ....
  assign dec_rs1en =  dec_rs1en_r; 
  assign dec_rs2en =  dec_rs2en_r; 
  assign dec_rdwen =  dec_rdwen_r; 
  assign dec_rs1x0 =  dec_rs1x0_r;
  assign dec_rs2x0 =  dec_rs2x0_r; 
  assign dec_misalgn = dec_misalgn_r; 
  assign dec_buserr = dec_buserr_r; 
  assign dec_ilegl = dec_ilegl_r; 
  assign dec_imm = dec_imm_r; 
  assign dec_info = dec_info_r;
  assign dec_pc  = dec_pc_r;

  //////////////////////////////////////////////////////////////
  // Instantiate the Regfile
  wire [`E203_XLEN-1:0] rf_rs1_nxt;
  wire [`E203_XLEN-1:0] rf_rs2_nxt;
  wire [`E203_XLEN-1:0] rf2ifu_rs1_nxt;

  e203_dec_regfile u_e203_dec_regfile(
    .read_src1_idx (dec_rs1idx ),
    .read_src2_idx (dec_rs2idx ),
    .read_src1_dat (rf_rs1_nxt),
    .read_src2_dat (rf_rs2_nxt),

    .i_rs1idx      (i_rs1idx ),
    .rf2ifu_rs1    (rf2ifu_rs1_nxt ),

    .x1_r          (rf2ifu_x1),
                    
    .wbck_dest_wen (rf_wbck_ena),
    .wbck_dest_idx (rf_wbck_rdidx),
    .wbck_dest_dat (rf_wbck_wdat),
    
    .test_mode     (test_mode),
    .clk           (clk_exu  ),
    .rst_n         (rst_n    ) 
  );

  wire flag_rf_rs1 = (dec_rs1idx == rf_wbck_rdidx ) & rf_wbck_ena ;
  wire flag_rf_rs11 = (dec_rs1idx == rf_wbck_o_rdidx_pre ) & rf_wbck_o_ena_pre ; 
  assign rf_rs1 = flag_rf_rs11 ? rf_wbck_o_wdat_pre : ( flag_rf_rs1 ? rf_wbck_wdat : rf_rs1_nxt ); 

  wire flag_rf_rs2 = (dec_rs2idx == rf_wbck_rdidx ) & rf_wbck_ena ;
  wire flag_rf_rs21 = (dec_rs2idx == rf_wbck_o_rdidx_pre ) & rf_wbck_o_ena_pre ; 
  assign rf_rs2 = flag_rf_rs21 ? rf_wbck_o_wdat_pre :  (flag_rf_rs2 ?  rf_wbck_wdat : rf_rs2_nxt ) ;

  wire flag_1idx_ifu1 = (i_rs1idx == rf_wbck_rdidx ) & rf_wbck_ena ;
  wire flag_1idx_ifu = (i_rs1idx == rf_wbck_o_rdidx_pre ) & rf_wbck_o_ena_pre ; 
  wire flag_1idx_ifu2 = (i_rs1idx == alu_wbck_o_rdidx_pre ) & alu_wbck_o_valid_pre ; 
  assign rf2ifu_rs1 =  flag_1idx_ifu2 ? alu_wbck_o_wdat_pre : ( flag_1idx_ifu ? rf_wbck_o_wdat_pre : (flag_1idx_ifu1 ? rf_wbck_wdat : rf2ifu_rs1_nxt) );   
 
 
  wire i_valid_ena  =  dec_o_ready; 
  wire i_pc_vld_ena = ifu_o_ir_pc_vld_set  ; 
  wire i_ir_ena  = ifu_o_ir_pc_vld_set  ;
  wire i_pc_ena  = ifu_o_ir_pc_vld_set  ;

  wire i_valid_r;
  wire i_pc_vld_r;
  wire [`E203_INSTR_SIZE-1:0] i_ir_r; 
  wire [`E203_PC_SIZE-1:0] i_pc_r;

  wire i_valid_nxt = i_valid; 
  wire i_pc_vld_nxt = i_pc_vld;
  wire [`E203_INSTR_SIZE-1:0] i_ir_nxt = i_ir;
  wire [`E203_PC_SIZE-1:0] i_pc_nxt = i_pc;

  sirv_gnrl_dfflr #(1) dec_valid_dfflr (i_valid_ena, i_valid_nxt, i_valid_r, clk, rst_n);
  sirv_gnrl_dfflr #(1) dec_pc_vld_dfflr (i_pc_vld_ena, i_pc_vld_nxt, i_pc_vld_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_INSTR_SIZE) i_ir_dfflr (i_ir_ena, i_ir_nxt, i_ir_r, clk, rst_n); 
  sirv_gnrl_dfflr #(`E203_PC_SIZE) i_pc_dfflr (i_pc_ena, i_pc_nxt,  i_pc_r, clk, rst_n);

  assign dec_o_valid  = i_valid_r;
  assign dec_o_pc_vld = i_pc_vld_r;
  assign dec_o_ir  = i_ir_r;
  assign dec_o_pc  = i_pc_r; 

  assign i_ready = dec_o_ready & i_valid ;
  
  assign dec_active =  dec_o_ready; 

endmodule                                      
                                               
