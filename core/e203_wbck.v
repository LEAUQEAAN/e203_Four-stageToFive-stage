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
//  The Write-Back module to arbitrate the write-back request to regfile
//
// ====================================================================

`include "e203_defines.v"

module e203_wbck(

  // input  i_valid,
  // output i_ready,

  //////////////////////////////////////////////////////////////
  // The ALU Write-Back Interface
  input  alu_wbck_i_valid, // Handshake valid
  output alu_wbck_i_ready, // Handshake ready
  input  [`E203_XLEN-1:0] alu_wbck_i_wdat,
  input  [`E203_RFIDX_WIDTH-1:0] alu_wbck_i_rdidx,
  
  //////////////////////////////////////////////////////////////
  // The Final arbitrated Write-Back Interface to Regfile
  output  rf_wbck_o_ena,
  output  [`E203_XLEN-1:0] rf_wbck_o_wdat,
  output  [`E203_RFIDX_WIDTH-1:0] rf_wbck_o_rdidx,

  //////////////////////////////////////////////////////////////
  // The Pre-Data rf_wbck_o_wdat for Decode
  output  rf_wbck_o_ena_pre,
  output  [`E203_XLEN-1:0] rf_wbck_o_wdat_pre,
  output  [`E203_RFIDX_WIDTH-1:0] rf_wbck_o_rdidx_pre,

  //////////////////////////////////////////////////////////////
  // The  LSU to LongWbck
  input  lsu_o_valid, // Handshake valid
  output lsu_o_ready, // Handshake ready
  input  [`E203_XLEN-1:0] lsu_o_wbck_wdat,
  input  [`E203_ITAG_WIDTH -1:0] lsu_o_wbck_itag,
  input  lsu_o_wbck_err , 
  input  lsu_o_cmt_ld,
  input  lsu_o_cmt_st,
  input  [`E203_ADDR_SIZE -1:0] lsu_o_cmt_badaddr,
  input  lsu_o_cmt_buserr , // The bus-error exception generated

  output  longp_excp_o_valid,
  input   longp_excp_o_ready,
  output  longp_excp_o_insterr,
  output  longp_excp_o_ld,
  output  longp_excp_o_st,
  output  longp_excp_o_buserr , // The load/store bus-error exception generated
  output [`E203_ADDR_SIZE-1:0] longp_excp_o_badaddr,
  output [`E203_PC_SIZE -1:0] longp_excp_o_pc,

  input  oitf_empty,
  input  [`E203_ITAG_WIDTH -1:0] oitf_ret_ptr,
  input  [`E203_RFIDX_WIDTH-1:0] oitf_ret_rdidx,
  input  [`E203_PC_SIZE-1:0] oitf_ret_pc,
  input  oitf_ret_rdwen,   
  input  oitf_ret_rdfpu,   
  output oitf_ret_ena,

  output wbck_active,
  input  clk,
  input  rst_n
  ); 


  wire  longp_wbck_o_valid; // Handshake valid
  wire longp_wbck_o_ready; // Handshake ready
  wire  [`E203_FLEN-1:0] longp_wbck_o_wdat;
  wire  [5-1:0] longp_wbck_o_flags;
  wire  [`E203_RFIDX_WIDTH-1:0] longp_wbck_o_rdidx;
  wire  longp_wbck_o_rdfpu;


  wire  rf_wbck_o_ena_nxt ;
  wire  [`E203_XLEN-1:0] rf_wbck_o_wdat_nxt ;
  wire  [`E203_RFIDX_WIDTH-1:0] rf_wbck_o_rdidx_nxt ;

  e203_wbck_wbck u_e203_wbck_wbck(
    .alu_wbck_i_valid   (alu_wbck_i_valid ), 
    .alu_wbck_i_ready   (alu_wbck_i_ready ),
    .alu_wbck_i_wdat    (alu_wbck_i_wdat  ),
    .alu_wbck_i_rdidx   (alu_wbck_i_rdidx ),

    .longp_wbck_i_valid (longp_wbck_o_valid ), 
    .longp_wbck_i_ready (longp_wbck_o_ready ),
    .longp_wbck_i_wdat  (longp_wbck_o_wdat  ),
    .longp_wbck_i_rdidx (longp_wbck_o_rdidx ),
    .longp_wbck_i_rdfpu (longp_wbck_o_rdfpu ),
    .longp_wbck_i_flags (longp_wbck_o_flags ),

    .rf_wbck_o_ena      (rf_wbck_o_ena_nxt    ),
    .rf_wbck_o_wdat     (rf_wbck_o_wdat_nxt   ),
    .rf_wbck_o_rdidx    (rf_wbck_o_rdidx_nxt  ),

    .wbck_active         (wbck_active),
    .clk                 (clk),
    .rst_n               (rst_n)
  );


  //////////////////////////////////////////////////////////////
  // Instantiate the Long-pipe Write-Back
  e203_exu_longpwbck u_e203_exu_longpwbck(
    .lsu_wbck_i_valid   (lsu_o_valid ),
    .lsu_wbck_i_ready   (lsu_o_ready ),
    .lsu_wbck_i_wdat    (lsu_o_wbck_wdat  ),
    .lsu_wbck_i_itag    (lsu_o_wbck_itag  ),
    .lsu_wbck_i_err     (lsu_o_wbck_err   ),
    .lsu_cmt_i_ld       (lsu_o_cmt_ld     ),
    .lsu_cmt_i_st       (lsu_o_cmt_st     ),
    .lsu_cmt_i_badaddr  (lsu_o_cmt_badaddr),
    .lsu_cmt_i_buserr   (lsu_o_cmt_buserr ),

    .longp_wbck_o_valid   (longp_wbck_o_valid ), 
    .longp_wbck_o_ready   (longp_wbck_o_ready ),
    .longp_wbck_o_wdat    (longp_wbck_o_wdat  ),
    .longp_wbck_o_rdidx   (longp_wbck_o_rdidx ),
    .longp_wbck_o_rdfpu   (longp_wbck_o_rdfpu ),
    .longp_wbck_o_flags   (longp_wbck_o_flags ),

    .longp_excp_o_ready   (longp_excp_o_ready  ),
    .longp_excp_o_valid   (longp_excp_o_valid  ),
    .longp_excp_o_ld      (longp_excp_o_ld     ),
    .longp_excp_o_st      (longp_excp_o_st     ),
    .longp_excp_o_buserr  (longp_excp_o_buserr ),
    .longp_excp_o_badaddr (longp_excp_o_badaddr),
    .longp_excp_o_insterr (longp_excp_o_insterr),
    .longp_excp_o_pc      (longp_excp_o_pc),

    .oitf_ret_rdidx      (oitf_ret_rdidx),
    .oitf_ret_rdwen      (oitf_ret_rdwen),
    .oitf_ret_rdfpu      (oitf_ret_rdfpu),
    .oitf_ret_pc         (oitf_ret_pc),
    .oitf_empty          (oitf_empty    ),
    .oitf_ret_ptr        (oitf_ret_ptr  ),
    .oitf_ret_ena        (oitf_ret_ena  ),

    .clk                 (clk          ),
    .rst_n               (rst_n        ) 
  );


  wire  rf_wbck_o_ena_r; 
  wire  [`E203_XLEN-1:0] rf_wbck_o_wdat_r;
  wire  [`E203_RFIDX_WIDTH-1:0] rf_wbck_o_rdidx_r; 

  sirv_gnrl_dfflr #(1) rf_wbck_o_ena_dfflr (1'b1, rf_wbck_o_ena_nxt, rf_wbck_o_ena_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_XLEN) rf_wbck_o_wdat_dfflr (1'b1, rf_wbck_o_wdat_nxt, rf_wbck_o_wdat_r, clk, rst_n);
  sirv_gnrl_dfflr #(`E203_RFIDX_WIDTH) rf_wbck_o_rdidx_dfflr (1'b1, rf_wbck_o_rdidx_nxt, rf_wbck_o_rdidx_r, clk, rst_n);

  assign rf_wbck_o_ena  = rf_wbck_o_ena_r; 
  assign rf_wbck_o_wdat  = rf_wbck_o_wdat_r; 
  assign rf_wbck_o_rdidx = rf_wbck_o_rdidx_r;

  assign  rf_wbck_o_ena_pre = rf_wbck_o_ena_nxt;
  assign  rf_wbck_o_wdat_pre = rf_wbck_o_wdat_nxt;
  assign  rf_wbck_o_rdidx_pre = rf_wbck_o_rdidx_nxt;

endmodule                                      
                                               
                                               
                                               
