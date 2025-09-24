`timescale 1ns / 1ps
// =============================================================================
//  Program : memory_write_back.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Writeback Pipeline Stage of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Aug/26/2025, by Jun-Kai Chen
//    Add support for 64-bit data.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
`include "aquila_config.vh"

module writeback #(
    parameter XLEN = 64
)(
    // System signals
    input               clk_i,
    input               rst_i,

    // Pipeline stall signal.
    input               stall_i,
    
    // Pipeline stall signal
    input               flush_i,

    // from Execute
    input               regfile_we_i,
    input  [4 : 0]      rd_addr_i,
    input               signex_sel_i,
    input  [XLEN-1 : 0] aligned_data_i,
    input  [XLEN-1 : 0] p_data_i,

    input               csr_we_i,
    input  [11: 0]      csr_we_addr_i,
    input  [XLEN-1 : 0] csr_we_data_i,

    // from Memory stage
    input  [2 : 0]      regfile_input_sel_i,

    // to Register File
    output              rd_we_o,
    output [4 : 0]      rd_addr_o,
    output [XLEN-1 : 0] rd_data_o,

    // PC of the current instruction
    input  [XLEN-1 : 0] pc_i,
    output [XLEN-1 : 0] pc_o,

    // System Jump operation
    input               sys_jump_i,
    input  [1 : 0]      sys_jump_csr_addr_i,
    output              sys_jump_o,
    output [1 : 0]      sys_jump_csr_addr_o,

    // Has the instruction fetch being successful
    input               fetch_valid_i,
    output              fetch_valid_o,

    // to CSR
    output              csr_we_o,
    output [11: 0]      csr_we_addr_o,
    output [XLEN-1 : 0] csr_we_data_o,

    // Exception info passed from Memory to CSR
    input               xcpt_valid_i,
    input  [3 : 0]      xcpt_cause_i,
    input  [XLEN-1 : 0] xcpt_tval_i,
    output              xcpt_valid_o,
    output [3 : 0]      xcpt_cause_o,
    output [XLEN-1 : 0] xcpt_tval_o
);

reg [XLEN-1 : 0]        rd_data;

reg                     sys_jump_r;
reg [ 1 : 0]            sys_jump_csr_addr_r;

reg                     xcpt_valid_r;
reg [ 3 : 0]            xcpt_cause_r;
reg [XLEN-1 : 0]        xcpt_tval_r;

// These are MEM/WB pipeline registers
always @(posedge clk_i)
begin
    if (rst_i || (flush_i && !stall_i)) // stall has higher priority than flush.
    begin
        sys_jump_r <= 0;
        sys_jump_csr_addr_r <= 0;
        xcpt_valid_r <= 0;
        xcpt_cause_r <= 0;
        xcpt_tval_r <= 0;
    end
    else if (stall_i)
    begin
        sys_jump_r <= sys_jump_r;
        sys_jump_csr_addr_r <= sys_jump_csr_addr_r;
        xcpt_valid_r <= xcpt_valid_r;
        xcpt_cause_r <= xcpt_cause_r;
        xcpt_tval_r <= xcpt_tval_r;
    end
    else if (xcpt_valid_i)
    begin
        sys_jump_r <= 0;
        sys_jump_csr_addr_r <= 0;
        xcpt_valid_r <= xcpt_valid_i;
        xcpt_cause_r <= xcpt_cause_i;
        xcpt_tval_r <= xcpt_tval_i;
    end
    else
    begin
        sys_jump_r <= sys_jump_i;
        sys_jump_csr_addr_r <= sys_jump_csr_addr_i;
        xcpt_valid_r <= xcpt_valid_i;
        xcpt_cause_r <= xcpt_cause_i;
        xcpt_tval_r <= xcpt_tval_i;
    end
end



// Register file input selection
always @(*)
begin
    case (regfile_input_sel_i)
        3'b000: // load byte
            rd_data = signex_sel_i ? {56'b0, aligned_data_i[7 : 0]} :
                                {{56{aligned_data_i[7]}}, aligned_data_i[7 : 0]};
        3'b001: // load half-word
            rd_data = signex_sel_i ? {48'b0, aligned_data_i[15: 0]} :
                                {{48{aligned_data_i[15]}}, aligned_data_i[15: 0]};
        3'b010: // load word
            rd_data = signex_sel_i ? {32'b0, aligned_data_i[31: 0]} :
                                {{32{aligned_data_i[31]}}, aligned_data_i[31: 0]};
        3'b011: // load double word
            rd_data = aligned_data_i;
        default: // pc+4, csr, execute result are from the processor
            rd_data = p_data_i;
    endcase
end

assign pc_o = pc_i;
assign fetch_valid_o = fetch_valid_i;

assign rd_we_o = regfile_we_i;
assign rd_addr_o = rd_addr_i;
assign rd_data_o = rd_data;

assign sys_jump_o = sys_jump_r;
assign sys_jump_csr_addr_o = sys_jump_csr_addr_r;
assign csr_we_o = csr_we_i;
assign csr_we_addr_o = csr_we_addr_i;
assign csr_we_data_o = csr_we_data_i;
assign xcpt_valid_o = xcpt_valid_r;
assign xcpt_cause_o = xcpt_cause_r;
assign xcpt_tval_o = xcpt_tval_r;

endmodule