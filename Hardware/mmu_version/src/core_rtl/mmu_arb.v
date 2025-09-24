`timescale 1 ns / 1 ps
// =============================================================================
//  Program : mmu.v
//  Author  : 
//  Date    : 
// -----------------------------------------------------------------------------
//  Description:
//  
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
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

module mmu_arb #(
) (
    // System input
    input           clk_i,
    input           rst_i,
    input           enable_i,

    // Stalling
    input           mem_stage_stalling_i,

    // Flush
    input           flush_i,

    // From processor to D-Cache
    input           d_req_valid_i,
    input           d_vipt_i,
    input  [63: 0]  d_req_paddr_i,
    input  [63: 0]  d_req_data_i,
    input           d_req_rw_i,
    input  [7 : 0]  d_req_byte_enable_i,

    output          d_rtrn_valid_o,
    output [63: 0]  d_rtrn_data_o,

    // From PTW to D-Cache
    input           ptw_req_valid_i,
    input  [63: 0]  ptw_req_paddr_i,
    input  [63: 0]  ptw_req_data_i,
    input           ptw_req_rw_i,
    input  [7 : 0]  ptw_req_byte_enable_i,

    output          ptw_rtrn_valid_o,
    output [63: 0]  ptw_rtrn_data_o,

    // To D-Cache
    output          d_req_valid_o,
    // d_strobe signal isn't used
    output [63: 0]  d_req_paddr_o,
    output [63: 0]  d_req_data_o,
    output          d_req_rw_o,
    output [7 : 0]  d_req_byte_enable_o,

    input           d_rtrn_valid_i,
    input  [63: 0]  d_rtrn_data_i
);

//==========================================================
// FSM
localparam [1 :0] S_IDLE          = 2'd0,
                  S_LOOKUP_PTW    = 2'd1,
                  S_LOOKUP_DCACHE = 2'd2;
// "Update" state isn't used in this circuit

reg  [1 : 0] S, S_nxt;

wire vipt_hit = d_rtrn_valid_i & d_vipt_i;

// -------------------------------------
// State registers
always @(posedge clk_i) begin
    if (rst_i) begin
        S <= S_IDLE;
    end
    else begin
        S <= S_nxt;
    end
end

// -------------------------------------
// Next state logic
always @(*) begin
    case (S)
        S_IDLE:
            S_nxt = (vipt_hit) ? S_IDLE : 
                    (ptw_req_valid_i) ? S_LOOKUP_PTW : 
                    (d_req_valid_i) ? S_LOOKUP_DCACHE :
                    S_IDLE;
        S_LOOKUP_PTW:
            S_nxt = (d_rtrn_valid_i) ? S_IDLE : S_LOOKUP_PTW;
        S_LOOKUP_DCACHE:
            S_nxt = (d_rtrn_valid_i) ? S_IDLE : S_LOOKUP_DCACHE;
        default:
            S_nxt = S_IDLE;
    endcase
end

//==========================================================
// User logic

// -------------------------------------
// Returning data when pipeline stalling
// -------------------------------------
reg         d_rtrn_valid_r;
reg [63: 0] d_rtrn_data_r;
wire        d_rtrn_valid_for_dcache = (S == S_LOOKUP_DCACHE & S_nxt == S_IDLE) | vipt_hit;

always @(posedge clk_i) begin
    if (rst_i || (d_rtrn_valid_o && !mem_stage_stalling_i)) begin
        d_rtrn_valid_r <= 1'b0;
    end
    else if (d_rtrn_valid_o && mem_stage_stalling_i) begin
        d_rtrn_valid_r <= 1'b1;
    end
end

always @(posedge clk_i) begin
    if (rst_i) begin
        d_rtrn_data_r <= 63'b0;
    end
    else if (d_rtrn_valid_for_dcache && mem_stage_stalling_i) begin
        d_rtrn_data_r <= d_rtrn_data_i;
    end
end

//==========================================================
// Output signals interface
assign d_req_valid_o = (S == S_IDLE ) & (ptw_req_valid_i | d_req_valid_i);
assign d_req_paddr_o = ((S == S_IDLE && ptw_req_valid_i) || S == S_LOOKUP_PTW) ? ptw_req_paddr_i : d_req_paddr_i;
assign d_req_data_o  = ((S == S_IDLE && ptw_req_valid_i) || S == S_LOOKUP_PTW) ? ptw_req_data_i  : d_req_data_i;
assign d_req_rw_o    = ((S == S_IDLE && ptw_req_valid_i) || S == S_LOOKUP_PTW) ? ptw_req_rw_i    : d_req_rw_i;
assign d_req_byte_enable_o = ((S == S_IDLE && ptw_req_valid_i) || S == S_LOOKUP_PTW) ? ptw_req_byte_enable_i : d_req_byte_enable_i;

assign d_rtrn_valid_o = (enable_i & d_rtrn_valid_r) ? 1'b1 : d_rtrn_valid_for_dcache;
assign d_rtrn_data_o  = (enable_i & d_rtrn_valid_r) ? d_rtrn_data_r : d_rtrn_data_i;

assign ptw_rtrn_valid_o = (S == S_LOOKUP_PTW) & (S_nxt == S_IDLE);
assign ptw_rtrn_data_o  = d_rtrn_data_i;

endmodule