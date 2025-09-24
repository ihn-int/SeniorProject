`timescale 1 ns / 1 ps
// =============================================================================
//  Program : tlb.v
//  Author  : 
//  Date    : 
// -----------------------------------------------------------------------------
//  Description:
//  
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Sep/24/2025, by Jun-Kai Chen
//    Add a new signal `work_for_dtlb' to fix a bug. This bug happens when fetch
//    page fault and data memory access occur simultaneously. The old circuit
//    mistakenly treating memory access as exception. Also modify the circuit to
//    support SV39 format.
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

module ptw (
    // System input
    input           clk_i,
    input           rst_i,

    // Stalling signals
    input           stall_if_i,
    input           stall_mem_i,

    // Flush siganl isn't used

    // ITLB Update
    input           i_req_valid_i,
    input           i_tlb_miss_i,
    input  [63: 0]  i_vaddr_i,

    output          i_req_ready_o,
    output [63: 0]  i_pte_o,
    output [1 : 0]  i_pte_size_o,

    // DTLB Update
    input           d_req_valid_i,
    input           d_tlb_miss_i,
    input  [63: 0]  d_vaddr_i,

    output          d_req_ready_o,
    output [63: 0]  d_pte_o,
    output [1 : 0]  d_pte_size_o,

    // To D-Cache (m is for memory)
    output          m_req_valid_o,
    output          m_req_rw_o,
    output [7 : 0]  m_byte_enable_o,
    output [63: 0]  m_req_addr_o,

    // From D-Cache
    input           m_req_ready_i,
    input  [63: 0]  m_rtrn_data_i,

    // From CSR
    input  [43: 0]  root_ppn_i,
    input  [15: 0]  asid_i,
    input  [1 : 0]  privilege_level_i,

    // Exception (Page Fault)
    output          i_xcpt_valid_o,
    output          d_xcpt_valid_o
);

//==========================================================
// FSM
// local parameter
localparam [1 : 0] S_IDLE   = 2'd0,
                   S_LOOKUP = 2'd1,
                   S_UPDATE = 2'd2;

// FSM signals
reg  [1 : 0] S, S_nxt;
reg          work_for_itlb_r;
reg          work_for_dtlb_r;
reg  [55: 0] req_addr_r;
reg  [1 : 0] pte_level_r;
reg          d_req_reserved; // DTLB req pending

// PTE field from memory
wire [8 : 0] i_vaddr_i_vpn_2, i_vaddr_i_vpn_1, i_vaddr_i_vpn_0;
wire [8 : 0] d_vaddr_i_vpn_2, d_vaddr_i_vpn_1, d_vaddr_i_vpn_0;

wire [25: 0] rtrn_data_ppn_2;
wire [8 : 0] rtrn_data_ppn_1;
wire [8 : 0] rtrn_data_ppn_0;
wire [1 : 0] rtrn_data_rsw;
wire         rtrn_data_D;
wire         rtrn_data_A;
wire         rtrn_data_G;
wire         rtrn_data_U;
wire         rtrn_data_X;
wire         rtrn_data_W;
wire         rtrn_data_R;
wire         rtrn_data_V;

reg  [63: 0] rtrn_data_r;
reg  [1 : 0] rtrn_pte_size_r;

reg          xcpt_valid_r;

// PTE vpn fields
assign i_vaddr_i_vpn_2 = i_vaddr_i[38:30];
assign i_vaddr_i_vpn_1 = i_vaddr_i[29:21];
assign i_vaddr_i_vpn_0 = i_vaddr_i[20:12];
assign d_vaddr_i_vpn_2 = d_vaddr_i[38:30];
assign d_vaddr_i_vpn_1 = d_vaddr_i[29:21];
assign d_vaddr_i_vpn_0 = d_vaddr_i[20:12];

// Return data field
// +---------------------------------------------------------------------------------------+
// |  N | PBMT | Reserved | PPN[2] | PPN[1] | PPN[0] | RSW | D | A | G | U | X | W | R | V |
// +---------------------------------------------------------------------------------------+
// | 63 |62  61|60      54|53    28|27    19|18    10|9   8| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 
// +---------------------------------------------------------------------------------------+
assign rtrn_data_ppn_2 = m_rtrn_data_i[53:28];
assign rtrn_data_ppn_1 = m_rtrn_data_i[27:19];
assign rtrn_data_ppn_0 = m_rtrn_data_i[18:10];
assign rtrn_data_rsw   = m_rtrn_data_i[9 : 8];
assign rtrn_data_D     = m_rtrn_data_i[7];
assign rtrn_data_A     = m_rtrn_data_i[6];
assign rtrn_data_G     = m_rtrn_data_i[5];
assign rtrn_data_U     = m_rtrn_data_i[4];
assign rtrn_data_X     = m_rtrn_data_i[3];
assign rtrn_data_W     = m_rtrn_data_i[2];
assign rtrn_data_R     = m_rtrn_data_i[1];
assign rtrn_data_V     = m_rtrn_data_i[0];

// Data request pending
always @(posedge clk_i) begin
    if (rst_i || (S == S_IDLE && d_req_reserved)) begin
        d_req_reserved <= 1'b0;
    end
    else if ((i_tlb_miss_i && i_req_valid_i) && (d_tlb_miss_i && d_req_valid_i)) begin
        d_req_reserved <= 1'b1;
    end
    else begin
        d_req_reserved <= d_req_reserved;
    end
end

// FSM State registers
always @(posedge clk_i) begin
    if (rst_i) begin
        S <= S_IDLE;
    end
    else begin
        S <= S_nxt;
    end
end

// FSM Next state logic
always @(*) begin
    case (S)
        S_IDLE: begin
            if ((i_tlb_miss_i && i_req_valid_i) || // ITLB miss
                (d_tlb_miss_i && d_req_valid_i) || // DTLB miss
                (d_req_reserved)) begin            // DTLB request pending
                S_nxt = S_LOOKUP;
            end
            else begin
                S_nxt = S_IDLE;
            end
        end
        S_LOOKUP: begin
            if (m_req_ready_i) begin    // request ready
                if (!rtrn_data_V ||
                    (rtrn_data_W && !rtrn_data_R)) begin
                    // A write-only page is illegal -> Page fault
                    S_nxt = S_IDLE;
                end
                else begin
                    if (rtrn_data_X || rtrn_data_R) begin
                        // Leaf page
                        S_nxt = S_UPDATE;
                    end
                    else begin
                        if (pte_level_r != 2'b00) begin // internal page
                            S_nxt = S_LOOKUP;
                        end
                        else begin //  A 4 KiB page with XRW == 3'b000
                            S_nxt = S_IDLE;
                        end
                    end
                end
            end
            else if (xcpt_valid_r) begin
                S_nxt = S_IDLE;
            end
            else begin
                S_nxt = S_LOOKUP;
            end
        end
        S_UPDATE: begin
            S_nxt = S_IDLE;
        end
        default: begin
            S_nxt = S_IDLE;
        end
    endcase
end

//==========================================================
// PTE Look up
// work_for_itlb_r
always @(posedge clk_i) begin
    if (S == S_IDLE) begin
        if (i_tlb_miss_i) begin
            work_for_itlb_r <= 1'b1;
        end
        else begin
            work_for_itlb_r <= 1'b0;
        end
    end
end

always @(posedge clk_i) begin
    if (S == S_IDLE) begin
        if (d_tlb_miss_i | d_req_reserved) begin
            work_for_dtlb_r <= 1'b1;
        end
        else begin
            work_for_dtlb_r <= 1'b0;
        end
    end
end

//==========================================================
// PTE Update
// ----------------------------------------------------
// | X | W | R | Meaning                              |
// |---|---|---|--------------------------------------|
// | 0 | 0 | 0 | Pointer to next level of page table. |
// | 0 | 0 | 1 | Read-only page.                      |
// | 0 | 1 | 0 | Reserved for future use.             |
// | 0 | 1 | 1 | Read-write page.                     |
// | 1 | 0 | 0 | Execute-only page.                   |
// | 1 | 0 | 1 | Read-execute page.                   |
// | 1 | 1 | 0 | Reserved for future use.             |
// | 1 | 1 | 1 | Read-write-execute page.             |
// ----------------------------------------------------

// req_addr_r
always @(posedge clk_i) begin
    if (S == S_IDLE) begin
        // a = sapt.ppn << 12 + va.vpn[2] * 8
        if (i_tlb_miss_i) begin // ITLB miss
            req_addr_r <= { root_ppn_i, i_vaddr_i_vpn_2, 3'b000 };
        end
        else begin  // DTLB miss or nothing
            req_addr_r <= { root_ppn_i, d_vaddr_i_vpn_2, 3'b000 };
        end
    end
    else if (S == S_LOOKUP) begin
        if (m_req_ready_i && rtrn_data_V &&         // return a valid page
            !(rtrn_data_W && !rtrn_data_R) &&       // not a reserved page
            !(rtrn_data_X || rtrn_data_R)) begin    // not a leaf page
            // continue look up
            case (pte_level_r)
                2'b10: begin    // level 2
                    // a = pte.ppn << 12 + va.vpn[1] * 8
                    req_addr_r <= (work_for_itlb_r) ? { m_rtrn_data_i[53:10],  i_vaddr_i_vpn_1, 3'b000 } : 
                                  (work_for_dtlb_r) ? { m_rtrn_data_i[53:10],  d_vaddr_i_vpn_1, 3'b000 } :
                                                      64'b0;
                end
                2'b01: begin    // level 1
                    // a = pte.ppn << 12 + va.vpn[0] * 8
                    req_addr_r <= (work_for_itlb_r) ? { m_rtrn_data_i[53:10],  i_vaddr_i_vpn_0, 3'b000 } : 
                                  (work_for_dtlb_r) ? { m_rtrn_data_i[53:10],  d_vaddr_i_vpn_0, 3'b000 } : 
                                                      64'b0;
                end
                default: begin  // level 0 or nothing -> page fault
                    req_addr_r <= req_addr_r;
                end
            endcase
        end
        else begin
            req_addr_r <= req_addr_r;
        end
    end
    else begin
        req_addr_r <= req_addr_r;
    end
end

// rtrn_pte_size_r
always @(posedge clk_i) begin
    if (rst_i || S_nxt == S_IDLE) begin
        rtrn_pte_size_r <= 2'b00;
    end
    else if (m_req_ready_i && rtrn_data_V &&        // return a valid page
             !(rtrn_data_W && !rtrn_data_R) &&      // not a reserved page
             (rtrn_data_X || rtrn_data_R)) begin    // a leaf page
        // Record the current level
        rtrn_pte_size_r <= pte_level_r;
    end
    else begin
        rtrn_pte_size_r <= rtrn_pte_size_r;
    end
end

// return data
always @(posedge clk_i) begin
    rtrn_data_r <= m_rtrn_data_i;
end

// pte_level_r
always @(posedge clk_i) begin
    if (rst_i || S_nxt == S_IDLE) begin
        pte_level_r <= 2'b10;
    end
    else if (S == S_LOOKUP && m_req_ready_i) begin
        pte_level_r <= { 1'b0, pte_level_r[1] };    // shift register
    end
    else begin
        pte_level_r <= pte_level_r;
    end
end

// xcpt_valid_r
always @(posedge clk_i) begin
    if (rst_i) begin
        xcpt_valid_r <= 1'b0;
    end
    else if (S == S_LOOKUP) begin
        if (m_req_ready_i) begin
            if (~rtrn_data_V ||                         // a invalid page
                (rtrn_data_W && ~rtrn_data_R)) begin    // a reserved encoding
                xcpt_valid_r <= 1'b1;
            end
            else begin
                if (!(rtrn_data_X || rtrn_data_R) &&    // not a leaf page
                    (~|pte_level_r)) begin              // level == 0
                    xcpt_valid_r <= 1'b1;
                end
                else begin
                    xcpt_valid_r <= xcpt_valid_r;
                end
            end
        end
        else begin
            xcpt_valid_r <= xcpt_valid_r;
        end
    end
    else if (xcpt_valid_r &&
            ((work_for_itlb_r && stall_if_i) ||
             (work_for_dtlb_r && stall_mem_i))) begin
        xcpt_valid_r <= 1'b1;
    end
    else begin
        xcpt_valid_r <= 1'b0;
    end
end




//==========================================================
// Output signals interface
// Update TLB
assign i_req_ready_o = (S == S_UPDATE) && work_for_itlb_r;
assign i_pte_o       = rtrn_data_r;
assign i_pte_size_o  = rtrn_pte_size_r;

assign d_req_ready_o = (S == S_UPDATE) && work_for_dtlb_r;
assign d_pte_o       = rtrn_data_r;
assign d_pte_size_o  = rtrn_pte_size_r;

// MMU request
assign m_req_valid_o    = (S == S_LOOKUP);
assign m_req_rw_o       = 1'b0;
assign m_byte_enable_o  = 8'hFF;
assign m_req_addr_o     = { 8'h00, req_addr_r[55: 0] }; // zero padding

// Exception
assign i_xcpt_valid_o = xcpt_valid_r & work_for_itlb_r;
assign d_xcpt_valid_o = xcpt_valid_r & work_for_dtlb_r;


endmodule