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
//  virtually indexed, physically tagged (VIPT) 
//
//  Virtual address for RISC-V:
//  -----------------------------------------------------------------------------------------------------------------------
//  |              VPN[1]                |              VPN[2]                |               page offset                 |
//  -----------------------------------------------------------------------------------------------------------------------
//  |                10                  |                10                  |                    12                     |
//  -----------------------------------------------------------------------------------------------------------------------
//
//  Dcache for 4KB 4-way-associative
//  -----------------------------------------------------------------------------------------------------------------------
//  |                                            TAG                                        |      Index     |   offset   |
//  -----------------------------------------------------------------------------------------------------------------------
//  |                                             24                                        |        5       |      3     |
//  -----------------------------------------------------------------------------------------------------------------------
//
//  For VIPT, index for dcache need to be in page offset.
//  If index for dcache is out of page offset, you can increase dacahe associativity to make index back into page offset.
//
//  Sep/24/2025, by Jun-Kai Chen
//    Modify the circuit to support SV39 virtual Memory. Also add a buffer at
//    output to prevent a bug which happens when fetch page fault and load use
//    hazard occur simultaneously.
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

module mmu #(
    parameter ITLB_ENTRIES = 4,
    parameter DTLB_ENTRIES = 4,
    parameter ASID_WIDTH   = 16
) (
    // System input
    input           clk_i,
    input           rst_i,

    input           flush_i,
    input           req_cancel_i,

    input           stall_instr_i,
    input           stall_data_i,
    input           stall_exe_i,
    input           stall_load_use_i,

    //  fence_i
    input           fence_i,

    // sfence.vma
    input           sfence_vma_valid_i,
    input           sfence_vma_type_i,
    input  [63: 0]  sfence_vma_vaddr_i,
    input  [63: 0]  sfence_vma_asid_i,

    // From CSR
    input           immu_enable_i,
    input           dmmu_enable_i,
    input  [43: 0]  root_ppn_i,
    input  [15: 0]  asid_i,
    input  [1 : 0]  privilege_level_i,
    input  [1 : 0]  ld_st_privilege_level_i,
    input           mxr_i,
    input           sum_i,

    // ---------------------------------
    // instruction -> mmu
    // ---------------------------------
    input           i_req_valid_i,
    input  [63: 0]  i_req_vaddr_i,
    input           i_req_branch_hit_i,
    input           i_req_branch_decision_i,

    // ---------------------------------
    // mmu -> I$
    // ---------------------------------
    output          i_req_valid_o,
    output [63: 0]  i_req_paddr_o,

    // ---------------------------------
    // data -> mmu
    // ---------------------------------
    input           d_req_valid_i,
    input  [63: 0]  d_req_vaddr_i,
    input  [63: 0]  d_req_data_i,
    input           d_req_rw_i,
    input  [7 : 0]  d_req_byte_enable_i,
    input           d_is_amo_i,

    // ---------------------------------
    // mmu -> D$
    // ---------------------------------
    output          d_req_valid_o,
    output          d_vipt_o,
    output          d_vipt_error_o,
    output [63: 0]  d_req_paddr_o,
    output [63: 0]  d_req_data_o,
    output          d_req_rw_o,
    output [7 : 0]  d_req_byte_enable_o,

    // ---------------------------------
    // I$ -> mmu
    // ---------------------------------
    input           i_rtrn_valid_i,
    input  [63: 0]  i_rtrn_data_i,

    // ---------------------------------
    // mmu -> instruction
    // ---------------------------------
    output          i_rtrn_valid_o,
    output [63: 0]  i_rtrn_data_o,
    output [63: 0]  i_rtrn_vaddr_o,
    output          i_rtrn_branch_hit_o,
    output          i_rtrn_branch_decision_o,
    output          i_rtrn_ignore_o,

    // ---------------------------------
    // D$ -> mmu
    // ---------------------------------
    input           d_rtrn_valid_i,
    input  [63: 0]  d_rtrn_data_i,

    // ---------------------------------
    // mmu -> data
    // ---------------------------------
    output          d_rtrn_valid_o,
    output [63: 0]  d_rtrn_data_o,

    // ---------------------------------
    // Exception
    // ---------------------------------
    output          i_xcpt_valid_o,
    output [3 : 0]  i_xcpt_cause_o,
    output [63: 0]  i_xcpt_tval_o,

    output          d_xcpt_valid_o,
    output [3 : 0]  d_xcpt_cause_o,
    output [63: 0]  d_xcpt_tval_o,

    output          page_lookup_o
);

//==========================================================
// Wire and Registers

// -------------------------------------
// ITLB
// -------------------------------------
reg             itlb_req_valid_r;
reg  [63: 0]    itlb_paddr_r;
reg  [63: 0]    itlb_vaddr_r;
reg             itlb_branch_hit_r;
reg             itlb_branch_decision_r;

reg             itlb_pte_U_r;
reg             itlb_pte_R_r;
reg             itlb_pte_X_r;
reg             itlb_pte_A_r;
reg             itlb_ppn_misaligned;

reg             itlb_empty;
reg             i_pmp_xcpt_valid;
reg  [3 : 0]    i_xcpt_cause;
reg  [63: 0]    i_xcpt_tval;
wire            itlb_hit;
wire [63: 0]    itlb_paddr;
wire [63: 0]    itlb_pte;

// -------------------------------------
// DTLB
// -------------------------------------
reg             dtlb_req_valid_r;
reg  [63: 0]    dtlb_paddr_r;
reg             dtlb_pte_U_r;
reg             dtlb_pte_R_r;
reg             dtlb_pte_W_r;
reg             dtlb_pte_D_r;
reg             dtlb_pte_X_r;
reg             dtlb_pte_A_r;
reg             dtlb_ppn_misaligned;

reg             d_pmp_xcpt_valid;
reg  [3 : 0]    d_xcpt_cause;
reg  [63: 0]    d_xcpt_tval;
wire            dtlb_hit;
wire [63: 0]    dtlb_paddr;
wire [63: 0]    dtlb_pte;

// -------------------------------------
// ITLB <-> PTW
// -------------------------------------
wire            i_req_reserved_restart;
wire            itlb_miss_valid = immu_enable_i & ~itlb_hit & 
                    (i_req_valid_i | i_req_reserved_restart);
//itlb miss when 1.immu enabled 2.itlb miss 3.instr request from pipeline or reserved
wire            ptw2itlb_update_valid;
wire [63: 0]    ptw2itlb_update_pte;
wire [1 : 0]    ptw2itlb_update_pte_size;

// -------------------------------------
// DTLB <-> PTW
// -------------------------------------
wire            dtlb_miss_valid = dmmu_enable_i & ~dtlb_hit & d_req_valid_i;

wire            ptw2dtlb_update_valid;
wire [63: 0]    ptw2dtlb_update_pte;
wire [1 : 0]    ptw2dtlb_update_pte_size;

// -------------------------------------
// DTLB <-> MMU Arbiter
// -------------------------------------
wire            dtlb2arb_req_valid;
wire [63: 0]    dtlb2arb_req_paddr;
wire [63: 0]    dtlb2arb_req_data;
wire            dtlb2arb_req_rw;
wire [7 : 0]    dtlb2arb_req_byte_enable;

// -------------------------------------
// PTW <-> MMU Arbiter
// -------------------------------------
wire            ptw2arb_req_valid;
wire            ptw2arb_req_rw;
wire [7 : 0]    ptw2arb_req_byte_enable;
wire [63: 0]    ptw2arb_req_addr;

wire            arb2ptw_rtrn_valid;
wire [63: 0]    arb2ptw_rtrn_data;

//==========================================================
// User Logic
assign page_lookup_o = ptw2arb_req_valid;

// -------------------------------------
// Instruction
// -------------------------------------

// -------------------------------------
// Delay registers
reg          update_itlb_done;
reg          i_rtrn_valid_delay;
reg  [1 : 0] itlb_pte_size;

// fetch  -> stall_pipeline || stall_data_hazard
// memory -> stall_pipeline
// stall_pipeline = stall_instr_fetch | stall_data_fetch | stall_from_exe

wire fetch_stage_stalling = stall_data_i  | stall_exe_i | stall_load_use_i;
wire mem_stage_stalling   = stall_instr_i | stall_exe_i;

always @(posedge clk_i) begin
    if (rst_i) begin
        update_itlb_done    <= 1'b0;
        i_rtrn_valid_delay  <= 1'b0;
        itlb_pte_size       <= 2'b00;
    end
    else begin
        update_itlb_done    <= ptw2itlb_update_valid;
        i_rtrn_valid_delay  <= i_rtrn_valid_i;
        itlb_pte_size       <= ptw2itlb_update_pte_size;
    end
end

// -------------------------------------
// Exception delay circuit
reg  i_ptw_xcpt_valid_r;    // instruction ptw exception delay register
wire i_ptw_xcpt_pipe_full  = i_ptw_xcpt_valid & ~itlb_empty;
wire i_ptw_xcpt_pipe_empty = i_ptw_xcpt_valid & itlb_empty;

always @(posedge clk_i) begin
    if (rst_i || !i_ptw_xcpt_valid && itlb_empty) begin
        i_ptw_xcpt_valid_r <= 1'b0;
    end
    else if (i_ptw_xcpt_pipe_full) begin
        i_ptw_xcpt_valid_r <= 1'b1;
    end
    else begin
        i_ptw_xcpt_valid_r <= i_ptw_xcpt_valid_r;
    end
end

// -------------------------------------
// Instruction request reserve circuit
reg  instr_fetching_r;

always @(posedge clk_i) begin
    if (rst_i || i_rtrn_valid_o) begin
        instr_fetching_r <= 1'b0;
    end
    else if (i_req_valid_o && !i_rtrn_valid_i) begin
        instr_fetching_r <= 1'b1;
    end
    else begin
        instr_fetching_r <= instr_fetching_r;
    end
end

reg  instr_req_reserved;
wire instr_stall_start = i_rtrn_valid_delay & ~i_rtrn_valid_i;
wire instr_stall_end   = ~i_rtrn_valid_delay & i_rtrn_valid_i;
wire instr_fetching    = (i_req_valid_o | instr_fetching_r) & ~i_rtrn_valid_i;

always @(posedge clk_i) begin
    if (rst_i || instr_stall_end || i_xcpt_valid_o) begin
        instr_req_reserved <= 1'b0;
    end
    else if (i_req_valid_i && instr_fetching) begin
        instr_req_reserved <= 1'b1;
    end
    else begin
        instr_req_reserved <= instr_req_reserved;
    end
end

assign i_req_reserved_restart = instr_req_reserved & instr_stall_end & ~fetch_stage_stalling;

// -------------------------------------
// ITLB status regs
wire update_immu = immu_enable_i & itlb_hit & 
    ((i_req_valid_i | i_req_reserved_restart) & ~instr_fetching | update_itlb_done) &
    ~sfence_vma_valid_i & ~fetch_stage_stalling;

wire i_ppn1_nzero = (|itlb_pte[27:19]);
wire i_ppn0_nzero = (|itlb_pte[18:10]);

always @(posedge clk_i) begin
    if (rst_i || 
        (flush_i && (!(instr_fetching || fetch_stage_stalling) || fence_i)) ||
        i_ptw_xcpt_pipe_full) begin
        itlb_req_valid_r       <= 1'b0;
        itlb_paddr_r           <= { 8'h00, {(56){i_ptw_xcpt_pipe_full}}} & itlb_paddr_r;
        itlb_empty             <= 1'b1;
        itlb_vaddr_r           <= { 25'b0, {(39){i_ptw_xcpt_pipe_full}}} & i_req_vaddr_i;
        itlb_branch_hit_r      <= 1'b0;
        itlb_branch_decision_r <= 1'b0;
        itlb_pte_U_r           <= 1'b0;
        itlb_pte_R_r           <= 1'b0;
        itlb_pte_X_r           <= 1'b0;
        itlb_pte_A_r           <= 1'b0;
    end
    else if (update_immu) begin
        itlb_req_valid_r       <= 1'b1;
        itlb_paddr_r           <= itlb_paddr;
        itlb_empty             <= 1'b0;
        itlb_vaddr_r           <= i_req_vaddr_i;
        itlb_branch_hit_r      <= i_req_branch_hit_i;
        itlb_branch_decision_r <= i_req_branch_decision_i;
        itlb_pte_U_r           <= itlb_pte[4];
        itlb_pte_R_r           <= itlb_pte[1];
        itlb_pte_X_r           <= itlb_pte[3];
        itlb_pte_A_r           <= itlb_pte[6];
        itlb_ppn_misaligned    <= (itlb_pte_size[1] & i_ppn1_nzero & i_ppn0_nzero) |
                                  (itlb_pte_size[0] & i_ppn0_nzero);
    end
    else if (!fetch_stage_stalling) begin
        itlb_req_valid_r       <= 1'b0;
    end
end

// -------------------------------------
// PMP (Physical Memory Protection)

// Instruction PMP exceptions
// 1. A bit not set
// 2. Superpage PPN misaligned
always @(*) begin
    i_pmp_xcpt_valid = itlb_req_valid_r &
                       (~itlb_pte_A_r |         // A bit not set
                        itlb_ppn_misaligned);   // Superpage PPN misaligned
end

// -------------------------------------------------
// |      12        |Instruction page fault        |
// -------------------------------------------------
always @(*) begin
    i_xcpt_cause = 4'd12;
    i_xcpt_tval = ~i_ptw_xcpt_pipe_empty ? itlb_vaddr_r : i_req_vaddr_i;
end

// -------------------------------------
// Data
// -------------------------------------

// -------------------------------------
// Delay registers
reg          update_dtlb_done;
reg  [1 : 0] dtlb_pte_size;

always @(posedge clk_i) begin
    if (rst_i) begin
        update_dtlb_done <= 1'b0;
        dtlb_pte_size    <= 2'b00;
    end
    else begin
        update_dtlb_done <= ptw2dtlb_update_valid;
        dtlb_pte_size    <= ptw2dtlb_update_pte_size;
    end
end

// -------------------------------------
// Pupeline stalling status for data accessing register
reg mem_stage_stalling_r;

always @(posedge clk_i) begin
    if (rst_i) begin
        mem_stage_stalling_r <= 0;
    end
    else if ((d_rtrn_valid_o || d_xcpt_valid_o) && dtlb_req_valid_r) begin
        mem_stage_stalling_r <= mem_stage_stalling;
    end
    else begin
        mem_stage_stalling_r <= mem_stage_stalling_r;
    end
end

// -------------------------------------
// DTLB status registers
wire d_ppn1_nzero = dtlb_pte[27:19];
wire d_ppn0_nzero = dtlb_pte[18:10];

always @(posedge clk_i) begin
    if (rst_i) begin
        dtlb_req_valid_r    <= 1'b0;
        dtlb_paddr_r        <= 1'b0;
        dtlb_pte_U_r        <= 1'b0;
        dtlb_pte_R_r        <= 1'b0;
        dtlb_pte_W_r        <= 1'b0;
        dtlb_pte_D_r        <= 1'b0;
        dtlb_pte_X_r        <= 1'b0;
        dtlb_pte_A_r        <= 1'b0;
        dtlb_ppn_misaligned <= 1'b0;
    end
    else if ((d_rtrn_valid_o || d_xcpt_valid_o) && dtlb_req_valid_r && 
              ~mem_stage_stalling) begin
        dtlb_req_valid_r    <= 1'b0;
        dtlb_paddr_r        <= dtlb_paddr_r;
        dtlb_pte_U_r        <= 1'b0;
        dtlb_pte_R_r        <= 1'b0;
        dtlb_pte_W_r        <= 1'b0;
        dtlb_pte_D_r        <= 1'b0;
        dtlb_pte_X_r        <= 1'b0;
        dtlb_pte_A_r        <= 1'b0;
        dtlb_ppn_misaligned <= 1'b0;
    end
    else if (dmmu_enable_i && dtlb_hit && (d_req_valid_i || update_dtlb_done) &&
             ~sfence_vma_valid_i) begin
        dtlb_req_valid_r    <= 1'b1;
        dtlb_paddr_r        <= dtlb_paddr;
        dtlb_pte_U_r        <= dtlb_pte[4];
        dtlb_pte_R_r        <= dtlb_pte[1];
        dtlb_pte_W_r        <= dtlb_pte[2];
        dtlb_pte_D_r        <= dtlb_pte[7];
        dtlb_pte_X_r        <= dtlb_pte[3];
        dtlb_pte_A_r        <= dtlb_pte[6];
        dtlb_ppn_misaligned <= (dtlb_pte_size[1] & d_ppn1_nzero & d_ppn0_nzero) |
                               (dtlb_pte_size[0] & d_ppn0_nzero);
    end
end

// -------------------------------------
// PMP (Physical Memory Protection)

// Data PMP exceptions:
// 1. A bit not set
// 2. R bit not set with MXR not set
// 3. R bit and X bit not set with MXR set
// 4. Wrtie to a page in U-Mode with U bit or D bit not set
// 5. ls/st a page in U-Mode with U bit not set
// 6. ls/st a page in S-Mode with U bit set and SUM not set
// 7. Superpage PPN misaligned

always @(*) begin
    d_pmp_xcpt_valid = dtlb_req_valid_r & (
        (~dtlb_pte_A_r) |
        (~dtlb_pte_R_r & ~mxr_i) |
        (~dtlb_pte_R_r & ~dtlb_pte_X_r & mxr_i) |
        ((d_req_rw_i | d_is_amo_i) & (~dtlb_pte_W_r | ~dtlb_pte_D_r)) |
        (ld_st_privilege_level_i == 2'b00 & ~dtlb_pte_U_r) |
        (ld_st_privilege_level_i == 2'b01 & dtlb_pte_U_r & ~sum_i) |
        (dtlb_ppn_misaligned));
end

// -------------------------------------------------
// |      13        |Load page fault               |
// -------------------------------------------------
// |      14        |Reserved                      |
// -------------------------------------------------
// |      15        |Store/AMO page fault          |
// -------------------------------------------------
always @(*) begin
    d_xcpt_cause = (d_is_amo_i | d_req_rw_i) ? 4'd15 : 4'd13;
    d_xcpt_tval  = d_req_vaddr_i;
end

// -------------------------------------
// VIPT (Virtual Index Physical Tag, only for cache accessing)
// -------------------------------------
reg d_vipt_delay;   // delay register for d_vipt_o

always @(posedge clk_i) begin
    d_vipt_delay <= d_vipt_o;
end

wire dmmu_update_vipt; // signals for update dtlb regsiters for vipt function
assign dmmu_update_vipt = (~dtlb_req_valid_r) & 
    (dtlb_hit & (d_req_valid_i | update_dtlb_done) & ~sfence_vma_valid_i);

`ifndef TEST_VIPT
    assign d_vipt_o = 1'b0;
    assign d_vipt_error_o = 1'b0;
`else
    assign d_vipt_o = dmmu_enable_i & dmmu_update_vipt & ~itlb_miss_valid & ~dtlb_miss_valid;
    assign d_vipt_error_o = dmmu_enable_i & d_vipt_delay & 
        (d_pmp_xcpt_valid | (dtlb_paddr_r[31:28] != 4'h8));
`endif

// -------------------------------------
// To Arbiter
// -------------------------------------
assign dtlb2arb_req_valid = (dmmu_enable_i) ? 
    (~d_xcpt_valid_o & dtlb_req_valid_r & ~mem_stage_stalling_r) :
    d_req_valid_i;
assign dtlb2arb_req_paddr = (dmmu_enable_i & dtlb_req_valid_r) ? 
    dtlb_paddr_r : d_req_vaddr_i;
assign dtlb2arb_req_data  = d_req_data_i;
assign dtlb2arb_req_rw    = d_req_rw_i;
assign dtlb2arb_req_byte_enable = d_req_byte_enable_i;

//==========================================================
// Output signals interface

// -----------------
// To I$
// -----------------
assign i_req_valid_o            = (immu_enable_i) ? 
    (~req_cancel_i & ~sfence_vma_valid_i & ~i_xcpt_valid_o & itlb_req_valid_r & ~fence_i) :
    i_req_valid_i;
assign i_req_paddr_o            = (immu_enable_i) ? 
    itlb_paddr_r : 
    i_req_vaddr_i;

// -----------------
// To Fetch
// -----------------
//==========================================================
// We need a queue to prevent the instruction is missed
// This happens when stall and immu exception are raised 

// -----------------
// local parameters
// -----------------
localparam BUF_SIZE = 2;
localparam BUF_BITS = $clog2(BUF_SIZE);

// -----------------
// current signals
// -----------------
wire         i_rtrn_valid;
wire [63: 0] i_rtrn_vaddr;
wire         i_rtrn_branch_hit;
wire         i_rtrn_branch_decision;
wire         i_rtrn_ignore;
wire         i_xcpt_valid;
assign i_rtrn_valid           = (immu_enable_i) ? 
    ((i_rtrn_valid_i & itlb_hit) | i_ptw_xcpt_pipe_full | i_rtrn_ignore):
    i_rtrn_valid_i;
assign i_rtrn_vaddr           = (immu_enable_i & ~i_ptw_xcpt_pipe_empty) ? 
    itlb_vaddr_r : 
    i_req_vaddr_i;
assign i_rtrn_branch_hit      = (immu_enable_i & ~i_ptw_xcpt_pipe_empty) ?
    itlb_branch_hit_r :
    i_req_branch_hit_i;
assign i_rtrn_branch_decision = (immu_enable_i & ~i_ptw_xcpt_pipe_empty) ?
    itlb_branch_decision_r :
    i_req_branch_decision_i;
assign i_rtrn_ignore          = immu_enable_i & itlb_empty & update_immu;
assign i_xcpt_valid = i_pmp_xcpt_valid | (i_ptw_xcpt_valid | i_ptw_xcpt_valid_r) & itlb_empty;

//------------------
// enable signals
//------------------
wire we, re;    // we: push, re: pop
reg  [BUF_BITS-1 : 0] wptr, rptr; // write pointer, read pointer
reg   [63: 0] last_vaddr;

assign we = (i_xcpt_valid | i_rtrn_valid) & (fetch_stage_stalling | buf_valid_r) & (last_vaddr != i_rtrn_vaddr);
assign re = ~fetch_stage_stalling & buf_valid_r;

// wptr
always @(posedge clk_i) begin
    if (rst_i)
        wptr <= 'b0;
    else if (we)
        wptr <= wptr + 'b1;
    else
        wptr <= wptr;
end

// rptr
always @(posedge clk_i) begin
    if (rst_i)
        rptr <= 'b0;
    else if (re)
        rptr <= rptr + 'b1;
    else
        rptr <= rptr;
end

// last_vaddr
always @(posedge clk_i) begin
    if (rst_i)
        last_vaddr <= 64'b0;
    else if (we)
        last_vaddr <= i_rtrn_vaddr;
    else
        last_vaddr <= last_vaddr;
end

//------------------
// buffer registers (wrtie port)
//------------------
reg          buf_valid                  [0 : BUF_SIZE-1];
reg          i_rtrn_valid_buf           [0 : BUF_SIZE-1];
reg  [63: 0] i_rtrn_data_buf            [0 : BUF_SIZE-1];
reg  [63: 0] i_rtrn_vaddr_buf           [0 : BUF_SIZE-1];
reg          i_rtrn_branch_hit_buf      [0 : BUF_SIZE-1];
reg          i_rtrn_branch_decision_buf [0 : BUF_SIZE-1];
reg          i_rtrn_ignore_buf          [0 : BUF_SIZE-1];
reg          i_xcpt_valid_buf           [0 : BUF_SIZE-1];
reg  [3 : 0] i_xcpt_cause_buf           [0 : BUF_SIZE-1];
reg  [63: 0] i_xcpt_tval_buf            [0 : BUF_SIZE-1];
integer i;
genvar g;


generate
for (g = 0; g < BUF_SIZE; g = g + 1) begin: buffer
    always @(posedge clk_i) begin
        if (rst_i || (re && rptr == g)) begin // clear
            buf_valid[g] <= 1'b0;
            
            i_rtrn_valid_buf[g] <= 1'b0;
            i_rtrn_data_buf[g] <= 64'b0;
            i_rtrn_vaddr_buf[g] <= 64'b0;
            i_rtrn_branch_hit_buf[g] <= 1'b0;
            i_rtrn_branch_decision_buf[g] <= 1'b0;
            i_rtrn_ignore_buf[g] <= 1'b0;
            
            i_xcpt_valid_buf[g] <= 1'b0;
            i_xcpt_cause_buf[g] <= 4'b0;
            i_xcpt_tval_buf[g] <= 64'b0;
        end
        else if (we && wptr == g) begin
            buf_valid[g] <= 1'b1;
            
            i_rtrn_valid_buf[g] <= i_rtrn_valid;
            i_rtrn_data_buf[g] <= i_rtrn_data_i;
            i_rtrn_vaddr_buf[g] <= i_rtrn_vaddr;
            i_rtrn_branch_hit_buf[g] <= i_rtrn_branch_hit;
            i_rtrn_branch_decision_buf[g] <= i_rtrn_branch_decision;
            i_rtrn_ignore_buf[g] <= i_rtrn_ignore;
            
            i_xcpt_valid_buf[g] <= i_xcpt_valid;
            i_xcpt_cause_buf[g] <= i_xcpt_cause;
            i_xcpt_tval_buf[g] <= i_xcpt_tval;
        end
        else begin
            buf_valid[g] <= buf_valid[g];
            
            i_rtrn_valid_buf[g] <= i_rtrn_valid_buf[g];
            i_rtrn_data_buf[g] <= i_rtrn_data_buf[g];
            i_rtrn_vaddr_buf[g] <= i_rtrn_vaddr_buf[g];
            i_rtrn_branch_hit_buf[g] <= i_rtrn_branch_hit_buf[g];
            i_rtrn_branch_decision_buf[g] <= i_rtrn_branch_decision_buf[g];
            i_rtrn_ignore_buf[g] <= i_rtrn_ignore_buf[g];
            
            i_xcpt_valid_buf[g] <= i_xcpt_valid_buf[g];
            i_xcpt_cause_buf[g] <= i_xcpt_cause_buf[g];
            i_xcpt_tval_buf[g] <= i_xcpt_tval_buf[g];
        end
    end 
end
endgenerate

//------------------
// read port
//------------------
reg          buf_valid_r;

reg          i_rtrn_valid_r;
reg  [63: 0] i_rtrn_data_r;
reg  [63: 0] i_rtrn_vaddr_r;
reg          i_rtrn_branch_hit_r;
reg          i_rtrn_branch_decision_r;
reg          i_rtrn_ignore_r;

reg          i_xcpt_valid_r;
reg  [3 : 0] i_xcpt_cause_r;
reg  [63: 0] i_xcpt_tval_r;

always @(*) begin
    buf_valid_r = buf_valid[rptr];
    
    i_rtrn_valid_r = i_rtrn_valid_buf[rptr];
    i_rtrn_data_r = i_rtrn_data_buf[rptr];
    i_rtrn_vaddr_r = i_rtrn_vaddr_buf[rptr];
    i_rtrn_branch_hit_r = i_rtrn_branch_hit_buf[rptr];
    i_rtrn_branch_decision_r = i_rtrn_branch_decision_buf[rptr];
    i_rtrn_ignore_r = i_rtrn_ignore_buf[rptr];
    
    i_xcpt_valid_r = i_xcpt_valid_buf[rptr];
    i_xcpt_cause_r = i_xcpt_cause_buf[rptr];
    i_xcpt_tval_r = i_xcpt_tval_buf[rptr];
end

//------------------
// output
//------------------
assign i_rtrn_valid_o = i_rtrn_valid | (buf_valid_r & i_rtrn_valid_r);
assign i_rtrn_data_o  = (buf_valid_r) ? i_rtrn_data_r : i_rtrn_data_i;
assign i_rtrn_vaddr_o           = (buf_valid_r) ? i_rtrn_vaddr_r : i_rtrn_vaddr;
assign i_rtrn_branch_hit_o      = (buf_valid_r) ? i_rtrn_branch_hit_r : i_rtrn_branch_hit;
assign i_rtrn_branch_decision_o = (buf_valid_r) ? i_rtrn_branch_decision_r : i_rtrn_branch_decision;
assign i_rtrn_ignore_o          = (buf_valid_r) ? i_rtrn_ignore_r : i_rtrn_ignore;

assign i_xcpt_valid_o = (buf_valid_r) ? i_xcpt_valid_r : i_xcpt_valid;
assign i_xcpt_cause_o = (buf_valid_r) ? i_xcpt_cause_r : i_xcpt_cause;
assign i_xcpt_tval_o  = (buf_valid_r) ? i_xcpt_tval_r  : i_xcpt_tval;

//==========================================================

// -----------------
// To Memory (exception)
// -----------------
assign d_xcpt_valid_o = d_pmp_xcpt_valid | d_ptw_xcpt_valid;
assign d_xcpt_cause_o = d_xcpt_cause;
assign d_xcpt_tval_o  = d_xcpt_tval;

//==========================================================
// Other modules

// -------------------------------------
//  TLB (Translation Lookaside Buffer)
// -------------------------------------
tlb #(
    .TLB_ENTRIES(ITLB_ENTRIES)
) itlb (
    // System input
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Flush signals
    .flush_i(sfence_vma_valid_i),
    .flush_type_i(sfence_vma_type_i),
    .flush_vaddr_i(sfence_vma_vaddr_i),
    .flush_asid_i(sfence_vma_asid_i),

    // CSR signals
    .csr_asid_i(asid_i),

    // Update signals
    .update_valid_i(ptw2itlb_update_valid),
    .update_tag_i(i_req_vaddr_i[38:12]),
    .update_pte_i(ptw2itlb_update_pte),
    .update_pte_size_i(ptw2itlb_update_pte_size),

    // Virtual address input
    .vaddr_i(i_req_vaddr_i),

    // Output
    .hit_o(itlb_hit),
    .paddr_o(itlb_paddr),
    .pte_o(itlb_pte)
);

tlb #(
    .TLB_ENTRIES(DTLB_ENTRIES)
) dtlb (
    // System input
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Flush signals
    .flush_i(sfence_vma_valid_i),
    .flush_type_i(sfence_vma_type_i),
    .flush_vaddr_i(sfence_vma_vaddr_i),
    .flush_asid_i(sfence_vma_asid_i),

    // CSR signals
    .csr_asid_i(asid_i),

    // Update signals
    .update_valid_i(ptw2dtlb_update_valid),
    .update_tag_i(d_req_vaddr_i[38:12]),
    .update_pte_i(ptw2dtlb_update_pte),
    .update_pte_size_i(ptw2dtlb_update_pte_size),

    // Virtual address input
    .vaddr_i(d_req_vaddr_i),

    // Output
    .hit_o(dtlb_hit),
    .paddr_o(dtlb_paddr),
    .pte_o(dtlb_pte)
);

// -------------------------------------
// PTW (Page Table Walk)
// -------------------------------------

ptw #(
) Ptw (
    // System input
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Stalling signals
    .stall_if_i(fetch_stage_stalling),
    .stall_mem_i(mem_stage_stalling),

    // Flush signal isn't used

    // ITLB Update
    .i_req_valid_i(i_req_valid_i | i_req_reserved_restart),
    .i_tlb_miss_i(itlb_miss_valid),
    .i_vaddr_i(i_req_vaddr_i),

    .i_req_ready_o(ptw2itlb_update_valid),
    .i_pte_o(ptw2itlb_update_pte),
    .i_pte_size_o(ptw2itlb_update_pte_size),

    // DTLB Update
    .d_req_valid_i(d_req_valid_i),
    .d_tlb_miss_i(dtlb_miss_valid),
    .d_vaddr_i(d_req_vaddr_i),

    .d_req_ready_o(ptw2dtlb_update_valid),
    .d_pte_o(ptw2dtlb_update_pte),
    .d_pte_size_o(ptw2dtlb_update_pte_size),

    // To D-Cache
    .m_req_valid_o(ptw2arb_req_valid),
    .m_req_rw_o(ptw2arb_req_rw),
    .m_byte_enable_o(ptw2arb_req_byte_enable),
    .m_req_addr_o(ptw2arb_req_addr),

    // From D-Cache,
    .m_req_ready_i(arb2ptw_rtrn_valid),
    .m_rtrn_data_i(arb2ptw_rtrn_data),

    // From CSR
    .root_ppn_i(root_ppn_i),
    .asid_i(asid_i),
    .privilege_level_i(privilege_level_i),

    // Exception (Page Fault)
    .i_xcpt_valid_o(i_ptw_xcpt_valid),
    .d_xcpt_valid_o(d_ptw_xcpt_valid)
);

// -------------------------------------
// MMU Arbiter
// -------------------------------------
mmu_arb #(
) MMU_arb (
    // System input
    .clk_i(clk_i),
    .rst_i(rst_i),
    .enable_i(dmmu_enable_i),

    // Stalling
    .mem_stage_stalling_i(mem_stage_stalling),

    // Flush
    .flush_i(flush_i),

    // From processor to D-Cache
    .d_req_valid_i(dtlb2arb_req_valid),
    .d_vipt_i(d_vipt_delay),
    .d_req_paddr_i(dtlb2arb_req_paddr),
    .d_req_data_i(dtlb2arb_req_data),
    .d_req_rw_i(dtlb2arb_req_rw),
    .d_req_byte_enable_i(dtlb2arb_req_byte_enable),

    .d_rtrn_valid_o(d_rtrn_valid_o),
    .d_rtrn_data_o(d_rtrn_data_o),

    // From PTW to D-Cache
    .ptw_req_valid_i(ptw2arb_req_valid),
    .ptw_req_paddr_i(ptw2arb_req_addr),
    .ptw_req_data_i(64'b0),
    .ptw_req_rw_i(ptw2arb_req_rw),
    .ptw_req_byte_enable_i(ptw2arb_req_byte_enable),
    
    .ptw_rtrn_valid_o(arb2ptw_rtrn_valid),
    .ptw_rtrn_data_o(arb2ptw_rtrn_data),
    
    // To D-Cache
    .d_req_valid_o(d_req_valid_o),
    // d_strobe signal isn't used
    .d_req_paddr_o(d_req_paddr_o),
    .d_req_data_o(d_req_data_o),
    .d_req_rw_o(d_req_rw_o),
    .d_req_byte_enable_o(d_req_byte_enable_o),

    .d_rtrn_valid_i(d_rtrn_valid_i),
    .d_rtrn_data_i(d_rtrn_data_i)
);

endmodule