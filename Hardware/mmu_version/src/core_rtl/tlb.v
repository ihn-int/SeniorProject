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
//    Modify the circuit to support SV39 format.
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

module tlb #(
    parameter TLB_ENTRIES = 4
)(
    // System input
    input  wire         clk_i,
    input  wire         rst_i,

    // Flush signals
    input  wire         flush_i,
    input  wire         flush_type_i,   // 0: rs1 == 0, 1: rs1 != 0
    input  wire [63: 0] flush_vaddr_i,  // rs1_data
    input  wire [63: 0] flush_asid_i,   // rs2_data

    // CSR signals
    input  wire [15: 0] csr_asid_i,

    // Update signals
    input  wire         update_valid_i,
    input  wire [26: 0] update_tag_i,   // VPN
    input  wire [63: 0] update_pte_i,
    input  wire [1 : 0] update_pte_size_i, // 0: 4 KiB, 1: 2 MiB, 2: 1 GiB

    // Virtual address input
    input  wire [63: 0] vaddr_i,

    // Output
    output wire         hit_o,
    output wire [63: 0] paddr_o,
    output wire [63: 0] pte_o
);

//==========================================================
// SV39 PTE (Page Table Entry)
// +---------------------------------------------------------------------------------------+
// |  N | PBMT | Reserved | PPN[2] | PPN[1] | PPN[0] | RSW | D | A | G | U | X | W | R | V |
// +---------------------------------------------------------------------------------------+
// | 63 |62  61|60      54|53    28|27    19|18    10|9   8| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 
// +---------------------------------------------------------------------------------------+
// VPN tag 
reg  [8 : 0] tag_vpn_2_r [0 : TLB_ENTRIES-1];
reg  [8 : 0] tag_vpn_1_r [0 : TLB_ENTRIES-1];
reg  [8 : 0] tag_vpn_0_r [0 : TLB_ENTRIES-1];

// PTE value
reg  [63: 0] pte_r       [0 : TLB_ENTRIES-1];
wire [25: 0] pte_ppn_2   [0 : TLB_ENTRIES-1];
wire [8 : 0] pte_ppn_1   [0 : TLB_ENTRIES-1];
wire [8 : 0] pte_ppn_0   [0 : TLB_ENTRIES-1];
wire [1 : 0] pte_rsw     [0 : TLB_ENTRIES-1];
wire         pte_D       [0 : TLB_ENTRIES-1];
wire         pte_A       [0 : TLB_ENTRIES-1];
wire         pte_G       [0 : TLB_ENTRIES-1];
wire         pte_U       [0 : TLB_ENTRIES-1];
wire         pte_X       [0 : TLB_ENTRIES-1];
wire         pte_W       [0 : TLB_ENTRIES-1];
wire         pte_R       [0 : TLB_ENTRIES-1];
wire         pte_V       [0 : TLB_ENTRIES-1];

// Page info
reg  [1 : 0] pte_size_r   [0 : TLB_ENTRIES-1];   // 0: 4KiB
reg          pte_valid_r  [0 : TLB_ENTRIES-1];
reg  [15: 0] pte_asid_r   [0 : TLB_ENTRIES-1];

// Virtual Address Field
wire [8 : 0] vaddr_vpn_2;
wire [8 : 0] vaddr_vpn_1;
wire [8 : 0] vaddr_vpn_0;

reg  [0 : TLB_ENTRIES-1] tlb_sel;
reg  [55: 0]             tlb_paddr [0 : TLB_ENTRIES-1];

reg  [$clog2(TLB_ENTRIES)-1 : 0] FIFO_cnt_r;

//==========================================================
// Virtual Address Translation
// -------------------------------------
// | VPN[2] | VPN[1] | VPN[0] | offset |
// -------------------------------------
// |38    30|29    21|20    12|11     0|
// -------------------------------------
// -------------------------------------
// | PPN[2] | PPN[1] | PPN[0] | offset |
// -------------------------------------
// |55    30|29    21|20    12|11     0|
// -------------------------------------
assign vaddr_vpn_2 = vaddr_i[38:30];
assign vaddr_vpn_1 = vaddr_i[29:21];
assign vaddr_vpn_0 = vaddr_i[20:12];

reg  vpn_2_match[0 : TLB_ENTRIES-1];
reg  vpn_1_match[0 : TLB_ENTRIES-1];
reg  vpn_0_match[0 : TLB_ENTRIES-1];

// Check VPN and tag
integer i;
always @(*) begin
    for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
        vpn_2_match[i] = (vaddr_vpn_2 == tag_vpn_2_r[i]);
        vpn_1_match[i] = (vaddr_vpn_1 == tag_vpn_1_r[i]);
        vpn_0_match[i] = (vaddr_vpn_0 == tag_vpn_0_r[i]);
    end
end

reg  [1 : 0] match_pte[0 : TLB_ENTRIES-1]; // correct page size && vpn tag
always @(*) begin
    for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
        if (pte_size_r[i] == 2'b10 && vpn_2_match[i]) begin
            match_pte[i] = 2'b10;
        end
        else if (pte_size_r[i] == 2'b01 && vpn_2_match[i] && vpn_1_match[i]) begin
            match_pte[i] = 2'b01;
        end
        else if (pte_size_r[i] == 2'b00 && vpn_2_match[i] && vpn_1_match[i] && vpn_0_match[i]) begin
            match_pte[i] = 2'b00;
        end
        else begin
            match_pte[i] = 2'b11;   // page not match
        end
    end
end

always @(*) begin
    for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
        if (pte_valid_r[i]) begin
            case (match_pte[i])
                2'b10: begin // 1 GiB page
                    tlb_sel[i] = 1'b1;
                    tlb_paddr[i] = {pte_ppn_2[i], vaddr_i[29:0]};
                end
                2'b01: begin // 2 MiB page
                    tlb_sel[i] = 1'b1;
                    tlb_paddr[i] = {pte_ppn_2[i], pte_ppn_1[i], vaddr_i[20: 0]};
                end
                2'b00: begin // 4 KiB page
                    tlb_sel[i] = 1'b1;
                    tlb_paddr[i] = {pte_ppn_2[i], pte_ppn_1[i], pte_ppn_0[i], vaddr_i[11: 0]};
                end
                2'b11: begin // This PTE is not selected
                    tlb_sel[i] = 1'b0;
                    tlb_paddr[i] = 'b0;
                end
            endcase
        end
        else begin  // This PTE is not selected
            tlb_sel[i] = 1'b0;
            tlb_paddr[i] = 'b0;
        end
    end
end

//==========================================================
// Update TLB
// +---------------------------------------------------------------------------------------+
// |  N | PBMT | Reserved | PPN[2] | PPN[1] | PPN[0] | RSW | D | A | G | U | X | W | R | V |
// +---------------------------------------------------------------------------------------+
// | 63 |62  61|60      54|53    28|27    19|18    10|9   8| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 
// +---------------------------------------------------------------------------------------+
// N is for Svnapot extension; PBMT is for Svpbmt extension
// Both of which are not supported here

reg  flush_vpn_2_match[0 : TLB_ENTRIES];
reg  flush_vpn_1_match[0 : TLB_ENTRIES];
reg  flush_vpn_0_match[0 : TLB_ENTRIES];
always @(*) begin
    for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
        flush_vpn_2_match[i] = (flush_vaddr_i[38:30] == tag_vpn_2_r[i]);
        flush_vpn_1_match[i] = (flush_vaddr_i[29:21] == tag_vpn_1_r[i]);
        flush_vpn_0_match[i] = (flush_vaddr_i[20:12] == tag_vpn_0_r[i]); 
    end
end

reg  flush_pte[0 : TLB_ENTRIES];   // correct page size and flush vpn
always @(*) begin
    for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
        flush_pte[i] <= (pte_size_r[i] == 2'b10 & flush_vpn_2_match[i]) |
                        (pte_size_r[i] == 2'b01 & flush_vpn_2_match[i] & flush_vpn_1_match[i]) |
                        (pte_size_r[i] == 2'b00 & flush_vpn_2_match[i] & flush_vpn_1_match[i] & flush_vpn_0_match[i]);
    end
end

// Valid
always @(posedge clk_i) begin
    if (rst_i) begin
        for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
            pte_valid_r[i] = 1'b0;
        end
    end
    else if (flush_i) begin
        for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
            case ({flush_type_i, |(flush_asid_i[15: 0])}) // {rs1, rs2}
                2'b11: begin // rs1 != 0, rs2 != 0
                // Check 1. leaf page with correct VA
                //       2. ASID match
                //       3. G == 0
                    if (flush_pte[i] &&
                        pte_asid_r[i] == flush_asid_i[15: 0] &&
                        pte_G[i] == 1'b0) begin
                            pte_valid_r[i] <= 1'b0;
                    end
                    else begin
                        pte_valid_r[i] <= pte_valid_r[i];
                    end
                end
                2'b10: begin // rs1 != 0, rs2 == 0
                // Check 1. leaf page with correct VA
                    if (flush_pte[i]) begin
                        pte_valid_r[i] <= 1'b0;
                    end
                    else begin
                        pte_valid_r[i] <= pte_valid_r[i];
                    end
                end
                2'b01: begin // rs1 == 0, rs2 != 0
                // Check 1. ASID match
                //       2. G == 0
                    if (pte_asid_r[i][15: 0] == flush_asid_i[15:0] &&
                        pte_G[i] == 0) begin
                            pte_valid_r[i] <= 1'b0;
                    end
                    else begin
                        pte_valid_r[i] <= pte_valid_r[i];
                    end
                end
                2'b00: begin // rs1 == 0, rs2 == 0
                // Clear all PTE
                    pte_valid_r[i] <= 1'b0;
                end
            endcase
        end
    end
    else if (update_valid_i) begin
        pte_valid_r[FIFO_cnt_r] <= 1'b1;
    end
end

// ASID
always @(posedge clk_i) begin
    if (update_valid_i) begin
        pte_asid_r[FIFO_cnt_r] <= csr_asid_i;
    end
end

// PTE
always @(posedge clk_i) begin
    if (update_valid_i) begin
        pte_r[FIFO_cnt_r] <= update_pte_i;
    end
end

// PTE Fields with generate
genvar g;
generate
    for (g = 0; g < TLB_ENTRIES; g = g + 1) begin: pte_gen
        assign pte_ppn_2[g] = pte_r[g][53:28];
        assign pte_ppn_1[g] = pte_r[g][27:19];
        assign pte_ppn_0[g] = pte_r[g][18:10];
        assign pte_rsw[g]   = pte_r[g][9 : 8];
        assign pte_D[g]     = pte_r[g][7];
        assign pte_A[g]     = pte_r[g][6];
        assign pte_G[g]     = pte_r[g][5];
        assign pte_U[g]     = pte_r[g][4];
        assign pte_X[g]     = pte_r[g][3];
        assign pte_W[g]     = pte_r[g][2];
        assign pte_R[g]     = pte_r[g][1];
        assign pte_V[g]     = pte_r[g][0];
    end
endgenerate

// Size
always @(posedge clk_i) begin
    if (update_valid_i) begin
        pte_size_r[FIFO_cnt_r] <= update_pte_size_i;
    end
end

// Tag
always @(posedge clk_i) begin
    if (rst_i) begin
        for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
            tag_vpn_2_r[i] <= 'b0;
            tag_vpn_1_r[i] <= 'b0;
            tag_vpn_0_r[i] <= 'b0;
        end
    end
    else if (update_valid_i) begin
        tag_vpn_2_r[FIFO_cnt_r] <= update_tag_i[26:18];
        tag_vpn_1_r[FIFO_cnt_r] <= update_tag_i[17: 9];
        tag_vpn_0_r[FIFO_cnt_r] <= update_tag_i[8 : 0];
    end
end

//==========================================================
// FIFO policy
always @(posedge clk_i) begin
    if (rst_i) begin
        FIFO_cnt_r <= 'b0;
    end
    else if (update_valid_i) begin
        FIFO_cnt_r <= FIFO_cnt_r + 1;
    end
    else begin
        FIFO_cnt_r <= FIFO_cnt_r;
    end
end

//==========================================================
// Output singals interface
// hit_o
assign hit_o = (|tlb_sel);

// paddr_o
generate
    for (g = 0; g < TLB_ENTRIES; g = g + 1) begin: paddr_o_gen
        wire [55: 0] paddr_temp;
        if (g == 0) begin
            assign paddr_temp = {(56){tlb_sel[g]}} & tlb_paddr[g];
        end
        else begin
            assign paddr_temp = paddr_o_gen[g-1].paddr_temp |
                ( {(56){tlb_sel[g]}} & tlb_paddr[g] );
        end
    end // paddr_o_gen
    // zero padding to 64 bits
    assign paddr_o = { 8'b0, paddr_o_gen[TLB_ENTRIES-1].paddr_temp[55: 0]};
endgenerate

// pte_o
generate
    for (g = 0; g < TLB_ENTRIES; g = g + 1) begin: pte_o_gen
        wire [63: 0] pte_temp;
        if (g == 0) begin
            assign pte_temp = {(64){tlb_sel[g]}} & pte_r[g];
        end
        else begin
            assign pte_temp = pte_o_gen[g-1].pte_temp |
                ( {(64){tlb_sel[g]}} & pte_r[g] );
        end
    end // pte_o_gen
    assign pte_o = pte_o_gen[TLB_ENTRIES-1].pte_temp;
endgenerate

endmodule   // tlb