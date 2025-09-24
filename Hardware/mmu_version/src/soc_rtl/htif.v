`timescale 1ns / 1ps
// =============================================================================
//  Program : htif.v
//  Author  : Jun-Kai Chen
//  Date    : Sep/24/2025
// -----------------------------------------------------------------------------
//  Description:
//  This circuit provides Host-Target InterFace(HTIF) functionality simulation,
//  handling access to `tohost' and `fromhost' addresses. Behavior fit
//  riscv-test requirements, printing `tohost' data.
//
//  This circuit is used only in `soc_tb.v'.
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
//  Copyright 2025,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Yang Ming Chiao Tung Uniersity (NYCU)
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

module htif #(
    parameter [15: 0] tohost_addr   = 16'h1000,
    parameter [15: 0] fromhost_addr = 16'h1040
) (
    // System input
    input               clk_i,
    input               rst_i,
    
    // Input signals
    input               p_strobe_i,
    input               p_rw_i,
    input  [15: 0]      p_addr_i,
    input  [63: 0]      p_data_i,
    
    // Output signals
    output reg          p_strobe_o,
    output reg [63: 0]  p_data_o
    );
    
wire access_tohost = (p_addr_i == tohost_addr);
wire access_fromhost = (p_addr_i == fromhost_addr);
wire write_en    = p_strobe_i & p_rw_i;
wire read_en     = p_strobe_i & ~p_rw_i; 

// tohost
reg  [63: 0] tohost;
always @(posedge clk_i) begin
    if (rst_i)
        tohost <= 64'b0;
    else if (write_en && access_tohost)
        tohost <= p_data_i;
    else
        tohost <= 64'b0;
end

// fromhost
reg  [63: 0] fromhost;
always @(posedge clk_i) begin
    if (rst_i)
        fromhost <= 64'b0;
    else if (write_en && access_fromhost)
        fromhost <= p_data_i;
    else
        fromhost <= fromhost;
end

// output ready
always @(posedge clk_i) begin
    if (rst_i)
        p_strobe_o <= 1'b0;
    else if (p_strobe_i)
        // raise ready when write and read
        p_strobe_o <= 1'b1;
    else
        p_strobe_o <= 1'b0;
end

// output data
wire [63: 0] dout = (access_tohost) ? tohost :
                    (access_fromhost) ? fromhost :
                    64'b0;
always @(posedge clk_i) begin
    if (rst_i)
        p_data_o <= 64'b0;
    else if (read_en)
        p_data_o <= dout;
    else
        p_data_o <= 64'b0;
end

// dump data
wire [7 : 0] char = tohost[7 : 0];
always @(posedge clk_i) begin
    if (|tohost)    // tohost is written --> dump
        if (tohost[63:48] == 16'h0101) begin    // cputchar
            if (char != 13)         // remove the `\r'
                $write("%c", char);
            if (char == 27) begin   // escape
                $display("breakpoint");
                $stop();
            end
            if (char == 3) begin    // ETX
                $display("ETX");
                $stop();
            end
        end
        else begin  // terminate code
            $write("Recieve terminate code: %d\n", tohost);
            $stop();
        end
end

endmodule
