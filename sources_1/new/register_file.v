`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.10.2026 20:59:50
// Design Name: 
// Module Name: register_file
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


//==============================================================
// 64-bit RISC-V Register File
// 32 registers x 64 bits
// Verilog-2001
//==============================================================

module register_file (
    input  wire        clk,
    input  wire        reset,

    // Read ports
    input  wire [4:0]  rs1_addr,
    input  wire [4:0]  rs2_addr,

    output wire [63:0] rs1_data,
    output wire [63:0] rs2_data,

    // Write port
    input  wire        reg_write,
    input  wire [4:0]  rd_addr,
    input  wire [63:0] rd_data
);

    //==========================================================
    // 32 x 64-bit registers
    //==========================================================

    reg [63:0] registers [0:31];

    integer i;

    //==========================================================
    // REGISTER WRITE
    //==========================================================

    always @(posedge clk) begin

        if (reset) begin

            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 64'd0;

        end
        else begin

            // x0 must always remain zero
            registers[0] <= 64'd0;

            // Write only when enabled and destination is not x0
            if (reg_write && (rd_addr != 5'd0))
                registers[rd_addr] <= rd_data;

        end
    end

    //==========================================================
    // REGISTER READ
    //==========================================================

    assign rs1_data =
        (rs1_addr == 5'd0) ? 64'd0 : registers[rs1_addr];

    assign rs2_data =
        (rs2_addr == 5'd0) ? 64'd0 : registers[rs2_addr];

endmodule