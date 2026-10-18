`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.10.2026 20:52:54
// Design Name: 
// Module Name: alu
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
// 64-bit RISC-V Arithmetic Logic Unit
// Project: Dual-Core RISC-V Processor with 3D IK Accelerator
// Language: Verilog-2001
//==============================================================

module alu (
    input  wire [63:0] a,
    input  wire [63:0] b,
    input  wire [3:0]  alu_control,

    output reg  [63:0] result,
    output wire        zero
);

    //==========================================================
    // ALU CONTROL CODES
    //==========================================================

    parameter ALU_ADD  = 4'b0000;
    parameter ALU_SUB  = 4'b0001;
    parameter ALU_AND  = 4'b0010;
    parameter ALU_OR   = 4'b0011;
    parameter ALU_XOR  = 4'b0100;
    parameter ALU_SLL  = 4'b0101;
    parameter ALU_SRL  = 4'b0110;
    parameter ALU_SRA  = 4'b0111;
    parameter ALU_SLT  = 4'b1000;
    parameter ALU_SLTU = 4'b1001;

    //==========================================================
    // ALU OPERATION
    //==========================================================

    always @(*) begin

        case (alu_control)

            // Addition
            ALU_ADD:
                result = a + b;

            // Subtraction
            ALU_SUB:
                result = a - b;

            // Bitwise AND
            ALU_AND:
                result = a & b;

            // Bitwise OR
            ALU_OR:
                result = a | b;

            // Bitwise XOR
            ALU_XOR:
                result = a ^ b;

            // Logical Shift Left
            ALU_SLL:
                result = a << b[5:0];

            // Logical Shift Right
            ALU_SRL:
                result = a >> b[5:0];

            // Arithmetic Shift Right
            ALU_SRA:
                result = $signed(a) >>> b[5:0];

            // Signed Less Than
            ALU_SLT:
                result = ($signed(a) < $signed(b)) ? 64'd1 : 64'd0;

            // Unsigned Less Than
            ALU_SLTU:
                result = (a < b) ? 64'd1 : 64'd0;

            // Default
            default:
                result = 64'd0;

        endcase
    end

    //==========================================================
    // ZERO FLAG
    //==========================================================

    assign zero = (result == 64'd0);

endmodule
