`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.10.2026 21:02:36
// Design Name: 
// Module Name: immediate_generator
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
// RISC-V 64-bit Immediate Generator
// Supports:
// I-type, S-type, B-type, U-type, J-type
// Verilog-2001
//==============================================================

module immediate_generator (
    input  wire [31:0] instruction,
    output reg  [63:0] immediate
);

    //==========================================================
    // OPCODE
    //==========================================================

    wire [6:0] opcode;

    assign opcode = instruction[6:0];

    //==========================================================
    // IMMEDIATE GENERATION
    //==========================================================

    always @(*) begin

        case (opcode)

            //==================================================
            // I-TYPE
            // LOAD, ALU-IMMEDIATE, JALR
            //==================================================

            7'b0000011,   // LOAD
            7'b0010011,   // OP-IMM
            7'b1100111:   // JALR
                immediate = {{52{instruction[31]}},
                             instruction[31:20]};


            //==================================================
            // S-TYPE
            // STORE
            //==================================================

            7'b0100011:
                immediate = {{52{instruction[31]}},
                             instruction[31:25],
                             instruction[11:7]};


            //==================================================
            // B-TYPE
            // BRANCH
            //==================================================

            7'b1100011:
                immediate = {{51{instruction[31]}},
                             instruction[31],
                             instruction[7],
                             instruction[30:25],
                             instruction[11:8],
                             1'b0};


            //==================================================
            // U-TYPE
            // LUI / AUIPC
            //==================================================

            7'b0110111,   // LUI
            7'b0010111:   // AUIPC
                immediate = {{32{instruction[31]}},
                             instruction[31:12],
                             12'b0};


            //==================================================
            // J-TYPE
            // JAL
            //==================================================

            7'b1101111:
                immediate = {{43{instruction[31]}},
                             instruction[31],
                             instruction[19:12],
                             instruction[20],
                             instruction[30:21],
                             1'b0};


            //==================================================
            // DEFAULT
            //==================================================

            default:
                immediate = 64'd0;

        endcase

    end

endmodule
