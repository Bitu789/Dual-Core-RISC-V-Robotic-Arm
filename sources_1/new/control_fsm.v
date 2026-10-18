`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.10.2026 21:08:46
// Design Name: 
// Module Name: control_fsm
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
// RISC-V Multi-Cycle Control FSM
// Project: Dual-Core 64-bit RISC-V Processor
// Language: Verilog-2001
//
// States:
// FETCH -> DECODE -> EXECUTE -> MEMORY -> WRITEBACK
//==============================================================

module control_fsm (
    input  wire        clk,
    input  wire        reset,

    // Instruction fields
    input  wire [6:0]  opcode,
    input  wire [2:0]  funct3,
    input  wire [6:0]  funct7,

    // Current instruction information
    input  wire        alu_zero,

    // Processor control outputs
    output reg         pc_write,
    output reg         ir_write,
    output reg         reg_write,

    output reg         mem_read,
    output reg         mem_write,

    output reg         alu_src_a,
    output reg         alu_src_b,

    output reg         branch_enable,
    output reg         jump_enable,

    output reg  [3:0]  alu_control,

    output reg  [2:0]  current_state
);

    //==========================================================
    // FSM STATES
    //==========================================================

    parameter STATE_FETCH      = 3'b000;
    parameter STATE_DECODE     = 3'b001;
    parameter STATE_EXECUTE    = 3'b010;
    parameter STATE_MEMORY     = 3'b011;
    parameter STATE_WRITEBACK  = 3'b100;

    reg [2:0] next_state;

    //==========================================================
    // RISC-V OPCODES
    //==========================================================

    parameter OPCODE_LOAD   = 7'b0000011;
    parameter OPCODE_OPIMM  = 7'b0010011;
    parameter OPCODE_STORE  = 7'b0100011;
    parameter OPCODE_OP     = 7'b0110011;
    parameter OPCODE_BRANCH = 7'b1100011;
    parameter OPCODE_JAL    = 7'b1101111;
    parameter OPCODE_JALR   = 7'b1100111;
    parameter OPCODE_LUI    = 7'b0110111;
    parameter OPCODE_AUIPC  = 7'b0010111;

    //==========================================================
    // ALU CONTROL CODES
    // These match alu.v
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
    // STATE REGISTER
    //==========================================================

    always @(posedge clk) begin

        if (reset)
            current_state <= STATE_FETCH;
        else
            current_state <= next_state;

    end

    //==========================================================
    // NEXT STATE LOGIC
    //==========================================================

    always @(*) begin

        next_state = STATE_FETCH;

        case (current_state)

            STATE_FETCH:
                next_state = STATE_DECODE;

            STATE_DECODE:
                next_state = STATE_EXECUTE;

            STATE_EXECUTE: begin

                case (opcode)

                    OPCODE_LOAD,
                    OPCODE_STORE:
                        next_state = STATE_MEMORY;

                    default:
                        next_state = STATE_WRITEBACK;

                endcase

            end

            STATE_MEMORY: begin

                case (opcode)

                    OPCODE_LOAD:
                        next_state = STATE_WRITEBACK;

                    OPCODE_STORE:
                        next_state = STATE_FETCH;

                    default:
                        next_state = STATE_FETCH;

                endcase

            end

            STATE_WRITEBACK:
                next_state = STATE_FETCH;

            default:
                next_state = STATE_FETCH;

        endcase

    end

    //==========================================================
    // CONTROL SIGNAL GENERATION
    //==========================================================

    always @(*) begin

        // Default values
        pc_write      = 1'b0;
        ir_write      = 1'b0;
        reg_write     = 1'b0;

        mem_read      = 1'b0;
        mem_write     = 1'b0;

        alu_src_a     = 1'b0;
        alu_src_b     = 1'b0;

        branch_enable = 1'b0;
        jump_enable   = 1'b0;

        alu_control   = ALU_ADD;

        case (current_state)

            //==================================================
            // FETCH
            //==================================================

            STATE_FETCH: begin

                ir_write = 1'b1;

                // ALU calculates PC + 4
                alu_src_a   = 1'b0;
                alu_src_b   = 1'b1;
                alu_control = ALU_ADD;

                pc_write = 1'b1;

            end


            //==================================================
            // DECODE
            //==================================================

            STATE_DECODE: begin

                // No register or memory write here.
                // Instruction fields are interpreted by core.
                alu_control = ALU_ADD;

            end


            //==================================================
            // EXECUTE
            //==================================================

            STATE_EXECUTE: begin

                case (opcode)

                    // -----------------------------------------
                    // Register-register ALU instructions
                    // -----------------------------------------

                    OPCODE_OP: begin

                        alu_src_a = 1'b1;
                        alu_src_b = 1'b0;

                        case (funct3)

                            3'b000: begin
                                if (funct7[5])
                                    alu_control = ALU_SUB;
                                else
                                    alu_control = ALU_ADD;
                            end

                            3'b111:
                                alu_control = ALU_AND;

                            3'b110:
                                alu_control = ALU_OR;

                            3'b100:
                                alu_control = ALU_XOR;

                            3'b001:
                                alu_control = ALU_SLL;

                            3'b101: begin
                                if (funct7[5])
                                    alu_control = ALU_SRA;
                                else
                                    alu_control = ALU_SRL;
                            end

                            3'b010:
                                alu_control = ALU_SLT;

                            3'b011:
                                alu_control = ALU_SLTU;

                            default:
                                alu_control = ALU_ADD;

                        endcase

                    end


                    // -----------------------------------------
                    // Immediate ALU instructions
                    // -----------------------------------------

                    OPCODE_OPIMM: begin

                        alu_src_a = 1'b1;
                        alu_src_b = 1'b1;

                        case (funct3)

                            3'b000:
                                alu_control = ALU_ADD;   // ADDI

                            3'b111:
                                alu_control = ALU_AND;   // ANDI

                            3'b110:
                                alu_control = ALU_OR;    // ORI

                            3'b100:
                                alu_control = ALU_XOR;   // XORI

                            3'b001:
                                alu_control = ALU_SLL;   // SLLI

                            3'b101: begin
                                if (funct7[5])
                                    alu_control = ALU_SRA; // SRAI
                                else
                                    alu_control = ALU_SRL; // SRLI
                            end

                            3'b010:
                                alu_control = ALU_SLT;   // SLTI

                            3'b011:
                                alu_control = ALU_SLTU;  // SLTIU

                            default:
                                alu_control = ALU_ADD;

                        endcase

                    end


                    // -----------------------------------------
                    // LOAD
                    // Address = register + immediate
                    // -----------------------------------------

                    OPCODE_LOAD: begin

                        alu_src_a   = 1'b1;
                        alu_src_b   = 1'b1;
                        alu_control = ALU_ADD;

                    end


                    // -----------------------------------------
                    // STORE
                    // Address = register + immediate
                    // -----------------------------------------

                    OPCODE_STORE: begin

                        alu_src_a   = 1'b1;
                        alu_src_b   = 1'b1;
                        alu_control = ALU_ADD;

                    end


                    // -----------------------------------------
                    // BRANCH
                    // -----------------------------------------

                    OPCODE_BRANCH: begin

                        alu_src_a   = 1'b1;
                        alu_src_b   = 1'b0;
                        alu_control = ALU_SUB;

                        branch_enable = 1'b1;

                    end


                    // -----------------------------------------
                    // JAL
                    // -----------------------------------------

                    OPCODE_JAL: begin

                        jump_enable = 1'b1;

                    end


                    // -----------------------------------------
                    // JALR
                    // -----------------------------------------

                    OPCODE_JALR: begin

                        jump_enable = 1'b1;

                    end


                    // -----------------------------------------
                    // LUI
                    // -----------------------------------------

                    OPCODE_LUI: begin

                        alu_src_a   = 1'b0;
                        alu_src_b   = 1'b1;
                        alu_control = ALU_ADD;

                    end


                    // -----------------------------------------
                    // AUIPC
                    // -----------------------------------------

                    OPCODE_AUIPC: begin

                        alu_src_a   = 1'b0;
                        alu_src_b   = 1'b1;
                        alu_control = ALU_ADD;

                    end


                    default: begin

                        alu_control = ALU_ADD;

                    end

                endcase

            end


            //==================================================
            // MEMORY
            //==================================================

            STATE_MEMORY: begin

                case (opcode)

                    OPCODE_LOAD:
                        mem_read = 1'b1;

                    OPCODE_STORE:
                        mem_write = 1'b1;

                    default: begin
                        mem_read  = 1'b0;
                        mem_write = 1'b0;
                    end

                endcase

            end


            //==================================================
            // WRITEBACK
            //==================================================

            STATE_WRITEBACK: begin

                case (opcode)

                    OPCODE_OP,
                    OPCODE_OPIMM,
                    OPCODE_LOAD,
                    OPCODE_LUI,
                    OPCODE_AUIPC,
                    OPCODE_JAL,
                    OPCODE_JALR:

                        reg_write = 1'b1;

                    default:
                        reg_write = 1'b0;

                endcase

            end


            default: begin

                pc_write      = 1'b0;
                ir_write      = 1'b0;
                reg_write     = 1'b0;
                mem_read      = 1'b0;
                mem_write     = 1'b0;
                branch_enable = 1'b0;
                jump_enable   = 1'b0;
                alu_control   = ALU_ADD;

            end

        endcase

    end

endmodule
