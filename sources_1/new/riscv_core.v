`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.10.2026 21:11:22
// Design Name: 
// Module Name: riscv_core
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
// 64-bit RISC-V Multi-Cycle Core
// Project:
// Dual-Core RISC-V Processor with Hardware-Accelerated
// 3D Robotic Arm Control
//
// Language: Verilog-2001
//
// Supported instruction classes:
//   R-type
//   I-type ALU
//   Load / Store
//   Branch
//   JAL / JALR
//   LUI / AUIPC
//
// This is a multi-cycle FSM processor, NOT a pipeline.
//==============================================================

module riscv_core (
    input  wire        clk,
    input  wire        reset,

    //==========================================================
    // MEMORY-MAPPED BUS INTERFACE
    //==========================================================

    output reg  [63:0] bus_addr,
    output reg  [63:0] bus_wdata,
    input  wire [63:0] bus_rdata,

    output reg         bus_read,
    output reg         bus_write,
    input  wire        bus_ready
);

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
    // Must match alu.v and control_fsm.v
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
    // PROCESSOR REGISTERS
    //==========================================================

    reg [63:0] pc;

    reg [31:0] instruction;

    reg [63:0] memory_address_register;
    reg [63:0] memory_data_register;

    reg [63:0] alu_result_register;

    //==========================================================
    // INSTRUCTION FIELDS
    //==========================================================

    wire [6:0] opcode;
    wire [4:0] rd;
    wire [2:0] funct3;
    wire [4:0] rs1;
    wire [4:0] rs2;
    wire [6:0] funct7;

    assign opcode = instruction[6:0];
    assign rd     = instruction[11:7];
    assign funct3 = instruction[14:12];
    assign rs1    = instruction[19:15];
    assign rs2    = instruction[24:20];
    assign funct7 = instruction[31:25];

    //==========================================================
    // CONTROL FSM SIGNALS
    //==========================================================

    wire       fsm_pc_write;
    wire       fsm_ir_write;
    wire       fsm_reg_write;

    wire       fsm_mem_read;
    wire       fsm_mem_write;

    wire       fsm_alu_src_a;
    wire       fsm_alu_src_b;

    wire       fsm_branch_enable;
    wire       fsm_jump_enable;

    wire [3:0] fsm_alu_control;

    wire [2:0] current_state;

    //==========================================================
    // REGISTER FILE SIGNALS
    //==========================================================

    wire [63:0] rs1_data;
    wire [63:0] rs2_data;

    reg         register_write_enable;
    reg  [4:0]  register_write_address;
    reg  [63:0] register_write_data;

    //==========================================================
    // IMMEDIATE
    //==========================================================

    wire [63:0] immediate;

    //==========================================================
    // ALU SIGNALS
    //==========================================================

    reg  [63:0] alu_input_a;
    reg  [63:0] alu_input_b;

    wire [63:0] alu_result;
    wire        alu_zero;

    //==========================================================
    // CONTROL FSM
    //==========================================================

    control_fsm control_unit (
        .clk            (clk),
        .reset          (reset),

        .opcode         (opcode),
        .funct3         (funct3),
        .funct7         (funct7),

        .alu_zero       (alu_zero),

        .pc_write       (fsm_pc_write),
        .ir_write       (fsm_ir_write),
        .reg_write      (fsm_reg_write),

        .mem_read       (fsm_mem_read),
        .mem_write      (fsm_mem_write),

        .alu_src_a      (fsm_alu_src_a),
        .alu_src_b      (fsm_alu_src_b),

        .branch_enable  (fsm_branch_enable),
        .jump_enable    (fsm_jump_enable),

        .alu_control    (fsm_alu_control),

        .current_state  (current_state)
    );

    //==========================================================
    // REGISTER FILE
    //==========================================================

    register_file registers (
        .clk        (clk),
        .reset      (reset),

        .rs1_addr    (rs1),
        .rs2_addr    (rs2),

        .rs1_data    (rs1_data),
        .rs2_data    (rs2_data),

        .reg_write   (register_write_enable),
        .rd_addr     (register_write_address),
        .rd_data     (register_write_data)
    );

    //==========================================================
    // IMMEDIATE GENERATOR
    //==========================================================

    immediate_generator immediate_unit (
        .instruction (instruction),
        .immediate   (immediate)
    );

    //==========================================================
    // ALU
    //==========================================================

    alu arithmetic_logic_unit (
        .a            (alu_input_a),
        .b            (alu_input_b),
        .alu_control  (fsm_alu_control),

        .result       (alu_result),
        .zero         (alu_zero)
    );

    //==========================================================
    // ALU INPUT SELECTION
    //==========================================================

    always @(*) begin

        // ALU input A
        if (fsm_alu_src_a)
            alu_input_a = rs1_data;
        else
            alu_input_a = pc;

        // ALU input B
        if (fsm_alu_src_b)
            alu_input_b = immediate;
        else
            alu_input_b = rs2_data;

    end

    //==========================================================
    // REGISTER WRITEBACK CONTROL
    //==========================================================

    always @(*) begin

        register_write_enable = 1'b0;
        register_write_address = 5'd0;
        register_write_data = 64'd0;

        if (fsm_reg_write) begin

            register_write_enable = 1'b1;
            register_write_address = rd;

            case (opcode)

                // R-type
                OPCODE_OP:
                    register_write_data = alu_result_register;

                // I-type ALU
                OPCODE_OPIMM:
                    register_write_data = alu_result_register;

                // Load
                OPCODE_LOAD:
                    register_write_data = memory_data_register;

                // LUI
                OPCODE_LUI:
                    register_write_data = immediate;

                // AUIPC
                OPCODE_AUIPC:
                    register_write_data = pc + immediate;

                // JAL
                OPCODE_JAL:
                    register_write_data = pc;

                // JALR
                OPCODE_JALR:
                    register_write_data = pc;

                default:
                    register_write_data = 64'd0;

            endcase

        end

    end

    //==========================================================
    // MEMORY-MAPPED BUS CONTROL
    //==========================================================

    always @(*) begin

        bus_addr  = 64'd0;
        bus_wdata = 64'd0;
        bus_read  = 1'b0;
        bus_write = 1'b0;

        case (current_state)

            //==================================================
            // FETCH
            //==================================================

            3'b000: begin

                bus_addr = pc;
                bus_read = 1'b1;

            end

            //==================================================
            // MEMORY
            //==================================================

            3'b011: begin

                bus_addr = memory_address_register;

                // LOAD
                if (opcode == OPCODE_LOAD)
                    bus_read = 1'b1;

                // STORE
                if (opcode == OPCODE_STORE) begin

                    bus_write = 1'b1;
                    bus_wdata = rs2_data;

                end

            end

            default: begin

                bus_addr  = 64'd0;
                bus_wdata = 64'd0;
                bus_read  = 1'b0;
                bus_write = 1'b0;

            end

        endcase

    end

    //==========================================================
    // SEQUENTIAL PROCESSOR LOGIC
    //==========================================================

    always @(posedge clk) begin

        if (reset) begin

            pc <= 64'd0;

            instruction <= 32'd0;

            memory_address_register <= 64'd0;
            memory_data_register <= 64'd0;

            alu_result_register <= 64'd0;

        end
        else begin

            //==================================================
            // FETCH
            //==================================================

            if (current_state == 3'b000) begin

                if (bus_ready) begin

                    instruction <= bus_rdata;

                    // Normal instruction advance
                    pc <= pc + 64'd4;

                end

            end


            //==================================================
            // EXECUTE
            //==================================================

            if (current_state == 3'b010) begin

                // Save ALU result
                alu_result_register <= alu_result;

                //================================================
                // LOAD / STORE ADDRESS CALCULATION
                //================================================

                if ((opcode == OPCODE_LOAD) ||
                    (opcode == OPCODE_STORE)) begin

                    memory_address_register <=
                        rs1_data + immediate;

                end

                //================================================
                // BRANCH
                //================================================

                if (opcode == OPCODE_BRANCH) begin

                    case (funct3)

                        // BEQ
                        3'b000: begin
                            if (rs1_data == rs2_data)
                                pc <= pc + immediate;
                        end

                        // BNE
                        3'b001: begin
                            if (rs1_data != rs2_data)
                                pc <= pc + immediate;
                        end

                        // BLT
                        3'b100: begin
                            if ($signed(rs1_data) <
                                $signed(rs2_data))
                                pc <= pc + immediate;
                        end

                        // BGE
                        3'b101: begin
                            if ($signed(rs1_data) >=
                                $signed(rs2_data))
                                pc <= pc + immediate;
                        end

                        // BLTU
                        3'b110: begin
                            if (rs1_data < rs2_data)
                                pc <= pc + immediate;
                        end

                        // BGEU
                        3'b111: begin
                            if (rs1_data >= rs2_data)
                                pc <= pc + immediate;
                        end

                        default:
                            pc <= pc;

                    endcase

                end

                //================================================
                // JAL
                //================================================

                if (opcode == OPCODE_JAL) begin

                    // PC was already advanced by 4 during fetch.
                    // Save return address and jump.
                    pc <= pc + immediate;

                end

                //================================================
                // JALR
                //================================================

                if (opcode == OPCODE_JALR) begin

                    pc <= (rs1_data + immediate) & ~64'd1;

                end

            end


            //==================================================
            // MEMORY READ
            //==================================================

            if (current_state == 3'b011) begin

                if ((opcode == OPCODE_LOAD) && bus_ready)
                    memory_data_register <= bus_rdata;

            end

        end

    end

endmodule