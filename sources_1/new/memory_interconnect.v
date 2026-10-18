`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.10.2026 21:21:38
// Design Name: 
// Module Name: memory_interconnect
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
// Memory-Mapped Interconnect
// Dual-Core RISC-V Robotic Control System
//
// Verilog-2001
//
// Address Map:
//
// 0x0000 - 0x0FFF : Shared Memory
// 0x1000 - 0x104F : 3D IK Accelerator
// 0x2000 - 0x2017 : PWM Controller
//
// Core 0 has priority when both cores request the bus.
//==============================================================

module memory_interconnect (

    input  wire        clk,
    input  wire        reset,

    //==========================================================
    // CORE 0 INTERFACE
    //==========================================================

    input  wire [63:0] core0_addr,
    input  wire [63:0] core0_wdata,
    output reg  [63:0] core0_rdata,
    input  wire        core0_read,
    input  wire        core0_write,
    output reg         core0_ready,

    //==========================================================
    // CORE 1 INTERFACE
    //==========================================================

    input  wire [63:0] core1_addr,
    input  wire [63:0] core1_wdata,
    output reg  [63:0] core1_rdata,
    input  wire        core1_read,
    input  wire        core1_write,
    output reg         core1_ready,

    //==========================================================
    // SHARED MEMORY INTERFACE
    //==========================================================

    output reg  [63:0] memory_addr,
    output reg  [63:0] memory_wdata,
    input  wire [63:0] memory_rdata,

    output reg         memory_read,
    output reg         memory_write,

    //==========================================================
    // IK ACCELERATOR INTERFACE
    //==========================================================

    output reg  [63:0] ik_addr,
    output reg  [63:0] ik_wdata,
    input  wire [63:0] ik_rdata,

    output reg         ik_read,
    output reg         ik_write,

    //==========================================================
    // PWM CONTROLLER INTERFACE
    //==========================================================

    output reg  [63:0] pwm_addr,
    output reg  [63:0] pwm_wdata,
    input  wire [63:0] pwm_rdata,

    output reg         pwm_read,
    output reg         pwm_write
);

    //==========================================================
    // ADDRESS RANGES
    //==========================================================

    parameter MEMORY_BASE = 64'h0000_0000_0000_0000;
    parameter MEMORY_END  = 64'h0000_0000_0000_0FFF;

    parameter IK_BASE     = 64'h0000_0000_0000_1000;
    parameter IK_END      = 64'h0000_0000_0000_104F;

    parameter PWM_BASE    = 64'h0000_0000_0000_2000;
    parameter PWM_END     = 64'h0000_0000_0000_2017;

    //==========================================================
    // SELECTED CORE
    //
    // Core 0 gets priority if both request simultaneously.
    //==========================================================

    reg selected_core;

    always @(*) begin

        if (core0_read || core0_write)
            selected_core = 1'b0;

        else if (core1_read || core1_write)
            selected_core = 1'b1;

        else
            selected_core = 1'b0;

    end

    //==========================================================
    // INTERCONNECT
    //==========================================================

    always @(*) begin

        //======================================================
        // DEFAULT VALUES
        //======================================================

        core0_rdata = 64'd0;
        core1_rdata = 64'd0;

        core0_ready = 1'b0;
        core1_ready = 1'b0;

        memory_addr  = 64'd0;
        memory_wdata = 64'd0;
        memory_read  = 1'b0;
        memory_write = 1'b0;

        ik_addr  = 64'd0;
        ik_wdata = 64'd0;
        ik_read  = 1'b0;
        ik_write = 1'b0;

        pwm_addr  = 64'd0;
        pwm_wdata = 64'd0;
        pwm_read  = 1'b0;
        pwm_write = 1'b0;

        //======================================================
        // CORE 0 SELECTED
        //======================================================

        if (selected_core == 1'b0) begin

            //==================================================
            // SHARED MEMORY
            //==================================================

            if ((core0_addr >= MEMORY_BASE) &&
                (core0_addr <= MEMORY_END)) begin

                memory_addr  = core0_addr;
                memory_wdata = core0_wdata;
                memory_read  = core0_read;
                memory_write = core0_write;

                core0_rdata = memory_rdata;
                core0_ready = 1'b1;

            end

            //==================================================
            // IK ACCELERATOR
            //==================================================

            else if ((core0_addr >= IK_BASE) &&
                     (core0_addr <= IK_END)) begin

                ik_addr  = core0_addr;
                ik_wdata = core0_wdata;
                ik_read  = core0_read;
                ik_write = core0_write;

                core0_rdata = ik_rdata;
                core0_ready = 1'b1;

            end

            //==================================================
            // PWM CONTROLLER
            //==================================================

            else if ((core0_addr >= PWM_BASE) &&
                     (core0_addr <= PWM_END)) begin

                pwm_addr  = core0_addr;
                pwm_wdata = core0_wdata;
                pwm_read  = core0_read;
                pwm_write = core0_write;

                core0_rdata = pwm_rdata;
                core0_ready = 1'b1;

            end

        end

        //======================================================
        // CORE 1 SELECTED
        //======================================================

        else begin

            //==================================================
            // SHARED MEMORY
            //==================================================

            if ((core1_addr >= MEMORY_BASE) &&
                (core1_addr <= MEMORY_END)) begin

                memory_addr  = core1_addr;
                memory_wdata = core1_wdata;
                memory_read  = core1_read;
                memory_write = core1_write;

                core1_rdata = memory_rdata;
                core1_ready = 1'b1;

            end

            //==================================================
            // IK ACCELERATOR
            //==================================================

            else if ((core1_addr >= IK_BASE) &&
                     (core1_addr <= IK_END)) begin

                ik_addr  = core1_addr;
                ik_wdata = core1_wdata;
                ik_read  = core1_read;
                ik_write = core1_write;

                core1_rdata = ik_rdata;
                core1_ready = 1'b1;

            end

            //==================================================
            // PWM CONTROLLER
            //==================================================

            else if ((core1_addr >= PWM_BASE) &&
                     (core1_addr <= PWM_END)) begin

                pwm_addr  = core1_addr;
                pwm_wdata = core1_wdata;
                pwm_read  = core1_read;
                pwm_write = core1_write;

                core1_rdata = pwm_rdata;
                core1_ready = 1'b1;

            end

        end

    end

endmodule