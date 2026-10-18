`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.10.2026 21:30:47
// Design Name: 
// Module Name: dual_core_robot_top
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
// Dual-Core RISC-V Processor with 3D Robotic Arm Control
//
// TOP LEVEL MODULE
//
// Architecture:
//   Core 0
//   Core 1
//       |
//       v
//   Memory Interconnect
//       |
//       +---- Shared Memory
//       +---- 3D IK Accelerator
//       +---- PWM Controller
//
// Language: Verilog-2001
//==============================================================
//==============================================================
// Dual-Core RISC-V Processor with 3D Robotic Arm Control
//
// TOP LEVEL MODULE
//
// Architecture:
//   Core 0
//   Core 1
//       |
//       v
//   Memory Interconnect
//       |
//       +---- Shared Memory
//       +---- 3D IK Accelerator
//       +---- PWM Controller
//
// Language: Verilog-2001
//==============================================================

module dual_core_robot_top (
    input  wire clk,
    input  wire reset,

    // Digital PWM outputs
    output wire pwm1,
    output wire pwm2,
    output wire pwm3
);

    //==========================================================
    // CORE 0 BUS
    //==========================================================

    wire [63:0] core0_addr;
    wire [63:0] core0_wdata;
    wire [63:0] core0_rdata;

    wire        core0_read;
    wire        core0_write;
    wire        core0_ready;


    //==========================================================
    // CORE 1 BUS
    //==========================================================

    wire [63:0] core1_addr;
    wire [63:0] core1_wdata;
    wire [63:0] core1_rdata;

    wire        core1_read;
    wire        core1_write;
    wire        core1_ready;


    //==========================================================
    // SHARED MEMORY BUS
    //==========================================================

    wire [63:0] memory_addr;
    wire [63:0] memory_wdata;
    wire [63:0] memory_rdata;

    wire        memory_read;
    wire        memory_write;


    //==========================================================
    // IK ACCELERATOR BUS
    //==========================================================

    wire [63:0] ik_addr;
    wire [63:0] ik_wdata;
    wire [63:0] ik_rdata;

    wire        ik_read;
    wire        ik_write;

    wire        ik_busy;
    wire        ik_done;
    wire        ik_error;


    //==========================================================
    // PWM CONTROLLER BUS
    //==========================================================

    wire [63:0] pwm_addr;
    wire [63:0] pwm_wdata;
    wire [63:0] pwm_rdata;

    wire        pwm_read;
    wire        pwm_write;


    //==========================================================
    // CORE 0
    //==========================================================

    riscv_core core0 (
        .clk        (clk),
        .reset      (reset),

        .bus_addr   (core0_addr),
        .bus_wdata  (core0_wdata),
        .bus_rdata  (core0_rdata),

        .bus_read   (core0_read),
        .bus_write  (core0_write),
        .bus_ready  (core0_ready)
    );


    //==========================================================
    // CORE 1
    //==========================================================

    riscv_core core1 (
        .clk        (clk),
        .reset      (reset),

        .bus_addr   (core1_addr),
        .bus_wdata  (core1_wdata),
        .bus_rdata  (core1_rdata),

        .bus_read   (core1_read),
        .bus_write  (core1_write),
        .bus_ready  (core1_ready)
    );


    //==========================================================
    // MEMORY INTERCONNECT
    //==========================================================

    memory_interconnect interconnect (
        .clk        (clk),
        .reset      (reset),

        // Core 0
        .core0_addr   (core0_addr),
        .core0_wdata  (core0_wdata),
        .core0_rdata  (core0_rdata),
        .core0_read   (core0_read),
        .core0_write  (core0_write),
        .core0_ready  (core0_ready),

        // Core 1
        .core1_addr   (core1_addr),
        .core1_wdata  (core1_wdata),
        .core1_rdata  (core1_rdata),
        .core1_read   (core1_read),
        .core1_write  (core1_write),
        .core1_ready  (core1_ready),

        // Shared memory
        .memory_addr   (memory_addr),
        .memory_wdata  (memory_wdata),
        .memory_rdata  (memory_rdata),
        .memory_read   (memory_read),
        .memory_write  (memory_write),

        // IK accelerator
        .ik_addr       (ik_addr),
        .ik_wdata      (ik_wdata),
        .ik_rdata      (ik_rdata),
        .ik_read       (ik_read),
        .ik_write      (ik_write),

        // PWM controller
        .pwm_addr      (pwm_addr),
        .pwm_wdata     (pwm_wdata),
        .pwm_rdata     (pwm_rdata),
        .pwm_read      (pwm_read),
        .pwm_write     (pwm_write)
    );


    //==========================================================
    // SHARED MEMORY
    //==========================================================

    shared_memory memory (
        .clk        (clk),
        .reset      (reset),

        .addr       (memory_addr),
        .wdata      (memory_wdata),
        .rdata      (memory_rdata),

        .read_en    (memory_read),
        .write_en   (memory_write)
    );


    //==========================================================
    // 3D IK ACCELERATOR
    //==========================================================

    ik_accelerator ik_unit (
        .clk        (clk),
        .reset      (reset),

        .addr       (ik_addr),
        .wdata      (ik_wdata),
        .rdata      (ik_rdata),

        .read_en    (ik_read),
        .write_en   (ik_write),

        .busy       (ik_busy),
        .done       (ik_done),
        .error      (ik_error)
    );


    //==========================================================
    // PWM CONTROLLER
    //==========================================================

    pwm_controller pwm_unit (
        .clk        (clk),
        .reset      (reset),

        .addr       (pwm_addr),
        .wdata      (pwm_wdata),
        .rdata      (pwm_rdata),

        .read_en    (pwm_read),
        .write_en   (pwm_write),

        .pwm1       (pwm1),
        .pwm2       (pwm2),
        .pwm3       (pwm3)
    );

endmodule
