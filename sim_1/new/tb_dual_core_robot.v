
`timescale 1ns / 1ps

module tb_dual_core_robot_final;

    reg clk;
    reg reset;

    wire pwm1;
    wire pwm2;
    wire pwm3;

    // DUT
    dual_core_robot_top dut (
        .clk   (clk),
        .reset (reset),
        .pwm1  (pwm1),
        .pwm2  (pwm2),
        .pwm3  (pwm3)
    );

    // Exactly 11 selected waveform signals
    wire [63:0] last_pc0;
    wire        ik_start;
    wire        ik_done;
    wire        ik_error;
    wire [63:0] theta1;
    wire [63:0] theta2;
    wire [63:0] theta3;

    assign last_pc0 = dut.core0.pc;

    assign ik_start =
        dut.ik_write &&
        (dut.ik_addr == 64'h1028) &&
        dut.ik_wdata[0];

    assign ik_done  = dut.ik_done;
    assign ik_error = dut.ik_error;

    assign theta1 = dut.ik_unit.theta1_reg;
    assign theta2 = dut.ik_unit.theta2_reg;
    assign theta3 = dut.ik_unit.theta3_reg;

    // 100 MHz clock
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Reset and simulation duration
    initial begin
        reset = 1'b1;

        repeat (5) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;

        repeat (5000) @(posedge clk);
        #1;

        $display("Simulation finished at %0t ns", $time);
        $display("Core 0 PC: %h", last_pc0);
        $display("IK start: %b, done: %b, error: %b",
                 ik_start, ik_done, ik_error);
        $display("Theta1=%0d Theta2=%0d Theta3=%0d",
                 theta1, theta2, theta3);
        $display("PWM1=%b PWM2=%b PWM3=%b",
                 pwm1, pwm2, pwm3);

        $finish;
    end

endmodule
