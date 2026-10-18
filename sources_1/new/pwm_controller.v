`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.10.2026 21:28:51
// Design Name: 
// Module Name: pwm_controller
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
// 3-Channel PWM Controller
// Dual-Core RISC-V 3D Robotic Arm
//
// Verilog-2001
//
// Address Map:
//   0x2000 : Joint 1 angle
//   0x2008 : Joint 2 angle
//   0x2010 : Joint 3 angle
//
// Output:
//   pwm1, pwm2, pwm3
//
// Angle input: 0 to 180 degrees
//==============================================================

module pwm_controller (
    input  wire        clk,
    input  wire        reset,

    input  wire [63:0] addr,
    input  wire [63:0] wdata,
    output reg  [63:0] rdata,

    input  wire        read_en,
    input  wire        write_en,

    output reg         pwm1,
    output reg         pwm2,
    output reg         pwm3
);

    //==========================================================
    // ANGLE REGISTERS
    //==========================================================

    reg [63:0] joint1_angle;
    reg [63:0] joint2_angle;
    reg [63:0] joint3_angle;

    //==========================================================
    // PWM COUNTER
    //==========================================================

    reg [7:0] pwm_counter;

    //==========================================================
    // PWM DUTY CYCLE
    //
    // 0 degrees   -> 0
    // 180 degrees -> 255
    //
    // duty = angle * 255 / 180
    //==========================================================

    reg [15:0] duty1;
    reg [15:0] duty2;
    reg [15:0] duty3;

    always @(*) begin

        duty1 = (joint1_angle * 255) / 180;
        duty2 = (joint2_angle * 255) / 180;
        duty3 = (joint3_angle * 255) / 180;

    end

    //==========================================================
    // MEMORY-MAPPED WRITE
    //==========================================================

    always @(posedge clk) begin

        if (reset) begin

            joint1_angle <= 64'd0;
            joint2_angle <= 64'd0;
            joint3_angle <= 64'd0;

        end
        else if (write_en) begin

            case (addr)

                // Joint 1
                64'h0000_0000_0000_2000:
                    joint1_angle <= wdata;

                // Joint 2
                64'h0000_0000_0000_2008:
                    joint2_angle <= wdata;

                // Joint 3
                64'h0000_0000_0000_2010:
                    joint3_angle <= wdata;

                default:
                    begin
                    end

            endcase

        end

    end

    //==========================================================
    // MEMORY-MAPPED READ
    //==========================================================

    always @(*) begin

        rdata = 64'd0;

        if (read_en) begin

            case (addr)

                64'h0000_0000_0000_2000:
                    rdata = joint1_angle;

                64'h0000_0000_0000_2008:
                    rdata = joint2_angle;

                64'h0000_0000_0000_2010:
                    rdata = joint3_angle;

                default:
                    rdata = 64'd0;

            endcase

        end

    end

    //==========================================================
    // PWM COUNTER
    //==========================================================

    always @(posedge clk) begin

        if (reset)
            pwm_counter <= 8'd0;

        else
            pwm_counter <= pwm_counter + 8'd1;

    end

    //==========================================================
    // PWM OUTPUT GENERATION
    //==========================================================

    always @(*) begin

        if (pwm_counter < duty1)
            pwm1 = 1'b1;
        else
            pwm1 = 1'b0;

        if (pwm_counter < duty2)
            pwm2 = 1'b1;
        else
            pwm2 = 1'b0;

        if (pwm_counter < duty3)
            pwm3 = 1'b1;
        else
            pwm3 = 1'b0;

    end

endmodule