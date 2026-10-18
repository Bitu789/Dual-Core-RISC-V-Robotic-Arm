`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.10.2026 19:44:34
// Design Name: 
// Module Name: ik_accelerator
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
// 3D Inverse Kinematics Hardware Accelerator
//
// Project:
// Dual-Core RISC-V Processor with Hardware-Accelerated
// 3D Robotic Arm Control
//
// Language: Verilog-2001
//
// Inputs:
//   X, Y, Z : target position
//   L1, L2  : arm link lengths
//
// Outputs:
//   theta1 : base angle
//   theta2 : shoulder angle
//   theta3 : elbow angle
//
// Angles are represented in integer degrees.
//
// Memory Map:
//   +0x00 : X
//   +0x08 : Y
//   +0x10 : Z
//   +0x18 : L1
//   +0x20 : L2
//   +0x28 : START
//   +0x30 : STATUS
//   +0x38 : THETA1
//   +0x40 : THETA2
//   +0x48 : THETA3
//
// STATUS:
//   bit 0 = DONE
//   bit 1 = BUSY
//   bit 2 = ERROR
//
//==============================================================

module ik_accelerator (
    input  wire        clk,
    input  wire        reset,

    input  wire [63:0] addr,
    input  wire [63:0] wdata,
    output reg  [63:0] rdata,

    input  wire        read_en,
    input  wire        write_en,

    output reg         busy,
    output reg         done,
    output reg         error
);

    //==========================================================
    // INPUT REGISTERS
    //==========================================================

    reg signed [63:0] x_reg;
    reg signed [63:0] y_reg;
    reg signed [63:0] z_reg;

    reg signed [63:0] l1_reg;
    reg signed [63:0] l2_reg;

    //==========================================================
    // OUTPUT REGISTERS
    //==========================================================

    reg signed [63:0] theta1_reg;
    reg signed [63:0] theta2_reg;
    reg signed [63:0] theta3_reg;

    //==========================================================
    // INTERNAL CALCULATION REGISTERS
    //==========================================================

    reg signed [63:0] radial_distance;
    reg signed [63:0] distance_squared;

    reg signed [63:0] numerator;
    reg signed [63:0] denominator;

    reg signed [63:0] d_value;

    //==========================================================
    // FSM STATES
    //==========================================================

    parameter STATE_IDLE       = 4'd0;
    parameter STATE_CALC_R     = 4'd1;
    parameter STATE_CHECK      = 4'd2;
    parameter STATE_THETA1     = 4'd3;
    parameter STATE_THETA3     = 4'd4;
    parameter STATE_THETA2     = 4'd5;
    parameter STATE_DONE       = 4'd6;
    parameter STATE_ERROR      = 4'd7;

    reg [3:0] state;

    //==========================================================
    // START REGISTER WRITE
    //==========================================================

    wire start_command;

    assign start_command =
        write_en &&
        (addr == 64'h0000_0000_0000_1028) &&
        (wdata[0] == 1'b1);

    //==========================================================
    // INTEGER ABSOLUTE VALUE FUNCTION
    //==========================================================

    function signed [63:0] abs_value;
        input signed [63:0] value;

        begin
            if (value < 0)
                abs_value = -value;
            else
                abs_value = value;
        end
    endfunction

    //==========================================================
    // INTEGER SQUARE ROOT
    //
    // Returns floor(sqrt(value))
    //
    // Example:
    // sqrt(12500) = 111.80...
    // Function returns 111.
    //
    //==========================================================

    function [63:0] integer_sqrt;
        input [127:0] value;

        reg [63:0] root;
        reg [63:0] trial_root;
        reg [127:0] trial_square;

        integer i;

        begin
            root = 64'd0;

            for (i = 63; i >= 0; i = i - 1) begin

                trial_root = root | (64'b1 << i);

                trial_square = trial_root * trial_root;

                if (trial_square <= value)
                    root = trial_root;

            end

            integer_sqrt = root;
        end

    endfunction

    //==========================================================
    // APPROXIMATE ATAN2
    //
    // Returns angle in degrees.
    //
    // This is an integer approximation intended for
    // RTL demonstration/control.
    //
    //==========================================================

    function signed [63:0] atan2_deg;
        input signed [63:0] y;
        input signed [63:0] x;

        reg signed [63:0] ax;
        reg signed [63:0] ay;
        reg signed [63:0] angle;

        begin

            ax = abs_value(x);
            ay = abs_value(y);

            // Both zero
            if ((ax == 0) && (ay == 0)) begin

                angle = 0;

            end

            // X = 0
            else if (ax == 0) begin

                if (y > 0)
                    angle = 90;
                else
                    angle = -90;

            end

            // Normal case
            else begin

                // Piecewise approximation
                if (ay <= ax)
                    angle = (ay * 45) / ax;
                else
                    angle = 90 - ((ax * 45) / ay);

                // Quadrant correction

                if ((x >= 0) && (y >= 0)) begin
                    angle = angle;
                end

                else if ((x < 0) && (y >= 0)) begin
                    angle = 180 - angle;
                end

                else if ((x < 0) && (y < 0)) begin
                    angle = -180 + angle;
                end

                else begin
                    angle = -angle;
                end

            end

            atan2_deg = angle;

        end

    endfunction

    //==========================================================
    // APPROXIMATE ACOS
    //
    // Input:
    //   d approximately -100 to +100
    //
    // Internally d represents D * 100.
    //
    // Output:
    //   Angle in degrees.
    //
    //==========================================================

    function signed [63:0] acos_deg;
        input signed [63:0] d;

        reg signed [63:0] value;
        reg signed [63:0] angle;

        begin

            value = d;

            if (value >= 100)
                angle = 0;

            else if (value <= -100)
                angle = 180;

            else if (value >= 90)
                angle = 26;

            else if (value >= 80)
                angle = 37;

            else if (value >= 70)
                angle = 46;

            else if (value >= 60)
                angle = 53;

            else if (value >= 50)
                angle = 60;

            else if (value >= 40)
                angle = 66;

            else if (value >= 30)
                angle = 73;

            else if (value >= 20)
                angle = 78;

            else if (value >= 10)
                angle = 84;

            else if (value >= 0)
                angle = 90;

            else if (value >= -10)
                angle = 96;

            else if (value >= -20)
                angle = 102;

            else if (value >= -30)
                angle = 107;

            else if (value >= -40)
                angle = 114;

            else if (value >= -50)
                angle = 120;

            else if (value >= -60)
                angle = 127;

            else if (value >= -70)
                angle = 134;

            else if (value >= -80)
                angle = 143;

            else if (value >= -90)
                angle = 154;

            else
                angle = 180;

            acos_deg = angle;

        end

    endfunction

    //==========================================================
    // MEMORY-MAPPED READ INTERFACE
    //==========================================================

    always @(*) begin

        rdata = 64'd0;

        if (read_en) begin

            case (addr)

                // X
                64'h0000_0000_0000_1000:
                    rdata = x_reg;

                // Y
                64'h0000_0000_0000_1008:
                    rdata = y_reg;

                // Z
                64'h0000_0000_0000_1010:
                    rdata = z_reg;

                // L1
                64'h0000_0000_0000_1018:
                    rdata = l1_reg;

                // L2
                64'h0000_0000_0000_1020:
                    rdata = l2_reg;

                // STATUS
                64'h0000_0000_0000_1030: begin

                    rdata = 64'd0;

                    rdata[0] = done;
                    rdata[1] = busy;
                    rdata[2] = error;

                end

                // THETA1
                64'h0000_0000_0000_1038:
                    rdata = theta1_reg;

                // THETA2
                64'h0000_0000_0000_1040:
                    rdata = theta2_reg;

                // THETA3
                64'h0000_0000_0000_1048:
                    rdata = theta3_reg;

                default:
                    rdata = 64'd0;

            endcase

        end

    end

    //==========================================================
    // IK PROCESSING FSM
    //==========================================================

   always @(posedge clk or posedge reset) begin

        if (reset) begin

            // Input registers
            x_reg <= 64'd0;
            y_reg <= 64'd0;
            z_reg <= 64'd0;

            l1_reg <= 64'd0;
            l2_reg <= 64'd0;

            // Output registers
            theta1_reg <= 64'd0;
            theta2_reg <= 64'd0;
            theta3_reg <= 64'd0;

            // Calculation registers
            radial_distance <= 64'd0;
            distance_squared <= 64'd0;

            numerator <= 64'd0;
            denominator <= 64'd0;
            d_value <= 64'd0;

            // Status
            busy <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;

            // FSM
            state <= STATE_IDLE;

        end

        else begin

            //==================================================
            // INPUT REGISTER WRITES
            //==================================================

            if (write_en) begin

                case (addr)

                    // X
                    64'h0000_0000_0000_1000:
                        x_reg <= wdata;

                    // Y
                    64'h0000_0000_0000_1008:
                        y_reg <= wdata;

                    // Z
                    64'h0000_0000_0000_1010:
                        z_reg <= wdata;

                    // L1
                    64'h0000_0000_0000_1018:
                        l1_reg <= wdata;

                    // L2
                    64'h0000_0000_0000_1020:
                        l2_reg <= wdata;

                    default: begin
                    end

                endcase

            end

            //==================================================
            // PROCESSING FSM
            //==================================================

            case (state)

                //================================================
                // IDLE
                //================================================

                STATE_IDLE: begin

                    busy <= 1'b0;

                    if (start_command) begin

                        busy <= 1'b1;
                        done <= 1'b0;
                        error <= 1'b0;

                        state <= STATE_CALC_R;

                    end

                end

                //================================================
                // CALCULATE RADIAL DISTANCE
                //
                // R = sqrt(X² + Y²)
                //================================================

                STATE_CALC_R: begin

                    radial_distance <=
                        integer_sqrt(
                            (x_reg * x_reg) +
                            (y_reg * y_reg)
                        );

                    state <= STATE_CHECK;

                end

                //================================================
                // REACHABILITY CHECK
                //
                // Minimum reach = |L1-L2|
                // Maximum planar reach = L1+L2
                //
                // The existing architecture uses radial distance
                // for this initial reachability check.
                //================================================

                STATE_CHECK: begin

                    distance_squared <=
                        (radial_distance * radial_distance) +
                        (z_reg * z_reg);

                    // Invalid link length
                    if ((l1_reg == 0) || (l2_reg == 0)) begin

                        error <= 1'b1;
                        busy <= 1'b0;

                        state <= STATE_ERROR;

                    end

                    // Too far
                    else if (radial_distance >
                             (l1_reg + l2_reg)) begin

                        error <= 1'b1;
                        busy <= 1'b0;

                        state <= STATE_ERROR;

                    end

                    // Too close
                    else if (radial_distance <
                             abs_value(l1_reg - l2_reg)) begin

                        error <= 1'b1;
                        busy <= 1'b0;

                        state <= STATE_ERROR;

                    end

                    // Valid
                    else begin

                        state <= STATE_THETA1;

                    end

                end

                //================================================
                // BASE ANGLE
                //
                // theta1 = atan2(Y,X)
                //================================================

                STATE_THETA1: begin

                    theta1_reg <=
                        atan2_deg(
                            y_reg,
                            x_reg
                        );

                    state <= STATE_THETA3;

                end

                //================================================
                // ELBOW ANGLE
                //
                // D =
                // (R² + Z² - L1² - L2²)
                // --------------------
                //       2L1L2
                //
                // D is scaled by 100.
                //================================================

                STATE_THETA3: begin

                    numerator =
                        (radial_distance * radial_distance) +
                        (z_reg * z_reg) -
                        (l1_reg * l1_reg) -
                        (l2_reg * l2_reg);

                    denominator =
                        2 * l1_reg * l2_reg;

                    if (denominator != 0) begin

                        d_value =
                            (numerator * 100) /
                            denominator;

                        // Clamp D to valid acos range
                        if (d_value > 100)
                            d_value = 100;

                        else if (d_value < -100)
                            d_value = -100;

                        theta3_reg <=
                            acos_deg(d_value);

                        state <= STATE_THETA2;

                    end

                    else begin

                        error <= 1'b1;
                        busy <= 1'b0;

                        state <= STATE_ERROR;

                    end

                end

                //================================================
                // SHOULDER ANGLE
                //
                // Simplified hardware approximation:
                //
                // theta2 = atan2(Z,R)
                //
                //================================================

                STATE_THETA2: begin

                    theta2_reg <=
                        atan2_deg(
                            z_reg,
                            radial_distance
                        );

                    state <= STATE_DONE;

                end

                //================================================
                // DONE
                //================================================

                STATE_DONE: begin

                    busy <= 1'b0;
                    done <= 1'b1;
                    error <= 1'b0;

                    state <= STATE_IDLE;

                end

                //================================================
                // ERROR
                //================================================

                STATE_ERROR: begin

                    busy <= 1'b0;
                    done <= 1'b0;
                    error <= 1'b1;

                    state <= STATE_IDLE;

                end

                //================================================
                // DEFAULT
                //================================================

                default: begin

                    busy <= 1'b0;
                    done <= 1'b0;
                    error <= 1'b0;

                    state <= STATE_IDLE;

                end

            endcase

        end

    end

endmodule