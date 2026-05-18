`timescale 1ns / 1ps

// NOTE: Purely combinational 9-MAC tree. On Zynq-7020, Vivado maps
// multiplications to DSP48E1 slices (>300MHz capability), so 100MHz
// timing closure is expected without explicit pipeline registers.
// If timing violations occur, consider adding a 1-stage pipeline
// (register acc_out) and adjusting the FSM in dronet_compute_engine
// to account for the 1-cycle output latency.
module dronet_conv3x3_pe (
    input  wire signed [7:0]  px00,
    input  wire signed [7:0]  px01,
    input  wire signed [7:0]  px02,
    input  wire signed [7:0]  px10,
    input  wire signed [7:0]  px11,
    input  wire signed [7:0]  px12,
    input  wire signed [7:0]  px20,
    input  wire signed [7:0]  px21,
    input  wire signed [7:0]  px22,
    input  wire signed [7:0]  wt00,
    input  wire signed [7:0]  wt01,
    input  wire signed [7:0]  wt02,
    input  wire signed [7:0]  wt10,
    input  wire signed [7:0]  wt11,
    input  wire signed [7:0]  wt12,
    input  wire signed [7:0]  wt20,
    input  wire signed [7:0]  wt21,
    input  wire signed [7:0]  wt22,
    input  wire signed [31:0] acc_in,
    output wire signed [31:0] acc_out
);

    wire signed [31:0] mac00 = px00 * wt00;
    wire signed [31:0] mac01 = px01 * wt01;
    wire signed [31:0] mac02 = px02 * wt02;
    wire signed [31:0] mac10 = px10 * wt10;
    wire signed [31:0] mac11 = px11 * wt11;
    wire signed [31:0] mac12 = px12 * wt12;
    wire signed [31:0] mac20 = px20 * wt20;
    wire signed [31:0] mac21 = px21 * wt21;
    wire signed [31:0] mac22 = px22 * wt22;

    assign acc_out = acc_in +
                     mac00 + mac01 + mac02 +
                     mac10 + mac11 + mac12 +
                     mac20 + mac21 + mac22;

endmodule
