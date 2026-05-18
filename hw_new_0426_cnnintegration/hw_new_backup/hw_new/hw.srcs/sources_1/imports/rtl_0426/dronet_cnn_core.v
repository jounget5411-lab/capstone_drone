`timescale 1ns / 1ps
`include "dronet_params.vh"

module dronet_cnn_core #(
    // -------------------------------------------------------
    // Weight / Bias file paths  (empty string = all zeros)
    // -------------------------------------------------------
    parameter CONV1_W_FILE = "",
    parameter CONV2_W_FILE = "",
    parameter CONV3_W_FILE = "",
    parameter CONV4_W_FILE = "",
    parameter CONV5_W_FILE = "",
    parameter DET_W_FILE   = "",
    parameter CONV1_B_FILE = "",
    parameter CONV2_B_FILE = "",
    parameter CONV3_B_FILE = "",
    parameter CONV4_B_FILE = "",
    parameter CONV5_B_FILE = "",
    parameter DET_B_FILE   = "",
    // -------------------------------------------------------
    // Per-layer quantization right-shift (default 8 for QAT)
    // -------------------------------------------------------
    parameter integer CONV1_SHIFT = 8,
    parameter integer CONV2_SHIFT = 8,
    parameter integer CONV3_SHIFT = 8,
    parameter integer CONV4_SHIFT = 8,
    parameter integer CONV5_SHIFT = 8,
    parameter integer DET_SHIFT   = 8
) (
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              soft_reset,
    input  wire                              start,
    input  wire                              test_pattern_enable,
    input  wire [15:0]                       frame_id,
    output wire [`DRONET_FRAME_ADDR_W-1:0]   frame_rd_addr,
    input  wire [7:0]                        frame_rd_data,
    output wire                              raw_wr_en,
    output wire [`DRONET_RAW_ADDR_W-1:0]     raw_wr_addr,
    output wire signed [7:0]                 raw_wr_data,
    output wire                              busy,
    output wire                              done_pulse,
    output reg  [15:0]                       raw_frame_id,
    output wire [3:0]                        current_step,
    output wire [23:0]                       cycles_left,
    output wire [7:0]                        dbg_last_frame_byte
);

    // FIX: Removed unused frame_buf_sel port.
    // Buffer selection is handled externally in dronet_accel_axi
    // (control_core_buf_sel → frame buffer cnn_buf_sel).

    dronet_compute_engine #(
        .CONV1_W_FILE (CONV1_W_FILE),
        .CONV2_W_FILE (CONV2_W_FILE),
        .CONV3_W_FILE (CONV3_W_FILE),
        .CONV4_W_FILE (CONV4_W_FILE),
        .CONV5_W_FILE (CONV5_W_FILE),
        .DET_W_FILE   (DET_W_FILE),
        .CONV1_B_FILE (CONV1_B_FILE),
        .CONV2_B_FILE (CONV2_B_FILE),
        .CONV3_B_FILE (CONV3_B_FILE),
        .CONV4_B_FILE (CONV4_B_FILE),
        .CONV5_B_FILE (CONV5_B_FILE),
        .DET_B_FILE   (DET_B_FILE),
        .CONV1_SHIFT  (CONV1_SHIFT),
        .CONV2_SHIFT  (CONV2_SHIFT),
        .CONV3_SHIFT  (CONV3_SHIFT),
        .CONV4_SHIFT  (CONV4_SHIFT),
        .CONV5_SHIFT  (CONV5_SHIFT),
        .DET_SHIFT    (DET_SHIFT)
    ) u_compute_engine (
        .clk(clk),
        .rst_n(rst_n),
        .soft_reset(soft_reset),
        .start(start),
        .test_pattern_enable(test_pattern_enable),
        .frame_rd_addr(frame_rd_addr),
        .frame_rd_data(frame_rd_data),
        .raw_wr_en(raw_wr_en),
        .raw_wr_addr(raw_wr_addr),
        .raw_wr_data(raw_wr_data),
        .busy(busy),
        .done_pulse(done_pulse),
        .current_step(current_step),
        .cycles_left(cycles_left),
        .dbg_last_frame_byte(dbg_last_frame_byte)
    );

    always @(posedge clk) begin
        if (!rst_n || soft_reset) begin
            raw_frame_id <= 16'd0;
        end else if (start) begin
            raw_frame_id <= frame_id;
        end
    end

endmodule
