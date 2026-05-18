`timescale 1ns / 1ps
`include "dronet_params.vh"

module dronet_compute_engine #(
    parameter integer PE_COUNT = 8,
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
    parameter CONV1_SHIFT = 8,
    parameter CONV2_SHIFT = 8,
    parameter CONV3_SHIFT = 8,
    parameter CONV4_SHIFT = 8,
    parameter CONV5_SHIFT = 8,
    parameter DET_SHIFT   = 8
) (
    input  wire                            clk,
    input  wire                            rst_n,
    input  wire                            soft_reset,
    input  wire                            start,
    input  wire                            test_pattern_enable,
    output reg  [`DRONET_FRAME_ADDR_W-1:0] frame_rd_addr,
    input  wire [7:0]                      frame_rd_data,
    output reg                             raw_wr_en,
    output reg  [`DRONET_RAW_ADDR_W-1:0]   raw_wr_addr,
    output reg  signed [7:0]               raw_wr_data,
    output reg                             busy,
    output reg                             done_pulse,
    output reg  [3:0]                      current_step,
    output reg  [23:0]                     cycles_left,
    output reg  [7:0]                      dbg_last_frame_byte
);

    localparam [3:0] ST_IDLE         = 4'd0;
    localparam [3:0] ST_SYNTH        = 4'd1;
    localparam [3:0] ST_LOAD         = 4'd2;
    localparam [3:0] ST_LAYER_PREP   = 4'd3;
    localparam [3:0] ST_3X3_ROW_CLR  = 4'd4;
    localparam [3:0] ST_3X3_IC_PREP  = 4'd5;
    localparam [3:0] ST_3X3_LOAD     = 4'd6;
    localparam [3:0] ST_3X3_SLIDE    = 4'd7;
    localparam [3:0] ST_3X3_WRITE    = 4'd8;
    localparam [3:0] ST_1X1_INIT     = 4'd9;
    localparam [3:0] ST_1X1_MAC      = 4'd10;
    localparam [3:0] ST_1X1_WRITE    = 4'd11;
    localparam [3:0] ST_POOL_INIT    = 4'd12;
    localparam [3:0] ST_POOL_ACC     = 4'd13;
    localparam [3:0] ST_POOL_WRITE   = 4'd14;
    localparam [3:0] ST_DONE         = 4'd15;

    localparam integer ACT_MEM_DEPTH = 115200;
    localparam integer LB_MAX_WIDTH  = `DRONET_INPUT_W + 2;
    localparam integer TEST_CELL_X   = 10;
    localparam integer TEST_CELL_Y   = 5;
    localparam integer TEST_CELL_IDX = (TEST_CELL_Y * `DRONET_GRID_W) + TEST_CELL_X;

    reg [3:0] state;

    (* ram_style = "block" *) reg signed [7:0] act_a [0:ACT_MEM_DEPTH-1];
    (* ram_style = "block" *) reg signed [7:0] act_b [0:ACT_MEM_DEPTH-1];

    reg signed [7:0] w_conv1 [0:71];
    reg signed [7:0] w_conv2 [0:1151];
    reg signed [7:0] w_conv3 [0:4607];
    reg signed [7:0] w_conv4 [0:511];
    reg signed [7:0] w_conv5 [0:4607];
    reg signed [7:0] w_det   [0:191];

    reg signed [31:0] b_conv1 [0:7];
    reg signed [31:0] b_conv2 [0:15];
    reg signed [31:0] b_conv3 [0:31];
    reg signed [31:0] b_conv4 [0:15];
    reg signed [31:0] b_conv5 [0:31];
    reg signed [31:0] b_det   [0:5];

    reg signed [31:0] acc_pe  [0:PE_COUNT-1];
    reg signed [31:0] acc_row [0:(PE_COUNT * `DRONET_INPUT_W)-1];

    reg [16:0] load_idx;
    reg [10:0] synth_idx;
    reg [5:0]  oc_tile_base;
    reg [5:0]  ic_idx;
    reg [8:0]  ox_idx;
    reg [6:0]  oy_idx;
    reg [3:0]  write_pe_idx;
    reg [8:0]  row_clr_idx;
    reg [8:0]  slide_x_idx;
    reg [8:0]  load_col_idx;
    reg [1:0]  row_load_sel;
    reg [1:0]  pool_idx;
    reg        mem_rd_phase;
    reg [16:0] act_rd_addr;
    reg signed [7:0] act_a_q, act_b_q;
    reg        act_rd_in_bounds;

    reg signed [7:0]  conv1x1_src;
    reg signed [7:0]  lb_load_data;
    reg signed [7:0]  pool_max;
    reg signed [7:0]  pool_val;
    reg signed [7:0]  out_val;
    reg               lb_load_en;

    reg               pe_active [0:PE_COUNT-1];
    reg signed [7:0]  pe1_weight [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w00 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w01 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w02 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w10 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w11 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w12 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w20 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w21 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w22 [0:PE_COUNT-1];

    wire signed [31:0] pe1_acc_next [0:PE_COUNT-1];
    wire signed [31:0] pe3_acc_next [0:PE_COUNT-1];

    wire [7:0] lb_window_x;
    wire signed [7:0] lb_tap00;
    wire signed [7:0] lb_tap01;
    wire signed [7:0] lb_tap02;
    wire signed [7:0] lb_tap10;
    wire signed [7:0] lb_tap11;
    wire signed [7:0] lb_tap12;
    wire signed [7:0] lb_tap20;
    wire signed [7:0] lb_tap21;
    wire signed [7:0] lb_tap22;

    integer i;
    integer pe;
    integer comb_ix;
    integer comb_iy;
    integer comb_addr_tmp;
    integer seq_ix;
    integer seq_iy;
    integer seq_addr_tmp;

    assign lb_window_x = slide_x_idx[7:0];
    
    function [23:0] cycles_for_step;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1:  cycles_for_step = 24'd187740;
                `DRONET_STEP_POOL1:  cycles_for_step = 24'd144000;
                `DRONET_STEP_CONV2:  cycles_for_step = 24'd299520;
                `DRONET_STEP_POOL2:  cycles_for_step = 24'd70400;
                `DRONET_STEP_CONV3:  cycles_for_step = 24'd265408;
                `DRONET_STEP_POOL3:  cycles_for_step = 24'd35200;
                `DRONET_STEP_CONV4:  cycles_for_step = 24'd17600;
                `DRONET_STEP_CONV5:  cycles_for_step = 24'd68464;
                `DRONET_STEP_DET:    cycles_for_step = 24'd8800;
                default:             cycles_for_step = 24'd0;
            endcase
        end
    endfunction
    
    function integer step_in_w;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1: step_in_w = 160;
                `DRONET_STEP_POOL1: step_in_w = 160;
                `DRONET_STEP_CONV2: step_in_w = 80;
                `DRONET_STEP_POOL2: step_in_w = 80;
                `DRONET_STEP_CONV3: step_in_w = 40;
                `DRONET_STEP_POOL3: step_in_w = 40;
                `DRONET_STEP_CONV4: step_in_w = 20;
                `DRONET_STEP_CONV5: step_in_w = 20;
                `DRONET_STEP_DET:   step_in_w = 20;
                default:            step_in_w = 0;
            endcase
        end
    endfunction

    function integer step_in_h;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1: step_in_h = 90;
                `DRONET_STEP_POOL1: step_in_h = 90;
                `DRONET_STEP_CONV2: step_in_h = 45;
                `DRONET_STEP_POOL2: step_in_h = 45;
                `DRONET_STEP_CONV3: step_in_h = 22;
                `DRONET_STEP_POOL3: step_in_h = 22;
                `DRONET_STEP_CONV4: step_in_h = 11;
                `DRONET_STEP_CONV5: step_in_h = 11;
                `DRONET_STEP_DET:   step_in_h = 11;
                default:            step_in_h = 0;
            endcase
        end
    endfunction

    function integer step_out_w;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1: step_out_w = 160;
                `DRONET_STEP_POOL1: step_out_w = 80;
                `DRONET_STEP_CONV2: step_out_w = 80;
                `DRONET_STEP_POOL2: step_out_w = 40;
                `DRONET_STEP_CONV3: step_out_w = 40;
                `DRONET_STEP_POOL3: step_out_w = 20;
                `DRONET_STEP_CONV4: step_out_w = 20;
                `DRONET_STEP_CONV5: step_out_w = 20;
                `DRONET_STEP_DET:   step_out_w = 20;
                default:            step_out_w = 0;
            endcase
        end
    endfunction

    function integer step_out_h;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1: step_out_h = 90;
                `DRONET_STEP_POOL1: step_out_h = 45;
                `DRONET_STEP_CONV2: step_out_h = 45;
                `DRONET_STEP_POOL2: step_out_h = 22;
                `DRONET_STEP_CONV3: step_out_h = 22;
                `DRONET_STEP_POOL3: step_out_h = 11;
                `DRONET_STEP_CONV4: step_out_h = 11;
                `DRONET_STEP_CONV5: step_out_h = 11;
                `DRONET_STEP_DET:   step_out_h = 11;
                default:            step_out_h = 0;
            endcase
        end
    endfunction

    function integer step_in_ch;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1: step_in_ch = 1;
                `DRONET_STEP_POOL1: step_in_ch = 8;
                `DRONET_STEP_CONV2: step_in_ch = 8;
                `DRONET_STEP_POOL2: step_in_ch = 16;
                `DRONET_STEP_CONV3: step_in_ch = 16;
                `DRONET_STEP_POOL3: step_in_ch = 32;
                `DRONET_STEP_CONV4: step_in_ch = 32;
                `DRONET_STEP_CONV5: step_in_ch = 16;
                `DRONET_STEP_DET:   step_in_ch = 32;
                default:            step_in_ch = 0;
            endcase
        end
    endfunction

    function integer step_out_ch;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1: step_out_ch = 8;
                `DRONET_STEP_POOL1: step_out_ch = 8;
                `DRONET_STEP_CONV2: step_out_ch = 16;
                `DRONET_STEP_POOL2: step_out_ch = 16;
                `DRONET_STEP_CONV3: step_out_ch = 32;
                `DRONET_STEP_POOL3: step_out_ch = 32;
                `DRONET_STEP_CONV4: step_out_ch = 16;
                `DRONET_STEP_CONV5: step_out_ch = 32;
                `DRONET_STEP_DET:   step_out_ch = 6;
                default:            step_out_ch = 0;
            endcase
        end
    endfunction

    function [0:0] step_is_pool;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_POOL1,
                `DRONET_STEP_POOL2,
                `DRONET_STEP_POOL3: step_is_pool = 1'b1;
                default:            step_is_pool = 1'b0;
            endcase
        end
    endfunction

    function [0:0] step_is_conv3;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1,
                `DRONET_STEP_CONV2,
                `DRONET_STEP_CONV3,
                `DRONET_STEP_CONV5: step_is_conv3 = 1'b1;
                default:            step_is_conv3 = 1'b0;
            endcase
        end
    endfunction

    function [0:0] step_relu_en;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1,
                `DRONET_STEP_CONV2,
                `DRONET_STEP_CONV3,
                `DRONET_STEP_CONV4,
                `DRONET_STEP_CONV5: step_relu_en = 1'b1;
                default:            step_relu_en = 1'b0;
            endcase
        end
    endfunction

    function [0:0] step_src_a;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1,
                `DRONET_STEP_CONV2,
                `DRONET_STEP_CONV3,
                `DRONET_STEP_CONV4,
                `DRONET_STEP_DET:   step_src_a = 1'b1;
                default:            step_src_a = 1'b0;
            endcase
        end
    endfunction

    function [0:0] step_dst_a;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_POOL1,
                `DRONET_STEP_POOL2,
                `DRONET_STEP_POOL3,
                `DRONET_STEP_CONV5: step_dst_a = 1'b1;
                default:            step_dst_a = 1'b0;
            endcase
        end
    endfunction

    function integer act_addr;
        input integer ch;
        input integer y;
        input integer x;
        input integer h;
        input integer w;
        begin
            act_addr = ((ch * h) + y) * w + x;
        end
    endfunction

    function integer raw_addr_calc;
        input integer ch;
        input integer y;
        input integer x;
        begin
            raw_addr_calc = (ch * `DRONET_CELLS) + (y * `DRONET_GRID_W) + x;
        end
    endfunction

    function integer row_acc_addr;
        input integer pe_idx;
        input integer x;
        begin
            row_acc_addr = (pe_idx * `DRONET_INPUT_W) + x;
        end
    endfunction

    function signed [7:0] weight_lookup;
        input [3:0] step;
        input integer oc;
        input integer ic;
        input integer ky;
        input integer kx;
        integer idx;
        begin
            weight_lookup = 8'sd0;
            case (step)
                `DRONET_STEP_CONV1: begin
                    idx = (((oc * 1) + ic) * 3 + ky) * 3 + kx;
                    weight_lookup = w_conv1[idx];
                end
                `DRONET_STEP_CONV2: begin
                    idx = (((oc * 8) + ic) * 3 + ky) * 3 + kx;
                    weight_lookup = w_conv2[idx];
                end
                `DRONET_STEP_CONV3: begin
                    idx = (((oc * 16) + ic) * 3 + ky) * 3 + kx;
                    weight_lookup = w_conv3[idx];
                end
                `DRONET_STEP_CONV4: begin
                    idx = (oc * 32) + ic;
                    weight_lookup = w_conv4[idx];
                end
                `DRONET_STEP_CONV5: begin
                    idx = (((oc * 16) + ic) * 3 + ky) * 3 + kx;
                    weight_lookup = w_conv5[idx];
                end
                `DRONET_STEP_DET: begin
                    idx = (oc * 32) + ic;
                    weight_lookup = w_det[idx];
                end
                default: begin
                    weight_lookup = 8'sd0;
                end
            endcase
        end
    endfunction

    function signed [31:0] bias_lookup;
        input [3:0] step;
        input integer oc;
        begin
            bias_lookup = 32'sd0;
            case (step)
                `DRONET_STEP_CONV1: bias_lookup = b_conv1[oc];
                `DRONET_STEP_CONV2: bias_lookup = b_conv2[oc];
                `DRONET_STEP_CONV3: bias_lookup = b_conv3[oc];
                `DRONET_STEP_CONV4: bias_lookup = b_conv4[oc];
                `DRONET_STEP_CONV5: bias_lookup = b_conv5[oc];
                `DRONET_STEP_DET:   bias_lookup = b_det[oc];
                default:            bias_lookup = 32'sd0;
            endcase
        end
    endfunction

    function signed [7:0] quantize_acc;
        input signed [31:0] value;
        input integer shift;
        input relu_en;
        reg signed [31:0] tmp;
        begin
            tmp = value;
            if (shift > 0) begin
                tmp = tmp >>> shift;
            end
            if (relu_en && (tmp < 0)) begin
                tmp = 0;
            end
            if (tmp > 127) begin
                quantize_acc = 8'sd127;
            end else if (tmp < -128) begin
                quantize_acc = -8'sd128;
            end else begin
                quantize_acc = tmp[7:0];
            end
        end
    endfunction

    // -----------------------------------------------------------------
    // Combinational BRAM read address + in-bounds flag + lb_load_en
    // -----------------------------------------------------------------
    // FIX: BRAM latency is 1 cycle (sync read). Previously act_rd_addr
    // was a reg updated nonblocking → effective latency = 2 cycles,
    // which misaligned data vs. consumers (3x3 line buffer, 1x1 MAC,
    // pool). Drive these combinationally so data_K is available in
    // act_a_q at the start of the cycle AFTER we issue addr_K.
    // -----------------------------------------------------------------
    always @(*) begin
        // Defaults
        conv1x1_src      = 8'sd0;
        lb_load_data     = 8'sd0;
        act_rd_addr      = 17'd0;
        act_rd_in_bounds = 1'b0;
        lb_load_en       = 1'b0;
        comb_ix          = 0;
        comb_iy          = 0;
        comb_addr_tmp    = 0;

        // ---- Address generator (drives BRAM read) ----
        case (state)
            ST_3X3_LOAD: begin
                comb_ix = load_col_idx - 1;
                comb_iy = oy_idx + row_load_sel - 1;
                if ((comb_ix >= 0) && (comb_ix < step_in_w(current_step)) &&
                    (comb_iy >= 0) && (comb_iy < step_in_h(current_step))) begin
                    act_rd_addr      = act_addr(ic_idx, comb_iy, comb_ix,
                                                step_in_h(current_step),
                                                step_in_w(current_step));
                    act_rd_in_bounds = 1'b1;
                end
                // lb_load_en fires during Phase 1 — data_K is in act_a_q now.
                lb_load_en = mem_rd_phase;
            end
            ST_1X1_INIT: begin
                // Pre-fetch ic=0 so data is ready at MAC cycle 0.
                act_rd_addr      = act_addr(0, oy_idx, ox_idx,
                                            step_in_h(current_step),
                                            step_in_w(current_step));
                act_rd_in_bounds = 1'b1;
            end
            ST_1X1_MAC: begin
                // Pre-fetch ic_idx+1 so it's ready next cycle.
                if (ic_idx < (step_in_ch(current_step) - 1)) begin
                    act_rd_addr      = act_addr(ic_idx + 1, oy_idx, ox_idx,
                                                step_in_h(current_step),
                                                step_in_w(current_step));
                    act_rd_in_bounds = 1'b1;
                end
            end
            ST_POOL_INIT: begin
                // Pre-fetch first pool pixel (pool_idx=0).
                act_rd_addr      = act_addr(oc_tile_base,
                                            oy_idx << 1,
                                            ox_idx << 1,
                                            step_in_h(current_step),
                                            step_in_w(current_step));
                act_rd_in_bounds = 1'b1;
            end
            ST_POOL_ACC: begin
                // Pre-fetch next pool pixel (pool_idx+1).
                if (pool_idx < 2'd3) begin
                    act_rd_addr      = act_addr(oc_tile_base,
                                                (oy_idx << 1) + (((pool_idx + 2'd1) >> 1) & 32'd1),
                                                (ox_idx << 1) + ((pool_idx + 2'd1) & 2'd1),
                                                step_in_h(current_step),
                                                step_in_w(current_step));
                    act_rd_in_bounds = 1'b1;
                end
            end
            default: begin
                // no read
            end
        endcase

        // ---- Data muxes consuming act_a_q / act_b_q ----
        if (state == ST_1X1_MAC) begin
            if (step_src_a(current_step)) begin
                conv1x1_src = act_a_q;
            end else begin
                conv1x1_src = act_b_q;
            end
        end

        if (state == ST_3X3_LOAD && mem_rd_phase) begin
            if (act_rd_in_bounds) begin
                if (step_src_a(current_step)) begin
                    lb_load_data = act_a_q;
                end else begin
                    lb_load_data = act_b_q;
                end
            end
        end

        for (pe = 0; pe < PE_COUNT; pe = pe + 1) begin
            if ((oc_tile_base + pe) < step_out_ch(current_step)) begin
                pe_active[pe]  = 1'b1;
                pe1_weight[pe] = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 0, 0);
                pe3_w00[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 0, 0);
                pe3_w01[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 0, 1);
                pe3_w02[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 0, 2);
                pe3_w10[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 1, 0);
                pe3_w11[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 1, 1);
                pe3_w12[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 1, 2);
                pe3_w20[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 2, 0);
                pe3_w21[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 2, 1);
                pe3_w22[pe]    = weight_lookup(current_step, oc_tile_base + pe, ic_idx, 2, 2);
            end else begin
                pe_active[pe]  = 1'b0;
                pe1_weight[pe] = 8'sd0;
                pe3_w00[pe]    = 8'sd0;
                pe3_w01[pe]    = 8'sd0;
                pe3_w02[pe]    = 8'sd0;
                pe3_w10[pe]    = 8'sd0;
                pe3_w11[pe]    = 8'sd0;
                pe3_w12[pe]    = 8'sd0;
                pe3_w20[pe]    = 8'sd0;
                pe3_w21[pe]    = 8'sd0;
                pe3_w22[pe]    = 8'sd0;
            end
        end
    end

    dronet_line_buffer_3x3 #(
        .MAX_WIDTH(LB_MAX_WIDTH),
        .ADDR_W(8)
    ) u_line_buffer_3x3 (
        .clk(clk),
        .load_en(lb_load_en),
        .load_row_sel(row_load_sel),
        .load_addr(load_col_idx[7:0]),
        .load_data(lb_load_data),
        .window_x(lb_window_x),
        .tap00(lb_tap00),
        .tap01(lb_tap01),
        .tap02(lb_tap02),
        .tap10(lb_tap10),
        .tap11(lb_tap11),
        .tap12(lb_tap12),
        .tap20(lb_tap20),
        .tap21(lb_tap21),
        .tap22(lb_tap22)
    );

    genvar g;
    generate
        for (g = 0; g < PE_COUNT; g = g + 1) begin : GEN_PES
            dronet_mac_pe u_mac_pe (
                .in_val(conv1x1_src),
                .weight(pe1_weight[g]),
                .acc_in(acc_pe[g]),
                .acc_out(pe1_acc_next[g])
            );

            dronet_conv3x3_pe u_conv3x3_pe (
                .px00(lb_tap00),
                .px01(lb_tap01),
                .px02(lb_tap02),
                .px10(lb_tap10),
                .px11(lb_tap11),
                .px12(lb_tap12),
                .px20(lb_tap20),
                .px21(lb_tap21),
                .px22(lb_tap22),
                .wt00(pe3_w00[g]),
                .wt01(pe3_w01[g]),
                .wt02(pe3_w02[g]),
                .wt10(pe3_w10[g]),
                .wt11(pe3_w11[g]),
                .wt12(pe3_w12[g]),
                .wt20(pe3_w20[g]),
                .wt21(pe3_w21[g]),
                .wt22(pe3_w22[g]),
                .acc_in(acc_row[row_acc_addr(g, slide_x_idx)]),
                .acc_out(pe3_acc_next[g])
            );
        end
    endgenerate

    // Synchronous read ports — required for BRAM inference of act_a/act_b
    always @(posedge clk) begin
        act_a_q <= act_a[act_rd_addr];
        act_b_q <= act_b[act_rd_addr];
    end

    initial begin
        for (i = 0; i < ACT_MEM_DEPTH; i = i + 1) begin
            act_a[i] = 8'sd0;
            act_b[i] = 8'sd0;
        end
        for (i = 0; i < (PE_COUNT * `DRONET_INPUT_W); i = i + 1) begin
            acc_row[i] = 32'sd0;
        end
        for (i = 0; i < 72; i = i + 1) begin
            w_conv1[i] = 8'sd0;
        end
        for (i = 0; i < 8; i = i + 1) begin
            b_conv1[i] = 32'sd0;
        end
        for (i = 0; i < 1152; i = i + 1) begin
            w_conv2[i] = 8'sd0;
        end
        for (i = 0; i < 16; i = i + 1) begin
            b_conv2[i] = 32'sd0;
            b_conv4[i] = 32'sd0;
        end
        for (i = 0; i < 4608; i = i + 1) begin
            w_conv3[i] = 8'sd0;
            w_conv5[i] = 8'sd0;
        end
        for (i = 0; i < 32; i = i + 1) begin
            b_conv3[i] = 32'sd0;
            b_conv5[i] = 32'sd0;
        end
        for (i = 0; i < 512; i = i + 1) begin
            w_conv4[i] = 8'sd0;
        end
        for (i = 0; i < 192; i = i + 1) begin
            w_det[i] = 8'sd0;
        end
        for (i = 0; i < 6; i = i + 1) begin
            b_det[i] = 32'sd0;
        end
        for (i = 0; i < PE_COUNT; i = i + 1) begin
            acc_pe[i] = 32'sd0;
        end

        if (CONV1_W_FILE != "") $readmemh(CONV1_W_FILE, w_conv1);
        if (CONV2_W_FILE != "") $readmemh(CONV2_W_FILE, w_conv2);
        if (CONV3_W_FILE != "") $readmemh(CONV3_W_FILE, w_conv3);
        if (CONV4_W_FILE != "") $readmemh(CONV4_W_FILE, w_conv4);
        if (CONV5_W_FILE != "") $readmemh(CONV5_W_FILE, w_conv5);
        if (DET_W_FILE   != "") $readmemh(DET_W_FILE,   w_det);
        if (CONV1_B_FILE != "") $readmemh(CONV1_B_FILE, b_conv1);
        if (CONV2_B_FILE != "") $readmemh(CONV2_B_FILE, b_conv2);
        if (CONV3_B_FILE != "") $readmemh(CONV3_B_FILE, b_conv3);
        if (CONV4_B_FILE != "") $readmemh(CONV4_B_FILE, b_conv4);
        if (CONV5_B_FILE != "") $readmemh(CONV5_B_FILE, b_conv5);
        if (DET_B_FILE   != "") $readmemh(DET_B_FILE,   b_det);
    end

    always @(posedge clk) begin
        if (!rst_n || soft_reset) begin
            state               <= ST_IDLE;
            frame_rd_addr       <= {`DRONET_FRAME_ADDR_W{1'b0}};
            raw_wr_en           <= 1'b0;
            raw_wr_addr         <= {`DRONET_RAW_ADDR_W{1'b0}};
            raw_wr_data         <= 8'sd0;
            busy                <= 1'b0;
            done_pulse          <= 1'b0;
            current_step        <= `DRONET_STEP_CONV1;
            cycles_left         <= 24'd0;
            dbg_last_frame_byte <= 8'd0;
            load_idx            <= 17'd0;
            synth_idx           <= 11'd0;
            oc_tile_base        <= 6'd0;
            ic_idx              <= 6'd0;
            ox_idx              <= 9'd0;
            oy_idx              <= 7'd0;
            write_pe_idx        <= 4'd0;
            row_clr_idx         <= 9'd0;
            slide_x_idx         <= 9'd0;
            load_col_idx        <= 9'd0;
            row_load_sel        <= 2'd0;
            pool_idx            <= 2'd0;
            pool_max            <= -8'sd128;
            mem_rd_phase        <= 1'b0;
            // act_rd_addr, act_rd_in_bounds, lb_load_en are now combinational
            for (i = 0; i < PE_COUNT; i = i + 1) begin
                acc_pe[i] <= 32'sd0;
            end
        end else begin
            raw_wr_en   <= 1'b0;
            done_pulse  <= 1'b0;
            // lb_load_en is now combinational

            case (state)
                ST_IDLE: begin
                    busy          <= 1'b0;
                    frame_rd_addr <= {`DRONET_FRAME_ADDR_W{1'b0}};
                    cycles_left   <= 24'd0;
                    if (start) begin
                        busy         <= 1'b1;
                        mem_rd_phase <= 1'b0;
                        if (test_pattern_enable) begin
                            state        <= ST_SYNTH;
                            synth_idx    <= 11'd0;
                            current_step <= `DRONET_STEP_DET;
                            cycles_left  <= `DRONET_RAW_BYTES;
                        end else begin
                            state         <= ST_LOAD;
                            load_idx      <= 17'd0;
                            frame_rd_addr <= {`DRONET_FRAME_ADDR_W{1'b0}};
                            current_step  <= `DRONET_STEP_CONV1;
                        end
                    end
                end

                ST_SYNTH: begin
                    raw_wr_en   <= 1'b1;
                    raw_wr_addr <= synth_idx[`DRONET_RAW_ADDR_W-1:0];
                    raw_wr_data <= 8'sd0;
                    if (synth_idx == (4 * `DRONET_CELLS) + TEST_CELL_IDX) begin
                        raw_wr_data <= 8'sd8;
                    end else if (synth_idx == (5 * `DRONET_CELLS) + TEST_CELL_IDX) begin
                        raw_wr_data <= 8'sd8;
                    end

                    if (cycles_left != 0) begin
                        cycles_left <= cycles_left - 24'd1;
                    end

                    if (synth_idx == (`DRONET_RAW_BYTES - 1)) begin
                        state <= ST_DONE;
                    end else begin
                        synth_idx <= synth_idx + 11'd1;
                    end
                end

                ST_LOAD: begin
                    if (!mem_rd_phase) begin
                        // Wait 1 cycle for synchronous frame buffer read
                        // (addr=0 was set in start transition; registered
                        //  output fb[0] becomes valid next cycle)
                        mem_rd_phase  <= 1'b1;
                        frame_rd_addr <= {{(`DRONET_FRAME_ADDR_W-1){1'b0}}, 1'b1};
                    end else begin
                        dbg_last_frame_byte <= frame_rd_data;
                        act_a[load_idx] <= $signed({1'b0, frame_rd_data}) - 9'sd128;
                        if (load_idx == (`DRONET_FRAME_PIXELS - 1)) begin
                            state        <= ST_LAYER_PREP;
                            current_step <= `DRONET_STEP_CONV1;
                            mem_rd_phase <= 1'b0;
                        end else begin
                            load_idx      <= load_idx + 17'd1;
                            frame_rd_addr <= frame_rd_addr + {{(`DRONET_FRAME_ADDR_W-1){1'b0}}, 1'b1};
                        end
                    end
                end

                ST_LAYER_PREP: begin
                    oc_tile_base    <= 6'd0;
                    ic_idx          <= 6'd0;
                    ox_idx          <= 9'd0;
                    oy_idx          <= 7'd0;
                    write_pe_idx    <= 4'd0;
                    row_clr_idx     <= 9'd0;
                    slide_x_idx     <= 9'd0;
                    load_col_idx    <= 9'd0;
                    row_load_sel    <= 2'd0;
                    pool_idx        <= 2'd0;
                    pool_max        <= -8'sd128;
                    mem_rd_phase    <= 1'b0;
                    // act_rd_in_bounds now combinational

                    cycles_left <= cycles_for_step(current_step);
    
                    if (step_is_pool(current_step)) begin
                        state <= ST_POOL_INIT;
                    end else if (step_is_conv3(current_step)) begin
                        state <= ST_3X3_ROW_CLR;
                    end else begin
                        state <= ST_1X1_INIT;
                    end 
                end
                
                ST_3X3_ROW_CLR: begin
                    for (i = 0; i < PE_COUNT; i = i + 1) begin
                        acc_row[row_acc_addr(i, row_clr_idx)] <= 32'sd0;
                    end

                    if (cycles_left != 0) begin
                        cycles_left <= cycles_left - 24'd1;
                    end

                    if (row_clr_idx == (step_out_w(current_step) - 1)) begin
                        row_clr_idx <= 9'd0;
                        ic_idx      <= 6'd0;
                        row_load_sel <= 2'd0;
                        load_col_idx <= 9'd0;
                        state <= ST_3X3_IC_PREP;
                    end else begin
                        row_clr_idx <= row_clr_idx + 9'd1;
                    end
                end

                ST_3X3_IC_PREP: begin
                    row_load_sel <= 2'd0;
                    load_col_idx <= 9'd0;
                    slide_x_idx  <= 9'd0;
                    state        <= ST_3X3_LOAD;
                end

                ST_3X3_LOAD: begin
                    if (cycles_left != 0) begin
                        cycles_left <= cycles_left - 24'd1;
                    end

                    if (!mem_rd_phase) begin
                        // Phase 0: combinational block presents BRAM addr
                        //          this cycle → act_a_q valid at end of P0.
                        mem_rd_phase <= 1'b1;
                    end else begin
                        // Phase 1: act_a_q = data_K; combinational lb_load_en
                        //          writes line buffer at end of P1.
                        dbg_last_frame_byte <= lb_load_data;
                        mem_rd_phase        <= 1'b0;

                        if (load_col_idx == (step_in_w(current_step) + 1)) begin
                            load_col_idx <= 9'd0;
                            if (row_load_sel == 2'd2) begin
                                state <= ST_3X3_SLIDE;
                            end else begin
                                row_load_sel <= row_load_sel + 2'd1;
                            end
                        end else begin
                            load_col_idx <= load_col_idx + 9'd1;
                        end
                    end
                end

                ST_3X3_SLIDE: begin
                    for (i = 0; i < PE_COUNT; i = i + 1) begin
                        if (pe_active[i]) begin
                            acc_row[row_acc_addr(i, slide_x_idx)] <= pe3_acc_next[i];
                        end
                    end

                    if (cycles_left != 0) begin
                        cycles_left <= cycles_left - 24'd1;
                    end

                    if (slide_x_idx == (step_out_w(current_step) - 1)) begin
                        slide_x_idx <= 9'd0;
                        if (ic_idx == (step_in_ch(current_step) - 1)) begin
                            ox_idx       <= 9'd0;
                            write_pe_idx <= 4'd0;
                            state        <= ST_3X3_WRITE;
                        end else begin
                            ic_idx <= ic_idx + 6'd1;
                            state  <= ST_3X3_IC_PREP;
                        end
                    end else begin
                        slide_x_idx <= slide_x_idx + 9'd1;
                    end
                end

                ST_3X3_WRITE: begin
                    if (write_pe_idx < PE_COUNT) begin
                        if ((oc_tile_base + write_pe_idx) < step_out_ch(current_step)) begin
                            out_val = quantize_acc(
                                acc_row[row_acc_addr(write_pe_idx, ox_idx)] +
                                bias_lookup(current_step, oc_tile_base + write_pe_idx),
                                (current_step == `DRONET_STEP_CONV1) ? CONV1_SHIFT :
                                (current_step == `DRONET_STEP_CONV2) ? CONV2_SHIFT :
                                (current_step == `DRONET_STEP_CONV3) ? CONV3_SHIFT :
                                CONV5_SHIFT,
                                step_relu_en(current_step)
                            );

                            seq_addr_tmp = act_addr(oc_tile_base + write_pe_idx, oy_idx, ox_idx,
                                                    step_out_h(current_step), step_out_w(current_step));
                            if (step_dst_a(current_step)) begin
                                act_a[seq_addr_tmp] <= out_val;
                            end else begin
                                act_b[seq_addr_tmp] <= out_val;
                            end
                        end

                        if (cycles_left != 0) begin
                            cycles_left <= cycles_left - 24'd1;
                        end

                        if (write_pe_idx == (PE_COUNT - 1)) begin
                            write_pe_idx <= 4'd0;
                            if (ox_idx == (step_out_w(current_step) - 1)) begin
                                ox_idx <= 9'd0;
                                if ((oc_tile_base + PE_COUNT) < step_out_ch(current_step)) begin
                                    oc_tile_base <= oc_tile_base + PE_COUNT;
                                    state        <= ST_3X3_ROW_CLR;
                                end else begin
                                    oc_tile_base <= 6'd0;
                                    if (oy_idx == (step_out_h(current_step) - 1)) begin
                                        oy_idx <= 7'd0;
                                        current_step <= current_step + 4'd1;
                                        state <= ST_LAYER_PREP;
                                    end else begin
                                        oy_idx <= oy_idx + 7'd1;
                                        state  <= ST_3X3_ROW_CLR;
                                    end
                                end
                            end else begin
                                ox_idx <= ox_idx + 9'd1;
                            end
                        end else begin
                            write_pe_idx <= write_pe_idx + 4'd1;
                        end
                    end
                end

                ST_1X1_INIT: begin
                    for (i = 0; i < PE_COUNT; i = i + 1) begin
                        if ((oc_tile_base + i) < step_out_ch(current_step)) begin
                            acc_pe[i] <= bias_lookup(current_step, oc_tile_base + i);
                        end else begin
                            acc_pe[i] <= 32'sd0;
                        end
                    end
                    ic_idx       <= 6'd0;
                    write_pe_idx <= 4'd0;
                    // Pre-fetch for ic=0 is done combinationally
                    // (see comb block at ST_1X1_INIT).
                    state        <= ST_1X1_MAC;
                end

                ST_1X1_MAC: begin
                    dbg_last_frame_byte <= conv1x1_src;

                    for (i = 0; i < PE_COUNT; i = i + 1) begin
                        if (pe_active[i]) begin
                            acc_pe[i] <= pe1_acc_next[i];
                        end
                    end

                    if (cycles_left != 0) begin
                        cycles_left <= cycles_left - 24'd1;
                    end

                    if (ic_idx == (step_in_ch(current_step) - 1)) begin
                        ic_idx <= 6'd0;
                        state  <= ST_1X1_WRITE;
                    end else begin
                        ic_idx <= ic_idx + 6'd1;
                        // Pre-fetch for ic_idx+1 is done combinationally.
                    end
                end

                ST_1X1_WRITE: begin
                    if (write_pe_idx < PE_COUNT) begin
                        if ((oc_tile_base + write_pe_idx) < step_out_ch(current_step)) begin
                            out_val = quantize_acc(
                                acc_pe[write_pe_idx],
                                (current_step == `DRONET_STEP_CONV4) ? CONV4_SHIFT : DET_SHIFT,
                                step_relu_en(current_step)
                            );

                            if (current_step == `DRONET_STEP_DET) begin
                                raw_wr_en   <= 1'b1;
                                raw_wr_addr <= raw_addr_calc(oc_tile_base + write_pe_idx, oy_idx, ox_idx);
                                raw_wr_data <= out_val;
                            end else begin
                                seq_addr_tmp = act_addr(oc_tile_base + write_pe_idx, oy_idx, ox_idx,
                                                        step_out_h(current_step), step_out_w(current_step));
                                if (step_dst_a(current_step)) begin
                                    act_a[seq_addr_tmp] <= out_val;
                                end else begin
                                    act_b[seq_addr_tmp] <= out_val;
                                end
                            end
                        end

                        if (cycles_left != 0) begin
                            cycles_left <= cycles_left - 24'd1;
                        end

                        if (write_pe_idx == (PE_COUNT - 1)) begin
                            write_pe_idx <= 4'd0;
                            if ((oc_tile_base + PE_COUNT) < step_out_ch(current_step)) begin
                                oc_tile_base <= oc_tile_base + PE_COUNT;
                                state <= ST_1X1_INIT;
                            end else begin
                                oc_tile_base <= 6'd0;
                                if (ox_idx == (step_out_w(current_step) - 1)) begin
                                    ox_idx <= 9'd0;
                                    if (oy_idx == (step_out_h(current_step) - 1)) begin
                                        oy_idx <= 7'd0;
                                        if (current_step == `DRONET_STEP_DET) begin
                                            state <= ST_DONE;
                                        end else begin
                                            current_step <= current_step + 4'd1;
                                            state <= ST_LAYER_PREP;
                                        end
                                    end else begin
                                        oy_idx <= oy_idx + 7'd1;
                                        state  <= ST_1X1_INIT;
                                    end
                                end else begin
                                    ox_idx <= ox_idx + 9'd1;
                                    state  <= ST_1X1_INIT;
                                end
                            end
                        end else begin
                            write_pe_idx <= write_pe_idx + 4'd1;
                        end
                    end
                end

                ST_POOL_INIT: begin
                    pool_idx <= 2'd0;
                    pool_max <= -8'sd128;
                    // Pre-fetch for pool_idx=0 done combinationally
                    state <= ST_POOL_ACC;
                end

                ST_POOL_ACC: begin
                    // Use pre-fetched data from BRAM registered output
                    if (step_src_a(current_step)) begin
                        pool_val = act_a_q;
                    end else begin
                        pool_val = act_b_q;
                    end
                    if (pool_val > pool_max) begin
                        pool_max <= pool_val;
                    end

                    if (cycles_left != 0) begin
                        cycles_left <= cycles_left - 24'd1;
                    end

                    if (pool_idx == 2'd3) begin
                        state <= ST_POOL_WRITE;
                    end else begin
                        pool_idx <= pool_idx + 2'd1;
                        // Pre-fetch for pool_idx+1 done combinationally
                    end
                end

                ST_POOL_WRITE: begin
                    seq_addr_tmp = act_addr(oc_tile_base, oy_idx, ox_idx, step_out_h(current_step), step_out_w(current_step));
                    if (step_dst_a(current_step)) begin
                        act_a[seq_addr_tmp] <= pool_max;
                    end else begin
                        act_b[seq_addr_tmp] <= pool_max;
                    end

                    if (cycles_left != 0) begin
                        cycles_left <= cycles_left - 24'd1;
                    end

                    if (oc_tile_base == (step_out_ch(current_step) - 1)) begin
                        oc_tile_base <= 6'd0;
                        if (ox_idx == (step_out_w(current_step) - 1)) begin
                            ox_idx <= 9'd0;
                            if (oy_idx == (step_out_h(current_step) - 1)) begin
                                oy_idx <= 7'd0;
                                current_step <= current_step + 4'd1;
                                state <= ST_LAYER_PREP;
                            end else begin
                                oy_idx <= oy_idx + 7'd1;
                                state <= ST_POOL_INIT;
                            end
                        end else begin
                            ox_idx <= ox_idx + 9'd1;
                            state <= ST_POOL_INIT;
                        end
                    end else begin
                        oc_tile_base <= oc_tile_base + 6'd1;
                        state <= ST_POOL_INIT;
                    end
                end

                ST_DONE: begin
                    busy        <= 1'b0;
                    done_pulse  <= 1'b1;
                    state       <= ST_IDLE;
                    cycles_left <= 24'd0;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
