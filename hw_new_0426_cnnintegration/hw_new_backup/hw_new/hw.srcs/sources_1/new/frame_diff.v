`timescale 1ns / 1ps
//
// ============================================================
// axis_frame_diff (registered output 버전)
//   - 기존: 모든 출력이 조합회로 → critical path 길이 증가
//   - 변경: 출력 4종(tdata/tvalid/tlast/tuser)을 1-stage register로 등록
//
//   효과:
//     frame_diff 내부의 8-bit subtract/abs/threshold 비교가
//     출력 register 앞에서 종료 → 다음 클럭에 feature_extractor 진입.
//     critical path 의 logic levels 가 줄어듦.
//
//   AXI-Stream 동작:
//     downstream(feature_extractor) 가 항상 ready 이므로 stall 없음.
//     ready 가 0일 때만 register hold (표준 skid 패턴 단순화).
// ============================================================
module axis_frame_diff #(
    parameter THRESHOLD = 30
)(
    input  wire        aclk,
    input  wire        aresetn,

    // AXI-Stream Slave: current frame pixel
    input  wire [7:0]  s_axis_curr_tdata,
    input  wire        s_axis_curr_tvalid,
    output wire        s_axis_curr_tready,
    input  wire        s_axis_curr_tlast,
    input  wire        s_axis_curr_tuser,

    // AXI-Stream Slave: previous frame pixel
    input  wire [7:0]  s_axis_prev_tdata,
    input  wire        s_axis_prev_tvalid,
    output wire        s_axis_prev_tready,
    input  wire        s_axis_prev_tlast,
    input  wire        s_axis_prev_tuser,

    // AXI-Stream Master: diff result
    output wire [7:0]  m_axis_diff_tdata,
    output wire        m_axis_diff_tvalid,
    input  wire        m_axis_diff_tready,
    output wire        m_axis_diff_tlast,
    output wire        m_axis_diff_tuser,

    // Debug
    output wire        sync_error
);

    // =========================================================
    // 조합 계산부 (기존과 동일)
    // =========================================================
    wire sync_ok = (s_axis_curr_tuser == s_axis_prev_tuser) &&
                   (s_axis_curr_tlast == s_axis_prev_tlast);

    assign sync_error = s_axis_curr_tvalid & s_axis_prev_tvalid & ~sync_ok;

    // Backpressure: 두 입력이 모두 valid 이고 출력이 받을 수 있을 때만 진행
    assign s_axis_curr_tready = s_axis_prev_tvalid & m_axis_diff_tready;
    assign s_axis_prev_tready = s_axis_curr_tvalid & m_axis_diff_tready;

    // 절대값 차이 (8-bit subtract + abs)
    wire [7:0] abs_diff = (s_axis_curr_tdata >= s_axis_prev_tdata) ?
                          (s_axis_curr_tdata - s_axis_prev_tdata) :
                          (s_axis_prev_tdata - s_axis_curr_tdata);

    // Threshold 비교 결과 (조합) - 다음 클럭에 register 됨
    wire [7:0] diff_tdata_comb  = (sync_ok && abs_diff >= THRESHOLD) ? 8'd1 : 8'd0;
    wire       diff_tvalid_comb = s_axis_curr_tvalid & s_axis_prev_tvalid;
    wire       diff_tlast_comb  = s_axis_curr_tlast;
    wire       diff_tuser_comb  = s_axis_curr_tuser;

    // =========================================================
    // 출력 1-stage register (★ critical path 단축)
    // =========================================================
    reg [7:0] m_axis_diff_tdata_r;
    reg       m_axis_diff_tvalid_r;
    reg       m_axis_diff_tlast_r;
    reg       m_axis_diff_tuser_r;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axis_diff_tdata_r  <= 8'd0;
            m_axis_diff_tvalid_r <= 1'b0;
            m_axis_diff_tlast_r  <= 1'b0;
            m_axis_diff_tuser_r  <= 1'b0;
        end else if (m_axis_diff_tready) begin
            // downstream 이 데이터를 받아간 cycle 에만 register update
            m_axis_diff_tdata_r  <= diff_tdata_comb;
            m_axis_diff_tvalid_r <= diff_tvalid_comb;
            m_axis_diff_tlast_r  <= diff_tlast_comb;
            m_axis_diff_tuser_r  <= diff_tuser_comb;
        end
        // m_axis_diff_tready 가 0 일 때는 register 가 hold
        // (feature_extractor 의 ready 는 항상 1 이므로 실질적으로 hold 발생 안 함)
    end

    assign m_axis_diff_tdata  = m_axis_diff_tdata_r;
    assign m_axis_diff_tvalid = m_axis_diff_tvalid_r;
    assign m_axis_diff_tlast  = m_axis_diff_tlast_r;
    assign m_axis_diff_tuser  = m_axis_diff_tuser_r;

endmodule