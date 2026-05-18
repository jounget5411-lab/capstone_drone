#include "xparameters.h"
#include "platform.h"
#include "ov5640/OV5640.h"
#include "ov5640/ScuGicInterruptController.h"
#include "ov5640/PS_GPIO.h"
#include "ov5640/AXI_VDMA.h"
#include "ov5640/PS_IIC.h"
#include "MIPI_D_PHY_RX.h"
#include "MIPI_CSI_2_RX.h"
#include "xuartps.h"
#include <cstdint>
#include <xil_io.h>
#include <xil_types.h>
#include "snn_weights.h"    //snn 가중치
// ============================================================
// VDMA1 (gray8 원본 저장 + prev frame 읽기)
// ============================================================
#define VDMA1_BASEADDR      XPAR_AXI_VDMA_1_BASEADDR

// VDMA S2MM (Write) register offsets
#define VDMA_S2MM_DMACR         0x30
#define VDMA_S2MM_DMASR         0x34
#define VDMA_S2MM_VSIZE         0xA0
#define VDMA_S2MM_HSIZE         0xA4
#define VDMA_S2MM_FRMDLY_STRIDE 0xA8
#define VDMA_S2MM_START_ADDR1   0xAC
#define VDMA_S2MM_START_ADDR2   0xB0
#define VDMA_S2MM_START_ADDR3   0xB4

// VDMA MM2S (Read) register offsets
#define VDMA_MM2S_DMACR         0x00
#define VDMA_MM2S_DMASR         0x04
#define VDMA_MM2S_VSIZE         0x50
#define VDMA_MM2S_HSIZE         0x54
#define VDMA_MM2S_FRMDLY_STRIDE 0x58
#define VDMA_MM2S_START_ADDR1   0x5C
#define VDMA_MM2S_START_ADDR2   0x60
#define VDMA_MM2S_START_ADDR3   0x64

//snn 관련
#define GPIO0_BASEADDR  XPAR_AXI_GPIO_0_BASEADDR  // 또는 직접 주소
#define GPIO0_CH1       (GPIO0_BASEADDR + 0x00)     // 채널1 (32bit)
#define GPIO0_CH2       (GPIO0_BASEADDR + 0x08)     // 채널2 (16bit)
#define GPIO1_BASEADDR  XPAR_AXI_GPIO_1_BASEADDR
#define GPIO1_DATA      (GPIO1_BASEADDR + 0x00)


// VDMA1 해상도/크기 설정
static constexpr u32 VDMA1_W = 160;
static constexpr u32 VDMA1_H = 90;
static constexpr u32 VDMA1_BPP = 1;
static constexpr u32 VDMA1_HSIZE = VDMA1_W * VDMA1_BPP;
static constexpr u32 VDMA1_STRIDE = VDMA1_HSIZE;
static constexpr u32 VDMA1_FRAME_BYTES = VDMA1_STRIDE * VDMA1_H;

// VDMA1 메모리 주소
#define MEM_BASE_ADDR_VDMA1 (XPAR_DDR_MEM_BASEADDR + 0x15000000)
static constexpr u32 VDMA1_FB0 = MEM_BASE_ADDR_VDMA1;
static constexpr u32 VDMA1_FB1 = VDMA1_FB0 + VDMA1_FRAME_BYTES;
static constexpr u32 VDMA1_FB2 = VDMA1_FB1 + VDMA1_FRAME_BYTES;

// ============================================================
// VDMA2 (frame diff 결과 저장)
// ============================================================
#define VDMA2_BASEADDR      0x43020000

static constexpr u32 VDMA2_W = 160;
static constexpr u32 VDMA2_H = 90;
static constexpr u32 VDMA2_BPP = 1;
static constexpr u32 VDMA2_HSIZE = VDMA2_W * VDMA2_BPP;
static constexpr u32 VDMA2_STRIDE = VDMA2_HSIZE;
static constexpr u32 VDMA2_FRAME_BYTES = VDMA2_STRIDE * VDMA2_H;

#define MEM_BASE_ADDR_VDMA2 (XPAR_DDR_MEM_BASEADDR + 0x15100000)
static constexpr u32 VDMA2_FB0 = MEM_BASE_ADDR_VDMA2;
static constexpr u32 VDMA2_FB1 = VDMA2_FB0 + VDMA2_FRAME_BYTES;
static constexpr u32 VDMA2_FB2 = VDMA2_FB1 + VDMA2_FRAME_BYTES;

// ============================================================
// VDMA1 S2MM (Write) 함수
// ============================================================
static void vdma1_s2mm_start(u32 fb0, u32 fb1, u32 fb2)
{
    // Reset
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_DMACR, 0x00000004);
    u32 timeout = 10000000;
    while ((Xil_In32(VDMA1_BASEADDR + VDMA_S2MM_DMACR) & 0x00000004) != 0) {
        if (--timeout == 0) {
            xil_printf("[VDMA1-W] reset timeout!\r\n");
            return;
        }
    }

    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_DMACR, 0x0001408B);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_START_ADDR1, fb0);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_START_ADDR2, fb1);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_START_ADDR3, fb2);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_FRMDLY_STRIDE, VDMA1_STRIDE);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_HSIZE, VDMA1_HSIZE);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_VSIZE, VDMA1_H);

    xil_printf("[VDMA1-W] start done\r\n");
}

// ============================================================
// VDMA1 MM2S (Read) 함수 - prev frame을 DDR에서 읽어서 Frame Diff IP로 전달
// ============================================================
static void vdma1_mm2s_start(u32 fb0, u32 fb1, u32 fb2)
{
    // Reset
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_DMACR, 0x00000004);
    u32 timeout = 10000000;
    while ((Xil_In32(VDMA1_BASEADDR + VDMA_MM2S_DMACR) & 0x00000004) != 0) {
        if (--timeout == 0) {
            xil_printf("[VDMA1-R] reset timeout!\r\n");
            return;
        }
    }

    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_DMACR, 0x0001408B);
    // Write와 같은 FB 주소를 읽음 (이전에 저장된 프레임 = prev)
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_START_ADDR1, fb0);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_START_ADDR2, fb1);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_START_ADDR3, fb2);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_FRMDLY_STRIDE, VDMA1_STRIDE);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_HSIZE, VDMA1_HSIZE);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_VSIZE, VDMA1_H);

    xil_printf("[VDMA1-R] start done\r\n");
}

// ============================================================
// VDMA2 S2MM (Write) 함수 - diff 결과를 DDR에 저장
// ============================================================
static void vdma2_s2mm_start(u32 fb0, u32 fb1, u32 fb2)
{
    // Reset
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_DMACR, 0x00000004);
    u32 timeout = 10000000;
    while ((Xil_In32(VDMA2_BASEADDR + VDMA_S2MM_DMACR) & 0x00000004) != 0) {
        if (--timeout == 0) {
            xil_printf("[VDMA2] reset timeout!\r\n");
            return;
        }
    }

    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_DMACR, 0x00000003);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_START_ADDR1, fb0);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_START_ADDR2, fb1);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_START_ADDR3, fb2);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_FRMDLY_STRIDE, VDMA2_STRIDE);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_HSIZE, VDMA2_HSIZE);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_VSIZE, VDMA2_H);

    xil_printf("[VDMA2] start done\r\n");
}
// threshold 설정
void load_threshold() {
    u32 val = (FC2_THR << 16) | FC1_THR;
    Xil_Out32(GPIO0_CH2, val);
}

// 가중치 1개 쓰기
void write_weight(u8 layer, u16 addr, int8_t data) {
    u32 val = ((u8)data << 13) | (addr << 2) | (layer << 1) | 1;
    Xil_Out32(GPIO0_CH1, val);
    // w_wr_en 내리기
    Xil_Out32(GPIO0_CH1, 0);
}

// 전체 가중치 로드
void load_weights() {
    xil_printf("Loading FC1 weights...\r\n");
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 144; j++) {
            write_weight(0, i * 144 + j, fc1_weight[i][j]);
        }
    }

    xil_printf("Loading FC2 weights...\r\n");
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 8; j++) {
            write_weight(1, i * 8 + j, fc2_weight[i][j]);
        }
    }

    xil_printf("All weights loaded!\r\n");
}

// 결과 읽기
void read_snn_result() {
    u32 val = Xil_In32(GPIO1_DATA);
    u8 done   = val & 0x01;
    u8 result = (val >> 1) & 0x01;
    u8 spk0   = (val >> 2) & 0x1F;
    u8 spk1   = (val >> 7) & 0x1F;
    xil_printf("SNN: done=%d result=%s spk0=%d spk1=%d\r\n",
               done, result ? "NON-FLY" : "FLY", spk0, spk1);
}


// ============================================================
// 기존 프로젝트 설정
// ============================================================
#define IRPT_CTL_DEVID      XPAR_XSCUGIC_0_BASEADDR
#define GPIO_DEVID           XPAR_GPIO0_BASEADDR
#define GPIO_IRPT_ID         XPAR_PS7_GPIO_0_INTR
#define CAM_I2C_DEVID        XPAR_I2C0_BASEADDR
#define CAM_I2C_IRPT_ID      XPAR_PS7_I2C_0_INTR
#define VDMA_DEVID           XPAR_AXI_VDMA_0_BASEADDR

#define VDMA_MM2S_IRPT_ID   XPAR_FABRIC_AXI_VDMA_0_INTR
#define VDMA_S2MM_IRPT_ID   XPAR_FABRIC_AXI_VDMA_0_INTR_1
#define CAM_I2C_SCLK_RATE   100000

#define DDR_BASE_ADDR        XPAR_DDR_MEM_BASEADDR
#define MEM_BASE_ADDR        (DDR_BASE_ADDR + 0x0A000000)

#define GAMMA_BASE_ADDR      XPAR_AXI_GAMMACORRECTION_0_BASEADDR

#define DEBUG_EN             0x0
#define UART_BASEADDR        XPS_UART0_BASEADDR

using namespace digilent;

void dFlushUart();
uint8_t dGetChar();
void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
                          OV5640& cam,
                          VideoOutput& vid,
                          Resolution res,
                          OV5640_cfg::mode_t mode);

// ============================================================
// main
// ============================================================
int main() 
{
    ScuGicInterruptController irpt_ctl(IRPT_CTL_DEVID);
    PS_GPIO<ScuGicInterruptController> gpio_driver(GPIO_DEVID, irpt_ctl, GPIO_IRPT_ID);
    PS_IIC<ScuGicInterruptController> iic_driver(CAM_I2C_DEVID, irpt_ctl, CAM_I2C_IRPT_ID, CAM_I2C_SCLK_RATE);
    OV5640 cam(iic_driver, gpio_driver);
    AXI_VDMA<ScuGicInterruptController> vdma_driver(VDMA_DEVID, MEM_BASE_ADDR, irpt_ctl, VDMA_MM2S_IRPT_ID, VDMA_S2MM_IRPT_ID);
    VideoOutput vid(XPAR_VTG_BASEADDR, XPAR_VIDEO_DYNCLK_BASEADDR);

    // 카메라 + HDMI 파이프라인 초기화
    pipeline_mode_change(vdma_driver, cam, vid,
        Resolution::R1280_720_60_PP,
        OV5640_cfg::mode_t::MODE_720P_1280_720_60fps);
    xil_printf("Video init done.\r\n");

    xil_printf("VDMA0 S2MM DMACR = 0x%08x\r\n", Xil_In32(XPAR_AXI_VDMA_0_BASEADDR + 0x30));
    xil_printf("VDMA0 MM2S DMACR = 0x%08x\r\n", Xil_In32(XPAR_AXI_VDMA_0_BASEADDR + 0x00));

    load_threshold();
    load_weights();
    // VDMA1 Write 시작 (gray8 원본 → DDR)
    xil_printf("Starting VDMA1 Write...\r\n");
    vdma1_s2mm_start(VDMA1_FB0, VDMA1_FB1, VDMA1_FB2);

    // VDMA2 Write 시작 (diff 결과 → DDR)
    xil_printf("Starting VDMA2 Write...\r\n");
    vdma2_s2mm_start(VDMA2_FB0, VDMA2_FB1, VDMA2_FB2);

    // 최소 1프레임 저장 대기
    for (volatile int i = 0; i < 5000000; ++i) { }

    // VDMA1 Read 시작 (prev frame DDR → Frame Diff IP)
    xil_printf("Starting VDMA1 Read...\r\n");
    vdma1_mm2s_start(VDMA1_FB0, VDMA1_FB1, VDMA1_FB2);

    xil_printf("All VDMAs started.\r\n");

    // ============================================================
    // 메뉴 루프
    // ============================================================
    uint8_t read_char0 = 0;
    uint8_t read_char1 = 0;
    uint8_t read_char2 = 0;
    uint8_t read_char4 = 0;
    uint8_t read_char5 = 0;
    uint16_t reg_addr;
    uint8_t reg_value;

    while (1)
    {
        xil_printf("\r\n\r\n\r\nPcam 5C MAIN OPTIONS\r\n");
        xil_printf("\r\nPlease press the key corresponding to the desired option:");
        xil_printf("\r\n  a. Change Resolution");
        xil_printf("\r\n  b. Change Liquid Lens Focus");
        xil_printf("\r\n  d. Change Image Format (Raw or RGB)");
        xil_printf("\r\n  e. Write a Register Inside the Image Sensor");
        xil_printf("\r\n  f. Read a Register Inside the Image Sensor");
        xil_printf("\r\n  g. Change Gamma Correction Factor Value");
        xil_printf("\r\n  h. Change AWB Settings");
        xil_printf("\r\n  z. Dump diff image (frame difference)");
        xil_printf("\r\n  y. Dump gray8 original image\r\n\r\n");

        read_char0 = getchar(); getchar();
        xil_printf("Read: %d\r\n", read_char0);



        switch (read_char0)
        {
        case 'a':
            xil_printf("\r\n  Please press the key corresponding to the desired resolution:");
            xil_printf("\r\n    1. 1280 x 720, 60fps");
            xil_printf("\r\n    2. 1920 x 1080, 15fps");
            xil_printf("\r\n    3. 1920 x 1080, 30fps");

            read_char1 = getchar(); getchar();
            xil_printf("\r\nRead: %d", read_char1);

            switch (read_char1)
            {
            case '1':
                pipeline_mode_change(vdma_driver, cam, vid,
                    Resolution::R1280_720_60_PP,
                    OV5640_cfg::mode_t::MODE_720P_1280_720_60fps);
                xil_printf("Resolution change done.\r\n");
                break;
            case '2':
                pipeline_mode_change(vdma_driver, cam, vid,
                    Resolution::R1920_1080_60_PP,
                    OV5640_cfg::mode_t::MODE_1080P_1920_1080_15fps);
                xil_printf("Resolution change done.\r\n");
                break;
            case '3':
                pipeline_mode_change(vdma_driver, cam, vid,
                    Resolution::R1920_1080_60_PP,
                    OV5640_cfg::mode_t::MODE_1080P_1920_1080_30fps);
                xil_printf("Resolution change done.\r\n");
                break;
            default:
                xil_printf("\r\n  Selection is outside the available options! Please retry...");
            }
            break;

        case 'b':
            xil_printf("\r\n\r\nPlease enter value of liquid lens register, in hex, with small letters (2 nibbles): 0x");
            while (read_char1 < 48) { read_char1 = getchar(); }
            while (read_char2 < 48) { read_char2 = getchar(); getchar(); }
            if (read_char1 <= 57) read_char1 -= 48; else read_char1 -= 87;
            if (read_char2 <= 57) read_char2 -= 48; else read_char2 -= 87;
            cam.writeRegLiquid((uint8_t)(16*read_char1 + read_char2));
            xil_printf("\r\nWrote to liquid lens controller: %x", (uint8_t)(16*read_char1 + read_char2));
            break;

        case 'd':
            xil_printf("\r\n  Please press the key corresponding to the desired setting:");
            xil_printf("\r\n    1. Select image format to be RGB, output still Raw");
            xil_printf("\r\n    2. Select image format & output to both be Raw");
            read_char1 = getchar(); getchar();
            xil_printf("\r\nRead: %d", read_char1);
            switch (read_char1)
            {
            case '1':
                cam.set_isp_format(OV5640_cfg::isp_format_t::ISP_RGB);
                xil_printf("Settings change done.\r\n");
                break;
            case '2':
                cam.set_isp_format(OV5640_cfg::isp_format_t::ISP_RAW);
                xil_printf("Settings change done.\r\n");
                break;
            default:
                xil_printf("\r\n  Selection is outside the available options! Please retry...");
            }
            break;

        case 'e':
            xil_printf("\r\nPlease enter address of image sensor register, in hex, with small letters (4 nibbles): \r\n");
            while (read_char1 < 48) { read_char1 = getchar(); }
            while (read_char2 < 48) { read_char2 = getchar(); }
            while (read_char4 < 48) { read_char4 = getchar(); }
            while (read_char5 < 48) { read_char5 = getchar(); getchar(); }
            if (read_char1 <= 57) read_char1 -= 48; else read_char1 -= 87;
            if (read_char2 <= 57) read_char2 -= 48; else read_char2 -= 87;
            if (read_char4 <= 57) read_char4 -= 48; else read_char4 -= 87;
            if (read_char5 <= 57) read_char5 -= 48; else read_char5 -= 87;
            reg_addr = 16*(16*(16*read_char1 + read_char2)+read_char4)+read_char5;
            xil_printf("Desired Register Address: %x\r\n", reg_addr);

            read_char1 = 0;
            read_char2 = 0;
            xil_printf("\r\nPlease enter value of image sensor register, in hex, with small letters (2 nibbles): \r\n");
            while (read_char1 < 48) { read_char1 = getchar(); }
            while (read_char2 < 48) { read_char2 = getchar(); getchar(); }
            if (read_char1 <= 57) read_char1 -= 48; else read_char1 -= 87;
            if (read_char2 <= 57) read_char2 -= 48; else read_char2 -= 87;
            reg_value = 16*read_char1 + read_char2;
            xil_printf("Desired Register Value: %x\r\n", reg_value);
            cam.writeReg(reg_addr, reg_value);
            xil_printf("Register write done.\r\n");
            break;

        case 'f':
            xil_printf("Please enter address of image sensor register, in hex, with small letters (4 nibbles): \r\n");
            while (read_char1 < 48) { read_char1 = getchar(); }
            while (read_char2 < 48) { read_char2 = getchar(); }
            while (read_char4 < 48) { read_char4 = getchar(); }
            while (read_char5 < 48) { read_char5 = getchar(); getchar(); }
            if (read_char1 <= 57) read_char1 -= 48; else read_char1 -= 87;
            if (read_char2 <= 57) read_char2 -= 48; else read_char2 -= 87;
            if (read_char4 <= 57) read_char4 -= 48; else read_char4 -= 87;
            if (read_char5 <= 57) read_char5 -= 48; else read_char5 -= 87;
            reg_addr = 16*(16*(16*read_char1 + read_char2)+read_char4)+read_char5;
            xil_printf("Desired Register Address: %x\r\n", reg_addr);
            cam.readReg(reg_addr, reg_value);
            xil_printf("Value of Desired Register: %x\r\n", reg_value);
            break;

        case 'g':
            xil_printf("  Please press the key corresponding to the desired Gamma factor:\r\n");
            xil_printf("    1. Gamma Factor = 1\r\n");
            xil_printf("    2. Gamma Factor = 1/1.2\r\n");
            xil_printf("    3. Gamma Factor = 1/1.5\r\n");
            xil_printf("    4. Gamma Factor = 1/1.8\r\n");
            xil_printf("    5. Gamma Factor = 1/2.2\r\n");
            read_char1 = getchar(); getchar();
            xil_printf("Read: %d\r\n", read_char1);
            read_char1 = read_char1 - 48;
            if ((read_char1 > 0) && (read_char1 < 6)) {
                Xil_Out32(GAMMA_BASE_ADDR, read_char1-1);
                xil_printf("Gamma value changed to option %d.\r\n", read_char1);
            } else {
                xil_printf("  Selection is outside the available options! Please retry...\r\n");
            }
            break;

        case 'h':
            xil_printf("  Please press the key corresponding to the desired AWB change:\r\n");
            xil_printf("    1. Enable Advanced AWB\r\n");
            xil_printf("    2. Enable Simple AWB\r\n");
            xil_printf("    3. Disable AWB\r\n");
            read_char1 = getchar(); getchar();
            xil_printf("Read: %d\r\n", read_char1);
            switch (read_char1)
            {
            case '1': cam.set_awb(OV5640_cfg::awb_t::AWB_ADVANCED); xil_printf("Enabled Advanced AWB\r\n"); break;
            case '2': cam.set_awb(OV5640_cfg::awb_t::AWB_SIMPLE);   xil_printf("Enabled Simple AWB\r\n");   break;
            case '3': cam.set_awb(OV5640_cfg::awb_t::AWB_DISABLED); xil_printf("Disabled AWB\r\n");         break;
            default:  xil_printf("  Selection is outside the available options! Please retry...\r\n");
            }
            break;

        // ============================================================
        // Dump: diff 결과 (VDMA2)
        // ============================================================
        case 'z':
        {
            xil_printf("P5\n160 90\n255\n");
            Xil_DCacheInvalidateRange((INTPTR)VDMA2_FB0, VDMA2_FRAME_BYTES);
            u8 *img_ptr = (u8 *)VDMA2_FB0;
            for (u32 i = 0; i < VDMA2_FRAME_BYTES; i++) {
                outbyte(img_ptr[i]);
            }
            break;
        }

        // ============================================================
        // Dump: gray8 원본 (VDMA1)
        // ============================================================
        case 'y':
        {
            xil_printf("P5\n160 90\n255\n");
            Xil_DCacheInvalidateRange((INTPTR)VDMA1_FB0, VDMA1_FRAME_BYTES);
            u8 *img_ptr = (u8 *)VDMA1_FB0;
            for (u32 i = 0; i < VDMA1_FRAME_BYTES; i++) {
                outbyte(img_ptr[i]);
            }
            break;
        }
        case 'r':
        {
            read_snn_result();
            break;
        }
        case 's':
{
    xil_printf("Continuous SNN monitoring... (reset to stop)\r\n");
    while (1) {
        u32 val = Xil_In32(GPIO1_DATA);
        u8 done   = val & 0x01;
        u8 result = (val >> 1) & 0x01;
        u8 spk0   = (val >> 2) & 0x1F;
        u8 spk1   = (val >> 7) & 0x1F;
        xil_printf("done=%d result=%s spk0=%d spk1=%d\r\n",
                   done, result ? "NON-FLY" : "FLY", spk0, spk1);
        for (volatile int i = 0; i < 5000000; ++i) { }
    }
    break;
}


        default:
            xil_printf("  Selection is outside the available options! Please retry...\r\n");
        }

        read_char1 = 0;
        read_char2 = 0;
        read_char4 = 0;
        read_char5 = 0;
    }

    return 0;
}

// ============================================================
// UART 유틸리티
// ============================================================
void dFlushUart()
{
    while (XUartPs_IsReceiveData(UART_BASEADDR))
        XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
}

uint8_t dGetChar()
{
    uint8_t chRxCh = '0';
    while (XUartPs_IsReceiveData(UART_BASEADDR))
    {
        chRxCh = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
        if (chRxCh == '\n') break;
    }
    return chRxCh;
}

// ============================================================
// 카메라 파이프라인 초기화
// ============================================================
void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
                          OV5640& cam, VideoOutput& vid,
                          Resolution res, OV5640_cfg::mode_t mode)
{
    vdma_driver.resetWrite();
    MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
    MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
#if (DEBUG_EN == 0x0)
    cam.reset();
#endif
    vdma_driver.configureWrite(timing[static_cast<int>(res)].h_active, timing[static_cast<int>(res)].v_active);
    Xil_Out32(GAMMA_BASE_ADDR, 3);
#if (DEBUG_EN == 0x0)
    cam.init();
    vdma_driver.enableWrite();
#endif
    MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
    MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
#if (DEBUG_EN == 0x0)
    cam.set_mode(mode);
    cam.set_awb(OV5640_cfg::awb_t::AWB_ADVANCED);
#endif
    vid.reset();
    vdma_driver.resetRead();
    vid.configure(res);
    vdma_driver.configureRead(timing[static_cast<int>(res)].h_active, timing[static_cast<int>(res)].v_active);
    vid.enable();
    vdma_driver.enableRead();
}