`include "DAP_Cmd.v"

module DAP_SWJ #(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    )(
        input clk,
        input resetn,
        input [31:0] clk_timer,
        input enable,

        // 串行时钟
        input sclk,
        input sclk_out,
        input sclk_negedge,
        input sclk_sampling,
        output sclk_sampling_en,

        // AHB MEM接口
        input ahb_write_en,
        input ahb_read_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        input dap_in_tvalid,
        output [`CMD_SWJ_RANGE] dap_in_tready,
        input [7:0] dap_in_tdata,

        output reg [9:0] ram_write_addr,
        output reg [7:0] ram_write_data,
        output reg ram_write_en,
        output reg [9:0] packet_len,

        input [`CMD_SWJ_RANGE] start,
        output reg [`CMD_SWJ_RANGE] done,

        output SWCLK_TCK_O,
        output SWDIO_TMS_T,
        output SWDIO_TMS_O,
        input SWDIO_TMS_I,
        input SWO_TDO_I,
        output TDI_O,
        // input RTCK_I,
        input SRST_I,
        output SRST_O,
        input TRST_I,
        output TRST_O,

        // swd模式信号
        output SWD_MODE
    );
    genvar gi;
    integer i;

    localparam [ADDRWIDTH-1:0] SWJ_CR_ADDR            = BASE_ADDR + 12'h000;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_WAIT_RETRY_ADDR    = BASE_ADDR + 12'h004;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_MATCH_RETRY_ADDR   = BASE_ADDR + 12'h006;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_SWD_CR_ADDR        = BASE_ADDR + 12'h008;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_CR_ADDR       = BASE_ADDR + 12'h00C;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF0_ADDR = BASE_ADDR + 12'h010;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF1_ADDR = BASE_ADDR + 12'h014;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF2_ADDR = BASE_ADDR + 12'h018;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF3_ADDR = BASE_ADDR + 12'h01C;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF4_ADDR = BASE_ADDR + 12'h020;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF5_ADDR = BASE_ADDR + 12'h024;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF6_ADDR = BASE_ADDR + 12'h028;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF7_ADDR = BASE_ADDR + 12'h02C;  // RW

    localparam [1:0] SEQ_STATUS_IDLE = 2'd0;    // 空闲状态
    localparam [1:0] SEQ_STATUS_BUSY = 2'd1;    // 忙状态
    localparam [1:0] SEQ_STATUS_DONE = 2'd2;    // 完成状态

    reg SWJ_CR;
    reg [15:0] SWJ_WAIT_RETRY;
    reg [15:0] SWJ_MATCH_RETRY;
    wire SWJ_CR_MODE = SWJ_CR; // 1: SWD; 0: JTAG


    reg [3:0] SWJ_SWD_CR;
    reg [3:0] SWJ_JTAG_CR;
    reg [19:0] SWJ_JTAG_IR_CONF_REG [0:7];

    wire [1:0] SWD_CONF_TURN = SWJ_SWD_CR[1:0];
    wire SWD_CONF_FORCE_DATA = SWJ_SWD_CR[2];
    wire SWD_CONF_TURN_CLK = SWJ_SWD_CR[3];

    wire [3:0] JTAG_CR_COUNT = SWJ_JTAG_CR[3:0];

    assign SWD_MODE = SWJ_CR_MODE;


    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            SWJ_CR <= 0;
            SWJ_SWD_CR <= 4'd0;
            SWJ_JTAG_CR <= 4'd0;
            for (i = 0; i < 8; i = i + 1) begin
                SWJ_JTAG_IR_CONF_REG[i] <= 20'd0;
            end
        end
        else begin
            if (ahb_write_en) begin
                case (ahb_addr[ADDRWIDTH-1:2])
                    SWJ_CR_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0]) begin
                            SWJ_CR <= ahb_wdata[0];
                        end
                    end
                    SWJ_WAIT_RETRY_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            SWJ_WAIT_RETRY[0+:8] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            SWJ_WAIT_RETRY[8+:8] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            SWJ_MATCH_RETRY[0+:8] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            SWJ_MATCH_RETRY[8+:8] <= ahb_wdata[24+:8];
                    end
                    SWJ_SWD_CR_ADDR[ADDRWIDTH-1:2]:
                        if (ahb_byte_strobe[0]) begin
                            SWJ_SWD_CR <= ahb_wdata[3:0];
                        end
                    SWJ_JTAG_CR_ADDR[ADDRWIDTH-1:2]:
                        if (ahb_byte_strobe[0]) begin
                            SWJ_JTAG_CR <= ahb_wdata[3:0];
                        end
                    SWJ_JTAG_IR_CONF0_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[0]);
                    SWJ_JTAG_IR_CONF1_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[1]);
                    SWJ_JTAG_IR_CONF2_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[2]);
                    SWJ_JTAG_IR_CONF3_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[3]);
                    SWJ_JTAG_IR_CONF4_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[4]);
                    SWJ_JTAG_IR_CONF5_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[5]);
                    SWJ_JTAG_IR_CONF6_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[6]);
                    SWJ_JTAG_IR_CONF7_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[7]);
                endcase
            end
        end
    end

    always @(*) begin : ahb_mem_read_ctrl
        if (ahb_read_en) begin
            case (ahb_addr[ADDRWIDTH-1:2]) /*synthesis parallel_case*/
                SWJ_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {31'd0, SWJ_CR};
                SWJ_WAIT_RETRY_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {SWJ_MATCH_RETRY, SWJ_WAIT_RETRY};
                SWJ_SWD_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_SWD_CR;
                SWJ_JTAG_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_CR;
                SWJ_JTAG_IR_CONF0_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[0];
                SWJ_JTAG_IR_CONF1_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[1];
                SWJ_JTAG_IR_CONF2_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[2];
                SWJ_JTAG_IR_CONF3_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[3];
                SWJ_JTAG_IR_CONF4_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[4];
                SWJ_JTAG_IR_CONF5_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[5];
                SWJ_JTAG_IR_CONF6_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[6];
                SWJ_JTAG_IR_CONF7_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[7];
                default:
                    ahb_rdata = {32{1'bx}};
            endcase
        end
        else begin
            ahb_rdata = {32{1'bx}};
        end
    end

    reg [15:0] seq_tx_cmd;
    reg [31:0] seq_tx_data;
    reg seq_tx_valid;
    wire seq_tx_full;
    reg [2:0] jtag_index;

    wire seq_rx_valid;
    wire [15:0] seq_rx_flag;
    wire [31:0] seq_rx_data;


    localparam [1:0] SWJ_SEQ_SM_READ_NUM = 2'd0;
    localparam [1:0] SWJ_SEQ_SM_READ_DATA = 2'd1;
    localparam [1:0] SWJ_SEQ_SM_WAIT = 2'd2;
    localparam [1:0] SWJ_SEQ_SM_DONE = 2'd3;


    reg [8:0] swj_seq_bit_num;
    reg [3:0] swj_seq_trans_num;
    reg [1:0] swj_seq_sm;
    reg [7:0] swj_seq_data;
    reg swj_seq_tx_valid;

    localparam [2:0] SWD_SEQ_SM_READ_SEQ_NUM = 3'd0;
    localparam [2:0] SWD_SEQ_SM_READ_SEQ_INFO = 3'd1;
    localparam [2:0] SWD_SEQ_SM_READ_DATA = 3'd2;
    localparam [2:0] SWD_SEQ_SM_WAIT = 3'd3;
    localparam [2:0] SWD_SEQ_SM_DONE = 3'd4;

    reg [2:0] swd_seq_sm;
    reg [7:0] swd_seq_num;
    reg swd_seq_dir;
    reg [6:0] swd_seq_bit_num;
    reg [3:0] swd_seq_trans_num;
    reg [7:0] swd_seq_send_data;
    reg swd_seq_tx_valid;


    localparam [2:0] WRITE_ABORT_READ_INDEX = 3'd0;
    localparam [2:0] WRITE_ABORT_READ_DATA0 = 3'd1;
    localparam [2:0] WRITE_ABORT_READ_DATA1 = 3'd2;
    localparam [2:0] WRITE_ABORT_READ_DATA2 = 3'd3;
    localparam [2:0] WRITE_ABORT_READ_DATA3 = 3'd4;
    localparam [2:0] WRITE_ABORT_WAIT_RESPONE = 3'd5;
    localparam [2:0] WRITE_ABORT_DONE = 3'd6;

    reg [2:0] write_abort_sm;
    reg [31:0] write_abort_data;

    localparam [2:0] SWJ_PINS_SM_READ_OUTPUT = 3'd0;
    localparam [2:0] SWJ_PINS_SM_READ_SELECT = 3'd1;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT0 = 3'd2;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT1 = 3'd3;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT2 = 3'd4;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT3 = 3'd5;
    localparam [2:0] SWJ_PINS_SM_WAIT_RESPONE = 3'd6;
    localparam [2:0] SWJ_PINS_SM_DONE = 3'd7;

    reg [2:0] swj_pins_sm;
    reg [5:0] swj_pins_output;
    reg [5:0] swj_pins_select;
    reg [31:0] swj_pins_wait_time;

    localparam [17:0] TRANS_BLOCK_SM_READ_INDEX          = 20'h0_0001;
    localparam [17:0] TRANS_BLOCK_SM_READ_COUNT_L        = 20'h0_0002;
    localparam [17:0] TRANS_BLOCK_SM_READ_COUNT_H        = 20'h0_0004;
    localparam [17:0] TRANS_BLOCK_SM_READ_REQUSET        = 20'h0_0008;
    localparam [17:0] TRANS_BLOCK_SM_READ_DATA0          = 20'h0_0010;
    localparam [17:0] TRANS_BLOCK_SM_READ_DATA1          = 20'h0_0020;
    localparam [17:0] TRANS_BLOCK_SM_READ_DATA2          = 20'h0_0040;
    localparam [17:0] TRANS_BLOCK_SM_READ_DATA3          = 20'h0_0080;
    localparam [17:0] TRANS_BLOCK_SM_PROCESS_REQ         = 20'h0_0100;
    localparam [17:0] TRANS_BLOCK_SM_TRANSFER            = 20'h0_0200;
    localparam [17:0] TRANS_BLOCK_SM_WAIT_RDBUFF         = 20'h0_0400;
    localparam [17:0] TRANS_BLOCK_SM_WRITE_DATA_0        = 20'h0_0800;
    localparam [17:0] TRANS_BLOCK_SM_WRITE_DATA_1        = 20'h0_1000;
    localparam [17:0] TRANS_BLOCK_SM_WRITE_DATA_2        = 20'h0_2000;
    localparam [17:0] TRANS_BLOCK_SM_WRITE_DATA_3        = 20'h0_4000;
    localparam [17:0] TRANS_BLOCK_SM_WRITE_COUNT_L       = 20'h0_8000;
    localparam [17:0] TRANS_BLOCK_SM_WRITE_COUNT_H       = 20'h1_0000;
    localparam [17:0] TRANS_BLOCK_SM_WRITE_RESPONSE      = 20'h2_0000;
    localparam [17:0] TRANS_BLOCK_SM_END                 = 20'h0_0000;


    reg [17:0] transfer_block_sm /*synthesis syn_encoding="onehot"*/;
    reg [15:0] transfer_block_request_cnt;
    reg [15:0] transfer_block_response_cnt;
    reg [3:0] transfer_block_requset;
    reg [31:0] transfer_block_data;
    reg transfer_block_need_post_read;
    reg transfer_block_need_rdbuff;
    reg transfer_block_seq_tx_valid;
    reg transfer_block_RnW;
    reg transfer_block_APnDP;
    reg transfer_block_err_flag;
    reg transfer_block_ir;

    localparam [26:0] TRANS_SM_READ_INDEX          = 27'h000_0001;
    localparam [26:0] TRANS_SM_READ_COUNT          = 27'h000_0002;
    localparam [26:0] TRANS_SM_READ_REQUSET        = 27'h000_0004;
    localparam [26:0] TRANS_SM_READ_DATA0          = 27'h000_0008;
    localparam [26:0] TRANS_SM_READ_DATA1          = 27'h000_0010;
    localparam [26:0] TRANS_SM_READ_DATA2          = 27'h000_0020;
    localparam [26:0] TRANS_SM_READ_DATA3          = 27'h000_0040;
    localparam [26:0] TRANS_SM_CHECK_POSTREAD      = 27'h000_0080;
    localparam [26:0] TRANS_SM_READ_AP             = 27'h000_0100;
    localparam [26:0] TRANS_SM_READ_DP             = 27'h000_0200;
    localparam [26:0] TRANS_SM_WRITE_APDP          = 27'h000_0400;
    localparam [26:0] TRANS_SM_MATCH_STEP1         = 27'h000_0800;
    localparam [26:0] TRANS_SM_MATCH_STEP2         = 27'h000_1000;
    localparam [26:0] TRANS_SM_MATCH_STEP3         = 27'h000_2000;
    localparam [26:0] TRANS_SM_CHECK_WIRTE         = 27'h000_4000;
    localparam [26:0] TRANS_SM_WRTE_COUNT          = 27'h000_8000;
    localparam [26:0] TRANS_SM_WRTE_STATUS         = 27'h001_0000;
    localparam [26:0] TRANS_SM_TRIGGER             = 27'h002_0000;
    localparam [26:0] TRANS_SM_WAIT                = 27'h004_0000;
    localparam [26:0] TRANS_SM_WRITE_DATA0         = 27'h008_0000;
    localparam [26:0] TRANS_SM_WRITE_DATA1         = 27'h010_0000;
    localparam [26:0] TRANS_SM_WRITE_DATA2         = 27'h020_0000;
    localparam [26:0] TRANS_SM_WRITE_DATA3         = 27'h040_0000;
    localparam [26:0] TRANS_SM_WRITE_TIME0         = 27'h080_0000;
    localparam [26:0] TRANS_SM_WRITE_TIME1         = 27'h100_0000;
    localparam [26:0] TRANS_SM_WRITE_TIME2         = 27'h200_0000;
    localparam [26:0] TRANS_SM_WRITE_TIME3         = 27'h400_0000;
    localparam [26:0] TRANS_SM_DONE                = 27'h000_0000;

    reg transfer_post_read;
    reg transfer_check_write;
    reg transfer_has_error;
    reg transfer_match_failed;
    reg [31:0] transfer_wdata;
    reg [26:0] transfer_sm /*synthesis syn_encoding="onehot"*/;
    reg [31:0] transfer_match_mask;
    reg [7:0] transfer_num;
    reg [7:0] transfer_cnt;
    reg [7:0] transfer_requset;
    reg [31:0] transfer_timestamp;
    reg [15:0] transfer_match_cnt;
    reg transfer_seq_tx_valid;

    reg [3:0] transfer_trig_requset;
    reg [26:0] transfer_trig_ret_sm;
    reg transfer_trig_en_wdata; // 启用数据段
    reg transfer_trig_en_wtime; // 启用时间戳段
    reg transfer_trig_en_cnt;   // 传输成功时完成计数增加
    reg transfer_trig_wtime_first; // 先写入时间戳，同时有data和time全开的效果
    reg transfer_trig_ir;
    reg transfer_trig_first_transfer;

    // Bit 0: APnDP: 0 = Debug Port (DP), 1 = Access Port (AP).
    // Bit 1: RnW: 0 = Write Register, 1 = Read Register.
    // Bit 2: A2 Register Address bit 2.
    // Bit 3: A3 Register Address bit 3.
    // Bit 4: Value Match
    // Bit 5: Match Mask
    // Bit 7: TD_TimeStamp request
    wire transfer_requset_APnDP = transfer_requset[0];
    wire transfer_requset_RnW = transfer_requset[1];
    wire transfer_requset_MATCH = transfer_requset[4];
    wire transfer_requset_MASK = transfer_requset[5];
    wire transfer_requset_TIMESTAMP = transfer_requset[7];


    // JTAG序列控制器状态机
    localparam [2:0] JTAG_SEQ_SM_READ_SEQ_COUNT = 3'd0;  // 读取序列数量
    localparam [2:0] JTAG_SEQ_SM_READ_SEQ_INFO = 3'd1;    // 读取序列信息（TCK周期数、TMS值、TDO捕获）
    localparam [2:0] JTAG_SEQ_SM_READ_DATA = 3'd2;        // 读取TDI数据
    localparam [2:0] JTAG_SEQ_SM_WAIT = 3'd3;             // 等待序列完成
    localparam [2:0] JTAG_SEQ_SM_DONE = 3'd4;             // 完成

    reg [2:0] jtag_seq_sm;
    reg jtag_seq_tms;
    reg jtag_seq_capture_tdo;
    reg [7:0] jtag_seq_num;          // 剩余序列数量
    reg [6:0] jtag_seq_bit_num;       // 当前序列的总位数
    reg [3:0] jtag_seq_trans_num;     // 当前块传输的位数（最多8位）
    reg [7:0] jtag_seq_send_data;     // 要发送的TDI数据
    reg jtag_seq_tx_valid;

    // JTAG IDCODE控制器状态机
    localparam [2:0] JTAG_IDCODE_SM_WAIT_INDEX = 3'd0;  // 等待Index
    localparam [2:0] JTAG_IDCODE_SM_WRITE_DATA0 = 3'd1;   // 等待IDCODE
    localparam [2:0] JTAG_IDCODE_SM_WRITE_DATA1 = 3'd2; // 写入数据1
    localparam [2:0] JTAG_IDCODE_SM_WRITE_DATA2 = 3'd3; // 写入数据2
    localparam [2:0] JTAG_IDCODE_SM_WRITE_DATA3 = 3'd4; // 写入数据3
    localparam [2:0] JTAG_IDCODE_SM_DONE = 3'd5;       // 完成

    reg [2:0] jtag_idcode_sm;
    reg jtag_idcode_seq_tx_valid;


    // RAM写入信号 - 每个状态机独立的寄存器
    // 注意: swj_seq, write_abort, swj_pins的packet_len始终为1
    //       所以这些状态机不需要packet_len寄存器
    reg [9:0] swj_seq_ram_write_addr;
    reg [7:0] swj_seq_ram_write_data;
    reg swj_seq_ram_write_en;

    reg [9:0] swd_seq_ram_write_addr;
    reg [7:0] swd_seq_ram_write_data;
    reg swd_seq_ram_write_en;
    reg [9:0] swd_seq_packet_len;

    reg [9:0] write_abort_ram_write_addr;
    reg [7:0] write_abort_ram_write_data;
    reg write_abort_ram_write_en;

    reg [9:0] swj_pins_ram_write_addr;
    reg [7:0] swj_pins_ram_write_data;
    reg swj_pins_ram_write_en;

    reg [9:0] transfer_block_ram_write_addr;
    reg [7:0] transfer_block_ram_write_data;
    reg transfer_block_ram_write_en;
    reg [9:0] transfer_block_packet_len;

    reg [9:0] transfer_ram_write_addr;
    reg [7:0] transfer_ram_write_data;
    reg transfer_ram_write_en;
    reg [9:0] transfer_packet_len;

    // JTAG序列控制器信号
    reg [9:0] jtag_seq_ram_write_addr;
    reg [7:0] jtag_seq_ram_write_data;
    reg jtag_seq_ram_write_en;

    // JTAG IDCODE控制器信号
    reg [9:0] jtag_idcode_ram_write_addr;
    reg [7:0] jtag_idcode_ram_write_data;
    reg jtag_idcode_ram_write_en;


    reg [`CMD_SWJ_RANGE] start_hold;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            start_hold <= 12'd0;
        end
        else begin
            if (start) begin
                start_hold <= start;
            end
        end
    end

    // wire [3:0] SEQ_CMD_TRANSFER = `SEQ_CMD_SWD_TRANSFER;
    wire [3:0] SEQ_CMD_TRANSFER = SWD_MODE ? `SEQ_CMD_SWD_TRANSFER : `SEQ_CMD_JTAG_TRANSFER;
    // Combinational logic for RAM write signals and seq_tx signals based on active state machine
    always @(*) begin
        // Default: no write
        ram_write_addr = 10'd0;
        ram_write_data = 8'd0;
        ram_write_en = 1'd0;
        packet_len = 10'd0;
        seq_tx_cmd = 16'd0;
        seq_tx_data = 32'd0;
        seq_tx_valid = 1'd0;

        casez ({start_hold, 2'd0}) /* synthesis parallel_case */
            `CMD_TRANSFER_BLOCK: begin
                ram_write_addr = transfer_block_ram_write_addr;
                ram_write_data = transfer_block_ram_write_data;
                ram_write_en = transfer_block_ram_write_en;
                packet_len = transfer_block_packet_len;
                // | 11 10 |      9 |    8  7  6 |     5 |  4 |  3 |  2 |   1 |     0 |
                // |  x  x | IDCODE | JTAG Index | Abort | IR | A3 | A2 | RnW | APnDP |
                seq_tx_cmd = {SEQ_CMD_TRANSFER, 2'd0, 1'd0, jtag_index, 1'd0, transfer_block_ir, transfer_block_requset};
                seq_tx_data = transfer_block_data;
                seq_tx_valid = transfer_block_seq_tx_valid;
            end
            `CMD_TRANSFER: begin
                ram_write_addr = transfer_ram_write_addr;
                ram_write_data = transfer_ram_write_data;
                ram_write_en = transfer_ram_write_en;
                packet_len = transfer_packet_len;
                // | 11 10 |      9 |    8  7  6 |     5 |  4 |  3 |  2 |   1 |     0 |
                // |  x  x | IDCODE | JTAG Index | Abort | IR | A3 | A2 | RnW | APnDP |
                seq_tx_cmd = {SEQ_CMD_TRANSFER, 2'd0, 1'd0, jtag_index, 1'd0, transfer_trig_ir, transfer_trig_requset};
                seq_tx_data = transfer_wdata;
                seq_tx_valid = (transfer_sm & TRANS_SM_TRIGGER) != 27'd0;
            end
            `CMD_WRITE_ABORT: begin
                ram_write_addr = write_abort_ram_write_addr;
                ram_write_data = write_abort_ram_write_data;
                ram_write_en = write_abort_ram_write_en;
                packet_len = 10'd1;  // Write Abort always returns 1 byte
                // | 11 10 |      9 |    8  7  6 |     5 |  4 |  3 |  2 |   1 |     0 |
                // |  x  x | IDCODE | JTAG Index | Abort | IR | A3 | A2 | RnW | APnDP |
                seq_tx_cmd = {SEQ_CMD_TRANSFER, 2'd0, 1'd0, jtag_index, 1'd1, 1'd1, 4'd0};
                seq_tx_data = write_abort_data;
                seq_tx_valid = (write_abort_sm == WRITE_ABORT_READ_DATA3) && dap_in_tvalid;
            end
            `CMD_SWJ_PINS: begin
                ram_write_addr = swj_pins_ram_write_addr;
                ram_write_data = swj_pins_ram_write_data;
                ram_write_en = swj_pins_ram_write_en;
                packet_len = 10'd1;  // SWJ Pins always returns 1 byte (pin status)
                seq_tx_cmd = {`SEQ_CMD_SWJ_PINS, swj_pins_output, swj_pins_select};
                seq_tx_data = swj_pins_wait_time;
                seq_tx_valid = (swj_pins_sm == SWJ_PINS_SM_READ_WAIT3) && dap_in_tvalid;
            end
            `CMD_SWJ_SEQUENCE: begin
                ram_write_addr = swj_seq_ram_write_addr;
                ram_write_data = swj_seq_ram_write_data;
                ram_write_en = swj_seq_ram_write_en;
                packet_len = 10'd1;  // SWJ Sequence always returns 1 byte
                seq_tx_cmd = {`SEQ_CMD_SWD_SEQ, 7'd0, 1'd0, swj_seq_trans_num};
                seq_tx_data = {24'd0, swj_seq_data};
                seq_tx_valid = swj_seq_tx_valid;
            end
            `CMD_SWD_SEQUENCE: begin
                ram_write_addr = swd_seq_ram_write_addr;
                ram_write_data = swd_seq_ram_write_data;
                ram_write_en = swd_seq_ram_write_en;
                packet_len = swd_seq_packet_len;
                seq_tx_cmd = {`SEQ_CMD_SWD_SEQ, 7'd0, swd_seq_dir, swd_seq_trans_num};
                seq_tx_data = {24'd0, swd_seq_send_data};
                seq_tx_valid = swd_seq_tx_valid;
            end
            `CMD_JTAG_SEQUENCE: begin
                ram_write_addr = jtag_seq_ram_write_addr;
                ram_write_data = jtag_seq_ram_write_data;
                ram_write_en = jtag_seq_ram_write_en;
                packet_len = jtag_seq_ram_write_addr + 1'd1;  // JTAG Sequence returns captured TDO data length
                seq_tx_cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, jtag_seq_tms, jtag_seq_trans_num};
                seq_tx_data = {24'd0, jtag_seq_send_data};
                seq_tx_valid = jtag_seq_tx_valid;
            end
            `CMD_JTAG_IDCODE: begin
                ram_write_addr = jtag_idcode_ram_write_addr;
                ram_write_data = jtag_idcode_ram_write_data;
                ram_write_en = jtag_idcode_ram_write_en;
                packet_len = 10'd5;
                // | 11 10 |      9 |    8  7  6 |     5 |  4 |  3 |  2 |   1 |     0 |
                // |  x  x | IDCODE | JTAG Index | Abort | IR | A3 | A2 | RnW | APnDP |
                seq_tx_cmd = {`SEQ_CMD_JTAG_TRANSFER, 3'h1, jtag_index, 6'h10};
                seq_tx_data = 32'd0;
                seq_tx_valid = jtag_idcode_seq_tx_valid;
            end
            default: begin
                // All signals already initialized to 0 above
            end
        endcase
    end

    // SWJ Sequence Controller
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            swj_seq_ram_write_en <= 1'd0;
            swj_seq_ram_write_data <= 8'd0;
            swj_seq_ram_write_addr <= 10'd0;
            swj_seq_bit_num <= 0;
            swj_seq_trans_num <= 0;
            swj_seq_sm <= 0;
            swj_seq_data <= 0;
            swj_seq_tx_valid <= 0;
            done[`CMD_SWJ_SEQUENCE_SHIFT] <= 1'd0;
        end
        else begin
            swj_seq_ram_write_en <= 1'd0;
            swj_seq_tx_valid <= 1'd0;

            if (start[`CMD_SWJ_SEQUENCE_SHIFT]) begin
                case (swj_seq_sm) /* synthesis parallel_case */
                    SWJ_SEQ_SM_READ_NUM: begin
                        if (dap_in_tvalid) begin
                            if (dap_in_tdata == 8'd0) begin
                                swj_seq_bit_num <= 9'd256;
                            end
                            else begin
                                swj_seq_bit_num <= dap_in_tdata;
                            end
                            swj_seq_sm <= SWJ_SEQ_SM_READ_DATA;
                        end
                    end
                    SWJ_SEQ_SM_READ_DATA: begin
                        if (dap_in_tvalid) begin
                            swj_seq_data <= dap_in_tdata;
                            swj_seq_tx_valid <= 1'd1;
                            swj_seq_trans_num <= (swj_seq_bit_num > 9'd8) ? 4'd8 : swj_seq_bit_num;
                            swj_seq_sm <= SWJ_SEQ_SM_WAIT;
                        end
                    end
                    SWJ_SEQ_SM_WAIT: begin
                        if (seq_rx_valid) begin
                            swj_seq_bit_num <= swj_seq_bit_num - swj_seq_trans_num;
                            if ((swj_seq_bit_num - swj_seq_trans_num) != 9'd0) begin
                                swj_seq_sm <= SWJ_SEQ_SM_READ_DATA;
                            end
                            else begin
                                swj_seq_ram_write_addr <= 10'd0;
                                swj_seq_ram_write_data <= 8'd0;
                                swj_seq_ram_write_en <= 1'd1;
                                swj_seq_sm <= SWJ_SEQ_SM_DONE;
                                done[`CMD_SWJ_SEQUENCE_SHIFT] <= 1'd1;
                            end
                        end
                    end
                    SWJ_SEQ_SM_DONE: begin
                    end
                endcase
            end
            else begin
                swj_seq_sm <= SWJ_SEQ_SM_READ_NUM;
                done[`CMD_SWJ_SEQUENCE_SHIFT] <= 1'd0;
            end
        end
    end

    // SWD Sequence Controller
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            swd_seq_ram_write_en <= 1'd0;
            swd_seq_ram_write_data <= 8'd0;
            swd_seq_ram_write_addr <= 10'd0;
            swd_seq_packet_len <= 1'd0;
            swd_seq_sm <= 0;
            swd_seq_num <= 0;
            swd_seq_dir <= 0;
            swd_seq_bit_num <= 0;
            swd_seq_trans_num <= 0;
            swd_seq_send_data <= 0;
            swd_seq_tx_valid <= 0;
            done[`CMD_SWD_SEQUENCE_SHIFT] <= 1'd0;
        end
        else begin
            swd_seq_ram_write_en <= 1'd0;
            swd_seq_tx_valid <= 1'd0;

            if (start[`CMD_SWD_SEQUENCE_SHIFT]) begin
                case (swd_seq_sm) /* synthesis parallel_case */
                    SWD_SEQ_SM_READ_SEQ_NUM: begin
                        if (dap_in_tvalid) begin
                            swd_seq_ram_write_addr <= 10'd0;
                            swd_seq_ram_write_data <= 8'd0;
                            swd_seq_ram_write_en <= 1'd1;
                            swd_seq_num <= dap_in_tdata;
                            swd_seq_sm <= SWD_SEQ_SM_READ_SEQ_INFO;
                        end
                    end
                    SWD_SEQ_SM_READ_SEQ_INFO: begin
                        if (dap_in_tvalid) begin
                            swd_seq_dir <= dap_in_tdata[7];
                            swd_seq_bit_num <= (dap_in_tdata[5:0] == 6'd0) ? 7'd64 : dap_in_tdata[5:0];
                            swd_seq_sm <= SWD_SEQ_SM_READ_DATA;
                        end
                    end
                    SWD_SEQ_SM_READ_DATA: begin
                        if (dap_in_tvalid || swd_seq_dir) begin
                            swd_seq_trans_num <= (swd_seq_bit_num > 9'd8) ? 4'd8 : swd_seq_bit_num;
                            swd_seq_send_data <= dap_in_tdata;
                            swd_seq_tx_valid <= 1'd1;
                            swd_seq_sm <= SWD_SEQ_SM_WAIT;
                        end
                    end
                    SWD_SEQ_SM_WAIT: begin
                        if (seq_rx_valid) begin
                            swd_seq_bit_num <= swd_seq_bit_num - swd_seq_trans_num;

                            if (swd_seq_dir) begin
                                swd_seq_ram_write_addr <= swd_seq_ram_write_addr + 1'd1;
                                swd_seq_ram_write_data <= seq_rx_data[7:0];
                                swd_seq_ram_write_en <= 1'd1;
                            end

                            if ((swd_seq_bit_num - swd_seq_trans_num) != 7'd0) begin
                                swd_seq_sm <= SWD_SEQ_SM_READ_DATA;
                            end
                            else begin
                                if (swd_seq_num - 1'd1 != 8'd0) begin
                                    swd_seq_num = swd_seq_num - 1'd1;
                                    swd_seq_sm <= SWD_SEQ_SM_READ_SEQ_INFO;
                                end
                                else begin
                                    swd_seq_sm <= SWD_SEQ_SM_DONE;
                                end
                            end
                        end
                    end
                    default: begin
                        swd_seq_packet_len <= swd_seq_ram_write_addr + 1'd1;
                        done[`CMD_SWD_SEQUENCE_SHIFT] <= 1'd1;
                    end
                endcase
            end
            else begin
                swd_seq_sm <= SWD_SEQ_SM_READ_SEQ_NUM;
                done[`CMD_SWD_SEQUENCE_SHIFT] <= 1'd0;
            end
        end
    end

    // SWD Write Abort Controller
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            write_abort_ram_write_en <= 1'd0;
            write_abort_ram_write_data <= 8'd0;
            write_abort_ram_write_addr <= 10'd0;
            write_abort_sm <= 3'd0;
            write_abort_data <= 32'd0;
            done[`CMD_WRITE_ABORT_SHIFT] <= 1'd0;
        end
        else begin
            write_abort_ram_write_en <= 1'd0;

            if (start[`CMD_WRITE_ABORT_SHIFT]) begin
                case (write_abort_sm) /* synthesis parallel_case */
                    WRITE_ABORT_READ_INDEX: begin
                        if (dap_in_tvalid) begin
                            write_abort_sm <= WRITE_ABORT_READ_DATA0;
                        end
                    end
                    WRITE_ABORT_READ_DATA0: begin
                        if (dap_in_tvalid) begin
                            write_abort_data[7:0] <= dap_in_tdata;
                            write_abort_sm <= WRITE_ABORT_READ_DATA1;
                        end
                    end
                    WRITE_ABORT_READ_DATA1: begin
                        if (dap_in_tvalid) begin
                            write_abort_data[15:8] <= dap_in_tdata;
                            write_abort_sm <= WRITE_ABORT_READ_DATA2;
                        end
                    end
                    WRITE_ABORT_READ_DATA2: begin
                        if (dap_in_tvalid) begin
                            write_abort_data[23:16] <= dap_in_tdata;
                            write_abort_sm <= WRITE_ABORT_READ_DATA3;
                        end
                    end
                    WRITE_ABORT_READ_DATA3: begin
                        if (dap_in_tvalid) begin
                            write_abort_data[31:24] <= dap_in_tdata;
                            write_abort_sm <= WRITE_ABORT_WAIT_RESPONE;
                        end
                    end
                    WRITE_ABORT_WAIT_RESPONE: begin
                        if (seq_rx_valid) begin
                            write_abort_ram_write_addr <= 10'd0;
                            write_abort_ram_write_en <= 1'd1;
                            // SWD模式判断ack，JTAG模式没有反馈返回成功
                            if (seq_rx_flag[3:0] == 4'b0001 || SWJ_CR_MODE == 1'b0) begin
                                write_abort_ram_write_data <= 8'h00;
                            end
                            else begin
                                write_abort_ram_write_data <= 8'hff;
                            end
                            write_abort_sm <= WRITE_ABORT_DONE;
                            done[`CMD_WRITE_ABORT_SHIFT] <= 1'd1;
                        end
                    end
                    WRITE_ABORT_DONE: begin
                    end
                endcase
            end
            else begin
                write_abort_sm <= WRITE_ABORT_READ_INDEX;
                done[`CMD_WRITE_ABORT_SHIFT] <= 1'd0;
            end
        end
    end

    // SWJ Pins Controller
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            swj_pins_ram_write_en <= 1'd0;
            swj_pins_ram_write_data <= 8'd0;
            swj_pins_ram_write_addr <= 10'd0;
            swj_pins_sm <= SWJ_PINS_SM_READ_OUTPUT;
            swj_pins_output <= 6'd0;
            swj_pins_select <= 6'd0;
            swj_pins_wait_time <= 32'd0;
            done[`CMD_SWJ_PINS_SHIFT] <= 1'd0;
        end
        else begin
            swj_pins_ram_write_en <= 1'd0;

            if (start[`CMD_SWJ_PINS_SHIFT]) begin
                case(swj_pins_sm) /* synthesis parallel_case */
                    SWJ_PINS_SM_READ_OUTPUT: begin
                        if (dap_in_tvalid) begin
                            swj_pins_output <= {dap_in_tdata[5], dap_in_tdata[7], dap_in_tdata[3:0]};
                            swj_pins_sm <= SWJ_PINS_SM_READ_SELECT;
                        end
                    end
                    SWJ_PINS_SM_READ_SELECT: begin
                        if (dap_in_tvalid) begin
                            swj_pins_select <= {dap_in_tdata[5], dap_in_tdata[7], dap_in_tdata[3:0]};
                            swj_pins_sm <= SWJ_PINS_SM_READ_WAIT0;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT0: begin
                        if (dap_in_tvalid) begin
                            swj_pins_wait_time[7:0] <= dap_in_tdata;
                            swj_pins_sm <= SWJ_PINS_SM_READ_WAIT1;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT1: begin
                        if (dap_in_tvalid) begin
                            swj_pins_wait_time[15:8] <= dap_in_tdata;
                            swj_pins_sm <= SWJ_PINS_SM_READ_WAIT2;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT2: begin
                        if (dap_in_tvalid) begin
                            swj_pins_wait_time[23:16] <= dap_in_tdata;
                            swj_pins_sm <= SWJ_PINS_SM_READ_WAIT3;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT3: begin
                        if (dap_in_tvalid) begin
                            swj_pins_wait_time[31:24] <= dap_in_tdata;
                            swj_pins_sm <= SWJ_PINS_SM_WAIT_RESPONE;
                        end
                    end
                    SWJ_PINS_SM_WAIT_RESPONE: begin
                        if (seq_rx_valid) begin
                            swj_pins_ram_write_addr <= 10'd0;
                            swj_pins_ram_write_data <= seq_rx_data[7:0];
                            swj_pins_ram_write_en <= 1'd1;
                            done[`CMD_SWJ_PINS_SHIFT] <= 1'd1;
                            swj_pins_sm <= SWJ_PINS_SM_DONE;
                        end
                    end
                    default: begin
                    end
                endcase
            end
            else begin
                swj_pins_sm <= SWJ_PINS_SM_READ_OUTPUT;
                done[`CMD_SWJ_PINS_SHIFT] <= 1'd0;
            end
        end
    end

    // SWD Block Transfer Controller
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            transfer_block_ram_write_en <= 1'd0;
            transfer_block_ram_write_data <= 8'd0;
            transfer_block_ram_write_addr <= 10'd0;
            transfer_block_packet_len <= 1'd0;
            transfer_block_sm <= TRANS_BLOCK_SM_READ_INDEX;
            transfer_block_request_cnt <= 16'd0;
            transfer_block_response_cnt <= 16'd0;
            transfer_block_requset <= 4'd0;
            transfer_block_data <= 32'd0;
            transfer_block_need_post_read <= 1'd0;
            transfer_block_need_rdbuff <= 1'd0;
            transfer_block_seq_tx_valid <= 1'd0;
            transfer_block_RnW <= 1'd0;
            transfer_block_APnDP <= 1'd0;
            transfer_block_err_flag <= 1'd0;
            transfer_block_ir <= 1'd0;
            done[`CMD_TRANSFER_BLOCK_SHIFT] <= 1'd0;
        end
        else begin
            transfer_block_ram_write_en <= 1'd0;
            transfer_block_seq_tx_valid <= 1'd0;

            if (start[`CMD_TRANSFER_BLOCK_SHIFT]) begin
                case (transfer_block_sm)
                    TRANS_BLOCK_SM_READ_INDEX: begin // 读取DAP Index抛弃
                        transfer_block_ram_write_addr <= 10'd2;
                        transfer_block_packet_len <= 10'd3;
                        transfer_block_err_flag <= 1'd0;
                        transfer_block_response_cnt <= 16'd0;
                        transfer_block_ir <= 1'd1;
                        if (dap_in_tvalid) begin
                            transfer_block_sm <= TRANS_BLOCK_SM_READ_COUNT_L;
                        end
                    end
                    TRANS_BLOCK_SM_READ_COUNT_L: begin
                        if (dap_in_tvalid) begin
                            transfer_block_request_cnt[7:0] <= dap_in_tdata;
                            transfer_block_sm <= TRANS_BLOCK_SM_READ_COUNT_H;
                        end
                    end
                    TRANS_BLOCK_SM_READ_COUNT_H: begin
                        if (dap_in_tvalid) begin
                            transfer_block_request_cnt[15:8] <= dap_in_tdata;
                            transfer_block_sm <= TRANS_BLOCK_SM_READ_REQUSET;
                        end
`ifdef SIMULATION
                        $display("Transmit Count: %d", {dap_in_tdata, transfer_block_request_cnt[7:0]});
`endif

                    end
                    TRANS_BLOCK_SM_READ_REQUSET: begin // 读requset
                        if (dap_in_tvalid) begin
                            transfer_block_requset <= dap_in_tdata[3:0];
                            transfer_block_RnW <= dap_in_tdata[1];
                            transfer_block_APnDP <= dap_in_tdata[0];
                            if (dap_in_tdata[1]) begin // RnW
                                if (SWD_MODE) begin
                                    // SWD读AP需要预读加RDBUFF
                                    transfer_block_need_post_read <= dap_in_tdata[0];
                                    transfer_block_need_rdbuff <= dap_in_tdata[0];
                                end
                                else begin
                                    // JTAG模式全需要预读加RDBUFF
                                    transfer_block_need_post_read <= 1'd1;
                                    transfer_block_need_rdbuff <= 1'd1;
                                end

                                transfer_block_sm <= TRANS_BLOCK_SM_PROCESS_REQ;
`ifdef SIMULATION

                                $display("transfer_block_need_post_read: %d", dap_in_tdata[0] || (SWD_MODE == 1'd0));
`endif

                            end
                            else begin
                                // 写模式都需要RDBUFF确认
                                transfer_block_need_post_read <= 1'd0;
                                transfer_block_need_rdbuff <= 1'd1;
                                transfer_block_sm <= TRANS_BLOCK_SM_READ_DATA0;
                            end
                        end
                    end
                    TRANS_BLOCK_SM_READ_DATA0: begin
                        if (dap_in_tvalid) begin
                            transfer_block_data[7:0] <= dap_in_tdata;
                            transfer_block_sm <= TRANS_BLOCK_SM_READ_DATA1;
                        end
                    end
                    TRANS_BLOCK_SM_READ_DATA1: begin
                        if (dap_in_tvalid) begin
                            transfer_block_data[15:8] <= dap_in_tdata;
                            transfer_block_sm <= TRANS_BLOCK_SM_READ_DATA2;
                        end
                    end
                    TRANS_BLOCK_SM_READ_DATA2: begin
                        if (dap_in_tvalid) begin
                            transfer_block_data[23:16] <= dap_in_tdata;
                            transfer_block_sm <= TRANS_BLOCK_SM_READ_DATA3;
                        end
                    end
                    TRANS_BLOCK_SM_READ_DATA3: begin
                        if (dap_in_tvalid) begin
                            transfer_block_data[31:24] <= dap_in_tdata;
                            transfer_block_sm <= TRANS_BLOCK_SM_PROCESS_REQ;
                        end
                    end
                    TRANS_BLOCK_SM_PROCESS_REQ: begin // 循环开始位置，根据请求类型触发读fifo数据或驱动SWD
                        if (!transfer_block_err_flag) begin
                            // 读请求直接驱动SWD
                            transfer_block_seq_tx_valid <= 1'd1;
                        end
                        transfer_block_sm <= TRANS_BLOCK_SM_TRANSFER;
                    end
                    TRANS_BLOCK_SM_TRANSFER: begin // 等待传输完成
                        if (!transfer_block_err_flag && (seq_rx_valid && seq_rx_flag[3:0] == 4'b0001)) begin // 正常传输完成
                            // 请求计数-1
                            transfer_block_request_cnt <= transfer_block_request_cnt - 1'd1;
                            // 响应计数+1
                            transfer_block_response_cnt <= transfer_block_response_cnt + 1'd1;
                            transfer_block_ir <= 1'd0; // 第一次传输完成后假定不需要IR
                            case (transfer_block_requset[1:0])
                                2'b11, 2'b10: begin // Read AP/DP
                                    if (transfer_block_need_post_read) begin
                                        if (transfer_block_request_cnt - 1'd1 == 16'd0) begin
                                            // 单次读AP传输，转RDBUFF，激活IR
                                            transfer_block_requset <= 4'b1110;
                                            transfer_block_ir <= 1'd1;
                                            transfer_block_seq_tx_valid <= 1'd1;
                                            transfer_block_sm <= TRANS_BLOCK_SM_WAIT_RDBUFF;
                                        end
                                        else begin
                                            // 多次传输的第一次传输，不写入数据直接触发下一次读取，不需要IR
                                            transfer_block_need_post_read <= 1'd0;
                                            transfer_block_seq_tx_valid <= 1'd1;
                                        end
                                    end
                                    else begin
                                        transfer_block_sm <= TRANS_BLOCK_SM_WRITE_DATA_0;
                                    end
                                end
                                2'b01, 2'b00: begin // Write AP/DP
                                    if (transfer_block_request_cnt - 1'd1 == 16'd0) begin
                                        // 最后一次传输，转RDBUFF，激活IR
                                        transfer_block_requset <= 4'b1110;
                                        transfer_block_ir <= 1'd1;
                                        transfer_block_seq_tx_valid <= 1'd1;
                                        transfer_block_sm <= TRANS_BLOCK_SM_WAIT_RDBUFF;
                                    end
                                    else begin
                                        transfer_block_sm <= TRANS_BLOCK_SM_READ_DATA0;
                                    end
                                end
                            endcase
                        end

                        if (transfer_block_err_flag || (seq_rx_valid && seq_rx_flag[3:0] != 4'b0001)) begin
                            transfer_block_err_flag <= 1'd1;
                            transfer_block_request_cnt <= transfer_block_request_cnt - 1'd1;
                            if (transfer_block_request_cnt - 1'd1 == 16'd0) begin
                                transfer_block_sm <= TRANS_BLOCK_SM_WRITE_COUNT_L;
                            end
                            else begin
                                if (transfer_block_requset[1]) begin
                                    transfer_block_sm <= TRANS_BLOCK_SM_PROCESS_REQ;
                                end
                                else begin
                                    transfer_block_sm <= TRANS_BLOCK_SM_READ_DATA0;
                                end
                            end
                        end
                    end
                    TRANS_BLOCK_SM_WAIT_RDBUFF: begin
                        if (seq_rx_valid) begin
                            if (transfer_block_RnW) begin
                                // 读模式需要写入数据
                                transfer_block_sm <= TRANS_BLOCK_SM_WRITE_DATA_0;
                            end
                            else begin
                                // 写模式确认ACK后抛弃数据
                                transfer_block_sm <= TRANS_BLOCK_SM_WRITE_COUNT_L;
                            end
                        end
                    end
                    TRANS_BLOCK_SM_WRITE_DATA_0: begin
                        transfer_block_ram_write_addr <= transfer_block_ram_write_addr + 1'd1;
                        transfer_block_ram_write_data <= seq_rx_data[7:0];
                        transfer_block_ram_write_en <= 1'd1;
                        transfer_block_sm <= TRANS_BLOCK_SM_WRITE_DATA_1;
                    end
                    TRANS_BLOCK_SM_WRITE_DATA_1: begin
                        transfer_block_ram_write_addr <= transfer_block_ram_write_addr + 1'd1;
                        transfer_block_ram_write_data <= seq_rx_data[15:8];
                        transfer_block_ram_write_en <= 1'd1;
                        transfer_block_sm <= TRANS_BLOCK_SM_WRITE_DATA_2;
                    end
                    TRANS_BLOCK_SM_WRITE_DATA_2: begin
                        transfer_block_ram_write_addr <= transfer_block_ram_write_addr + 1'd1;
                        transfer_block_ram_write_data <= seq_rx_data[23:16];
                        transfer_block_ram_write_en <= 1'd1;
                        transfer_block_sm <= TRANS_BLOCK_SM_WRITE_DATA_3;
                    end
                    TRANS_BLOCK_SM_WRITE_DATA_3: begin
                        transfer_block_ram_write_addr <= transfer_block_ram_write_addr + 1'd1;
                        transfer_block_ram_write_data <= seq_rx_data[31:24];
                        transfer_block_ram_write_en <= 1'd1;
                        if (transfer_block_request_cnt == 16'd0) begin
                            // 最后一次传输
                            if (transfer_block_need_rdbuff) begin
                                // 需要RDBUFF，清除标志并触发RDBUFF
                                transfer_block_need_rdbuff <= 1'd0;
                                transfer_block_requset <= 4'b1110;
                                transfer_block_seq_tx_valid <= 1'd1;
                                transfer_block_sm <= TRANS_BLOCK_SM_WAIT_RDBUFF;
                            end
                            else begin
                                // 不需要RDBUFF结束传输
                                transfer_block_sm <= TRANS_BLOCK_SM_WRITE_COUNT_L;
                            end
                        end
                        else begin
                            // 继续请求
                            transfer_block_sm <= TRANS_BLOCK_SM_PROCESS_REQ;
                        end
                    end
                    TRANS_BLOCK_SM_WRITE_COUNT_L: begin
                        transfer_block_ram_write_addr <= 10'd0;
                        transfer_block_ram_write_data <= transfer_block_response_cnt[7:0];
                        transfer_block_ram_write_en <= 1'd1;
                        transfer_block_packet_len <= transfer_block_ram_write_addr + 1'd1;
                        transfer_block_sm <= TRANS_BLOCK_SM_WRITE_COUNT_H;
`ifdef SIMULATION

                        $display("Transfer Block Write Head");
`endif

                    end
                    TRANS_BLOCK_SM_WRITE_COUNT_H: begin
                        transfer_block_ram_write_addr <= 10'd1;
                        transfer_block_ram_write_data <= transfer_block_response_cnt[15:8];
                        transfer_block_ram_write_en <= 1'd1;
                        transfer_block_sm <= TRANS_BLOCK_SM_WRITE_RESPONSE;
                    end
                    TRANS_BLOCK_SM_WRITE_RESPONSE: begin
                        transfer_block_ram_write_addr <= 10'd2;
                        transfer_block_ram_write_data <= {5'd0, seq_rx_flag[2:0]};
                        transfer_block_ram_write_en <= 1'd1;
                        transfer_block_sm <= TRANS_BLOCK_SM_END;
                        done[`CMD_TRANSFER_BLOCK_SHIFT] <= 1'd1;
                    end
                endcase
            end
            else begin
                transfer_block_sm <= TRANS_BLOCK_SM_READ_INDEX;
                done[`CMD_TRANSFER_BLOCK_SHIFT] <= 1'd0;
            end
        end
    end

    // SWD Transfer Controller
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            transfer_ram_write_en <= 1'd0;
            transfer_ram_write_data <= 8'd0;
            transfer_ram_write_addr <= 10'd0;
            transfer_packet_len <= 1'd0;
            transfer_post_read <= 0;
            transfer_check_write <= 0;
            transfer_has_error <= 0;
            transfer_match_failed <= 1'd0;
            transfer_wdata <= 32'd0;
            transfer_sm <= TRANS_SM_READ_INDEX;
            transfer_match_mask <= 32'd0;
            transfer_num <= 8'd0;
            transfer_cnt <= 8'd0;
            transfer_requset <= 8'd0;
            transfer_timestamp <= 1'd0;
            transfer_match_cnt <= 1'd0;
            transfer_seq_tx_valid <= 1'd0;
            transfer_trig_requset <= 4'd0;
            transfer_trig_ret_sm <= 26'd0;
            transfer_trig_en_wdata <= 1'd0;
            transfer_trig_en_wtime <= 1'd0;
            transfer_trig_en_cnt <= 1'd0;
            transfer_trig_wtime_first <= 1'd0;
            transfer_trig_ir <= 1'd0;
            transfer_trig_first_transfer <= 1'd0;
            done[`CMD_TRANSFER_SHIFT] <= 1'd0;
        end
        else begin
            transfer_ram_write_en <= 1'd0;
            transfer_seq_tx_valid <= 1'd0;

            if (start[`CMD_TRANSFER_SHIFT]) begin
                case(transfer_sm)
                    TRANS_SM_READ_INDEX: begin
                        transfer_post_read <= 1'd0;
                        transfer_check_write <= 1'd0;
                        transfer_has_error <= 1'd0;
                        transfer_match_failed <= 1'd0;
                        transfer_packet_len <= 10'd2;
                        transfer_ram_write_addr <= 10'd1;
                        transfer_trig_first_transfer <= 1'd1;
                        if (dap_in_tvalid) begin
                            transfer_sm <= TRANS_SM_READ_COUNT;
                        end
                    end
                    TRANS_SM_READ_COUNT: begin
                        if (dap_in_tvalid) begin
                            transfer_sm <= TRANS_SM_READ_REQUSET;
                            transfer_num <= dap_in_tdata;
                            transfer_cnt <= 8'd0;
`ifdef SIMULATION

                            $display("transfer_num %d", dap_in_tdata);
`endif

                        end
                    end
                    TRANS_SM_READ_REQUSET: begin
                        if (transfer_num) begin
                            if (dap_in_tvalid) begin
                                transfer_num <= transfer_num - 1'd1;
                                transfer_requset <= dap_in_tdata;
                                if (dap_in_tdata[1] == 1'd0 || dap_in_tdata[5:4] != 2'd0) begin
                                    // 写请求或设置MASK有数据段的请求
                                    transfer_sm <= TRANS_SM_READ_DATA0;
                                end
                                else begin
                                    // READ/MATCH请求先检查是否需求处理post_read
                                    transfer_sm <= transfer_has_error ? TRANS_SM_READ_REQUSET : TRANS_SM_CHECK_POSTREAD;
                                end
`ifdef SIMULATION

                                $display("transfer_requset RnW:%b APnDP:%b", dap_in_tdata[1], dap_in_tdata[0]);
`endif

                            end
                        end
                        else begin
                            transfer_sm <= transfer_has_error ? TRANS_SM_WRTE_COUNT : TRANS_SM_CHECK_WIRTE;
                        end
                    end
                    TRANS_SM_READ_DATA0: begin
                        if (dap_in_tvalid) begin
                            transfer_wdata[7:0] <= dap_in_tdata;
                            if (transfer_requset_MASK) begin
                                transfer_match_mask[7:0] <= dap_in_tdata;
                            end
                            transfer_sm <= TRANS_SM_READ_DATA1;
                        end
                    end
                    TRANS_SM_READ_DATA1: begin
                        if (dap_in_tvalid) begin
                            transfer_wdata[15:8] <= dap_in_tdata;
                            if (transfer_requset_MASK) begin
                                transfer_match_mask[15:8] <= dap_in_tdata;
                            end
                            transfer_sm <= TRANS_SM_READ_DATA2;
                        end
                    end
                    TRANS_SM_READ_DATA2: begin
                        if (dap_in_tvalid) begin
                            transfer_wdata[23:16] <= dap_in_tdata;
                            if (transfer_requset_MASK) begin
                                transfer_match_mask[23:16] <= dap_in_tdata;
                            end
                            transfer_sm <= TRANS_SM_READ_DATA3;
                        end
                    end
                    TRANS_SM_READ_DATA3: begin
                        if (dap_in_tvalid) begin
                            transfer_wdata[31:24] <= dap_in_tdata;
                            if (transfer_has_error) begin
                                transfer_sm <= TRANS_SM_READ_REQUSET;
                            end
                            else if (transfer_requset_MASK) begin
                                transfer_match_mask[31:24] <= dap_in_tdata;
                                transfer_cnt <= transfer_cnt + 1'd1;
                                transfer_sm <= TRANS_SM_READ_REQUSET;
`ifdef SIMULATION

                                $display("WRITE MASK %08x", {dap_in_tdata, transfer_match_mask[23:0]});
`endif

                            end
                            else begin
                                transfer_sm <= TRANS_SM_CHECK_POSTREAD;
                            end
                        end
                    end
                    TRANS_SM_CHECK_POSTREAD: begin
`ifdef SIMULATION
                        $display("TRANS_SM_CHECK_POSTREAD %d", transfer_post_read);
`endif

                        // 需要先清除post_read状态的请求：MATCH、WriteAP、WriteDP、ReadDP (ReadAP之外所有请求)
                        if (transfer_post_read && (transfer_requset_MATCH || {transfer_requset_RnW, transfer_requset_APnDP} != 2'b11)) begin
                            transfer_post_read <= 1'd0;
                            // RDBUFF
                            transfer_trig_requset <= 4'b1110;
                            // 写入延迟数据
                            transfer_trig_en_wdata <= 1'd1;
                            // 读请求按配置写入时间戳，写请求没有时间戳
                            transfer_trig_en_wtime <= transfer_requset_RnW ? transfer_requset_TIMESTAMP : 1'd0;
                            // 不计入命令数量
                            transfer_trig_en_cnt <= 1'd0;
                            transfer_trig_wtime_first <= 1'd0;

                            if (transfer_requset_MATCH) begin // 匹配读取请求
                                transfer_trig_ret_sm <= TRANS_SM_MATCH_STEP1;
                            end
                            else if (transfer_requset_RnW == 1'd0) begin // 写请求
                                transfer_trig_ret_sm <= TRANS_SM_WRITE_APDP;
                            end
                            else begin // 读DP请求
                                transfer_trig_ret_sm <= TRANS_SM_READ_DP;
                            end
                            transfer_trig_ir <= transfer_trig_first_transfer | transfer_trig_requset[0];
                            transfer_sm <= TRANS_SM_TRIGGER;
                        end
                        else begin
                            if (transfer_requset_MATCH) begin // 匹配读取请求
                                transfer_sm <= transfer_requset_APnDP ? TRANS_SM_MATCH_STEP1 : TRANS_SM_MATCH_STEP2;
                            end
                            else if (transfer_requset_RnW == 1'd0) begin // 写请求
                                transfer_sm <= TRANS_SM_WRITE_APDP;
                            end
                            else if (transfer_requset_APnDP) begin // 读AP请求
                                transfer_sm <= TRANS_SM_READ_AP;
                            end
                            else begin
                                transfer_sm <= TRANS_SM_READ_DP;
                            end
                        end
                    end
                    TRANS_SM_READ_AP: begin // 读AP
                        //                requset, en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                        TRANS_TRIGGER(transfer_requset[3:0], transfer_post_read, transfer_requset_TIMESTAMP, 1'd1, 1'd0, TRANS_SM_READ_REQUSET);
                        transfer_check_write <= 1'd0;
                        transfer_post_read <= 1'd1;
`ifdef SIMULATION

                        $display("TRANS_SM_READ_AP");
`endif

                    end

                    TRANS_SM_READ_DP: begin // 读DP
                        // JTAG模式下需要预读取
                        //                requset,          en_wdata, en_wtime, en_cnt, wtime_first,             ret_sm
                        TRANS_TRIGGER(transfer_requset[3:0], (SWD_MODE ? 1'd1 : transfer_post_read), 1'd0, 1'd1, transfer_requset_TIMESTAMP, TRANS_SM_READ_REQUSET);
                        transfer_check_write <= 1'd0;
                        transfer_post_read <= (SWD_MODE == 1'd0);
                    end

                    TRANS_SM_WRITE_APDP: begin // 写AP/DP
                        //                requset,              en_wdata,       en_wtime,          en_cnt, wtime_first, ret_sm
                        TRANS_TRIGGER(transfer_requset[3:0], 1'd0, transfer_requset_TIMESTAMP, 1'd1, 1'd0, TRANS_SM_READ_REQUSET);
                        transfer_check_write <= 1'd1;
                    end

                    TRANS_SM_MATCH_STEP1: begin // 匹配第一步，AP预读取
                        //                requset,           en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                        TRANS_TRIGGER(transfer_requset[3:0], 1'd0, 1'd0, 1'd0, 1'd0, TRANS_SM_MATCH_STEP2);
                    end

                    TRANS_SM_MATCH_STEP2: begin // 匹配第二步，读取正式数据
                        //                requset,           en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                        TRANS_TRIGGER(transfer_requset[3:0], 1'd0, 1'd0, 1'd0, 1'd0, TRANS_SM_MATCH_STEP3);
                    end

                    TRANS_SM_MATCH_STEP3: begin // 匹配第三步，循环读取匹配
                        transfer_check_write <= 1'd0;
                        if ((transfer_match_mask & seq_rx_data) == transfer_wdata) begin
                            transfer_cnt <= transfer_cnt + 1'd1;
                            transfer_sm <= TRANS_SM_READ_REQUSET;
                        end
                        else if (transfer_match_cnt + 1'd1 == SWJ_MATCH_RETRY) begin
                            transfer_match_failed <= 1'd1;
                            transfer_has_error <= 1'd1;
                            transfer_sm <= TRANS_SM_READ_REQUSET;
                        end
                        else begin
                            //                requset,           en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                            TRANS_TRIGGER(transfer_requset[3:0], 1'd0, 1'd0, 1'd0, 1'd0, TRANS_SM_MATCH_STEP3);
                            transfer_match_cnt <= transfer_match_cnt + 1'd1;
                        end
                    end
                    TRANS_SM_CHECK_WIRTE: begin
`ifdef SIMULATION
                        $display("TRANS_SM_CHECK_WIRTE pr:%d wc:%d", transfer_post_read, transfer_check_write);
`endif

                        if (transfer_post_read) begin
                            //                requset, en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                            TRANS_TRIGGER(4'b1110, 1'd1, 1'd0, 1'd0, 1'd0, TRANS_SM_WRTE_COUNT);
                        end
                        else if (transfer_check_write) begin
                            //                requset, en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                            TRANS_TRIGGER(4'b1110, 1'd0, 1'd0, 1'd0, 1'd0, TRANS_SM_WRTE_COUNT);
                        end
                        else begin
                            transfer_sm <= TRANS_SM_WRTE_COUNT;
                        end
                    end
                    TRANS_SM_WRTE_COUNT: begin
                        transfer_ram_write_addr <= 10'd0;
                        transfer_ram_write_data <= transfer_cnt;
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= transfer_sm << 1'd1;
                    end
                    TRANS_SM_WRTE_STATUS: begin
                        transfer_ram_write_addr <= 10'd1;
                        transfer_ram_write_data <= {3'd0, transfer_match_failed, seq_rx_flag[3:0]};
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= TRANS_SM_DONE;
                    end
                    TRANS_SM_TRIGGER: begin
                        transfer_trig_first_transfer <= 1'd0;
                        transfer_timestamp <= clk_timer;
                        transfer_sm <= TRANS_SM_WAIT;
                    end
                    TRANS_SM_WAIT: begin
                        if (seq_rx_valid) begin
                            if (seq_rx_flag[3:0] == 4'b0001) begin
                                if (transfer_trig_en_cnt) begin
                                    transfer_cnt <= transfer_cnt + 1'd1;
                                end
                                if (transfer_trig_wtime_first) begin
                                    transfer_sm <= TRANS_SM_WRITE_TIME0;
                                end
                                else if (transfer_trig_en_wdata) begin
                                    transfer_sm <= TRANS_SM_WRITE_DATA0;
                                end
                                else if (transfer_trig_en_wtime) begin
                                    transfer_sm <= TRANS_SM_WRITE_TIME0;
                                end
                                else begin
                                    transfer_sm <= transfer_trig_ret_sm;
                                end
                            end
                            else begin
                                transfer_has_error <= 1'd1;
                                transfer_sm <= TRANS_SM_READ_REQUSET;
                            end
                        end
                    end
                    TRANS_SM_WRITE_DATA0: begin
                        transfer_packet_len <= transfer_packet_len + 1'd1;
                        transfer_ram_write_addr <= transfer_ram_write_addr + 1'd1;
                        transfer_ram_write_data <= seq_rx_data[7:0];
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= transfer_sm << 1'd1;
                    end
                    TRANS_SM_WRITE_DATA1: begin
                        transfer_packet_len <= transfer_packet_len + 1'd1;
                        transfer_ram_write_addr <= transfer_ram_write_addr + 1'd1;
                        transfer_ram_write_data <= seq_rx_data[15:8];
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= transfer_sm << 1'd1;
                    end
                    TRANS_SM_WRITE_DATA2: begin
                        transfer_packet_len <= transfer_packet_len + 1'd1;
                        transfer_ram_write_addr <= transfer_ram_write_addr + 1'd1;
                        transfer_ram_write_data <= seq_rx_data[23:16];
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= transfer_sm << 1'd1;
                    end
                    TRANS_SM_WRITE_DATA3: begin
                        transfer_packet_len <= transfer_packet_len + 1'd1;
                        transfer_ram_write_addr <= transfer_ram_write_addr + 1'd1;
                        transfer_ram_write_data <= seq_rx_data[31:24];
                        transfer_ram_write_en <= 1'd1;
                        if (!transfer_trig_en_wtime || transfer_trig_wtime_first) begin
                            transfer_sm <= transfer_trig_ret_sm;
                        end
                        else begin
                            transfer_sm <= transfer_sm << 1'd1;
                        end
                    end
                    TRANS_SM_WRITE_TIME0: begin
                        transfer_packet_len <= transfer_packet_len + 1'd1;
                        transfer_ram_write_addr <= transfer_ram_write_addr + 1'd1;
                        transfer_ram_write_data <= transfer_timestamp[7:0];
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= transfer_sm << 1'd1;
                    end
                    TRANS_SM_WRITE_TIME1: begin
                        transfer_packet_len <= transfer_packet_len + 1'd1;
                        transfer_ram_write_addr <= transfer_ram_write_addr + 1'd1;
                        transfer_ram_write_data <= transfer_timestamp[15:8];
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= transfer_sm << 1'd1;
                    end
                    TRANS_SM_WRITE_TIME2: begin
                        transfer_packet_len <= transfer_packet_len + 1'd1;
                        transfer_ram_write_addr <= transfer_ram_write_addr + 1'd1;
                        transfer_ram_write_data <= transfer_timestamp[23:16];
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= transfer_sm << 1'd1;
                    end
                    TRANS_SM_WRITE_TIME3: begin
                        transfer_packet_len <= transfer_packet_len + 1'd1;
                        transfer_ram_write_addr <= transfer_ram_write_addr + 1'd1;
                        transfer_ram_write_data <= transfer_timestamp[31:24];
                        transfer_ram_write_en <= 1'd1;
                        transfer_sm <= transfer_trig_wtime_first ? TRANS_SM_WRITE_DATA0 : transfer_trig_ret_sm;
                    end
                endcase

                if (transfer_sm == TRANS_SM_DONE) begin
                    done[`CMD_TRANSFER_SHIFT] <= 1'd1;
                end
            end
            else begin
                transfer_sm <= TRANS_SM_READ_INDEX;
                done[`CMD_TRANSFER_SHIFT] <= 1'd0;
            end
        end
    end



    // JTAG Sequence Controller
    // 根据您的协议说明：
    // 第一字节是Number of Sequences
    // 之后跟随Number of Sequences个数据包
    // 每个数据包开头是Sequence Info (Bit[5:0]=TCK周期数1-64, Bit[6]=TMS值, Bit[7]=TDO捕获)
    // 之后跟随TDI Data (LSB first, padded to BYTE boundary)
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            jtag_seq_ram_write_en <= 1'd0;
            jtag_seq_ram_write_data <= 8'd0;
            jtag_seq_ram_write_addr <= 10'd0;
            jtag_seq_sm <= JTAG_SEQ_SM_READ_SEQ_COUNT;
            jtag_seq_tms <= 1'd0;
            jtag_seq_capture_tdo <= 1'd0;
            jtag_seq_num <= 8'd0;
            jtag_seq_bit_num <= 7'd0;
            jtag_seq_trans_num <= 4'd0;
            jtag_seq_send_data <= 8'd0;
            jtag_seq_tx_valid <= 1'd0;
            done[`CMD_JTAG_SEQUENCE_SHIFT] <= 1'd0;
        end
        else begin
            jtag_seq_ram_write_en <= 1'd0;
            jtag_seq_tx_valid <= 1'd0;

            if (start[`CMD_JTAG_SEQUENCE_SHIFT]) begin
                case (jtag_seq_sm) /* synthesis parallel_case */
                    JTAG_SEQ_SM_READ_SEQ_COUNT: begin
                        // 读取Number of Sequences
                        if (dap_in_tvalid) begin
                            jtag_seq_num <= dap_in_tdata;
                            jtag_seq_ram_write_addr <= 10'd0;
                            jtag_seq_ram_write_data <= 8'd0;
                            jtag_seq_ram_write_en <= 1'd1;
                            jtag_seq_sm <= JTAG_SEQ_SM_READ_SEQ_INFO;
                        end
                    end

                    JTAG_SEQ_SM_READ_SEQ_INFO: begin
                        // 读取Sequence Info (TCK cycles, TMS, TDO capture)
                        if (dap_in_tvalid) begin
                            // Bit[5:0]: Number of TCK cycles (1-64, 64 encoded as 0)
                            jtag_seq_bit_num <= (dap_in_tdata[5:0] == 6'd0) ? 7'd64 : {1'b0, dap_in_tdata[5:0]};
                            // Bit[6]: TMS value
                            jtag_seq_tms <= dap_in_tdata[6];
                            // Bit[7]: TDO Capture
                            jtag_seq_capture_tdo <= dap_in_tdata[7];
                            jtag_seq_sm <= JTAG_SEQ_SM_READ_DATA;
                        end
                    end

                    JTAG_SEQ_SM_READ_DATA: begin
                        // 读取TDI Data (分块传输，每块最多8位)
                        if (dap_in_tvalid) begin
                            jtag_seq_send_data <= dap_in_tdata;
                            jtag_seq_trans_num <= (jtag_seq_bit_num > 7'd8) ? 4'd8 : jtag_seq_bit_num[3:0];
                            jtag_seq_tx_valid <= 1'd1;
                            jtag_seq_sm <= JTAG_SEQ_SM_WAIT;
                        end
                    end

                    JTAG_SEQ_SM_WAIT: begin
                        // 等待序列传输完成
                        if (seq_rx_valid) begin
                            // 更新剩余位数
                            jtag_seq_bit_num <= jtag_seq_bit_num - jtag_seq_trans_num;

                            // 如果需要捕获TDO，写入返回数据
                            if (jtag_seq_capture_tdo) begin
                                jtag_seq_ram_write_addr <= jtag_seq_ram_write_addr + 1'd1;
                                jtag_seq_ram_write_data <= seq_rx_data[7:0];
                                jtag_seq_ram_write_en <= 1'd1;
                            end

                            // 检查当前序列是否完成
                            if ((jtag_seq_bit_num - jtag_seq_trans_num) != 7'd0) begin
                                // 当前序列还有数据，继续读取
                                jtag_seq_sm <= JTAG_SEQ_SM_READ_DATA;
                            end
                            else begin
                                // 当前序列完成，检查是否还有更多序列
                                if (jtag_seq_num - 1'd1 != 8'd0) begin
                                    // 还有更多序列
                                    jtag_seq_num <= jtag_seq_num - 1'd1;
                                    jtag_seq_sm <= JTAG_SEQ_SM_READ_SEQ_INFO;
                                end
                                else begin
                                    // 所有序列完成
                                    jtag_seq_sm <= JTAG_SEQ_SM_DONE;
                                    done[`CMD_JTAG_SEQUENCE_SHIFT] <= 1'd1;
                                end
                            end
                        end
                    end

                    JTAG_SEQ_SM_DONE: begin
                        // 保持完成状态
                    end
                endcase
            end
            else begin
                jtag_seq_sm <= JTAG_SEQ_SM_READ_SEQ_COUNT;
                done[`CMD_JTAG_SEQUENCE_SHIFT] <= 1'd0;
            end
        end
    end

    // JTAG IDCODE Controller
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            jtag_idcode_ram_write_en <= 1'd0;
            jtag_idcode_ram_write_data <= 8'd0;
            jtag_idcode_ram_write_addr <= 10'd0;
            jtag_idcode_sm <= JTAG_IDCODE_SM_WAIT_INDEX;
            jtag_idcode_seq_tx_valid <= 1'd0;
            done[`CMD_JTAG_IDCODE_SHIFT] <= 1'd0;
        end
        else begin
            jtag_idcode_ram_write_en <= 1'd0;
            jtag_idcode_seq_tx_valid <= 1'd0;

            if (start[`CMD_JTAG_IDCODE_SHIFT]) begin
                case (jtag_idcode_sm) /* synthesis parallel_case */
                    JTAG_IDCODE_SM_WAIT_INDEX: begin
                        if (dap_in_tvalid) begin
                            // 写入Status
                            jtag_idcode_ram_write_addr <= 10'd0;
                            jtag_idcode_ram_write_data <= 8'h00;
                            jtag_idcode_ram_write_en <= 1'd1;
                            jtag_idcode_sm <= JTAG_IDCODE_SM_WRITE_DATA0;
                            jtag_idcode_seq_tx_valid <= 1'd1;
                        end
                    end
                    JTAG_IDCODE_SM_WRITE_DATA0: begin
                        if (seq_rx_valid) begin
                            // 写入IDCODE的第一字节
                            jtag_idcode_ram_write_addr <= 10'd1;
                            jtag_idcode_ram_write_data <= seq_rx_data[7:0];
                            jtag_idcode_ram_write_en <= 1'd1;
                            jtag_idcode_sm <= JTAG_IDCODE_SM_WRITE_DATA1;
                        end
                    end
                    JTAG_IDCODE_SM_WRITE_DATA1: begin
                        // 继续写入剩余字节
                        jtag_idcode_ram_write_addr <= 10'd2;
                        jtag_idcode_ram_write_data <= seq_rx_data[15:8];
                        jtag_idcode_ram_write_en <= 1'd1;
                        jtag_idcode_sm <= JTAG_IDCODE_SM_WRITE_DATA2;
                    end
                    JTAG_IDCODE_SM_WRITE_DATA2: begin
                        // 继续写入剩余字节
                        jtag_idcode_ram_write_addr <= 10'd3;
                        jtag_idcode_ram_write_data <= seq_rx_data[23:16];
                        jtag_idcode_ram_write_en <= 1'd1;
                        jtag_idcode_sm <= JTAG_IDCODE_SM_WRITE_DATA3;
                    end
                    JTAG_IDCODE_SM_WRITE_DATA3: begin
                        // 继续写入剩余字节
                        jtag_idcode_ram_write_addr <= 10'd4;
                        jtag_idcode_ram_write_data <= seq_rx_data[31:24];
                        jtag_idcode_ram_write_en <= 1'd1;
                        jtag_idcode_sm <= JTAG_IDCODE_SM_DONE;
                        done[`CMD_JTAG_IDCODE_SHIFT] <= 1'd1;
                    end
                    JTAG_IDCODE_SM_DONE: begin
                    end
                endcase
            end
            else begin
                jtag_idcode_sm <= JTAG_IDCODE_SM_WAIT_INDEX;
                done[`CMD_JTAG_IDCODE_SHIFT] <= 1'd0;
            end
        end
    end

    // JTAG IR 配置解码逻辑
    // 根据命令选择对应的配置
    reg [19:0] jtag_ir_conf_selected;
    reg jtag_index_recv_status;
    wire [3:0] jtag_ir_len = jtag_ir_conf_selected[19:16];
    wire [7:0] jtag_ir_before_len = jtag_ir_conf_selected[15:8];
    wire [7:0] jtag_ir_after_len = jtag_ir_conf_selected[7:0];

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            jtag_index_recv_status <= 1'd0;
            jtag_ir_conf_selected <= 20'd0;
            jtag_index <= 3'd0;
        end
        else begin
            if (jtag_index_recv_status == 1'd0) begin
                if ((start[`CMD_WRITE_ABORT_SHIFT] | start[`CMD_TRANSFER_SHIFT] | start[`CMD_TRANSFER_BLOCK_SHIFT] | start[`CMD_JTAG_IDCODE_SHIFT]) & dap_in_tvalid) begin
                    jtag_index_recv_status <= 1'd1;
                    jtag_index <= dap_in_tdata[2:0];
                    jtag_ir_conf_selected <= SWJ_JTAG_IR_CONF_REG[dap_in_tdata[2:0]];
                end
            end
            else if (done) begin
                jtag_index_recv_status <= 1'd0;
            end
        end
    end

    assign dap_in_tready[`CMD_SWJ_SEQUENCE_SHIFT] = ~swj_seq_sm[1];

    assign dap_in_tready[`CMD_SWD_SEQUENCE_SHIFT] =
           (swd_seq_sm == SWD_SEQ_SM_READ_SEQ_NUM) ||
           (swd_seq_sm == SWD_SEQ_SM_READ_SEQ_INFO) ||
           (swd_seq_sm == SWD_SEQ_SM_READ_DATA && swd_seq_dir == 1'd0);

    assign dap_in_tready[`CMD_WRITE_ABORT_SHIFT] = (write_abort_sm < WRITE_ABORT_WAIT_RESPONE);

    assign dap_in_tready[`CMD_SWJ_PINS_SHIFT] = swj_pins_sm < SWJ_PINS_SM_WAIT_RESPONE;

    assign dap_in_tready[`CMD_TRANSFER_BLOCK_SHIFT] = transfer_block_sm[7:0] != 8'd0;

    assign dap_in_tready[`CMD_TRANSFER_SHIFT] = transfer_sm[2] ? (transfer_num != 8'd0) : transfer_sm[6:0] != 7'd0;

    assign dap_in_tready[`CMD_JTAG_SEQUENCE_SHIFT] =
           (jtag_seq_sm == JTAG_SEQ_SM_READ_SEQ_COUNT || jtag_seq_sm == JTAG_SEQ_SM_READ_SEQ_INFO || jtag_seq_sm == JTAG_SEQ_SM_READ_DATA);

    assign dap_in_tready[`CMD_JTAG_IDCODE_SHIFT] = (jtag_idcode_sm == JTAG_IDCODE_SM_WAIT_INDEX);

    task AHB_WRITE_REG32;
        output [31:0] optreg;
        begin
            if (ahb_byte_strobe[0])
                optreg[ 0+:8] = ahb_wdata[ 0+:8];
            if (ahb_byte_strobe[1])
                optreg[ 8+:8] = ahb_wdata[ 8+:8];
            if (ahb_byte_strobe[2])
                optreg[16+:8] = ahb_wdata[16+:8];
            if (ahb_byte_strobe[3])
                optreg[24+:8] = ahb_wdata[24+:8];
        end
    endtask

    task TRANS_TRIGGER;
        input [3:0] requset;
        input en_wdata;
        input en_wtime;
        input en_cnt;
        input wtime_first;
        input [31:0] ret_sm;
        begin
            transfer_trig_requset <= requset;
            transfer_trig_ret_sm <= ret_sm;
            transfer_trig_en_wdata <= en_wdata;
            transfer_trig_en_wtime <= en_wtime;
            transfer_trig_en_cnt <= en_cnt;
            transfer_trig_wtime_first <= wtime_first;
            transfer_trig_ir <= transfer_trig_first_transfer | (transfer_trig_requset[0] ^ requset[0]);
            transfer_sm <= TRANS_SM_TRIGGER;
        end
    endtask

`ifdef SIMULATION

    task print_requset;
        input [3:0] requset;
        input ir_en;
        begin
            case (requset[1:0])
                2'b00: begin
                    $display(">>>> W DP %X IR %b", requset[3:2] << 2, ir_en);
                end
                2'b10: begin
                    if (requset[3:2] == 2'b00) begin
                        $display(">>>> R DP %X IR %b", requset[3:2] << 2, ir_en);
                    end
                    else begin
                        $display(">>>> RDBUFF IR %b", ir_en);
                    end
                end
                2'b01: begin
                    $display(">>>> W AP %X IR %b", requset[3:2] << 2, ir_en);
                end
                2'b11: begin
                    $display(">>>> R AP %X IR %b", requset[3:2] << 2, ir_en);
                end
            endcase
        end
    endtask


    always @(posedge clk) begin
        if (seq_tx_valid) begin
            print_requset(seq_tx_cmd[3:0], seq_tx_cmd[4]);
        end
    end
`endif

    DAP_Seqence dap_seqence_inst(
                    // 控制器时钟
                    .clk(clk),
                    .resetn(resetn),

                    // 串行时钟
                    .sclk(sclk),
                    .sclk_out(sclk_out),
                    .sclk_negedge(sclk_negedge),
                    .sclk_sampling(sclk_sampling),
                    .sclk_sampling_en(sclk_sampling_en),

                    // 控制器输入输出
                    .seq_tx_valid(seq_tx_valid),
                    .seq_tx_cmd(seq_tx_cmd),
                    .seq_tx_data(seq_tx_data),
                    .seq_tx_full(seq_tx_full),

                    .seq_rx_valid(seq_rx_valid),
                    .seq_rx_flag(seq_rx_flag),
                    .seq_rx_data(seq_rx_data),

                    .DAP_TRANS_WAIT_RETRY(SWJ_WAIT_RETRY),
                    .SWD_TURN_CYCLE(SWD_CONF_TURN),
                    .SWD_CONF_FORCE_DATA(SWD_CONF_FORCE_DATA),
                    .SWD_CONF_TURN_CLK(SWD_CONF_TURN_CLK),

                    // `ifdef USE_JTAG
                    // JTAG IR配置信号 (解码后的值由DAP_SWJ根据cmd[8:6]选择)
                    .JTAG_COUNT(JTAG_CR_COUNT),
                    .JTAG_IR_LEN(jtag_ir_len),
                    .JTAG_IR_BEFORE_LEN(jtag_ir_before_len),
                    .JTAG_IR_AFTER_LEN(jtag_ir_after_len),
                    // `endif

                    // GPIO
                    .SWCLK_TCK_O(SWCLK_TCK_O),
                    .SWDIO_TMS_T(SWDIO_TMS_T),
                    .SWDIO_TMS_O(SWDIO_TMS_O),
                    .SWDIO_TMS_I(SWDIO_TMS_I),
                    .SWO_TDO_I(SWO_TDO_I),
                    .TDI_O(TDI_O),
                    // input RTCK_I,
                    .SRST_I(SRST_I),
                    .SRST_O(SRST_O),
                    .TRST_I(TRST_I),
                    .TRST_O(TRST_O)
                );
endmodule
