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

    localparam [1:0] SEQ_STATUS_IDLE = 2'd0;
    localparam [1:0] SEQ_STATUS_BUSY = 2'd1;
    localparam [1:0] SEQ_STATUS_DONE = 2'd2;

    reg SWJ_CR;
    reg [15:0] SWJ_WAIT_RETRY;
    reg [15:0] SWJ_MATCH_RETRY;
    wire SWJ_CR_MODE = SWJ_CR; // 0: SWD; 1: JTAG


    reg [31:0] SWJ_SWD_CR;
    reg [7:0] SWJ_JTAG_CR;
    reg [31:0] SWJ_JTAG_IR_CONF_REG [0:7];

    wire [11:0] SWD_CONF_TURN = SWJ_SWD_CR[11:0];
    wire SWD_CONF_FORCE_DATA = SWJ_SWD_CR[31];

    wire [7:0] JTAG_CR_COUNT = SWJ_JTAG_CR[7:0];
    wire [13:0] JTAG_IR_BEFORE_CONF [0:7];
    wire [13:0] JTAG_IR_AFTER_CONF [0:7];
    wire [3:0] JTAG_IR_LEN_CONF [0:7];

    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : jtag_ir_conf_loop
            assign JTAG_IR_BEFORE_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][18+:14];
            assign JTAG_IR_AFTER_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][4+:14];
            assign JTAG_IR_LEN_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][3:0];
        end
    endgenerate
    assign SWD_MODE = SWJ_CR_MODE;

    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            SWJ_CR <= 0;
            SWJ_SWD_CR <= 9'd0;
            SWJ_JTAG_CR <= 8'd0;
            for (i = 0; i < 8; i = i + 1) begin
                SWJ_JTAG_IR_CONF_REG[i] <= 32'd0;
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
                        AHB_WRITE_REG32(SWJ_SWD_CR);
                    SWJ_JTAG_CR_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_CR);
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

    wire seq_rx_valid;
    wire [15:0] seq_rx_flag;
    wire [31:0] seq_rx_data;

    DAP_Seqence dap_seqence_inst(
                    // 控制器时钟
                    .clk(clk),
                    .resetn(resetn),

                    // 串行时钟
                    .sclk(sclk),
                    .sclk_out(sclk_out),
                    .sclk_negedge(sclk_negedge),
                    .sclk_sampling(sclk_sampling),

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


    localparam [2:0] SWJ_PINS_SM_READ_OUTPUT = 3'd0;
    localparam [2:0] SWJ_PINS_SM_READ_SELECT = 3'd1;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT0 = 3'd2;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT1 = 3'd3;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT2 = 3'd4;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT3 = 3'd5;
    localparam [2:0] SWJ_PINS_SM_READ_WAIT_RESPONE = 3'd6;
    localparam [2:0] SWJ_PINS_SM_READ_DONE = 3'd7;

    reg [2:0] swj_pins_sm;
    reg [5:0] swj_pins_output;
    reg [5:0] swj_pins_select;
    reg [31:0] swj_pins_wait_time;

    localparam [17:0] SWD_BTRANS_SM_READ_INDEX          = 20'h0_0001;
    localparam [17:0] SWD_BTRANS_SM_READ_COUNT_L        = 20'h0_0002;
    localparam [17:0] SWD_BTRANS_SM_READ_COUNT_H        = 20'h0_0004;
    localparam [17:0] SWD_BTRANS_SM_READ_REQUSET        = 20'h0_0008;
    localparam [17:0] SWD_BTRANS_SM_READ_DATA0          = 20'h0_0010;
    localparam [17:0] SWD_BTRANS_SM_READ_DATA1          = 20'h0_0020;
    localparam [17:0] SWD_BTRANS_SM_READ_DATA2          = 20'h0_0040;
    localparam [17:0] SWD_BTRANS_SM_READ_DATA3          = 20'h0_0080;
    localparam [17:0] SWD_BTRANS_SM_PROCESS_REQ         = 20'h0_0100;
    localparam [17:0] SWD_BTRANS_SM_TRANSFER            = 20'h0_0200;
    localparam [17:0] SWD_BTRANS_SM_WAIT_RDBUFF         = 20'h0_0400;
    localparam [17:0] SWD_BTRANS_SM_WRITE_DATA_0        = 20'h0_0800;
    localparam [17:0] SWD_BTRANS_SM_WRITE_DATA_1        = 20'h0_1000;
    localparam [17:0] SWD_BTRANS_SM_WRITE_DATA_2        = 20'h0_2000;
    localparam [17:0] SWD_BTRANS_SM_WRITE_DATA_3        = 20'h0_4000;
    localparam [17:0] SWD_BTRANS_SM_WRITE_COUNT_L       = 20'h0_8000;
    localparam [17:0] SWD_BTRANS_SM_WRITE_COUNT_H       = 20'h1_0000;
    localparam [17:0] SWD_BTRANS_SM_WRITE_RESPONSE      = 20'h2_0000;
    localparam [17:0] SWD_BTRANS_SM_END                 = 20'h0_0000;


    reg [17:0] swd_block_trans_sm /*synthesis syn_encoding="onehot"*/;
    reg [15:0] swd_block_trans_request_cnt;
    reg [15:0] swd_block_trans_response_cnt;
    reg [3:0] swd_block_trans_req;
    reg [31:0] swd_block_trans_data;
    reg swd_block_trans_seq_tx_valid;
    reg swd_block_trans_RnW;
    reg swd_block_trans_err_flag;

    localparam [26:0] SWD_TRANS_SM_READ_INDEX          = 27'h000_0001;
    localparam [26:0] SWD_TRANS_SM_READ_COUNT          = 27'h000_0002;
    localparam [26:0] SWD_TRANS_SM_READ_REQUSET        = 27'h000_0004;
    localparam [26:0] SWD_TRANS_SM_READ_DATA0          = 27'h000_0008;
    localparam [26:0] SWD_TRANS_SM_READ_DATA1          = 27'h000_0010;
    localparam [26:0] SWD_TRANS_SM_READ_DATA2          = 27'h000_0020;
    localparam [26:0] SWD_TRANS_SM_READ_DATA3          = 27'h000_0040;
    localparam [26:0] SWD_TRANS_SM_CHECK_POSTREAD      = 27'h000_0080;
    localparam [26:0] SWD_TRANS_SM_READ_AP             = 27'h000_0100;
    localparam [26:0] SWD_TRANS_SM_READ_DP             = 27'h000_0200;
    localparam [26:0] SWD_TRANS_SM_WRITE_APDP          = 27'h000_0400;
    localparam [26:0] SWD_TRANS_SM_MATCH_STEP1         = 27'h000_0800;
    localparam [26:0] SWD_TRANS_SM_MATCH_STEP2         = 27'h000_1000;
    localparam [26:0] SWD_TRANS_SM_MATCH_STEP3         = 27'h000_2000;
    localparam [26:0] SWD_TRANS_SM_CHECK_WIRTE         = 27'h000_4000;
    localparam [26:0] SWD_TRANS_SM_WRTE_COUNT          = 27'h000_8000;
    localparam [26:0] SWD_TRANS_SM_WRTE_STATUS         = 27'h001_0000;
    localparam [26:0] SWD_TRANS_SM_TRIGGER             = 27'h002_0000;
    localparam [26:0] SWD_TRANS_SM_WAIT                = 27'h004_0000;
    localparam [26:0] SWD_TRANS_SM_WRITE_DATA0         = 27'h008_0000;
    localparam [26:0] SWD_TRANS_SM_WRITE_DATA1         = 27'h010_0000;
    localparam [26:0] SWD_TRANS_SM_WRITE_DATA2         = 27'h020_0000;
    localparam [26:0] SWD_TRANS_SM_WRITE_DATA3         = 27'h040_0000;
    localparam [26:0] SWD_TRANS_SM_WRITE_TIME0         = 27'h080_0000;
    localparam [26:0] SWD_TRANS_SM_WRITE_TIME1         = 27'h100_0000;
    localparam [26:0] SWD_TRANS_SM_WRITE_TIME2         = 27'h200_0000;
    localparam [26:0] SWD_TRANS_SM_WRITE_TIME3         = 27'h400_0000;
    localparam [26:0] SWD_TRANS_SM_DONE                = 27'h000_0000;



    reg swd_trans_post_read;
    reg swd_trans_check_write;
    reg swd_trans_has_error;
    reg swd_trans_match_failed;
    reg [31:0] swd_trans_wdata;
    reg [26:0] swd_trans_sm /*synthesis syn_encoding="onehot"*/;
    reg [31:0] swd_trans_match_mask;
    reg [7:0] swd_trans_num;
    reg [7:0] swd_trans_cnt;
    reg [7:0] swd_trans_requset;
    reg [31:0] swd_trans_timestamp;
    reg [15:0] swd_trans_match_cnt;
    reg swd_trans_seq_tx_valid;

    reg [3:0] swd_trans_trig_requset;
    reg [26:0] swd_trans_trig_ret_sm;
    reg swd_trans_trig_en_wdata; // 启用数据段
    reg swd_trans_trig_en_wtime; // 启用时间戳段
    reg swd_trans_trig_en_cnt;   // 传输成功时完成计数增加
    reg swd_trans_trig_wtime_first; // 先写入时间戳，同时有data和time全开的效果

    // Bit 0: APnDP: 0 = Debug Port (DP), 1 = Access Port (AP).
    // Bit 1: RnW: 0 = Write Register, 1 = Read Register.
    // Bit 2: A2 Register Address bit 2.
    // Bit 3: A3 Register Address bit 3.
    // Bit 4: Value Match
    // Bit 5: Match Mask
    // Bit 7: TD_TimeStamp request
    wire swd_trans_requset_APnDP = swd_trans_requset[0];
    wire swd_trans_requset_RnW = swd_trans_requset[1];
    wire swd_trans_requset_MATCH = swd_trans_requset[4];
    wire swd_trans_requset_MASK = swd_trans_requset[5];
    wire swd_trans_requset_TIMESTAMP = swd_trans_requset[7];



    always @(*) begin
        casez ({start, 2'd0}) /* synthesis parallel_case */
            `CMD_TRANSFER_BLOCK: begin
                seq_tx_cmd = {`SEQ_CMD_SWD_TRANSFER, 8'd0, swd_block_trans_req};
                seq_tx_data = swd_block_trans_data;
                seq_tx_valid = swd_block_trans_seq_tx_valid;

                // ram_write_addr
                // ram_write_data
                // ram_write_en
                // packet_len
            end
            `CMD_TRANSFER: begin
                seq_tx_cmd = {`SEQ_CMD_SWD_TRANSFER, 8'd0, swd_trans_trig_requset};
                seq_tx_data = swd_trans_wdata;
                seq_tx_valid = (swd_trans_sm & SWD_TRANS_SM_TRIGGER) != 27'd0;
            end
            `CMD_WRITE_ABORT: begin
                seq_tx_cmd = 16'd0;
                seq_tx_data = 32'd0;
                seq_tx_valid = 1'd0;
            end
            `CMD_SWJ_PINS: begin
                seq_tx_cmd = {`SEQ_CMD_SWJ_PINS, swj_pins_output, swj_pins_select};
                seq_tx_data = swj_pins_wait_time;
                seq_tx_valid = (swj_pins_sm == SWJ_PINS_SM_READ_WAIT3) && dap_in_tvalid;

                // ram_write_addr <= 10'd0;
                // ram_write_data <= seq_rx_data[7:0];
                // ram_write_en <= (swj_pins_sm == SWJ_PINS_SM_READ_WAIT_RESPONE) && seq_rx_valid;
                // packet_len <= 1'd1;
            end
            `CMD_SWJ_SEQUENCE: begin
                seq_tx_cmd = {`SEQ_CMD_SWD_SEQ, 7'd0, 1'd0, swj_seq_trans_num};
                seq_tx_data = {24'd0, swj_seq_data};
                seq_tx_valid = swj_seq_tx_valid;
            end
            `CMD_SWD_SEQUENCE: begin
                seq_tx_cmd = {`SEQ_CMD_SWD_SEQ, 7'd0, swd_seq_dir, swd_seq_trans_num};
                seq_tx_data = {24'd0, swd_seq_send_data};
                seq_tx_valid = swd_seq_tx_valid;
            end
            default: begin
                seq_tx_cmd = 16'd0;
                seq_tx_data = 32'd0;
                seq_tx_valid = 1'd0;
            end
        endcase
    end

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            ram_write_en <= 1'd0;
            ram_write_data <= 8'd0;
            ram_write_addr <= 10'd0;
            done <= 0;
            packet_len <= 1'd0;

            swj_seq_bit_num <= 0;
            swj_seq_trans_num <= 0;
            swj_seq_sm <= 0;
            swj_seq_data <= 0;
            swj_seq_tx_valid <= 0;

            swd_seq_sm <= 0;
            swd_seq_num <= 0;
            swd_seq_dir <= 0;
            swd_seq_bit_num <= 0;
            swd_seq_trans_num <= 0;
            swd_seq_send_data <= 0;
            swd_seq_tx_valid <= 0;

            swj_pins_sm <= SWJ_PINS_SM_READ_OUTPUT;
            swj_pins_output <= 6'd0;
            swj_pins_select <= 6'd0;
            swj_pins_wait_time <= 32'd0;

            swd_block_trans_sm <= SWD_BTRANS_SM_READ_INDEX;
            swd_block_trans_request_cnt <= 16'd0;
            swd_block_trans_response_cnt <= 16'd0;
            swd_block_trans_req <= 4'd0;
            swd_block_trans_data <= 32'd0;
            swd_block_trans_seq_tx_valid <= 1'd0;
            swd_block_trans_RnW <= 1'd0;
            swd_block_trans_err_flag <= 1'd0;

            swd_trans_post_read <= 0;
            swd_trans_check_write <= 0;
            swd_trans_has_error <= 0;
            swd_trans_match_failed <= 1'd0;
            swd_trans_wdata <= 32'd0;
            swd_trans_sm <= SWD_TRANS_SM_READ_INDEX;
            swd_trans_match_mask <= 32'd0;
            swd_trans_num <= 8'd0;
            swd_trans_cnt <= 8'd0;
            swd_trans_requset <= 8'd0;
            swd_trans_timestamp <= 1'd0;
            swd_trans_match_cnt <= 1'd0;
            swd_trans_seq_tx_valid <= 1'd0;
            swd_trans_trig_requset <= 4'd0;
            swd_trans_trig_ret_sm <= 26'd0;
            swd_trans_trig_en_wdata <= 1'd0;
            swd_trans_trig_en_wtime <= 1'd0;
            swd_trans_trig_en_cnt <= 1'd0;
            swd_trans_trig_wtime_first <= 1'd0;
        end
        else begin
            ram_write_en <= 1'd0;
            swd_trans_seq_tx_valid <= 1'd0;
            swd_block_trans_seq_tx_valid <= 1'd0;
            swj_seq_tx_valid <= 1'd0;
            swd_seq_tx_valid <= 1'd0;

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
                                ram_write_addr <= 10'd0;
                                ram_write_data <= 8'd0;
                                packet_len <= 1'd1;
                                ram_write_en <= 1'd1;
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

            if (start[`CMD_SWD_SEQUENCE_SHIFT]) begin
                case (swd_seq_sm) /* synthesis parallel_case */
                    SWD_SEQ_SM_READ_SEQ_NUM: begin
                        if (dap_in_tvalid) begin
                            ram_write_addr <= 10'd0;
                            ram_write_data <= 8'd0;
                            ram_write_en <= 1'd1;

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
                                packet_len <= ram_write_addr + 10'd1;
                                ram_write_addr <= ram_write_addr + 10'd1;
                                ram_write_data <= seq_rx_data[7:0];
                                ram_write_en <= 1'd1;
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
                                    done[`CMD_SWD_SEQUENCE_SHIFT] <= 1'd1;
                                end
                            end

                        end
                    end

                    default: begin
                    end
                endcase
            end
            else begin
                swd_seq_sm <= SWD_SEQ_SM_READ_SEQ_NUM;
                done[`CMD_SWD_SEQUENCE_SHIFT] <= 1'd0;
            end


            if (start[`CMD_SWJ_PINS_SHIFT]) begin
                case(swj_pins_sm) /* synthesis parallel_case */
                    SWJ_PINS_SM_READ_OUTPUT: begin
                        if (dap_in_tvalid) begin
                            swj_pins_output <= {dap_in_tdata[5], dap_in_tdata[7], dap_in_tdata[3:0]};
                            swj_seq_sm <= SWJ_PINS_SM_READ_SELECT;
                        end
                    end
                    SWJ_PINS_SM_READ_SELECT: begin
                        if (dap_in_tvalid) begin
                            swj_pins_select <= {dap_in_tdata[5], dap_in_tdata[7], dap_in_tdata[3:0]};
                            swj_seq_sm <= SWJ_PINS_SM_READ_WAIT0;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT0: begin
                        if (dap_in_tvalid) begin
                            swj_pins_wait_time[7:0] <= dap_in_tdata;
                            swj_seq_sm <= SWJ_PINS_SM_READ_WAIT1;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT1: begin
                        if (dap_in_tvalid) begin
                            swj_pins_wait_time[15:8] <= dap_in_tdata;
                            swj_seq_sm <= SWJ_PINS_SM_READ_WAIT2;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT2: begin
                        if (dap_in_tvalid) begin
                            swj_pins_wait_time[23:16] <= dap_in_tdata;
                            swj_seq_sm <= SWJ_PINS_SM_READ_WAIT3;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT3: begin
                        if (dap_in_tvalid) begin
                            swj_pins_wait_time[31:24] <= dap_in_tdata;
                            swj_seq_sm <= SWJ_PINS_SM_READ_WAIT_RESPONE;
                        end
                    end
                    SWJ_PINS_SM_READ_WAIT_RESPONE: begin
                        if (seq_rx_valid) begin
                            ram_write_addr <= 10'd0;
                            ram_write_data <= seq_rx_data[7:0];
                            ram_write_en <= 1'd1;
                            packet_len <= 1'd1;
                            done[`CMD_SWJ_PINS_SHIFT] <= 1'd1;
                            swj_pins_sm <= SWJ_PINS_SM_READ_DONE;
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

            if (start[`CMD_TRANSFER_BLOCK_SHIFT] && SWJ_CR_MODE == 1'd0) begin
                case (swd_block_trans_sm)
                    SWD_BTRANS_SM_READ_INDEX: begin // 读取DAP Index抛弃
                        ram_write_addr <= 10'd2;
                        packet_len <= 10'd3;
                        swd_block_trans_err_flag <= 1'd0;
                        if (dap_in_tvalid) begin
                            swd_block_trans_sm <= SWD_BTRANS_SM_READ_COUNT_L;
                        end
                    end
                    SWD_BTRANS_SM_READ_COUNT_L: begin
                        if (dap_in_tvalid) begin
                            swd_block_trans_request_cnt <= {dap_in_tdata, swd_block_trans_request_cnt[15:8]};
                            swd_block_trans_sm <= SWD_BTRANS_SM_READ_COUNT_H;
                        end
                    end
                    SWD_BTRANS_SM_READ_COUNT_H: begin
                        if (dap_in_tvalid) begin
                            swd_block_trans_request_cnt <= {dap_in_tdata, swd_block_trans_request_cnt[15:8]};
                            swd_block_trans_sm <= SWD_BTRANS_SM_READ_REQUSET;
                        end
                    end
                    SWD_BTRANS_SM_READ_REQUSET: begin // 读requset
                        if (dap_in_tvalid) begin
                            swd_block_trans_req <= dap_in_tdata[3:0];
                            swd_block_trans_RnW <= dap_in_tdata[1];
                            if (dap_in_tdata[1]) begin
                                swd_block_trans_sm <= SWD_BTRANS_SM_PROCESS_REQ;
                            end
                            else begin
                                swd_block_trans_sm <= SWD_TRANS_SM_READ_DATA0;
                            end
                        end
                    end
                    SWD_TRANS_SM_READ_DATA0: begin
                        if (dap_in_tvalid) begin
                            swd_block_trans_data[7:0] <= dap_in_tdata;
                            swd_block_trans_sm <= SWD_TRANS_SM_READ_DATA1;
                        end
                    end
                    SWD_TRANS_SM_READ_DATA1: begin
                        if (dap_in_tvalid) begin
                            swd_block_trans_data[15:8] <= dap_in_tdata;
                            swd_block_trans_sm <= SWD_TRANS_SM_READ_DATA2;
                        end
                    end
                    SWD_TRANS_SM_READ_DATA2: begin
                        if (dap_in_tvalid) begin
                            swd_block_trans_data[23:16] <= dap_in_tdata;
                            swd_block_trans_sm <= SWD_TRANS_SM_READ_DATA3;
                        end
                    end
                    SWD_TRANS_SM_READ_DATA3: begin
                        if (dap_in_tvalid) begin
                            swd_block_trans_data[31:24] <= dap_in_tdata;
                            swd_block_trans_sm <= SWD_BTRANS_SM_PROCESS_REQ;
                        end
                    end
                    SWD_BTRANS_SM_PROCESS_REQ: begin // 循环开始位置，根据请求类型触发读fifo数据或驱动SWD
                        if (!swd_block_trans_err_flag) begin
                            // 读请求直接驱动SWD
                            swd_block_trans_seq_tx_valid <= 1'd1;
                        end
                        swd_block_trans_sm <= SWD_BTRANS_SM_TRANSFER;
                    end
                    SWD_BTRANS_SM_TRANSFER: begin // 等待传输完成
                        if (!swd_block_trans_err_flag && (seq_rx_valid && seq_rx_flag[2:0] == 3'b001)) begin // 正常传输完成
                            // 请求计数-1
                            swd_block_trans_request_cnt <= swd_block_trans_request_cnt - 1'd1;
                            // 响应计数+1
                            swd_block_trans_response_cnt <= swd_block_trans_response_cnt + 1'd1;

                            case (swd_block_trans_req[1:0])
                                2'b11: begin // Read AP
                                    if (swd_block_trans_response_cnt == 16'd0) begin
                                        // 第一次传输
                                        swd_block_trans_sm <= SWD_BTRANS_SM_PROCESS_REQ;
                                    end
                                    else begin
                                        swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_DATA_0;
                                    end
                                end
                                2'b01: begin // Write AP
                                    if (swd_block_trans_request_cnt - 1'd1 == 16'd0) begin
                                        // 最后一次传输
                                        swd_block_trans_req <= 4'b1110;
                                        swd_block_trans_seq_tx_valid <= 1'd1;
                                        swd_block_trans_sm <= SWD_BTRANS_SM_WAIT_RDBUFF;
                                    end
                                    else begin
                                        swd_block_trans_sm <= SWD_BTRANS_SM_PROCESS_REQ;
                                    end
                                end
                                2'b10: begin // Read DP
                                    swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_DATA_0;
                                end
                                2'b00: begin // Write DP
                                    if (swd_block_trans_request_cnt - 1'd1 == 16'd0) begin
                                        // 完成
                                        swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_COUNT_L;
                                    end
                                    else begin
                                        swd_block_trans_sm <= SWD_BTRANS_SM_PROCESS_REQ;
                                    end
                                end
                            endcase
                        end

                        if (swd_block_trans_err_flag || (seq_rx_valid && seq_rx_flag[2:0] != 3'b001)) begin // 错误状态
                            swd_block_trans_err_flag <= 1'd1;
                            swd_block_trans_request_cnt <= swd_block_trans_request_cnt - 1'd1;
                            if (swd_block_trans_request_cnt - 1'd1 == 16'd0) begin
                                swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_COUNT_L;
                            end
                            else begin
                                swd_block_trans_sm <= SWD_BTRANS_SM_PROCESS_REQ;
                            end
                        end
                    end

                    SWD_BTRANS_SM_WAIT_RDBUFF: begin
                        if (seq_rx_valid) begin
                            if (swd_block_trans_RnW) begin
                                swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_DATA_0;
                            end
                            else begin
                                swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_COUNT_L;
                            end
                        end
                    end

                    SWD_BTRANS_SM_WRITE_DATA_0: begin
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= seq_rx_data[7:0];
                        ram_write_en <= 1'd1;
                        swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_DATA_1;
                    end
                    SWD_BTRANS_SM_WRITE_DATA_1: begin
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= seq_rx_data[15:8];
                        ram_write_en <= 1'd1;
                        swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_DATA_2;
                    end
                    SWD_BTRANS_SM_WRITE_DATA_2: begin
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= seq_rx_data[23:16];
                        ram_write_en <= 1'd1;
                        swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_DATA_3;
                    end
                    SWD_BTRANS_SM_WRITE_DATA_3: begin
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= seq_rx_data[31:24];
                        ram_write_en <= 1'd1;

                        if (swd_block_trans_req[0] == 1'd0) begin // 读DP
                            if (swd_block_trans_request_cnt == 16'd0) begin
                                swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_COUNT_L;
                            end
                            else begin
                                swd_block_trans_sm <= SWD_BTRANS_SM_PROCESS_REQ;
                            end
                        end
                        else begin // 读AP
                            if (swd_block_trans_request_cnt == 16'd0) begin // 最后一次读AP接RDBUFF
                                swd_block_trans_req <= 4'b1110;
                                swd_block_trans_seq_tx_valid <= 1'd1;
                                swd_block_trans_sm <= SWD_BTRANS_SM_WAIT_RDBUFF;
                            end
                            else begin
                                swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_COUNT_L;
                            end
                        end
                    end

                    SWD_BTRANS_SM_WRITE_COUNT_L: begin
                        ram_write_addr <= 0;
                        ram_write_data <= swd_block_trans_response_cnt[7:0];
                        ram_write_en <= 1'd1;
                        packet_len <= ram_write_addr + 1'd1;
                        swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_COUNT_H;
                    end
                    SWD_BTRANS_SM_WRITE_COUNT_H: begin
                        ram_write_addr <= 1;
                        ram_write_data <= swd_block_trans_response_cnt[15:8];
                        ram_write_en <= 1'd1;
                        swd_block_trans_sm <= SWD_BTRANS_SM_WRITE_RESPONSE;
                    end
                    SWD_BTRANS_SM_WRITE_RESPONSE: begin
                        ram_write_addr <= 2;
                        ram_write_data <= {5'd0, seq_rx_flag[2:0]};
                        ram_write_en <= 1'd1;
                        swd_block_trans_sm <= SWD_BTRANS_SM_END;
                        done[`CMD_TRANSFER_BLOCK_SHIFT] <= 1'd1;
                    end
                endcase
            end
            else begin
                swd_block_trans_sm <= SWD_BTRANS_SM_READ_INDEX;
                done[`CMD_TRANSFER_BLOCK_SHIFT] <= 1'd0;
            end

            if (start[`CMD_TRANSFER_SHIFT] && SWJ_CR_MODE == 1'd0) begin
                case(swd_trans_sm)
                    SWD_TRANS_SM_READ_INDEX: begin
                        swd_trans_post_read <= 1'd0;
                        swd_trans_check_write <= 1'd0;
                        swd_trans_has_error <= 1'd0;
                        swd_trans_match_failed <= 1'd0;
                        packet_len <= 10'd2;
                        ram_write_addr <= 10'd1;
                        if (dap_in_tvalid) begin
                            swd_trans_sm <= SWD_TRANS_SM_READ_COUNT;
                        end
                    end
                    SWD_TRANS_SM_READ_COUNT: begin
                        if (dap_in_tvalid) begin
                            swd_trans_sm <= SWD_TRANS_SM_READ_REQUSET;
                            swd_trans_num <= dap_in_tdata;
                            swd_trans_cnt <= 8'd0;
                        end
                    end
                    SWD_TRANS_SM_READ_REQUSET: begin
                        if (swd_trans_num) begin
                            if (dap_in_tvalid) begin
                                swd_trans_num <= swd_trans_num - 1'd1;
                                swd_trans_requset <= dap_in_tdata;
                                if (dap_in_tdata[1] == 1'd0 || dap_in_tdata[5:4] != 2'd0) begin
                                    // 写请求或设置MASK有数据段的请求
                                    swd_trans_sm <= SWD_TRANS_SM_READ_DATA0;
                                end
                                else begin
                                    // READ/MATCH请求先检查是否需求处理post_read
                                    swd_trans_sm <= swd_trans_has_error ? SWD_TRANS_SM_READ_REQUSET : SWD_TRANS_SM_CHECK_POSTREAD;
                                end
                            end
                        end
                        else begin
                            swd_trans_sm <= swd_trans_has_error ? SWD_TRANS_SM_WRTE_COUNT : SWD_TRANS_SM_CHECK_WIRTE;
                        end
                    end
                    SWD_TRANS_SM_READ_DATA0: begin
                        if (dap_in_tvalid) begin
                            swd_trans_wdata[7:0] <= dap_in_tdata;
                            if (swd_trans_requset_MASK) begin
                                swd_trans_match_mask[7:0] <= dap_in_tdata;
                            end
                            swd_trans_sm <= swd_trans_sm << 1'd1;
                        end
                    end
                    SWD_TRANS_SM_READ_DATA1: begin
                        if (dap_in_tvalid) begin
                            swd_trans_wdata[15:8] <= dap_in_tdata;
                            if (swd_trans_requset_MASK) begin
                                swd_trans_match_mask[15:8] <= dap_in_tdata;
                            end
                            swd_trans_sm <= swd_trans_sm << 1'd1;
                        end
                    end
                    SWD_TRANS_SM_READ_DATA2: begin
                        if (dap_in_tvalid) begin
                            swd_trans_wdata[23:16] <= dap_in_tdata;
                            if (swd_trans_requset_MASK) begin
                                swd_trans_match_mask[23:16] <= dap_in_tdata;
                            end
                            swd_trans_sm <= swd_trans_sm << 1'd1;
                        end
                    end
                    SWD_TRANS_SM_READ_DATA3: begin
                        if (dap_in_tvalid) begin
                            swd_trans_wdata[31:24] <= dap_in_tdata;
                            if (swd_trans_has_error) begin
                                swd_trans_sm <= SWD_TRANS_SM_READ_REQUSET;
                            end
                            else if (swd_trans_requset_MASK) begin
                                swd_trans_match_mask[31:24] <= dap_in_tdata;
                                swd_trans_cnt <= swd_trans_cnt + 1'd1;
                                swd_trans_sm <= SWD_TRANS_SM_READ_REQUSET;
`ifdef SIMULATION

                                $display("WRITE MASK %08x", {dap_in_tdata, swd_trans_match_mask[23:0]});
`endif

                            end
                            else begin
                                swd_trans_sm <= SWD_TRANS_SM_CHECK_POSTREAD;
                            end
                        end
                    end
                    SWD_TRANS_SM_CHECK_POSTREAD: begin
`ifdef SIMULATION
                        $display("SWD_TRANS_SM_CHECK_POSTREAD %d", swd_trans_post_read);
`endif

                        if (swd_trans_post_read) begin
                            // 需要先清除post_read状态的请求：MATCH、WriteAP、WriteDP、ReadDP (ReadAP之外所有请求)
                            if (swd_trans_requset_MATCH || {swd_trans_requset_RnW, swd_trans_requset_APnDP} != 2'b11) begin
                                swd_trans_post_read <= 1'd0;
                                // RDBUFF
                                swd_trans_trig_requset <= 4'b1110;
                                // 写入延迟数据
                                swd_trans_trig_en_wdata <= 1'd1;
                                // 读请求按配置写入时间戳，写请求没有时间戳
                                swd_trans_trig_en_wtime <= swd_trans_requset_RnW ? swd_trans_requset_TIMESTAMP : 1'd0;
                                // 不计入命令数量
                                swd_trans_trig_en_cnt <= 1'd0;
                                swd_trans_trig_wtime_first <= 1'd0;

                                if (swd_trans_requset_MATCH) begin // 匹配读取请求
                                    swd_trans_trig_ret_sm <= SWD_TRANS_SM_MATCH_STEP1;
                                end
                                else if (swd_trans_requset_RnW == 1'd0) begin // 写请求
                                    swd_trans_trig_ret_sm <= SWD_TRANS_SM_WRITE_APDP;
                                end
                                else begin // 读DP请求
                                    swd_trans_trig_ret_sm <= SWD_TRANS_SM_READ_DP;
                                end

                                swd_trans_sm <= SWD_TRANS_SM_TRIGGER;
                            end
                        end
                        else begin
                            if (swd_trans_requset_MATCH) begin // 匹配读取请求
                                swd_trans_sm <= swd_trans_requset_APnDP ? SWD_TRANS_SM_MATCH_STEP1 : SWD_TRANS_SM_MATCH_STEP2;
                            end
                            else if (swd_trans_requset_RnW == 1'd0) begin // 写请求
                                swd_trans_sm <= SWD_TRANS_SM_WRITE_APDP;
                            end
                            else if (swd_trans_requset_APnDP) begin // 读AP请求
                                swd_trans_sm <= SWD_TRANS_SM_READ_AP;
                            end
                            else begin
                                swd_trans_sm <= SWD_TRANS_SM_READ_DP;
                            end
                        end
                    end
                    SWD_TRANS_SM_READ_AP: begin // 读AP
                        //                requset, en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                        SWD_TRANS_TRIGGER(swd_trans_requset[3:0], swd_trans_post_read, swd_trans_requset_TIMESTAMP, 1'd1, 1'd0, SWD_TRANS_SM_READ_REQUSET);
                        swd_trans_check_write <= 1'd0;
                        swd_trans_post_read <= 1'd1;
`ifdef SIMULATION

                        $display("SWD_TRANS_SM_READ_AP");
`endif

                    end

                    SWD_TRANS_SM_READ_DP: begin // 读DP
                        //                requset, en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                        SWD_TRANS_TRIGGER(swd_trans_requset[3:0], 1'd1, swd_trans_requset_TIMESTAMP, 1'd1, 1'd1, SWD_TRANS_SM_READ_REQUSET);
                        swd_trans_check_write <= 1'd0;
                    end

                    SWD_TRANS_SM_WRITE_APDP: begin // 写AP/DP
                        //                requset,              en_wdata,       en_wtime,          en_cnt, wtime_first, ret_sm
                        SWD_TRANS_TRIGGER(swd_trans_requset[3:0], 1'd0, swd_trans_requset_TIMESTAMP, 1'd1, 1'd0, SWD_TRANS_SM_READ_REQUSET);
                        swd_trans_check_write <= 1'd1;
                    end

                    SWD_TRANS_SM_MATCH_STEP1: begin // 匹配第一步，AP预读取
                        //                requset,           en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                        SWD_TRANS_TRIGGER(swd_trans_requset[3:0], 1'd0, 1'd0, 1'd0, 1'd0, SWD_TRANS_SM_MATCH_STEP2);
                    end

                    SWD_TRANS_SM_MATCH_STEP2: begin // 匹配第二步，读取正式数据
                        //                requset,           en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                        SWD_TRANS_TRIGGER(swd_trans_requset[3:0], 1'd0, 1'd0, 1'd0, 1'd0, SWD_TRANS_SM_MATCH_STEP3);
                    end

                    SWD_TRANS_SM_MATCH_STEP3: begin // 匹配第三步，循环读取匹配
                        swd_trans_check_write <= 1'd0;
                        if (swd_trans_match_mask & seq_rx_data[31:0] == swd_trans_wdata) begin
                            swd_trans_cnt <= swd_trans_cnt + 1'd1;
                            swd_trans_sm <= SWD_TRANS_SM_READ_REQUSET;
                        end
                        else if (swd_trans_match_cnt + 1'd1 == SWJ_MATCH_RETRY) begin
                            swd_trans_match_failed <= 1'd1;
                            swd_trans_has_error <= 1'd1;
                            swd_trans_sm <= SWD_TRANS_SM_READ_REQUSET;
                        end
                        else begin
                            //                requset,           en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                            SWD_TRANS_TRIGGER(swd_trans_requset[3:0], 1'd0, 1'd0, 1'd0, 1'd0, SWD_TRANS_SM_MATCH_STEP3);
                            swd_trans_match_cnt <= swd_trans_match_cnt + 1'd1;
                        end
                    end

                    SWD_TRANS_SM_CHECK_WIRTE: begin
`ifdef SIMULATION
                        $display("SWD_TRANS_SM_CHECK_WIRTE pr:%d wc:%d", swd_trans_post_read, swd_trans_check_write);
`endif

                        if (swd_trans_post_read) begin
                            //                requset, en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                            SWD_TRANS_TRIGGER(4'b1110, 1'd1, 1'd0, 1'd0, 1'd0, SWD_TRANS_SM_WRTE_COUNT);
                        end
                        else if (swd_trans_check_write) begin
                            //                requset, en_wdata, en_wtime, en_cnt, wtime_first, ret_sm
                            SWD_TRANS_TRIGGER(4'b1110, 1'd0, 1'd0, 1'd0, 1'd0, SWD_TRANS_SM_WRTE_COUNT);
                        end
                        else begin
                            swd_trans_sm <= SWD_TRANS_SM_WRTE_COUNT;
                        end
                    end

                    SWD_TRANS_SM_WRTE_COUNT: begin
                        ram_write_addr <= 10'd0;
                        ram_write_data <= swd_trans_cnt;
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= swd_trans_sm << 1'd1;
                    end
                    SWD_TRANS_SM_WRTE_STATUS: begin
                        ram_write_addr <= 10'd1;
                        ram_write_data <= {4'd0, swd_trans_match_failed, seq_rx_flag[2:0]};
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= SWD_TRANS_SM_DONE;
                    end
                    SWD_TRANS_SM_TRIGGER: begin
`ifdef SIMULATION
                        case (swd_trans_trig_requset[1:0])
                            2'b00: begin
                                $display("W DP %X", swd_trans_trig_requset[3:2] << 2);
                            end
                            2'b10: begin
                                if (swd_trans_trig_requset[3:2] == 2'b00) begin
                                    $display("R DP %X", swd_trans_trig_requset[3:2] << 2);
                                end
                                else begin
                                    $display("RDBUFF");
                                end
                            end
                            2'b01: begin
                                $display("W AP %X", swd_trans_trig_requset[3:2] << 2);
                            end
                            2'b11: begin
                                $display("R AP %X", swd_trans_trig_requset[3:2] << 2);
                            end
                        endcase
`endif

                        swd_trans_timestamp <= clk_timer;
                        swd_block_trans_sm <= SWD_TRANS_SM_WAIT;
                        swd_trans_sm <= SWD_TRANS_SM_WAIT;
                    end
                    SWD_TRANS_SM_WAIT: begin
                        if (seq_rx_valid) begin
                            if (seq_rx_flag[2:0] == 3'b001) begin
                                if (swd_trans_trig_en_cnt) begin
                                    swd_trans_cnt <= swd_trans_cnt + 1'd1;
                                end
                                if (swd_trans_trig_wtime_first) begin
                                    swd_trans_sm <= SWD_TRANS_SM_WRITE_TIME0;
                                end
                                else if (swd_trans_trig_en_wdata) begin
                                    swd_trans_sm <= SWD_TRANS_SM_WRITE_DATA0;
                                end
                                else if (swd_trans_trig_en_wtime) begin
                                    swd_trans_sm <= SWD_TRANS_SM_WRITE_TIME0;
                                end
                                else begin
                                    swd_trans_sm <= swd_trans_trig_ret_sm;
                                end
                            end
                            else begin
                                swd_trans_has_error <= 1'd1;
                                swd_trans_sm <= SWD_TRANS_SM_READ_REQUSET;
                            end
                        end
                    end
                    SWD_TRANS_SM_WRITE_DATA0: begin
                        packet_len <= packet_len + 1'd1;
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= seq_rx_data[7:0];
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= swd_trans_sm << 1'd1;
                    end
                    SWD_TRANS_SM_WRITE_DATA1: begin
                        packet_len <= packet_len + 1'd1;
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= seq_rx_data[15:8];
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= swd_trans_sm << 1'd1;
                    end
                    SWD_TRANS_SM_WRITE_DATA2: begin
                        packet_len <= packet_len + 1'd1;
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= seq_rx_data[23:16];
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= swd_trans_sm << 1'd1;
                    end
                    SWD_TRANS_SM_WRITE_DATA3: begin
                        packet_len <= packet_len + 1'd1;
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= seq_rx_data[31:24];
                        ram_write_en <= 1'd1;
                        if (!swd_trans_trig_en_wtime || swd_trans_trig_wtime_first) begin
                            swd_trans_sm <= swd_trans_trig_ret_sm;
                        end
                        else begin
                            swd_trans_sm <= swd_trans_sm << 1'd1;
                        end
                    end
                    SWD_TRANS_SM_WRITE_TIME0: begin
                        packet_len <= packet_len + 1'd1;
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= swd_trans_timestamp[7:0];
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= swd_trans_sm << 1'd1;
                    end
                    SWD_TRANS_SM_WRITE_TIME1: begin
                        packet_len <= packet_len + 1'd1;
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= swd_trans_timestamp[15:8];
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= swd_trans_sm << 1'd1;
                    end
                    SWD_TRANS_SM_WRITE_TIME2: begin
                        packet_len <= packet_len + 1'd1;
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= swd_trans_timestamp[23:16];
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= swd_trans_sm << 1'd1;
                    end
                    SWD_TRANS_SM_WRITE_TIME3: begin
                        packet_len <= packet_len + 1'd1;
                        ram_write_addr <= ram_write_addr + 1'd1;
                        ram_write_data <= swd_trans_timestamp[31:24];
                        ram_write_en <= 1'd1;
                        swd_trans_sm <= swd_trans_trig_wtime_first ? SWD_TRANS_SM_WRITE_DATA0 : swd_trans_trig_ret_sm;
                    end
                endcase

                if (swd_trans_sm == SWD_TRANS_SM_DONE) begin
                    done[`CMD_TRANSFER_SHIFT] <= 1'd1;
                end
            end
            else begin
                swd_trans_sm <= SWD_TRANS_SM_READ_INDEX;
                done[`CMD_TRANSFER_SHIFT] <= 1'd0;
            end
        end
    end

    assign dap_in_tready[`CMD_SWJ_SEQUENCE_SHIFT] = ~swj_seq_sm[1];

    assign dap_in_tready[`CMD_SWD_SEQUENCE_SHIFT] =
           (swd_seq_sm == SWD_SEQ_SM_READ_SEQ_NUM) ||
           (swd_seq_sm == SWD_SEQ_SM_READ_SEQ_INFO) ||
           (swd_seq_sm == SWD_SEQ_SM_READ_DATA && swd_seq_dir == 1'd0);

    assign dap_in_tready[`CMD_SWJ_PINS_SHIFT] = swj_pins_sm < SWJ_PINS_SM_READ_WAIT_RESPONE;

    assign dap_in_tready[`CMD_TRANSFER_BLOCK_SHIFT] = (SWJ_CR_MODE == 1'd0) ?
           (swd_block_trans_sm[7:0] != 8'd0) :
           1'd0;

    assign dap_in_tready[`CMD_TRANSFER_SHIFT] = (SWJ_CR_MODE == 1'd0) ?
           (swd_trans_sm[2] ? (swd_trans_num != 8'd0) : swd_trans_sm[6:0] != 7'd0) :
           1'd0;

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

    task SWD_TRANS_TRIGGER;
        input [3:0] requset;
        input en_wdata;
        input en_wtime;
        input en_cnt;
        input wtime_first;
        input [31:0] ret_sm;
        begin
            swd_trans_trig_requset <= requset;
            swd_trans_trig_ret_sm <= ret_sm;
            swd_trans_trig_en_wdata <= en_wdata;
            swd_trans_trig_en_wtime <= en_wtime;
            swd_trans_trig_en_cnt <= en_cnt;
            swd_trans_trig_wtime_first <= wtime_first;
            swd_trans_sm <= SWD_TRANS_SM_TRIGGER;
        end
    endtask
endmodule
