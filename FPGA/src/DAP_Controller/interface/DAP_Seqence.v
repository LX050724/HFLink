`include "DAP_Cmd.v"

// `define USE_FIFO

module DAP_Seqence (
        // 控制器时钟
        input clk,
        input resetn,

        // 串行时钟
        input sclk,
        input sclk_out,
        input sclk_negedge,
        input sclk_sampling,
        output sclk_sampling_en,

        // 控制器输入输出
        input seq_tx_valid,
        input [15:0] seq_tx_cmd,
        input [31:0] seq_tx_data,
        output seq_tx_full,

        output seq_rx_valid,
        output [15:0] seq_rx_flag,
        output [31:0] seq_rx_data,

        input [15:0] DAP_TRANS_WAIT_RETRY,
        input [1:0] SWD_TURN_CYCLE,
        input SWD_CONF_FORCE_DATA,
        input SWD_CONF_TURN_CLK,

`ifdef USE_JTAG
        // JTAG IR配置信号
        input [3:0] JTAG_COUNT,
        input [3:0] JTAG_IR_LEN,
        input [7:0] JTAG_IR_BEFORE_LEN,
        input [7:0] JTAG_IR_AFTER_LEN,
`endif

        // GPIO
        output SWCLK_TCK_O,
        output reg SWDIO_TMS_T,
        output reg SWDIO_TMS_O,
        input SWDIO_TMS_I,
        input SWO_TDO_I,
        output reg TDI_O,
        // input RTCK_I,
        input SRST_I,
        output reg SRST_O,
        input TRST_I,
        output reg TRST_O
    );

`ifdef USE_FIFO

    reg tx_nxt;
    wire [1:0] seq_tx_fifo_rnum;
    wire tx_valid = seq_tx_fifo_rnum[0];
    wire [15:0] tx_cmd;
    wire [31:0] tx_data;

    fifo_top seq_tx_fifo(
                 .WrReset(~resetn), //input WrReset
                 .RdReset(~resetn), //input RdReset
                 .WrClk(clk), //input WrClk
                 .WrEn(seq_tx_valid), //input WrEn
                 .Data({seq_tx_cmd, seq_tx_data}), //input [47:0] Data

                 .RdClk(sclk), //input RdClk
                 .RdEn(1'd1), //input RdEn
                 .Q({tx_cmd, tx_data}), //output [47:0] Q
                 .Rnum(seq_tx_fifo_rnum), //output [1:0] Rnum
                 .Empty(), //output Empty
                 .Full() //output Full
             );

    reg rx_valid;
    reg [15:0] rx_flag;
    reg [31:0] rx_data;
    wire [1:0] seq_rx_fifo_rnum;
    assign seq_rx_valid = seq_rx_fifo_rnum[0];
    fifo_top seq_rx_fifo(
                 .WrReset(~resetn), //input WrReset
                 .RdReset(~resetn), //input RdReset
                 .WrClk(sclk), //input WrClk
                 .WrEn(rx_valid), //input WrEn
                 .Data({rx_flag, rx_data}), //input [47:0] Data

                 .RdClk(clk), //input RdClk
                 .RdEn(1'd1), //input RdEn
                 .Q({seq_rx_flag, seq_rx_data}), //output [47:0] Q
                 .Rnum(Rnum), //output [1:0] Rnum
                 .Empty(), //output Empty
                 .Full() //output Full
             );

`else
    reg rx_valid;
    reg rx_valid2;
    reg [15:0] rx_flag;
    reg [31:0] rx_data;


    reg tx_valid_ff1;
    reg tx_valid_ff2;
    reg tx_valid;
    reg tx_nxt;
    reg [15:0] tx_cmd;
    reg [31:0] tx_data;
    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            tx_valid_ff1 <= 1'd0;
            tx_valid_ff2 <= 1'd0;
            tx_cmd <= 16'd0;
            tx_data <= 32'd0;
            tx_valid <= 1'd0;
        end
        else begin
            if (tx_valid_ff1 == 1 && tx_valid_ff2 == 0) begin
                tx_valid <= 1'd1;
                tx_cmd <= seq_tx_cmd;
                tx_data <= seq_tx_data;
            end
            else if (tx_nxt) begin
                tx_valid <= 1'd0;
            end
            tx_valid_ff1 <= seq_tx_valid;
            tx_valid_ff2 <= tx_valid_ff1;
        end
    end

    reg rx_valid_ff1;
    reg rx_valid_ff2;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            rx_valid_ff1 <= 1'd0;
            rx_valid_ff2 <= 1'd0;
        end
        else begin
            rx_valid_ff1 <= rx_valid;
            rx_valid_ff2 <= rx_valid_ff1;
        end
    end

    assign seq_rx_flag = rx_flag;
    assign seq_rx_data = rx_data;
    assign seq_rx_valid = rx_valid_ff2;

`endif

    // JTAG IR 配置值直接来自输入端口，由DAP_SWJ根据cmd[8:6]解码后传递
    wire [2:0] jtag_index = tx_cmd[8:6];
    wire [7:0] jtag_ir_after_len = JTAG_IR_AFTER_LEN;
    wire [3:0] jtag_ir_len = JTAG_IR_LEN;
    wire [7:0] jtag_ir_before_len = JTAG_IR_BEFORE_LEN;


    wire [3:0] current_cmd = tx_cmd[15:12];

    reg clock_oen;
    reg clock_idle;
    assign SWCLK_TCK_O = clock_oen ? sclk_out : clock_idle;

    reg [7:0] swj_busy;
    reg delay_clk_en;

    reg [1:0] swj_seq_sm;
    reg [7:0] swj_seq_count;



    // JTAG IR registers
    reg [3:0] jtag_ir_sm;
    reg [31:0] jtag_ir_tx_data;
    reg [7:0] jtag_ir_tx_count;

    // JTAG IDCODE registers
    reg jtag_idcode_sm;
    reg [3:0] jtag_idcode_tx_sm;
    reg [5:0] jtag_idcode_tx_count;
    reg [3:0] jtag_idcode_rx_sm;
    reg [31:0] jtag_idcode_rx_data;
    reg [5:0] jtag_idcode_rx_count;

    reg swd_seq_sm;
    reg swd_seq_dir;
    reg [3:0] swd_seq_num;
    reg [7:0] swd_seq_tx_data;
    reg [7:0] swd_seq_rx_data;
    reg [3:0] swd_seq_tx_count;
    reg [3:0] swd_seq_rx_count;

    reg swj_pin_sm;
    reg [7:0] swj_pin_select_reg;
    reg [7:0] swj_pin_output_reg;
    reg [31:0] swj_us_cnt;
    reg [7:0] swj_tick_cnt;
    wire [7:0] swj_pin_output = {tx_cmd[10], 1'd0, tx_cmd[11], 1'd0, tx_cmd[9:6]};
    wire [7:0] swj_pin_select = {tx_cmd[ 4], 1'd0, tx_cmd[ 5], 1'd0, tx_cmd[3:0]};
    wire [31:0] swj_pin_delay = seq_tx_data;
    wire [7:0] swj_pin_read = {
             SRST_I,
             1'd0,
             TRST_I,
             1'd0,
             SWO_TDO_I,
             TDI_O,
             SWDIO_TMS_I,
             SWCLK_TCK_O
         };

    localparam [1:0] SWD_TRANS_SM_IDLE = 2'd0;
    localparam [1:0] SWD_TRANS_SM_WORKING = 2'd1;
    localparam [1:0] SWD_TRANS_SM_CHECK = 2'd2;

    localparam [3:0] SWD_TRANS_IO_START = 4'd0;
    localparam [3:0] SWD_TRANS_IO_APnDP = 4'd1;
    localparam [3:0] SWD_TRANS_IO_RnW = 4'd2;
    localparam [3:0] SWD_TRANS_IO_A2 = 4'd3;
    localparam [3:0] SWD_TRANS_IO_A3 = 4'd4;
    localparam [3:0] SWD_TRANS_IO_PARITY = 4'd5;
    localparam [3:0] SWD_TRANS_IO_STOP = 4'd6;
    localparam [3:0] SWD_TRANS_IO_PARK = 4'd7;
    localparam [3:0] SWD_TRANS_IO_TURN1 = 4'd8;
    localparam [3:0] SWD_TRANS_IO_ACK0 = 4'd9;
    localparam [3:0] SWD_TRANS_IO_ACK1  = 4'd10;
    localparam [3:0] SWD_TRANS_IO_ACK2 = 4'd11;
    localparam [3:0] SWD_TRANS_IO_TURN2 = 4'd12;
    localparam [3:0] SWD_TRANS_IO_DATA = 4'd13;
    localparam [3:0] SWD_TRANS_IO_DATA_PATIYY = 4'd14;
    localparam [3:0] SWD_TRANS_IO_DONE = 4'd15;

    reg [1:0] swd_trans_sm;
    reg [1:0] swd_trans_turn_cycle;
    reg swd_force_data;
    reg [15:0] swd_trans_retry_max;
    reg [15:0] swd_trans_retry_cnt;

    reg [3:0] swd_trans_tx_sm;
    reg [11:0] swd_trans_tx_cnt;
    reg swd_trans_tx_APnDP;
    reg swd_trans_tx_RnW;
    reg [3:2] swd_trans_tx_ADDR;
    reg [31:0] swd_trans_tx_data;
    reg swd_trans_tx_parity;

    reg [3:0] swd_trans_rx_sm;
    reg [11:0] swd_trans_rx_cnt;
    reg [31:0] swd_trans_rx_data;
    reg swd_trans_rx_parity;
    reg [2:0] swd_trans_rx_ack;

    localparam [3:0] JTAG_EXIT2_DR = 4'h0;
    localparam [3:0] JTAG_EXIT1_DR = 4'h1;
    localparam [3:0] JTAG_SHIFT_DR = 4'h2;
    localparam [3:0] JTAG_PAUSE_DR = 4'h3;
    localparam [3:0] JTAG_SELECT_IR_SCAN = 4'h4;
    localparam [3:0] JTAG_UPDATE_DR = 4'h5;
    localparam [3:0] JTAG_CAPTURE_DR = 4'h6;
    localparam [3:0] JTAG_SELECT_DR_SCAN = 4'h7;
    localparam [3:0] JTAG_EXIT2_IR = 4'h8;
    localparam [3:0] JTAG_EXIT1_IR = 4'h9;
    localparam [3:0] JTAG_SHIFT_IR = 4'hA;
    localparam [3:0] JTAG_PAUSE_IR = 4'hB;
    localparam [3:0] JTAG_RUN_TEST_IDLE = 4'hC;
    localparam [3:0] JTAG_UPDATE_IR = 4'hD;
    localparam [3:0] JTAG_CAPUTRE_IR = 4'hE;
    localparam [3:0] JTAG_TEST_LOGIC_RESET = 4'hF;

    assign sclk_sampling_en = delay_clk_en;

    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
`ifdef USE_FIFO
            rx_valid <= 0;
`else
            rx_valid <= 0;
            rx_valid2 <= 0;
`endif

            clock_oen <= 1'd0;
            clock_idle <= 1'd1;
            rx_flag <= 16'd0;
            tx_nxt <= 1'd0;
            rx_data <= 32'd0;
            SWDIO_TMS_T <= 1'd0;
            SWDIO_TMS_O <= 1'd0;
            TDI_O <= 1'd1;
            SRST_O <= 1'd1;
            TRST_O <= 1'd1;
            swj_busy <= 8'd0;
            delay_clk_en <= 1'd0;

            swj_seq_sm <= 0;
            swj_seq_count <= 0;

            // JTAG IR registers reset
            jtag_ir_sm <= 4'd0;
            jtag_ir_tx_data = 32'd0;
            jtag_ir_tx_count = 8'd0;

            // JTAG IDCODE registers reset
            jtag_idcode_sm <= 1'd0;
            jtag_idcode_tx_sm <= 4'd0;
            jtag_idcode_tx_count <= 6'd0;
            jtag_idcode_rx_sm <= 4'd0;
            jtag_idcode_rx_data <= 32'd0;
            jtag_idcode_rx_count <= 6'd0;

            swd_seq_sm <= 1'd0;
            swd_seq_dir <= 1'd0;
            swd_seq_num <= 4'd0;
            swd_seq_tx_data <= 8'd0;
            swd_seq_rx_data <= 8'd0;
            swd_seq_tx_count <= 4'd0;
            swd_seq_rx_count <= 4'd0;

            swj_pin_select_reg <= 8'd0;
            swj_pin_output_reg <= 8'd0;
            swj_pin_sm <= 1'd0;
            swj_us_cnt <= 32'd0;
            swj_tick_cnt <= 8'd0;


            swd_trans_sm <= 2'd0;
            swd_trans_turn_cycle <= 2'd0;
            swd_trans_retry_max <= 16'd0;
            swd_force_data <= 1'd0;
            swd_trans_retry_cnt <= 16'd0;
            swd_trans_tx_sm <= 4'd0;
            swd_trans_tx_cnt <= 12'd0;
            swd_trans_tx_APnDP <= 1'd0;
            swd_trans_tx_RnW <= 1'd0;
            swd_trans_tx_ADDR <= 2'd0;
            swd_trans_tx_parity <= 1'd0;
            swd_trans_tx_data <= 32'd0;
            swd_trans_rx_sm <= 4'd0;
            swd_trans_rx_cnt <= 12'd0;
            swd_trans_rx_data <= 32'd0;
            swd_trans_rx_parity <= 1'd0;
            swd_trans_rx_ack <= 3'd0;

        end
        else begin
            tx_nxt <= 1'd0;

`ifdef USE_FIFO

            rx_valid <= 1'd0;
`else
            rx_valid2 <= rx_valid;
            if (rx_valid2)
                rx_valid <= 1'd0;
`endif

            delay_clk_en <= 1'd1;



            // SEQ_CMD_SWD_SEQ
            case (swd_seq_sm)
                1'd0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWD_SEQ && swj_busy == 8'd0) begin
                        tx_nxt <= 1'd1;
                        swd_seq_num <= tx_cmd[3:0];
                        swd_seq_dir <= tx_cmd[4];
                        swd_seq_tx_count <= 4'd0;
                        swd_seq_rx_count <= 4'd0;
                        swd_seq_tx_data <= tx_data[7:0];
                        swd_seq_rx_data <= 8'd0;

                        swd_seq_sm <= 2'd1;
                        delay_clk_en <= 1'd0;
                    end
                end
                1'd1: begin
                    if (sclk_negedge) begin
                        if (swd_seq_tx_count != swd_seq_num) begin
                            swd_seq_tx_count <= swd_seq_tx_count + 1'd1;
                            {swd_seq_tx_data, SWDIO_TMS_O} <= {1'd0, swd_seq_tx_data};

                            SWDIO_TMS_T <= swd_seq_dir;
                            clock_oen <= 1'd1;
                        end
                        else begin
                            clock_oen <= 1'd0;
                        end
                    end

                    if (sclk_sampling) begin
                        if (swd_seq_rx_count != swd_seq_num) begin
                            swd_seq_rx_count <= swd_seq_rx_count + 1'd1;
                            swd_seq_rx_data[swd_seq_rx_count] <= SWDIO_TMS_I;
                        end
                        else begin
                            // swd_seq_rx_done <= 1'd1;
                            SWDIO_TMS_T <= 1'd0;
                            SWDIO_TMS_O <= 1'd0;
                        end

                        if (swd_seq_tx_count == swd_seq_num && swd_seq_rx_count == swd_seq_num) begin
                            rx_data[7:0] <= swd_seq_rx_data;
                            rx_flag <= swd_seq_num;
                            rx_valid <= 1'd1;
                            swj_busy[0] <= 1'd0;
                            swd_seq_sm <= 1'd0;
                        end
                    end
                end
            endcase

            // SEQ_CMD_SWJ_PINS
            case (swj_pin_sm) /*synthesis parallel_case*/
                0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWJ_PINS && swj_busy == 8'd0) begin
                        // 位 0：SWCLK/TCK
                        if (swj_pin_select[0]) begin
                            clock_idle <= swj_pin_output[0];
                        end

                        // 位 1：SWDIO/TMS
                        if (swj_pin_select[1]) begin
                            SWDIO_TMS_T <= 1'd0;
                            SWDIO_TMS_O <= swj_pin_output[1];
                        end
                        else begin
                            SWDIO_TMS_T <= 1'd1;
                        end

                        // 位 2：TDI
                        if (swj_pin_select[2]) begin
                            TDI_O <= swj_pin_output[2];
                        end

                        // 位 5：nTRST
                        if (swj_pin_select[5]) begin
                            TRST_O <= swj_pin_output[5];
                        end

                        // 位 7：nRESET
                        if (swj_pin_select[7]) begin
                            SRST_O <= swj_pin_output[7];
                        end

                        swj_pin_select_reg <= swj_pin_select;
                        swj_pin_output_reg <= swj_pin_output;
                        swj_pin_sm <= 1'd1;
                        swj_us_cnt <= swj_pin_delay;
                        swj_tick_cnt <= 8'd0;
                        swj_busy[1] <= 1'd1;
                        tx_nxt <= 1'd1;
                    end
                end

                1: begin
                    if (swj_tick_cnt < 8'd120) begin
                        swj_tick_cnt <= swj_tick_cnt + 1'd1;
                    end
                    else begin
                        swj_tick_cnt <= 8'd0;
                        swj_us_cnt <= swj_us_cnt - 1'd1;
                    end

                    if (
                        (swj_us_cnt == 32'd0) ||
                        ((swj_pin_select_reg & (swj_pin_read ^ swj_pin_output_reg)) == 8'd0)
                    ) begin
                        rx_data[7:0] <= swj_pin_read;
                        rx_flag <= 16'd0;
                        rx_valid <= 1'd1;
                        swj_pin_sm <= 1'd0;
                        swj_busy[1] <= 1'd0;
                    end
                end
            endcase

            // SEQ_CMD_SWD_TRANSFER
            case (swd_trans_sm) /*synthesis parallel_case*/
                SWD_TRANS_SM_IDLE: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWD_TRANSFER && swj_busy == 8'd0) begin
                        tx_nxt <= 1'd1;
                        swj_busy[2] <= 1'd1;

                        // 装载数据
                        swd_trans_tx_data <= tx_data[31:0];
                        // 装载配置信息
                        swd_trans_retry_cnt <= 16'd0;
                        swd_trans_retry_max <= DAP_TRANS_WAIT_RETRY;
                        swd_trans_turn_cycle <= SWD_TURN_CYCLE;
                        swd_force_data <= SWD_CONF_FORCE_DATA;
                        // 装载请求头
                        swd_trans_tx_APnDP <= tx_cmd[0];
                        swd_trans_tx_RnW <= tx_cmd[1];
                        swd_trans_tx_ADDR <= tx_cmd[3:2];

                        swd_trans_rx_parity <= 1'd0;

                        delay_clk_en <= 0; // 复位延迟时钟标志
                        swd_trans_sm <= SWD_TRANS_SM_WORKING;
                    end
                end

                SWD_TRANS_SM_WORKING: begin
                    if (sclk_negedge) begin
                        swd_trans_tx_sm <= swd_trans_tx_sm + 4'd1;
                        case (swd_trans_tx_sm)
                            SWD_TRANS_IO_START: begin
                                swd_trans_tx_cnt <= 12'd0;
                                clock_oen <= 1'd1;
                                SWDIO_TMS_O <= 1'd1;
                                SWDIO_TMS_T <= 1'd0;
                                // delay_clk_en <= 0;
                            end

                            SWD_TRANS_IO_APnDP: begin
                                SWDIO_TMS_O <= swd_trans_tx_APnDP;
                            end

                            SWD_TRANS_IO_RnW: begin
                                SWDIO_TMS_O <= swd_trans_tx_RnW;
                            end

                            SWD_TRANS_IO_A2: begin
                                SWDIO_TMS_O <= swd_trans_tx_ADDR[2];
                            end

                            SWD_TRANS_IO_A3: begin
                                SWDIO_TMS_O <= swd_trans_tx_ADDR[3];
                            end

                            SWD_TRANS_IO_PARITY: begin
                                SWDIO_TMS_O <= swd_trans_tx_APnDP ^ swd_trans_tx_RnW ^ swd_trans_tx_ADDR[2] ^ swd_trans_tx_ADDR[3];
                            end

                            SWD_TRANS_IO_STOP: begin
                                SWDIO_TMS_O <= 1'd0;
                            end

                            SWD_TRANS_IO_PARK: begin
                                SWDIO_TMS_O <= 1'd1;
                                swd_trans_tx_parity <= 1'd0;
                            end

                            SWD_TRANS_IO_TURN1: begin
                                SWDIO_TMS_T <= 1'd1;
                                if (swd_trans_tx_cnt[1:0] == swd_trans_turn_cycle) begin
                                    swd_trans_tx_cnt <= 12'd0;
                                    clock_oen <= SWD_CONF_TURN_CLK;
                                end
                                else begin
                                    swd_trans_tx_cnt <= swd_trans_tx_cnt + 1'd1;
                                    swd_trans_tx_sm <= SWD_TRANS_IO_TURN1;
                                    if (swd_trans_tx_cnt == 12'd1) begin
                                        clock_oen <= SWD_CONF_TURN_CLK;
                                    end
                                end
                            end
                            SWD_TRANS_IO_ACK0: begin
                                clock_oen <= 1'd1;
                            end
                            // SWD_TRANS_IO_ACK1: // Nothing
                            // SWD_TRANS_IO_ACK2: // Nothing
                            SWD_TRANS_IO_TURN2: begin
                                // 无论读写都进入SWD_TRANS_IO_TURN2
                                // 读模式下用于等待ACK结果，ACK失败生成TURN数量时钟后退出，ACK成功生成TURN段之后进入数据段生成全部时钟
                                if (swd_trans_tx_cnt[1:0] == swd_trans_turn_cycle) begin
                                    swd_trans_tx_cnt <= 12'd0;
                                    clock_oen <= swd_trans_tx_RnW | SWD_CONF_TURN_CLK;

                                    // 判断RX ACK状态
                                    if (swd_trans_rx_ack == 3'b001 || swd_force_data) begin // OK
                                        swd_trans_tx_sm <= SWD_TRANS_IO_DATA;
                                    end
                                    else begin // Wait / Error / Other
                                        swd_trans_tx_sm <= SWD_TRANS_IO_DONE;
                                    end
                                end
                                else begin
                                    swd_trans_tx_cnt <= swd_trans_tx_cnt + 1'd1;
                                    swd_trans_tx_sm <= SWD_TRANS_IO_TURN2;
                                    if (swd_trans_tx_cnt == 12'd1) begin
                                        clock_oen <= swd_trans_tx_RnW | SWD_CONF_TURN_CLK;
                                    end
                                end
                            end
                            SWD_TRANS_IO_DATA: begin
                                clock_oen <= 1'd1;
                                {swd_trans_tx_data, SWDIO_TMS_O} <= {1'd0, swd_trans_tx_data}; // 移位输出
                                if (swd_trans_tx_RnW) begin
                                    SWDIO_TMS_O <= 1'd0;
                                end

                                swd_trans_tx_parity <= swd_trans_tx_parity ^ swd_trans_tx_data[0];
                                if (swd_trans_tx_cnt == 12'd31) begin
                                    swd_trans_tx_cnt <= 12'd0;
                                end
                                else begin
                                    swd_trans_tx_cnt <= swd_trans_tx_cnt + 1'd1;
                                    swd_trans_tx_sm <= SWD_TRANS_IO_DATA;
                                end
                            end
                            SWD_TRANS_IO_DATA_PATIYY: begin
                                SWDIO_TMS_O <= swd_trans_tx_RnW ? 1'd0 : swd_trans_tx_parity;
                            end
                            SWD_TRANS_IO_DONE: begin
                                swd_trans_tx_sm <= SWD_TRANS_IO_DONE;
                                swd_trans_sm <= SWD_TRANS_SM_CHECK;
                                clock_oen <= 1'd0;
                                SWDIO_TMS_O <= 1'd0;
                                SWDIO_TMS_T <= 1'd0;
                            end
                        endcase
                    end

                    if (sclk_sampling) begin
                        swd_trans_rx_sm <= swd_trans_rx_sm + 4'd1;
                        case (swd_trans_rx_sm)
                            SWD_TRANS_IO_START: begin
                                swd_trans_rx_cnt <= 12'd0;
                                swd_trans_rx_ack <= 3'd0;
                            end
                            SWD_TRANS_IO_TURN1: begin
                                if (swd_trans_rx_cnt == swd_trans_turn_cycle) begin
                                    swd_trans_rx_cnt <= 12'd0;
                                end
                                else begin
                                    swd_trans_rx_cnt <= swd_trans_rx_cnt + 1'd1;
                                    swd_trans_rx_sm <= SWD_TRANS_IO_TURN1;
                                end
                            end
                            SWD_TRANS_IO_ACK0: begin
                                swd_trans_rx_ack[0] <= SWDIO_TMS_I;
                            end
                            SWD_TRANS_IO_ACK1: begin
                                swd_trans_rx_ack[1] <= SWDIO_TMS_I;
                            end
                            SWD_TRANS_IO_ACK2: begin
                                swd_trans_rx_ack[2] <= SWDIO_TMS_I;
                                swd_trans_rx_parity <= 1'd0;
                                if ({SWDIO_TMS_I, swd_trans_rx_ack[1:0]} == 3'b001 || SWD_CONF_FORCE_DATA) begin
                                    // OK
                                    // 读请求ACK后跟数据段
                                    // 写请求ACK后无读取内容
                                    swd_trans_rx_sm <= swd_trans_tx_RnW ? SWD_TRANS_IO_DATA : SWD_TRANS_IO_DONE;
                                    SWDIO_TMS_T <= swd_trans_tx_RnW;
                                end
                                else begin
                                    // WAIT / ERROR / Other
                                    swd_trans_rx_sm <= SWD_TRANS_IO_DONE;
                                end
                            end
                            SWD_TRANS_IO_DATA: begin
                                swd_trans_rx_data <= {SWDIO_TMS_I, swd_trans_rx_data[31:1]};
                                swd_trans_rx_parity <= swd_trans_rx_parity ^ SWDIO_TMS_I;
                                if (swd_trans_rx_cnt == 12'd31) begin
                                    swd_trans_rx_cnt <= 12'd0;
                                end
                                else begin
                                    swd_trans_rx_cnt <= swd_trans_rx_cnt + 1'd1;
                                    swd_trans_rx_sm <= SWD_TRANS_IO_DATA;
                                end
                            end
                            SWD_TRANS_IO_DATA_PATIYY: begin
                                swd_trans_rx_parity <= swd_trans_rx_parity ^ SWDIO_TMS_I;
                                SWDIO_TMS_O <= 1'd0;
                                SWDIO_TMS_T <= 1'd0;
                            end
                            SWD_TRANS_IO_DONE: begin
                                swd_trans_rx_sm <= SWD_TRANS_IO_DONE;
                            end
                        endcase
                    end
                end

                SWD_TRANS_SM_CHECK: begin
                    swd_trans_tx_sm <= SWD_TRANS_IO_START;
                    swd_trans_rx_sm <= SWD_TRANS_IO_START;
                    if (swd_trans_rx_ack == 3'b010 && swd_trans_retry_cnt != swd_trans_retry_max) begin
                        // WAIT 重试
                        swd_trans_retry_cnt <= swd_trans_retry_cnt + 1'd1;
                        delay_clk_en <= 1'd0;
                        swd_trans_sm <= SWD_TRANS_SM_WORKING;
                    end
                    else begin
                        // OK / WAIT Timeout / ERROR / Other
                        swd_trans_sm <= SWD_TRANS_SM_IDLE;

                        rx_data <= swd_trans_rx_data;
                        case (swd_trans_rx_ack)
                            3'b001, 3'b010, 3'b100:
                                rx_flag <= {12'd0, swd_trans_rx_parity, swd_trans_rx_ack};
                            default:
                                rx_flag <= {12'd0, swd_trans_rx_parity, 3'b111};
                        endcase
                        rx_valid <= 1'd1;
                        swj_busy[2] <= 1'd0;
                    end
                end
            endcase

`ifdef USE_JTAG
            // SEQ_CMD_JTAG_IR
            case (jtag_ir_sm)
                // Idle state - wait for command
                4'd0: begin // BEGIN
                    if (tx_valid && current_cmd == `SEQ_CMD_JTAG_IR && swj_busy == 8'd0) begin
                        tx_nxt <= 1'd1;
                        swj_busy[3] <= 1'd1;

                        // Get JTAG index from bits [8:6] of tx_cmd
                        jtag_ir_tx_data <= tx_data;

                        // Start clock and set TMS high
                        clock_oen <= 1'd1;
                        jtag_ir_sm <= 4'd1;
                    end
                end

                4'd1: begin // JTAG_RUN_TEST_IDLE -> JTAG_SELECT_DR_SCAN
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd1;
                        SWDIO_TMS_T <= 1'd0;
                        TDI_O <= 1'd0;
                        jtag_ir_sm <= 4'd2;

                    end
                end

                4'd2: begin // JTAG_SELECT_DR_SCAN -> JTAG_SELECT_IR_SCAN
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd1;
                        TDI_O <= 1'd0;
                        jtag_ir_sm <= 4'd3;
                    end
                end

                4'd3: begin // JTAG_SELECT_IR_SCAN -> JTAG_CAPUTRE_IR
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd0;
                        jtag_ir_tx_count <= jtag_ir_before_len;
                        jtag_ir_sm <= 4'd4;
                    end
                end

                4'd4: begin // JTAG_CAPUTRE_IR -> BEFOR==0 ? JTAG_SHIFT_IR : JTAG_SHIFT_IR.BEFOR
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd0;
                        jtag_ir_sm <= (jtag_ir_tx_count == 8'd0) ? 4'd6 : 4'd5;
                    end
                end

                4'd5: begin // JTAG_SHIFT_IR.BEFOR -> JTAG_SHIFT_IR
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd0;
                        TDI_O <= 1'd1;
                        if (jtag_ir_tx_count != 8'd0) begin
                            jtag_ir_tx_count <= jtag_ir_tx_count - 1'd1;
                        end
                        else begin
                            jtag_ir_tx_count <= {4'd0, jtag_ir_len};
                            jtag_ir_sm <= 4'd8;
                        end
                    end
                end

                4'd6: begin // JTAG_SHIFT_IR -> AFTER==0 ? JTAG_EXIT1_IR : JTAG_SHIFT_IR.AFTER
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd0;
                        {jtag_ir_tx_data, TDI_O} <= {1'd0, jtag_ir_tx_data};

                        if (jtag_ir_tx_count != 8'd0) begin
                            jtag_ir_tx_count <= jtag_ir_tx_count - 1'd1;
                        end
                        else begin
                            jtag_ir_tx_count <= jtag_ir_after_len;
                            if (jtag_ir_after_len == 8'd0) begin
                                jtag_ir_sm <= 4'd8;
                                SWDIO_TMS_O <= 1'd1;
                            end
                            else begin
                                jtag_ir_sm <= 4'd7;
                            end
                        end
                    end
                end

                4'd7: begin // JTAG_SHIFT_IR.AFTER -> JTAG_EXIT1_IR
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd1;
                        TDI_O <= 1'd1;
                        if (jtag_ir_tx_count != 8'd0) begin
                            jtag_ir_tx_count <= jtag_ir_tx_count - 1'd1;
                        end
                        else begin
                            jtag_ir_sm <= 4'd8;
                        end
                    end
                end

                4'd8: begin // JTAG_EXIT1_IR -> JTAG_UPDATE_IR
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd1;
                        jtag_ir_sm <= 4'd9;
                    end
                end

                4'd9: begin // JTAG_UPDATE_IR -> JTAG_RUN_TEST_IDLE
                    if (sclk_negedge) begin
                        SWDIO_TMS_O <= 1'd0;
                        jtag_ir_sm <= 4'd10;
                    end
                end

                4'd10: begin // FINISH
                    if (sclk_negedge) begin
                        SWDIO_TMS_T <= 1'd0;
                        SWDIO_TMS_O <= 1'd0;
                        clock_oen <= 1'd0;
                        rx_valid <= 1'd1;
                        swj_busy[3] <= 1'd0;
                        jtag_ir_sm <= 4'd0;
                    end
                end
            endcase

            // SEQ_CMD_JTAG_IDCODE
            if (jtag_idcode_sm == 1'd0) begin
                if (tx_valid && current_cmd == `SEQ_CMD_JTAG_IDCODE && swj_busy == 8'd0) begin
                    swj_busy[4] <= 1'd1;
                    jtag_idcode_sm <= 1'd1;
                    jtag_idcode_tx_sm <= 4'd0;
                    jtag_idcode_rx_sm <= 4'd0;
                    // 计算shift需要多少时钟
                    jtag_idcode_rx_data <= 32'd0;
                    delay_clk_en <= 0; // 复位延迟时钟标志
                end
            end
            else begin
                // JTAG IDCODE TX State Machine (sclk_negedge driven)
                if (sclk_negedge) begin
                    case (jtag_idcode_tx_sm)
                        4'd0: begin // RUN_TEST_IDLE -> SELECT_DR_SCAN
                            clock_oen <= 1'd1;
                            SWDIO_TMS_O <= 1'd1;
                            SWDIO_TMS_T <= 1'd0;
                            TDI_O <= 1'd0;
                            jtag_idcode_tx_sm <= 4'd1;
                        end

                        4'd1: begin // SELECT_DR_SCAN -> CAPTURE_DR
                            SWDIO_TMS_O <= 1'd0;
                            jtag_idcode_tx_sm <= 4'd2;
                        end

                        4'd2: begin // CAPTURE_DR -> SHIFT_DR (first bit)
                            SWDIO_TMS_O <= 1'd0;
                            // JTAG IDCODE指令的SHIFT_DR阶段需要发送32+(TAP数量-1)个时钟
                            jtag_idcode_tx_count <= JTAG_COUNT + 8'd30;
                            jtag_idcode_tx_sm <= 4'd3;
                        end

                        4'd3: begin // SHIFT_DR BEFORE+32+AFTER
                            if (jtag_ir_tx_count != 8'd0) begin
                                jtag_ir_tx_count <= jtag_ir_tx_count - 1'd1;
                            end
                            else begin
                                SWDIO_TMS_O <= 1'd1;
                                jtag_idcode_tx_sm <= 4'd4;
                            end
                        end

                        4'd4: begin // EXIT1_DR -> UPDATE_DR
                            SWDIO_TMS_O <= 1'd1;
                            jtag_idcode_tx_sm <= 4'd5;
                        end

                        4'd5: begin // UPDATE_DR -> RUN_TEST_IDLE
                            SWDIO_TMS_O <= 1'd0;
                            jtag_idcode_tx_sm <= 4'd6;
                        end

                        4'd6: begin // FINISH
                            clock_oen <= 1'd0;
                        end
                    endcase
                end

                // JTAG IDCODE RX State Machine (sclk_sampling driven)
                if (sclk_sampling) begin
                    case (jtag_idcode_rx_sm)
                        // RUN_TEST_IDLE -> SELECT_DR_SCAN -> CAPTURE_DR -> SHIFT_DR
                        4'd0: begin
                            jtag_idcode_rx_sm <= 4'd1;    
                        end 
                        4'd1: begin
                            jtag_idcode_rx_sm <= 4'd2;
                        end
                        4'd2: begin
                            jtag_idcode_rx_count <= jtag_index;
                            jtag_idcode_rx_sm <= (jtag_index == 3'd0) ? 4'd4 : 4'd3;
                        end

                        4'd3: begin // SHIFT_DR - idle cycles
                            if (jtag_idcode_rx_count > 0) begin
                                jtag_idcode_rx_count <= jtag_idcode_rx_count - 1'd1;
                            end
                            else begin
                                jtag_idcode_rx_sm <= 4'd4;
                                jtag_idcode_rx_count <= 5'd31;  // 32 bits to sample
                            end
                        end

                        4'd4: begin // SHIFT_DR - sample 32 bits from TDO
                            if (jtag_idcode_rx_count > 0) begin
                                jtag_idcode_rx_count <= jtag_idcode_rx_count - 1'd1;
                                jtag_idcode_rx_data <= {SWO_TDO_I, jtag_idcode_rx_data[31:1]};
                            end
                            else begin
                                jtag_idcode_rx_sm <= 4'd5;
                            end
                        end

                        4'd5: begin // FINISH
                        end
                    endcase
                end

                if (jtag_idcode_tx_sm == 4'd6 && jtag_idcode_rx_sm == 4'd5) begin
                    jtag_idcode_tx_sm <= 4'd0;
                    jtag_idcode_rx_sm <= 4'd0;
                    jtag_idcode_sm <= 1'd0;
                    rx_data <= jtag_idcode_rx_data;
                    rx_valid <= 1'd1;
                    swj_busy[4] <= 1'd0;
                end
            end
`endif

        end
    end

endmodule
