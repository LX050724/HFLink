`include "DAP_Cmd.v"

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

        // JTAG IR配置信号
        input [3:0] JTAG_COUNT,
        input [3:0] JTAG_IR_LEN,
        input [7:0] JTAG_IR_BEFORE_LEN,
        input [7:0] JTAG_IR_AFTER_LEN,

        // GPIO
        output SWCLK_TCK_O,
        output reg SWDIO_TMS_T,
        output reg SWDIO_TMS_O,
        input SWDIO_TMS_I,
        input SWO_TDO_I,
        output reg TDI_O,
        // input RTCK_I,
        input SRST_I,
        output SRST_O,
        input TRST_I,
        output TRST_O
    );

    localparam CMD_INDEX_SWD_SEQ = 0;
    localparam CMD_INDEX_SWJ_PINS = 1;
    localparam CMD_INDEX_SWD_TRANS = 2;
    localparam CMD_INDEX_JTAG_SEQ = 3;
    localparam CMD_INDEX_JTAG_TRANS = 4;
    localparam CMD_INDEX_MAX = 5;

    reg [CMD_INDEX_MAX-1:0] swj_busy;
    reg [CMD_INDEX_MAX-1:0] rx_valid;
    reg [CMD_INDEX_MAX-1:0] rx_valid_hold;
    reg rx_valid_delay;
    reg [15:0] rx_flag;
    reg [31:0] rx_data;


    reg tx_valid_ff1;
    reg tx_valid_ff2;
    reg tx_valid;
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
            else if (swj_busy) begin
                tx_valid <= 1'd0;
            end
            tx_valid_ff1 <= seq_tx_valid;
            tx_valid_ff2 <= tx_valid_ff1;
        end
    end

    reg rx_valid_ff1;
    reg rx_valid_ff2;
    reg rx_valid_ff3;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            rx_valid_ff1 <= 1'd0;
            rx_valid_ff2 <= 1'd0;
            rx_valid_ff3 <= 1'd0;
        end
        else begin
            rx_valid_ff1 <= rx_valid != 5'd0 || rx_valid_delay;
            rx_valid_ff2 <= rx_valid_ff1;
            rx_valid_ff3 <= rx_valid_ff2;
        end
    end

    assign seq_rx_flag = rx_flag;
    assign seq_rx_data = rx_data;
    assign seq_rx_valid = rx_valid_ff2 & (rx_valid_ff3 ^ rx_valid_ff2);  // rx_valid在当前周期被置位，前一个周期没有被置位，说明是新数据到达
    // assign seq_rx_valid = rx_valid_ff2;

    // ========== 输出多路复用器 ==========
    always @(*) begin
        casez((rx_valid | rx_valid_hold)) /*synthesis parallel_case*/
            5'b????1: begin // SWD SEQ
                rx_flag = 16'd0;
                rx_data = {24'd0, swd_seq_rx_shift};
            end
            5'b???1?: begin // SWJ PINS
                rx_flag = 16'd0;
                rx_data = {24'd0, swj_pins_rx_data};
            end
            5'b??1??: begin // SWD TRANSFER
                rx_flag = {12'd0, swd_trans_rx_flag};
                rx_data = swd_trans_rx_shift;
            end
            5'b?1???: begin // JTAG SEQ
                rx_flag = 16'd0;
                rx_data = {24'd0, jtag_seq_rx_shift};
            end
            5'b1????: begin // JTAG TRANSFER
                rx_flag = {13'd0, jtag_trans_rx_flag};
                rx_data = jtag_trans_rx_data;
            end
            default: begin
                rx_flag = 16'd0;
                rx_data = 32'd0;
            end
        endcase
    end

    wire [3:0] current_cmd = tx_cmd[15:12];

    // ========== 各命令独立clock_oen寄存器数组 ==========
    reg [CMD_INDEX_MAX-1:0] clock_oen;
    reg [CMD_INDEX_MAX-1:0] delay_clk_en;

    // ========== 各命令独立RX寄存器 ==========
    // SWD_SEQ
    reg [7:0] swd_seq_rx_shift;

    // SWJ_PINS
    reg [7:0] swj_pins_rx_data;

    // SWD_TRANSFER
    reg [31:0] swd_trans_rx_shift;
    reg [3:0] swd_trans_rx_flag;

    reg [2:0] jtag_trans_rx_flag;

    // ========== SWJ_PINS 基础GPIO寄存器 (缺省值/空闲状态) ==========
    reg swj_pins_swclk_o_reg;
    reg swj_pins_swdio_o_reg;
    reg swj_pins_swdio_t_reg;
    reg swj_pins_tdi_o_reg;
    reg swj_pins_srst_o_reg;
    reg swj_pins_trst_o_reg;

    // ========== SWD_SEQ 专用GPIO寄存器 (仅swdio) ==========
    reg swd_seq_swdio_o_reg;
    reg swd_seq_swdio_t_reg;

    // ========== SWD_TRANSFER 专用GPIO寄存器 (仅swdio) ==========
    reg swd_trans_swdio_o_reg;
    reg swd_trans_swdio_t_reg;

    // ========== JTAG_SEQ 专用GPIO寄存器 ==========
    reg jtag_seq_tms_o_reg;
    reg jtag_seq_tdi_o_reg;

    // ========== JTAG_TRANS 专用GPIO寄存器 ==========
    reg jtag_trans_tms_o_reg;
    reg jtag_trans_tdi_o_reg;


    // ========== GPIO 输出选择器 ==========
    always @(*) begin
        casez (swj_busy) /*synthesis parallel_case*/
            5'b????1: begin  // SWD_SEQ 活跃
                SWDIO_TMS_O = swd_seq_swdio_o_reg;
                SWDIO_TMS_T = swd_seq_swdio_t_reg;
                TDI_O       = swj_pins_tdi_o_reg;
            end
            5'b???1?: begin // SWJ_PINS 活跃
                SWDIO_TMS_O = swj_pins_swdio_o_reg;
                SWDIO_TMS_T = swj_pins_swdio_t_reg;
                TDI_O       = swj_pins_tdi_o_reg;
            end
            5'b??1??: begin  // SWD_TRANSFER 活跃
                SWDIO_TMS_O = swd_trans_swdio_o_reg;
                SWDIO_TMS_T = swd_trans_swdio_t_reg;
                TDI_O       = swj_pins_tdi_o_reg;
            end
            5'b?1???: begin  // JTAG_SEQ 活跃
                SWDIO_TMS_O = jtag_seq_tms_o_reg;
                SWDIO_TMS_T = ~clock_oen[CMD_INDEX_JTAG_SEQ];
                TDI_O       = jtag_seq_tdi_o_reg;
            end
            5'b1????: begin  // JTAG_TRANS 活跃
                SWDIO_TMS_O = jtag_trans_tms_o_reg;
                SWDIO_TMS_T = ~clock_oen[CMD_INDEX_JTAG_TRANS];
                TDI_O       = jtag_trans_tdi_o_reg;
            end
            default: begin
                SWDIO_TMS_O = 1'd0;
                SWDIO_TMS_T = 1'd0;
                TDI_O       = 1'd0;
            end
        endcase
    end
    assign SRST_O = swj_pins_srst_o_reg;
    assign TRST_O = swj_pins_trst_o_reg;
    assign SWCLK_TCK_O = clock_oen ? sclk_out : 1'd0;
    // ==============================================

    // ========== SWD_SEQ 状态机寄存器 ==========
    reg swd_seq_sm;
    reg swd_seq_dir;
    reg [3:0] swd_seq_num;
    reg [7:0] swd_seq_tx_data;
    reg [3:0] swd_seq_tx_count;
    reg [3:0] swd_seq_rx_count;

    // ========== SWJ_PINS 状态机寄存器 ==========
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

    // ========== SWD_TRANSFER 状态机寄存器 ==========
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
    reg [31:0] swd_trans_tx_shift;
    reg [31:0] swd_trans_tx_data;
    reg swd_trans_tx_parity;

    reg [3:0] swd_trans_rx_sm;
    reg [11:0] swd_trans_rx_cnt;
    reg swd_trans_rx_parity;
    reg [2:0] swd_trans_rx_ack;

    // ========== JTAG 状态机寄存器 ==========
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

    reg jtag_seq_sm;
    reg jtag_seq_tms;
    reg [3:0] jtag_seq_num;
    reg [3:0] jtag_seq_tx_count;
    reg [3:0] jtag_seq_rx_count;
    reg [7:0] jtag_seq_tx_data;
    reg [7:0] jtag_seq_rx_shift;

    reg jtag_trans_sm;
    reg [15:0] jtag_trans_retry_cnt;
    reg [15:0] jtag_trans_retry_max;
    reg [2:0] jtag_trans_index;
    reg [4:0] jtag_trans_tx_sm;
    reg jtag_trans_idcode;
    reg jtag_trans_abort;
    reg jtag_trans_ir;
    reg jtag_trans_APnDP;
    reg [34:0] jtag_trans_tx_data;
    reg [34:0] jtag_trans_tx_shift;
    reg [7:0] jtag_trans_tx_count;
    reg [34:0] jtag_trans_rx_shift;
    reg [7:0] jtag_trans_rx_count;

    wire [2:0] jtag_trans_rx_ack = {jtag_trans_rx_shift[2], jtag_trans_rx_shift[0], jtag_trans_rx_shift[1]};
    wire [31:0] jtag_trans_rx_data = jtag_trans_rx_shift[34:3];

    assign sclk_sampling_en = delay_clk_en == 5'h1f;  // 只要有任一命令的延时使能被关闭就复位采样时钟

    // ============================================================================
    // rx_valid_hold 合并逻辑
    // ============================================================================
    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            rx_valid_hold <= 5'd0;
            rx_valid_delay <= 1'd0;
        end
        else begin
            rx_valid_hold <= swj_busy != 5'd0 ? rx_valid : rx_valid | rx_valid_hold;
            rx_valid_delay <= rx_valid != 5'd0;
        end
    end

    // ============================================================================
    // SWD_SEQ 命令实现
    // ============================================================================
    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            swd_seq_sm <= 1'd0;
            swd_seq_dir <= 1'd0;
            swd_seq_num <= 4'd0;
            swd_seq_tx_data <= 8'd0;
            swd_seq_rx_shift <= 8'd0;
            swd_seq_tx_count <= 4'd0;
            swd_seq_rx_count <= 4'd0;
            swd_seq_swdio_o_reg <= 1'd0;
            swd_seq_swdio_t_reg <= 1'd0;
            rx_valid[CMD_INDEX_SWD_SEQ] <= 1'd0;
            clock_oen[CMD_INDEX_SWD_SEQ] <= 1'd0;
            swj_busy[CMD_INDEX_SWD_SEQ] <= 1'd0;
            delay_clk_en[CMD_INDEX_SWD_SEQ] <= 1'd0;
        end
        else begin
            rx_valid[CMD_INDEX_SWD_SEQ] <= 1'd0;
            delay_clk_en[CMD_INDEX_SWD_SEQ] <= 1'd1;  // 默认不延时，状态机内部根据需要开启

            case (swd_seq_sm)
                1'd0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWD_SEQ && swj_busy == 8'd0) begin
                        swj_busy[CMD_INDEX_SWD_SEQ] <= 1'd1;
                        swd_seq_num <= tx_cmd[3:0];
                        swd_seq_dir <= tx_cmd[4];
                        swd_seq_tx_count <= 4'd0;
                        swd_seq_rx_count <= 4'd0;
                        swd_seq_tx_data <= tx_data[7:0];
                        swd_seq_rx_shift <= 8'd0;
                        swd_seq_sm <= 2'd1;
                        delay_clk_en[CMD_INDEX_SWD_SEQ] <= 1'd0;
                    end
                end
                1'd1: begin
                    if (sclk_negedge) begin
                        if (swd_seq_tx_count != swd_seq_num) begin
                            swd_seq_tx_count <= swd_seq_tx_count + 1'd1;
                            {swd_seq_tx_data, swd_seq_swdio_o_reg} <= {1'd0, swd_seq_tx_data};

                            swd_seq_swdio_t_reg <= swd_seq_dir;
                            clock_oen[CMD_INDEX_SWD_SEQ] <= 1'd1;
                        end
                        else begin
                            clock_oen[CMD_INDEX_SWD_SEQ] <= 1'd0;
                        end
                    end

                    if (sclk_sampling) begin
                        if (swd_seq_rx_count != swd_seq_num) begin
                            swd_seq_rx_count <= swd_seq_rx_count + 1'd1;
                            swd_seq_rx_shift[swd_seq_rx_count] <= SWDIO_TMS_I;
                        end
                        else begin
                            // swd_seq_rx_done <= 1'd1;
                            swd_seq_swdio_t_reg <= 1'd0;
                            swd_seq_swdio_o_reg <= 1'd0;
                            rx_valid[CMD_INDEX_SWD_SEQ] <= 1'd1;
                            swj_busy[CMD_INDEX_SWD_SEQ] <= 1'd0;
                            swd_seq_sm <= 1'd0;
                        end
                    end
                end
            endcase
        end
    end

    // ============================================================================
    // SWJ_PINS 命令实现
    // ============================================================================
    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            swj_pin_sm <= 1'd0;
            swj_pin_select_reg <= 8'd0;
            swj_pin_output_reg <= 8'd0;
            swj_us_cnt <= 32'd0;
            swj_tick_cnt <= 8'd0;
            swj_pins_swdio_o_reg <= 1'd0;
            swj_pins_swdio_t_reg <= 1'd0;
            swj_pins_tdi_o_reg <= 1'd0;
            swj_pins_srst_o_reg <= 1'd1;
            swj_pins_trst_o_reg <= 1'd1;
            swj_pins_rx_data <= 8'd0;
            rx_valid[CMD_INDEX_SWJ_PINS] <= 1'd0;
            delay_clk_en[CMD_INDEX_SWJ_PINS] <= 1'd0;
            clock_oen[CMD_INDEX_SWJ_PINS] <= 1'd0;
            swj_busy[CMD_INDEX_SWJ_PINS] <= 1'd0;
            swj_pins_swclk_o_reg <= 1'd0;
        end
        else begin
            rx_valid[CMD_INDEX_SWJ_PINS] <= 1'd0;
            delay_clk_en[CMD_INDEX_SWJ_PINS] <= 1'd1;

            case (swj_pin_sm) /*synthesis parallel_case*/
                1'd0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWJ_PINS && swj_busy == 8'd0) begin
                        // 位 0：SWCLK/TCK
                        if (swj_pin_select[0]) begin
                            swj_pins_swclk_o_reg <= swj_pin_output[0];
                        end

                        // 位 1：SWDIO/TMS
                        if (swj_pin_select[1]) begin
                            swj_pins_swdio_t_reg <= 1'd0;
                            swj_pins_swdio_o_reg <= swj_pin_output[1];
                        end
                        else begin
                            swj_pins_swdio_t_reg <= 1'd1;
                        end

                        // 位 2：TDI
                        if (swj_pin_select[2]) begin
                            swj_pins_tdi_o_reg <= swj_pin_output[2];
                        end

                        // 位 5：nTRST
                        if (swj_pin_select[5]) begin
                            swj_pins_trst_o_reg <= swj_pin_output[5];
                        end

                        // 位 7：nRESET
                        if (swj_pin_select[7]) begin
                            swj_pins_srst_o_reg <= swj_pin_output[7];
                        end

                        swj_pin_select_reg <= swj_pin_select;
                        swj_pin_output_reg <= swj_pin_output;
                        swj_pin_sm <= 1'd1;
                        swj_us_cnt <= swj_pin_delay;
                        swj_tick_cnt <= 8'd0;
                        swj_busy[CMD_INDEX_SWJ_PINS] <= 1'd1;
                    end
                end

                1'd1: begin
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
                        swj_pins_rx_data <= swj_pin_read;
                        swj_pin_sm <= 1'd0;
                        rx_valid[CMD_INDEX_SWJ_PINS] <= 1'd1;
                        swj_busy[CMD_INDEX_SWJ_PINS] <= 1'd0;
                    end
                end
            endcase
        end
    end

    // ============================================================================
    // SWD_TRANSFER 命令实现
    // ============================================================================
    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
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
            swd_trans_tx_shift <= 32'd0;
            swd_trans_rx_sm <= 4'd0;
            swd_trans_rx_cnt <= 12'd0;
            swd_trans_rx_shift <= 32'd0;
            swd_trans_rx_parity <= 1'd0;
            swd_trans_rx_ack <= 3'd0;
            swd_trans_rx_flag <= 4'd0;
            swd_trans_swdio_o_reg <= 1'd0;
            swd_trans_swdio_t_reg <= 1'd0;
            rx_valid[CMD_INDEX_SWD_TRANS] <= 1'd0;
            delay_clk_en[CMD_INDEX_SWD_TRANS] <= 1'd0;
            clock_oen[CMD_INDEX_SWD_TRANS] <= 1'd0;
            swj_busy[CMD_INDEX_SWD_TRANS] <= 1'd0;
        end
        else begin
            rx_valid[CMD_INDEX_SWD_TRANS] <= 1'd0;
            delay_clk_en[CMD_INDEX_SWD_TRANS] <= 1'd1;

            case (swd_trans_sm) /*synthesis parallel_case*/
                SWD_TRANS_SM_IDLE: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWD_TRANSFER && swj_busy == 8'd0) begin
                        swj_busy[CMD_INDEX_SWD_TRANS] <= 1'd1;

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

                        delay_clk_en[CMD_INDEX_SWD_TRANS] <= 1'd0; // 复位延迟时钟标志
                        swd_trans_sm <= SWD_TRANS_SM_WORKING;
                    end
                end

                SWD_TRANS_SM_WORKING: begin
                    if (sclk_negedge) begin
                        swd_trans_tx_sm <= swd_trans_tx_sm + 4'd1;
                        case (swd_trans_tx_sm) /*synthesis parallel_case*/
                            SWD_TRANS_IO_START: begin
                                swd_trans_tx_cnt <= 12'd0;
                                clock_oen[CMD_INDEX_SWD_TRANS] <= 1'd1;
                                swd_trans_swdio_o_reg <= 1'd1;
                                swd_trans_swdio_t_reg <= 1'd0;
                                swd_trans_tx_shift <= swd_trans_tx_data;
                                // delay_clk_en <= 0;
                            end

                            SWD_TRANS_IO_APnDP: begin
                                swd_trans_swdio_o_reg <= swd_trans_tx_APnDP;
                            end

                            SWD_TRANS_IO_RnW: begin
                                swd_trans_swdio_o_reg <= swd_trans_tx_RnW;
                            end

                            SWD_TRANS_IO_A2: begin
                                swd_trans_swdio_o_reg <= swd_trans_tx_ADDR[2];
                            end

                            SWD_TRANS_IO_A3: begin
                                swd_trans_swdio_o_reg <= swd_trans_tx_ADDR[3];
                            end

                            SWD_TRANS_IO_PARITY: begin
                                swd_trans_swdio_o_reg <= swd_trans_tx_APnDP ^ swd_trans_tx_RnW ^ swd_trans_tx_ADDR[2] ^ swd_trans_tx_ADDR[3];
                            end

                            SWD_TRANS_IO_STOP: begin
                                swd_trans_swdio_o_reg <= 1'd0;
                            end

                            SWD_TRANS_IO_PARK: begin
                                swd_trans_swdio_o_reg <= 1'd1;
                                swd_trans_tx_parity <= 1'd0;
                            end

                            SWD_TRANS_IO_TURN1: begin
                                swd_trans_swdio_t_reg <= 1'd1;
                                if (swd_trans_tx_cnt[1:0] == swd_trans_turn_cycle) begin
                                    swd_trans_tx_cnt <= 12'd0;
                                    clock_oen[CMD_INDEX_SWD_TRANS] <= SWD_CONF_TURN_CLK;
                                end
                                else begin
                                    swd_trans_tx_cnt <= swd_trans_tx_cnt + 1'd1;
                                    swd_trans_tx_sm <= SWD_TRANS_IO_TURN1;
                                    if (swd_trans_tx_cnt == 12'd1) begin
                                        clock_oen[CMD_INDEX_SWD_TRANS] <= SWD_CONF_TURN_CLK;
                                    end
                                end
                            end
                            SWD_TRANS_IO_ACK0: begin
                                clock_oen[CMD_INDEX_SWD_TRANS] <= 1'd1;
                            end
                            // SWD_TRANS_IO_ACK1: // Nothing
                            // SWD_TRANS_IO_ACK2: // Nothing
                            SWD_TRANS_IO_TURN2: begin
                                // 无论读写都进入SWD_TRANS_IO_TURN2
                                // 读模式下用于等待ACK结果，ACK失败生成TURN数量时钟后退出，ACK成功生成TURN段之后进入数据段生成全部时钟
                                if (swd_trans_tx_cnt[1:0] == swd_trans_turn_cycle) begin
                                    swd_trans_tx_cnt <= 12'd0;
                                    clock_oen[CMD_INDEX_SWD_TRANS] <= swd_trans_tx_RnW | SWD_CONF_TURN_CLK;

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
                                        clock_oen[CMD_INDEX_SWD_TRANS] <= swd_trans_tx_RnW | SWD_CONF_TURN_CLK;
                                    end
                                end
                            end
                            SWD_TRANS_IO_DATA: begin
                                clock_oen[CMD_INDEX_SWD_TRANS] <= 1'd1;
                                {swd_trans_tx_shift, swd_trans_swdio_o_reg} <= {1'd0, swd_trans_tx_shift}; // 移位输出
                                if (swd_trans_tx_RnW) begin
                                    swd_trans_swdio_o_reg <= 1'd0;
                                end

                                swd_trans_tx_parity <= swd_trans_tx_parity ^ swd_trans_tx_shift[0];
                                if (swd_trans_tx_cnt == 12'd31) begin
                                    swd_trans_tx_cnt <= 12'd0;
                                end
                                else begin
                                    swd_trans_tx_cnt <= swd_trans_tx_cnt + 1'd1;
                                    swd_trans_tx_sm <= SWD_TRANS_IO_DATA;
                                end
                            end
                            SWD_TRANS_IO_DATA_PATIYY: begin
                                swd_trans_swdio_o_reg <= swd_trans_tx_RnW ? 1'd0 : swd_trans_tx_parity;
                            end
                            SWD_TRANS_IO_DONE: begin
                                swd_trans_tx_sm <= SWD_TRANS_IO_DONE;
                                swd_trans_sm <= SWD_TRANS_SM_CHECK;
                                clock_oen[CMD_INDEX_SWD_TRANS] <= 1'd0;
                                swd_trans_swdio_o_reg <= 1'd0;
                                swd_trans_swdio_t_reg <= 1'd0;
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
                                    swd_trans_swdio_t_reg <= swd_trans_tx_RnW;
                                end
                                else begin
                                    // WAIT / ERROR / Other
                                    swd_trans_rx_sm <= SWD_TRANS_IO_DONE;
                                end
                            end
                            SWD_TRANS_IO_DATA: begin
                                swd_trans_rx_shift <= {SWDIO_TMS_I, swd_trans_rx_shift[31:1]};
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
                        delay_clk_en[CMD_INDEX_SWD_TRANS] <= 1'd0;
                        swd_trans_sm <= SWD_TRANS_SM_WORKING;
                    end
                    else begin
                        // OK / WAIT Timeout / ERROR / Other
                        swd_trans_sm <= SWD_TRANS_SM_IDLE;
                        case (swd_trans_rx_ack)
                            3'b001, 3'b010, 3'b100:
                                swd_trans_rx_flag <= {swd_trans_rx_parity, swd_trans_rx_ack};
                            default:
                                swd_trans_rx_flag <= {swd_trans_rx_parity, 3'b111};
                        endcase
                        rx_valid[CMD_INDEX_SWD_TRANS] <= 1'd1;
                        swj_busy[CMD_INDEX_SWD_TRANS] <= 1'd0;
                    end
                end
            endcase
        end
    end

    // ============================================================================
    // JTAG_SEQ 命令实现
    // ============================================================================
    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            jtag_seq_sm <= 1'd0;
            jtag_seq_tms <= 1'd0;
            jtag_seq_num <= 4'd0;
            jtag_seq_tx_count <= 4'd0;
            jtag_seq_rx_count <= 4'd0;
            jtag_seq_tx_data <= 8'd0;
            jtag_seq_rx_shift <= 8'd0;
            jtag_seq_tms_o_reg <= 1'd0;
            jtag_seq_tdi_o_reg <= 1'd0;
            rx_valid[CMD_INDEX_JTAG_SEQ] <= 1'd0;
            delay_clk_en[CMD_INDEX_JTAG_SEQ] <= 1'd0;
            clock_oen[CMD_INDEX_JTAG_SEQ] <= 1'd0;
            swj_busy[CMD_INDEX_JTAG_SEQ] <= 1'd0;
        end
        else begin
            rx_valid[CMD_INDEX_JTAG_SEQ] <= 1'd0;
            delay_clk_en[CMD_INDEX_JTAG_SEQ] <= 1'd1;

            case (jtag_seq_sm)
                1'd0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_JTAG_SEQ && swj_busy == 8'd0) begin
                        swj_busy[CMD_INDEX_JTAG_SEQ] <= 1'd1;
                        jtag_seq_sm <= 4'd1;
                        jtag_seq_num <= tx_cmd[3:0];
                        jtag_seq_tx_count <= 4'd0;
                        jtag_seq_rx_count <= 4'd0;
                        jtag_seq_tx_data <= tx_data[7:0];
                        jtag_seq_rx_shift <= 8'd0;
                        jtag_seq_tms <= tx_cmd[4];
                        delay_clk_en[CMD_INDEX_JTAG_SEQ] <= 1'd0; // 复位延迟时钟标志
                    end
                end
                1'd1: begin
                    if (sclk_negedge) begin
                        if (jtag_seq_tx_count != jtag_seq_num) begin
                            clock_oen[CMD_INDEX_JTAG_SEQ] <= 1'd1;
                            {jtag_seq_tx_data, jtag_seq_tdi_o_reg} <= {1'd0, jtag_seq_tx_data};
                            jtag_seq_tms_o_reg <= jtag_seq_tms;
                            jtag_seq_tx_count <= jtag_seq_tx_count + 1'd1;
                        end
                        else begin
                            jtag_seq_tms_o_reg <= 1'd0;
                            jtag_seq_tdi_o_reg <= 1'd0;
                            clock_oen[CMD_INDEX_JTAG_SEQ] <= 1'd0;
                        end
                    end

                    if (sclk_sampling) begin
                        if (jtag_seq_rx_count != jtag_seq_num) begin
                            jtag_seq_rx_count <= jtag_seq_rx_count + 1'd1;
                            jtag_seq_rx_shift[jtag_seq_rx_count] <= SWO_TDO_I;
                        end
                        else begin
                            jtag_seq_tms_o_reg <= 1'd0;
                            rx_valid[CMD_INDEX_JTAG_SEQ] <= 1'd1;
                            swj_busy[CMD_INDEX_JTAG_SEQ] <= 1'd0;
                            jtag_seq_sm <= 4'd0;
                        end
                    end
                end
            endcase
        end
    end

    // ============================================================================
    // JTAG_TRANSFER 命令实现
    // ============================================================================
    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            jtag_trans_sm <= 1'd0;
            jtag_trans_retry_cnt <= 16'd0;
            jtag_trans_retry_max <= 16'd0;
            jtag_trans_index <= 3'd0;
            jtag_trans_tx_sm <= 5'd0;
            jtag_trans_tx_data <= 35'd0;
            jtag_trans_tx_shift <= 35'd0;
            jtag_trans_tx_count <= 8'd0;
            jtag_trans_rx_shift <= 35'd0;
            jtag_trans_rx_count <= 8'd0;
            jtag_trans_rx_flag <= 3'd0;
            jtag_trans_idcode <= 1'd0;
            jtag_trans_abort <= 1'd0;
            jtag_trans_APnDP <= 1'd0;
            jtag_trans_ir <= 1'd0;
            jtag_trans_tms_o_reg <= 1'd0;
            jtag_trans_tdi_o_reg <= 1'd0;
            rx_valid[CMD_INDEX_JTAG_TRANS] <= 1'd0;
            delay_clk_en[CMD_INDEX_JTAG_TRANS] <= 1'd0;
            clock_oen[CMD_INDEX_JTAG_TRANS] <= 1'd0;
            swj_busy[CMD_INDEX_JTAG_TRANS] <= 1'd0;
        end
        else begin
            rx_valid[CMD_INDEX_JTAG_TRANS] <= 1'd0;

            case (jtag_trans_sm)
                1'd0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_JTAG_TRANSFER && swj_busy == 8'd0) begin
                        swj_busy[CMD_INDEX_JTAG_TRANS] <= 1'd1;
                        jtag_trans_tx_data <= {tx_data[31:0], tx_cmd[3:1]};
                        jtag_trans_APnDP <= tx_cmd[0];
                        jtag_trans_ir <= tx_cmd[4];
                        jtag_trans_abort <= tx_cmd[5];
                        jtag_trans_index <= tx_cmd[8:6];
                        jtag_trans_idcode <= tx_cmd[9];
                        jtag_trans_tx_sm <= tx_cmd[4] ? 5'd0 : 5'd9; // 启用IR段从0状态开始，没有启用从9状态开始
                        jtag_trans_retry_max <= DAP_TRANS_WAIT_RETRY;
                        jtag_trans_retry_cnt <= 16'd0;
                        jtag_trans_rx_count <= 8'd0;
                        jtag_trans_sm <= 1'd1;
                        delay_clk_en[CMD_INDEX_JTAG_TRANS] <= 1'd0; // 复位延迟时钟标志
                    end
                    else begin
                        delay_clk_en[CMD_INDEX_JTAG_TRANS] <= 1'd1;
                    end
                end
                1'd1: begin
                    if (sclk_negedge) begin
                        jtag_trans_tx_sm <= jtag_trans_tx_sm + 1'd1;
                        case (jtag_trans_tx_sm)
                            5'd0: begin  // Run-Test/IDLE -> SELECT_DR_SCAN
                                clock_oen[CMD_INDEX_JTAG_TRANS] <= 1'd1;
                                jtag_trans_tdi_o_reg <= 1'd0;
                                jtag_trans_tms_o_reg <= 1'd1;
                            end
                            5'd1: begin // SELECT_DR_SCAN -> SELECT_IR_SCAN
                                jtag_trans_tms_o_reg <= 1'd1;
                            end
                            5'd2: begin // SELECT_IR_SCAN -> CAPTURE_IR
                                jtag_trans_tms_o_reg <= 1'd0;
                            end
                            5'd3: begin // CAPTURE_IR -> Shift-IR Before
                                jtag_trans_tms_o_reg <= 1'd0;
                                jtag_trans_tx_count <= JTAG_IR_BEFORE_LEN;
                                if (JTAG_IR_BEFORE_LEN == 0) begin
                                    jtag_trans_tx_count <= JTAG_IR_LEN;
                                    jtag_trans_tx_sm <= jtag_trans_tx_sm + 2;
                                end
                                /* 解码IR指令 */
                                if (jtag_trans_idcode) begin
                                    jtag_trans_tx_shift <= 8'b11111110;
                                end
                                else if (jtag_trans_abort) begin
                                    jtag_trans_tx_shift <= 8'b11111000;
                                end
                                else if (jtag_trans_APnDP) begin
                                    jtag_trans_tx_shift <= 8'b11111011;
                                end
                                else begin
                                    jtag_trans_tx_shift <= 8'b11111010;
                                end
                            end
                            5'd4: begin // Shift-IR Before
                                jtag_trans_tms_o_reg <= 1'd0;
                                jtag_trans_tdi_o_reg <= 1'd1;
                                jtag_trans_tx_count <= jtag_trans_tx_count - 1'd1;
                                if (jtag_trans_tx_count - 1'd1 != 8'd0) begin
                                    jtag_trans_tx_sm <= jtag_trans_tx_sm;
                                end
                                else begin
                                    jtag_trans_tx_count <= JTAG_IR_LEN;
                                end
                            end
                            5'd5: begin // Shift-IR
                                jtag_trans_tms_o_reg <= 1'd0;
                                {jtag_trans_tx_shift, jtag_trans_tdi_o_reg} <= {1'd0, jtag_trans_tx_shift};
                                jtag_trans_tx_count <= jtag_trans_tx_count - 1'd1;
                                if (jtag_trans_tx_count - 1'd1 != 8'd0) begin
                                    jtag_trans_tx_sm <= jtag_trans_tx_sm;
                                end
                                else begin
                                    jtag_trans_tx_count <= JTAG_IR_AFTER_LEN;
                                    if (JTAG_IR_AFTER_LEN == 0) begin
                                        jtag_trans_tms_o_reg <= 1'd1; // EXIT1-IR
                                        jtag_trans_tx_sm <= jtag_trans_tx_sm + 2;
                                    end
                                end
                            end
                            5'd6: begin // Shift-IR After
                                jtag_trans_tms_o_reg <= 1'd0;
                                jtag_trans_tdi_o_reg <= 1'd1;
                                jtag_trans_tx_count <= jtag_trans_tx_count - 1'd1;
                                if (jtag_trans_tx_count - 1'd1 != 8'd0) begin
                                    jtag_trans_tx_sm <= jtag_trans_tx_sm;
                                end
                                else begin
                                    jtag_trans_tms_o_reg <= 1'd1; // EXIT1-IR
                                end
                            end
                            5'd7: begin  // EXIT1-IR -> UPDATE-IR
                                jtag_trans_tms_o_reg <= 1'd1;
                            end
                            5'd8: begin // UPDATE-IR -> Run-Test/IDLE
                                jtag_trans_tms_o_reg <= 1'd0;
                            end
                            5'd9: begin // Run-Test/IDLE -> SELECT_DR_SCAN
                                clock_oen[CMD_INDEX_JTAG_TRANS] <= 1'd1;
                                jtag_trans_tms_o_reg <= 1'd1;
                                jtag_trans_tx_shift <= jtag_trans_tx_data;
                            end
                            5'd10: begin // SELECT_DR_SCAN -> Capture-DR
                                jtag_trans_tms_o_reg <= 1'd0;
                            end
                            5'd11: begin // Capture-DR -> Shift-DR Before
                                jtag_trans_tms_o_reg <= 1'd0;
                                jtag_trans_tx_count <= jtag_trans_index;
                                if (jtag_trans_index == 3'd0) begin
                                    if (jtag_trans_idcode) begin
                                        jtag_trans_tx_count <= 6'd32;
                                        jtag_trans_rx_count <= 6'd32;
                                    end
                                    else begin
                                        jtag_trans_tx_count <= 6'd35;
                                        jtag_trans_rx_count <= 6'd35;
                                    end
                                    jtag_trans_tx_sm <= jtag_trans_tx_sm + 2; // Shift-DR
                                end
                            end
                            5'd12: begin // Shift-DR Before
                                jtag_trans_tms_o_reg <= 1'd0;
                                jtag_trans_tdi_o_reg <= 1'd0;
                                jtag_trans_tx_count <= jtag_trans_tx_count - 1'd1;
                                if (jtag_trans_tx_count - 1'd1 != 8'd0) begin
                                    jtag_trans_tx_sm <= jtag_trans_tx_sm;
                                end
                                else begin
                                    if (jtag_trans_idcode) begin
                                        jtag_trans_tx_count <= 6'd32;
                                        jtag_trans_rx_count <= 6'd32;
                                    end
                                    else begin
                                        jtag_trans_tx_count <= 6'd35;
                                        jtag_trans_rx_count <= 6'd35;
                                    end
                                end
                            end
                            5'd13: begin // Shift-DR
                                delay_clk_en[CMD_INDEX_JTAG_TRANS] <= 1'd1;
                                jtag_trans_tms_o_reg <= 1'd0;
                                {jtag_trans_tx_shift, jtag_trans_tdi_o_reg} <= {1'd0, jtag_trans_tx_shift};
                                jtag_trans_tx_count <= jtag_trans_tx_count - 1'd1;
                                if (jtag_trans_tx_count - 1'd1 != 8'd0) begin
                                    jtag_trans_tx_sm <= jtag_trans_tx_sm;
                                end
                                else begin
                                    jtag_trans_tx_count <= JTAG_COUNT - jtag_trans_index;
                                    if (JTAG_COUNT - jtag_trans_index == 0) begin
                                        jtag_trans_tms_o_reg <= 1'd1;  // -> Exit1-DR
                                        jtag_trans_tx_sm <= jtag_trans_tx_sm + 2;
                                    end
                                end
                            end
                            5'd14: begin // Shift-DR After -> Exit1-DR
                                jtag_trans_tms_o_reg <= 1'd0;
                                jtag_trans_tdi_o_reg <= 1'd0;
                                jtag_trans_tx_count <= jtag_trans_tx_count - 1'd1;
                                if (jtag_trans_tx_count - 1'd1 != 8'd0) begin
                                    jtag_trans_tx_sm <= jtag_trans_tx_sm;
                                end
                                else begin
                                    jtag_trans_tms_o_reg <= 1'd1; // -> Exit1-DR
                                end
                            end

                            5'd15: begin // Exit1-DR -> Update-DR
                                jtag_trans_tms_o_reg <= 1'd1;
                                jtag_trans_tdi_o_reg <= 1'd0;
                            end
                            5'd16: begin // Update-DR -> Run-Test/IDLE
                                jtag_trans_tms_o_reg <= 1'd0;
                                jtag_trans_tdi_o_reg <= 1'd0;
                            end
                            5'd17: begin // Run-Test/IDLE -> FINISH
                                jtag_trans_tms_o_reg <= 1'd0;
                                jtag_trans_tdi_o_reg <= 1'd0;
                                clock_oen[CMD_INDEX_JTAG_TRANS] <= 1'd0;
                            end
                            5'd18: begin
                                jtag_trans_tx_sm <= jtag_trans_tx_sm;
                            end
                        endcase
                    end

                    if (sclk_sampling) begin
                        if (jtag_trans_rx_count != 0) begin
                            jtag_trans_rx_count <= jtag_trans_rx_count - 1'd1;
                            jtag_trans_rx_shift <= {SWO_TDO_I, jtag_trans_rx_shift[34:1]};
                        end
                    end

                    if (jtag_trans_tx_sm == 5'd18) begin
                        if ((jtag_trans_rx_ack == 3'b010) && (jtag_trans_retry_cnt != jtag_trans_retry_max) && !(jtag_trans_abort | jtag_trans_idcode)) begin
                            // WAIT 重试
                            jtag_trans_tx_sm <= 5'd9;
                            jtag_trans_retry_cnt <= jtag_trans_retry_cnt + 1'd1;
                            delay_clk_en[CMD_INDEX_JTAG_TRANS] <= 1'd0;
                        end
                        else begin
                            // OK / WAIT Timeout / ERROR / Other / Abort / IDCODE
                            if (jtag_trans_abort | jtag_trans_idcode) begin
                                jtag_trans_rx_flag <= 3'd001;
                            end
                            else begin
                                case (jtag_trans_rx_ack)
                                    3'b001, 3'b010, 3'b100:
                                        jtag_trans_rx_flag <= jtag_trans_rx_ack;
                                    default:
                                        jtag_trans_rx_flag <= 3'b111;
                                endcase
                            end

                            jtag_trans_sm <= 1'd0;
                            delay_clk_en[CMD_INDEX_JTAG_TRANS] <= 1'd1;
                            rx_valid[CMD_INDEX_JTAG_TRANS] <= 1'd1;
                            swj_busy[CMD_INDEX_JTAG_TRANS] <= 1'd0;
                        end
                    end
                end
            endcase
        end
    end

endmodule
