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

        // 控制器输入输出
        input seq_tx_valid,
        input [15:0] seq_tx_cmd,
        input [63:0] seq_tx_data,
        output seq_tx_full,

        output seq_rx_valid,
        output [15:0] seq_rx_flag,
        output [63:0] seq_rx_data,

        input [15:0] DAP_TRANS_WAIT_RETRY,
        input [11:0] SWD_TURN_CYCLE,


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

    reg [15:0] dap_trans_wait_retry_ff;
    reg [11:0] swd_turn_cycle_ff;
    // reg [15:0] dap_trans_wait_retry;
    // reg [7:0] swd_turn_cycle;

    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            dap_trans_wait_retry_ff <= 0;
            swd_turn_cycle_ff <= 0;
            // dap_trans_wait_retry <= 0;
            // swd_turn_cycle <= 0;
        end
        else begin
            dap_trans_wait_retry_ff <= DAP_TRANS_WAIT_RETRY;
            swd_turn_cycle_ff <= SWD_TURN_CYCLE;
            // dap_trans_wait_retry <= dap_trans_wait_retry_ff;
            // swd_turn_cycle <= swd_turn_cycle_ff;
        end
    end


    reg [63:0] tx_shift_reg;
    reg [63:0] rx_shift_reg;

    reg rx_valid;
    reg rx_valid2;
    reg [15:0] rx_flag;
    reg [63:0] rx_data;


    reg tx_valid_ff1;
    reg tx_valid_ff2;
    reg tx_valid;
    reg tx_nxt;
    reg [15:0] tx_cmd;
    reg [63:0] tx_data;
    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            tx_valid_ff1 <= 1'd0;
            tx_valid_ff2 <= 1'd0;
            tx_cmd <= 16'd0;
            tx_data <= 64'd0;
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

    wire [3:0] current_cmd = tx_cmd[15:12];

    reg clock_oen;
    reg clock_idle;
    assign SWCLK_TCK_O = clock_oen ? sclk_out : clock_idle;

    reg swj_busy;
    reg delay_clk_en;

    reg [1:0] swj_seq_sm;
    reg [7:0] swj_seq_count;

    reg swd_seq_sm;
    reg [7:0] swd_seq_cmd;
    reg [6:0] swd_seq_tx_count;
    reg [6:0] swd_seq_rx_count;

    reg swj_pin_sm;
    reg [7:0] swj_pin_select_reg;
    reg [7:0] swj_pin_output_reg;
    reg [31:0] swj_us_cnt;
    reg [7:0] swj_tick_cnt;
    wire [7:0] swj_pin_output = seq_tx_data[7:0];
    wire [7:0] swj_pin_select = seq_tx_data[15:8];
    wire [31:0] swj_pin_delay = seq_tx_data[47:16];
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
    reg [11:0] swd_trans_turn_cycle;
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

    wire sclk_sampling_en = (delay_clk_en || sclk_negedge) && sclk_sampling;

    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            rx_valid <= 0;
            rx_valid2 <= 0;
            rx_shift_reg <= 64'd0;
            tx_shift_reg <= 64'd0;
            clock_oen <= 1'd0;
            clock_idle <= 1'd1;
            rx_flag <= 16'd0;
            tx_nxt <= 1'd0;
            rx_data <= 64'd0;
            SWDIO_TMS_T <= 1'd0;
            SWDIO_TMS_O <= 1'd0;
            TDI_O <= 1'd1;
            SRST_O <= 1'd1;
            TRST_O <= 1'd1;
            swj_busy <= 1'd0;
            delay_clk_en <= 1'd0;

            swj_seq_sm <= 0;
            swj_seq_count <= 0;

            swd_seq_sm <= 1'd0;
            swd_seq_cmd <= 8'd0;
            swd_seq_tx_count <= 7'd0;
            swd_seq_rx_count <= 7'd0;

            swj_pin_select_reg <= 8'd0;
            swj_pin_output_reg <= 8'd0;
            swj_pin_sm <= 1'd0;
            swj_us_cnt <= 32'd0;
            swj_tick_cnt <= 8'd0;


            swd_trans_sm <= 2'd0;
            swd_trans_turn_cycle <= 12'd0;
            swd_trans_retry_max <= 16'd0;
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
            rx_valid2 <= rx_valid;
            if (rx_valid2)
                rx_valid <= 1'd0;

            if (sclk_negedge)
                delay_clk_en <= 1'd1;

            // SEQ_CMD_SWD_SEQ
            case (swd_seq_sm)
                1'd0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWD_SEQ && swj_busy == 0) begin
                        tx_nxt <= 1'd1;
                        tx_shift_reg <= tx_data;
                        // tx_shift_reg[32] <= 0;
                        swd_seq_cmd <= tx_cmd;
                        SWDIO_TMS_T <= tx_cmd[7];
                        swd_seq_tx_count <= tx_cmd[6:0];
                        swd_seq_rx_count <= tx_cmd[6:0];
                        swd_seq_sm <= 2'd1;
                        delay_clk_en <= 0;
                    end
                end
                1'd1: begin
                    if (sclk_negedge) begin
                        if (swd_seq_tx_count) begin
                            clock_oen <= 1'd1;
                            swd_seq_tx_count <= swd_seq_tx_count - 7'd1;
                            {tx_shift_reg, SWDIO_TMS_O} <= {1'd0, tx_shift_reg}; // 移位输出
                        end
                        else begin
                            clock_oen <= 1'd0; // 关闭时钟输出
                            SWDIO_TMS_T <= 1'd1;
                            if (swd_seq_rx_count == 7'd0) begin
                                rx_data <= rx_shift_reg;
                                rx_flag <= swd_seq_cmd;
                                rx_valid <= 1'd1;
                                swd_seq_sm <= 1'd0;
                                swj_busy <= 1'd0;
                            end
                        end
                    end

                    if (sclk_sampling_en && swd_seq_rx_count) begin
                        swd_seq_rx_count <= swd_seq_rx_count - 7'd1;
                        rx_shift_reg <= {SWDIO_TMS_I, rx_shift_reg[63:1]};
                        // rx_shift_reg[swd_seq_rx_count] <= SWDIO_TMS_I;
                    end
                end
            endcase

            // SEQ_CMD_SWJ_PINS
            case (swj_pin_sm) /*synthesis parallel_case*/
                0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWJ_PINS && swj_busy == 0) begin
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
                        swj_busy <= 1'd1;
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
                        rx_data <= {56'd0, swj_pin_read};
                        rx_flag <= 16'd0;
                        rx_valid <= 1'd1;
                        swj_pin_sm <= 1'd0;
                        swj_busy <= 1'd0;
                    end
                end
            endcase

            // SEQ_CMD_SWD_TRANSFER
            case (swd_trans_sm) /*synthesis parallel_case*/
                SWD_TRANS_SM_IDLE: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWD_TRANSFER && swj_busy == 0) begin
                        tx_nxt <= 1'd1;
                        swj_busy <= 1'd1;

                        // 装载数据
                        swd_trans_tx_data <= tx_data[31:0];
                        // 装载配置信息
                        swd_trans_retry_cnt <= 16'd0;
                        swd_trans_retry_max <= dap_trans_wait_retry_ff;
                        swd_trans_turn_cycle <= swd_turn_cycle_ff;
                        // 装载请求头
                        swd_trans_tx_APnDP <= tx_cmd[0];
                        swd_trans_tx_RnW <= tx_cmd[1];
                        swd_trans_tx_ADDR <= tx_cmd[3:2];

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
                                delay_clk_en <= 0;
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
                                if (swd_trans_tx_cnt == swd_trans_turn_cycle) begin
                                    swd_trans_tx_cnt <= 12'd0;
                                end
                                else begin
                                    swd_trans_tx_cnt <= swd_trans_tx_cnt + 1'd1;
                                    swd_trans_tx_sm <= SWD_TRANS_IO_TURN1;
                                end
                            end
                            // SWD_TRANS_IO_ACK0: // Nothing
                            // SWD_TRANS_IO_ACK1: // Nothing
                            // SWD_TRANS_IO_ACK2: // Nothing
                            SWD_TRANS_IO_TURN2: begin
                                // 无论读写都进入SWD_TRANS_IO_TURN2
                                // 读模式下用于等待ACK结果，ACK失败生成TURN数量时钟后退出，ACK成功生成TURN段之后进入数据段生成全部时钟
                                if (swd_trans_tx_cnt == swd_trans_turn_cycle) begin
                                    swd_trans_tx_cnt <= 12'd0;

                                    // 判断RX ACK状态
                                    if (swd_trans_rx_ack == 3'd001) begin // OK
                                        swd_trans_tx_sm <= SWD_TRANS_IO_DATA;
                                    end
                                    else begin // Wait / Error / Other
                                        swd_trans_tx_sm <= SWD_TRANS_IO_DONE;
                                    end
                                end
                                else begin
                                    swd_trans_tx_cnt <= swd_trans_tx_cnt + 1'd1;
                                    swd_trans_tx_sm <= SWD_TRANS_IO_TURN2;
                                end
                            end
                            SWD_TRANS_IO_DATA: begin
                                {swd_trans_tx_data, SWDIO_TMS_O} <= {1'd0, swd_trans_tx_data}; // 移位输出
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
                                SWDIO_TMS_O <= swd_trans_tx_parity;
                            end
                            SWD_TRANS_IO_DONE: begin
                                swd_trans_tx_sm <= SWD_TRANS_IO_DONE;
                                clock_oen <= 1'd0;
                                SWDIO_TMS_O <= 1'd0;
                                SWDIO_TMS_T <= 1'd0;
                            end
                        endcase
                    end

                    if (sclk_sampling_en) begin
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
                                if ({SWDIO_TMS_I, swd_trans_rx_ack[1:0]} == 3'b001) begin
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

                    if (sclk_negedge) begin
                        if (swd_trans_tx_RnW) begin
                            // 读模式下rx结束但ACK错误结束
                            if (swd_trans_rx_sm == SWD_TRANS_IO_DONE && swd_trans_rx_ack != 3'b001) begin
                                swd_trans_sm <= SWD_TRANS_SM_CHECK;
                            end
                        end
                        else begin
                            // 写模式下tx结束但ACK错误结束
                            if (swd_trans_tx_sm == SWD_TRANS_IO_DONE && swd_trans_rx_ack != 3'b001) begin
                                swd_trans_sm <= SWD_TRANS_SM_CHECK;
                            end
                        end

                        if (swd_trans_tx_sm == SWD_TRANS_IO_DONE && swd_trans_rx_sm == SWD_TRANS_IO_DONE) begin
                            swd_trans_sm <= SWD_TRANS_SM_CHECK;
                        end
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

                        rx_data <= {32'd0, swd_trans_rx_data};
                        case (swd_trans_rx_ack)
                            3'b001, 3'b010, 3'b100:
                                rx_flag <= {13'd0, swd_trans_rx_ack};
                            default:
                                rx_flag <= {13'd0, 3'b111};
                        endcase
                        rx_valid <= 1'd1;
                        swj_busy <= 1'd0;
                    end
                end
            endcase


        end
    end

endmodule
