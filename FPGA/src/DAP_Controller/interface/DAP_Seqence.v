`include "DAP_Cmd.v"

module DAP_Seqence (
        // 控制器时钟
        input clk,
        input resetn,

        // 串行时钟
        input sclk,
        input sclk_out,
        input sclk_pulse,
        input sclk_delay_pulse,

        // 控制器输入输出
        input seq_tx_valid,
        input [15:0] seq_tx_cmd,
        input [63:0] seq_tx_data,
        output seq_tx_full,

        output seq_rx_valid,
        output [15:0] seq_rx_flag,
        output [63:0] seq_rx_data,


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
    assign SWCLK_TCK_O = clock_oen ? ~sclk_out : clock_idle;

    reg [31:0] MATCH_MASK;
    reg swj_busy;

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

    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            rx_valid <= 0;
            rx_valid2 <= 0;
            tx_shift_reg <= 64'd0;
            clock_oen <= 1'd0;
            clock_idle <= 1'd1;
            rx_flag <= 0;
            tx_nxt <= 1'd0;
            rx_data <= 64'd0;
            SWDIO_TMS_T <= 1'd1;
            SWDIO_TMS_O <= 1'd0;
            MATCH_MASK <= 32'd0;
            swj_busy <= 1'd0;

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
        end
        else begin
            rx_valid2 <= rx_valid;
            if (rx_valid2)
                rx_valid <= 1'd0;

            // SEQ_CMD_SWD_SEQ
            case (swd_seq_sm)
                1'd0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWD_SEQ && swj_busy == 0) begin
                        tx_nxt <= 1'd1;
                        tx_shift_reg <= tx_data;
                        swd_seq_cmd <= tx_cmd;
                        SWDIO_TMS_T <= tx_cmd[7];
                        swd_seq_tx_count <= tx_cmd[6:0];
                        swd_seq_rx_count <= tx_cmd[6:0];
                        swd_seq_sm <= 2'd1;
                    end
                end
                1'd1: begin
                    if (sclk_pulse) begin
                        if (swd_seq_tx_count) begin
                            clock_oen <= 1'd1;
                            swd_seq_tx_count <= swd_seq_tx_count - 7'd1;
                            {tx_shift_reg[62:0], SWDIO_TMS_O} <= tx_shift_reg; // 移位输出
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

                    if (sclk_delay_pulse && swd_seq_rx_count) begin
                        swd_seq_rx_count <= swd_seq_rx_count - 7'd1;
                        rx_shift_reg <= {rx_shift_reg[62:0], SWDIO_TMS_I};
                    end
                end
            endcase

            // SEQ_CMD_SWJ_PINS
            case (swj_pin_sm)
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
                        swj_us_cnt == 32'd0 ||
                        (swj_pin_select_reg & (swj_pin_read ^ swj_pin_output_reg)) == 8'd0
                    ) begin
                        rx_data <= {56'd0, swj_pin_read};
                        rx_flag <= 16'd0;
                        rx_valid <= 1'd1;
                        swj_pin_sm <= 1'd0;
                        swj_busy <= 1'd0;
                    end
                end
            endcase

        end
    end

endmodule
