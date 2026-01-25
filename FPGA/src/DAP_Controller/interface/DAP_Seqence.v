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

    reg rx_valid;
    reg rx_valid2;
    reg [15:0] rx_flag;
    reg [63:0] rx_data;


    reg tx_valid_ff1;
    reg tx_valid_ff2;
    reg tx_valid;
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
            else begin
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
    assign SWCLK_TCK_O = clock_oen ? ~sclk_out : 1'd0;

    reg [31:0] MATCH_MASK;

    reg [3:0] swj_seq_sm;
    reg [7:0] swj_seq_count;
    reg swj_busy;

    always @(posedge sclk or negedge resetn) begin
        if (!resetn) begin
            swj_seq_sm <= 0;
            rx_valid <= 0;
            rx_valid2 <= 0;
            swj_seq_count <= 0;
            tx_shift_reg <= 64'd0;
            clock_oen <= 0;
            rx_flag <= 0;
            rx_data <= 64'd0;
            swj_busy <= 1'd0;
            SWDIO_TMS_T <= 1'd1;
            SWDIO_TMS_O <= 1'd0;
            MATCH_MASK <= 32'd0;
        end
        else begin
            rx_valid2 <= rx_valid;
            if (rx_valid2)
                rx_valid <= 1'd0;

            case (swj_seq_sm)
                0: begin
                    if (tx_valid && current_cmd == `SEQ_CMD_SWJ_SEQ && swj_busy == 0) begin
                        swj_seq_count <= tx_cmd[7:0];
                        tx_shift_reg <= tx_data;
                        swj_busy <= 1'd1;
                        SWDIO_TMS_T <= 1'd0; // TODO 是否需要turn周期
                        swj_seq_sm <= 2;
                    end
                end
                1: begin
                    if (sclk_pulse) begin
                        swj_seq_sm <= 1'd2;
                    end
                end
                2: begin
                    if (sclk_pulse) begin
                        if (swj_seq_count == 0) begin
                            clock_oen <= 1'd0; // 关闭时钟输出
                            rx_flag <= 16'd0;
                            rx_data <= 64'd0;
                            rx_valid <= 1'd1; // 反馈
                            swj_seq_sm <= 2'd0; // 复位状态机
                            swj_busy <= 1'd0;
                        end
                        else begin
                            clock_oen <= 1'd1;   // 开启时钟输出
                            {tx_shift_reg[62:0], SWDIO_TMS_O} <= tx_shift_reg; // 移位输出
                            swj_seq_count <= swj_seq_count - 1; // 计数递减
                        end
                    end
                end
            endcase

        end
    end

endmodule
