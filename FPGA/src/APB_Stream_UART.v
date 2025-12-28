module APB_Stream_UART#(
        parameter ADDRWIDTH = 4
    ) (
        // APB
        input PCLK,
        input PWRITE,
        input PSEL,
        input PENABLE,
        input [ADDRWIDTH-1:0] PADDR,
        input [3:0] PSTRB,
        input [31:0] PWDATA,
        output reg [31:0] PRDATA,
        output PREADY,
        input PRESETn,

        // AXIStream TX
        input tx_tvalid,
        output reg tx_tready,
        input [7:0] tx_tdata,

        // AXIStream RX
        output reg rx_tvalid,
        output reg [7:0] rx_tdata,

        // UART
        output reg UART_TX,
        input UART_RX,
        output reg UART_DE,
        output UART_RTS,
        output UART_DTR
    );


    reg [31:0] uart_cr_reg;
    reg [31:0] uart_baud_reg;

    wire UART_CR_EN = uart_cr_reg[0];         // 串口使能
    wire UART_CR_EN_PARTY = uart_cr_reg[1];   // 校验位使能
    wire UART_CR_PARTY_ODD = uart_cr_reg[2];  // 0:偶校验 1:奇校验
    wire [1:0] UART_CR_STOP_BIT = uart_cr_reg[4:3]; // 停止位长度 0: 1b; 1: 1.5b; other: 2b
    wire UART_CR_DTR = uart_cr_reg[30];
    wire UART_CR_RTS = uart_cr_reg[31];
    wire [27:0] UART_BUAD_DIV_I = uart_baud_reg[4+:28];
    wire [3:0] UART_BUAD_DIV_Q = uart_baud_reg[3:0];

    assign UART_RTS = UART_CR_EN & UART_CR_RTS;
    assign UART_DTR = UART_CR_EN & UART_CR_DTR;

    /***************************** APB接口 *****************************/

    assign PREADY = 1'd1;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            uart_cr_reg <= 32'd0;
        end
        else if (PSEL && !PENABLE) begin
            if (PWRITE) begin
                case (PADDR[ADDRWIDTH-1:2])
                    2'd0: begin
                        if (PSTRB[0])
                            uart_cr_reg[0+:8] <= PWDATA[0+:8];
                        if (PSTRB[1])
                            uart_cr_reg[8+:8] <= PWDATA[8+:8];
                        if (PSTRB[2])
                            uart_cr_reg[16+:8] <= PWDATA[16+:8];
                        if (PSTRB[3])
                            uart_cr_reg[24+:8] <= PWDATA[24+:8];
                    end
                    2'd1: begin
                        if (PSTRB[0])
                            uart_baud_reg[0+:8] <= PWDATA[0+:8];
                        if (PSTRB[1])
                            uart_baud_reg[8+:8] <= PWDATA[8+:8];
                        if (PSTRB[2])
                            uart_baud_reg[16+:8] <= PWDATA[16+:8];
                        if (PSTRB[3])
                            uart_baud_reg[24+:8] <= PWDATA[24+:8];
                    end
                endcase
            end
            else begin
                case (PADDR[ADDRWIDTH-1:2])
                    2'd0:
                        PRDATA <= uart_cr_reg;
                    2'd1:
                        PRDATA <= uart_baud_reg;
                    default:
                        PRDATA <= 32'd0;
                endcase
            end
        end
    end

    /***************************** 波特率发生器 *****************************/
    reg [27:0] uart_baud_cnt_i;
    reg [3:0] uart_baud_cnt_q;
    reg uart_baud_clk_reg;
    wire uart_baud_clk = uart_baud_clk_reg;
    wire uart_baud_clk_div4 = (uart_baud_cnt_q[1:0] == 2'd0) & uart_baud_clk;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn || !UART_CR_EN) begin
            uart_baud_cnt_i <= 28'd0;
            uart_baud_cnt_q <= 4'd0;
        end
        else begin
            if (uart_baud_cnt_i == 28'd0) begin
                uart_baud_clk_reg <= 1'd1;
                // 倒序比较均匀注入
                if ({uart_baud_cnt_q[0], uart_baud_cnt_q[1], uart_baud_cnt_q[2], uart_baud_cnt_q[3]} < UART_BUAD_DIV_Q)
                    uart_baud_cnt_i <= UART_BUAD_DIV_I + 28'd1;
                else
                    uart_baud_cnt_i <= UART_BUAD_DIV_I;
                uart_baud_cnt_q <= uart_baud_cnt_q + 4'd1;
            end
            else begin
                uart_baud_clk_reg <= 1'd0;
                uart_baud_cnt_i <= uart_baud_cnt_i - 28'd1;
            end
        end
    end

    /***************************** 串口发送 *****************************/
    reg [3:0] tx_sm;
    reg tx_data_ready;
    reg [7:0] tx_data_buf;
    reg [7:0] tx_shift_reg;
    reg tx_party_reg;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn || !UART_CR_EN) begin
            tx_sm <= 13'd0;
            tx_party_reg <= 1'd0;
            tx_shift_reg <= 8'd0;
            tx_data_ready <= 1'd0;
            tx_tready <= 1'd1;
            tx_data_buf <= 8'd0;
            UART_TX <= 1'd1;
            UART_DE <= 1'd0;
        end
        else begin
            if (!tx_data_ready) begin
                tx_tready <= 1'd1;
                if (tx_tready & tx_tvalid) begin
                    tx_tready <= 1'd0;
                    tx_data_buf <= tx_tdata;
                    tx_data_ready <= 1'd1;
                end
            end

            case (tx_sm)
                0: begin
                    if (tx_data_ready && uart_baud_clk_div4) begin
                        tx_party_reg <= 1'd0;
                        tx_shift_reg <= tx_data_buf;
                        tx_data_ready <= 1'd0;
                        UART_TX <= 1'd0;
                        UART_DE <= 1'd1;
                        tx_sm <= 4'd1;
                    end
                end
                1,2,3,4,5,6,7,8: begin
                    if (uart_baud_clk_div4) begin
                        // 已发送起始位
                        UART_TX <= tx_shift_reg[0];
                        tx_party_reg <= tx_party_reg ^ tx_shift_reg[0];
                        tx_shift_reg <= tx_shift_reg >> 1;
                        tx_sm <= tx_sm + 1'd1;
                    end
                end
                9: begin
                    if (uart_baud_clk_div4) begin
                        // 已发送数据
                        if (UART_CR_EN_PARTY) begin
                            // 支持校验 发送校验位转到发送停止位状态
                            UART_TX <= UART_CR_PARTY_ODD ? ~tx_party_reg : tx_party_reg;
                            tx_sm <= 4'd10;
                        end
                        else begin
                            // 不支持校验 发送停止位转到停止位延迟状态
                            UART_TX <= 1'b1;
                            tx_sm <= 4'd11;
                        end
                    end
                end
                10: begin
                    if (uart_baud_clk_div4) begin
                        // 已发送校验位
                        // 发送停止位转到停止位延迟状态
                        UART_TX <= 1'b1;
                        tx_sm <= 4'd11;
                    end
                end
                11: begin
                    if (uart_baud_clk_div4) begin
                        // 已发送1b停止位时间
                        if (UART_CR_STOP_BIT == 2'd0) begin
                            if (tx_data_ready) begin
                                tx_data_ready <= 1'd0;
                                tx_party_reg <= 1'd0;
                                tx_shift_reg <= tx_data_buf;
                                UART_TX <= 1'd0;
                                UART_DE <= 1'd1;
                                tx_sm <= 4'd1;
                            end
                            else begin
                                UART_DE <= 1'd0;
                                tx_sm <= 4'd0;
                            end
                        end
                        else begin
                            tx_sm <= 4'd12;
                        end
                    end
                end
                12: begin
                    if (uart_baud_clk)
                        tx_sm <= 4'd13;
                end
                13: begin
                    if (uart_baud_clk) begin
                        // 已发送1.5b停止位时间
                        if (UART_CR_STOP_BIT == 2'd1) begin
                            if (tx_data_ready) begin
                                tx_data_ready <= 1'd0;
                                tx_party_reg <= 1'd0;
                                tx_shift_reg <= tx_data_buf;
                                UART_TX <= 1'd0;
                                UART_DE <= 1'd1;
                                tx_sm <= 4'd1;
                            end
                            else begin
                                UART_DE <= 1'd0;
                                tx_sm <= 4'd0;
                            end
                        end
                        else begin
                            tx_sm <= 4'd14;
                        end
                    end
                end
                14: begin
                    if (uart_baud_clk)
                        tx_sm <= 4'd15;
                end
                15: begin
                    if (uart_baud_clk) begin
                        // 已发送2b停止位时间
                        if (tx_data_ready) begin
                            tx_data_ready <= 1'd0;
                            tx_party_reg <= 1'd0;
                            tx_shift_reg <= tx_data_buf;
                            UART_TX <= 1'd0;
                            UART_DE <= 1'd1;
                            tx_sm <= 4'd1;
                        end
                        else begin
                            UART_DE <= 1'd0;
                            tx_sm <= 4'd0;
                        end
                    end
                end
            endcase
        end
    end


    /***************************** 串口接收 *****************************/

    reg uart_rx0;
    reg uart_rx1;
    wire uart_rx = uart_rx1;
    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn || !UART_CR_EN) begin
            uart_rx0 <= 1'd1;
            uart_rx1 <= 1'd1;
        end
        else begin
            uart_rx0 <= UART_RX;
            uart_rx1 <= uart_rx0;
        end
    end

    reg [3:0] uart_oversimple_shift;
    reg [1:0] uart_oversimple_cnt;
    wire [2:0] uart_oversimple_add = uart_oversimple_shift[0] + uart_oversimple_shift[1] + uart_oversimple_shift[2] + uart_oversimple_shift[3];
    wire uart_oversimple_result = uart_oversimple_add > 3'd2;

    reg rx_find_start;
    reg [3:0] rx_sm;
    reg [7:0] rx_data_shift;
    reg rx_data_valid;
    wire rx_data_party = (rx_data_shift[0] ^ rx_data_shift[1] ^ rx_data_shift[2] ^ rx_data_shift[3] ^ rx_data_shift[4] ^ rx_data_shift[5] ^ rx_data_shift[6] ^ rx_data_shift[7]);

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn || !UART_CR_EN) begin
            uart_oversimple_shift <= 4'hf;
            uart_oversimple_cnt <= 2'd0;
            rx_data_valid <= 1'd0;
            rx_data_shift <= 8'd0;
            rx_sm <= 4'd0;
            rx_tdata <= 8'd0;
            rx_tvalid <= 1'd0;
            rx_find_start <= 1'd0;
        end
        else begin
            rx_tvalid <= 1'd0;
            if (uart_baud_clk) begin
                uart_oversimple_shift <= {uart_oversimple_shift[2:0], uart_rx};
                if (rx_sm == 4'd0 && rx_find_start == 1'd0)
                    uart_oversimple_cnt <= 2'd0;
                else
                    uart_oversimple_cnt <= uart_oversimple_cnt + 2'd1;

                case (rx_sm)
                    0:
                        if (rx_find_start == 1'd0 && uart_rx == 1'd0) begin
                            // 找到起始位
                            uart_oversimple_cnt <= 2'd1;
                            rx_find_start <= 1'd1;
                        end else if (uart_oversimple_cnt == 2'b11) begin
                            if (uart_oversimple_result == 0)
                                rx_sm <= 1'd1;
                            rx_find_start <= 1'd0;
                            uart_oversimple_cnt <= 2'd0;
                        end
                    1,2,3,4,5,6,7,8:
                        if (uart_oversimple_cnt == 2'b11) begin
                            // 接收数据位
                            rx_data_shift <= {uart_oversimple_result, rx_data_shift[7:1]};
                            rx_sm <= rx_sm + 4'd1;
                        end
                    9:
                        if (uart_oversimple_cnt == 2'b11) begin
                            // 接收停止位/
                            if (UART_CR_EN_PARTY) begin
                                // 校验位
                                rx_data_valid <= (rx_data_party ^ uart_oversimple_result == UART_CR_PARTY_ODD);
                                rx_sm <= 10;
                            end
                            else begin
                                rx_data_valid <= 1'd1;
                                // 停止位
                                if (uart_oversimple_result) begin
                                    // 正确接收1b时间
                                    if (UART_CR_STOP_BIT == 2'd0) begin
                                        rx_tdata <= rx_data_shift;
                                        rx_tvalid <= 1'd1;
                                        rx_sm <= 0;
                                    end
                                    else begin
                                        rx_sm <= 11;
                                    end
                                end
                                else begin
                                    // 错误
                                    rx_sm <= 0;
                                end
                            end
                        end
                    10:
                        if (uart_oversimple_cnt == 2'b11) begin
                            if (UART_CR_STOP_BIT == 2'd0) begin
                                if (uart_oversimple_shift[0] | uart_oversimple_shift[1]) begin
                                    // 正确接收1b时间
                                    rx_tdata <= rx_data_shift;
                                    rx_tvalid <= rx_data_valid;
                                    rx_sm <= 0;
                                end else begin
                                    rx_sm <= 11;
                                end
                            end
                            else begin
                                rx_sm <= 11;
                            end
                        end
                    11:
                        if (uart_oversimple_cnt == 2'b01) begin
                            if (UART_CR_STOP_BIT == 2'd1) begin
                                if (uart_oversimple_shift[0] | uart_oversimple_shift[1]) begin
                                    // 正确接收1.5b时间
                                    rx_tdata <= rx_data_shift;
                                    rx_tvalid <= rx_data_valid;
                                end
                                rx_sm <= 0;
                            end
                            else begin
                                rx_sm <= 12;
                            end
                        end
                    12:
                        if (uart_oversimple_cnt == 2'b11) begin
                            if (uart_oversimple_shift[0] | uart_oversimple_shift[1]) begin
                                // 正确接收2b时间
                                rx_tdata <= rx_data_shift;
                                rx_tvalid <= rx_data_valid;
                            end
                            rx_sm <= 0;
                        end
                endcase
            end
        end
    end

endmodule

