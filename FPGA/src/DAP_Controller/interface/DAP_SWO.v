`timescale 1 ns / 10 ps

module DAP_SWO #(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    )(
        input clk,
        input clk_200M,
        input resetn,

        // AHB MEM接口
        input ahb_write_en,
        input ahb_read_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        input [1:0] LOC_SWO_DDR_Q,
        
        input [13:0] fifo_size,
        output swo_active,

        output reg [7:0] swo_byte,
        output reg       swo_valid,
        output wire      swo_fifo_clear,      // FIFO 复位请求（连接到 DAP_USB_Transfer.clear_data）
        input  wire      swo_fifo_clear_done  // FIFO 复位完成（来自 DAP_USB_Transfer.clear_done）
    );

    localparam [ADDRWIDTH-1:0] SWO_CTRL_ADDR           = BASE_ADDR + 0;  // {BIT_TIME[15:0], CR[15:8]=EDGE_DEC, CR[7:0]=CTRL}
    localparam [ADDRWIDTH-1:0] SWO_BIT_TIME_ADDR       = BASE_ADDR + 4;  // {BIT_TIME[15:0], CR[15:8]=EDGE_DEC, CR[7:0]=CTRL}
    localparam [ADDRWIDTH-1:0] SWO_PARAM_ADDR          = BASE_ADDR + 8;  // {BIT_DEC_LOW[15:0], BIT_DEC_HIGH[15:0]}

    reg [15:0] SWO_CR;                  // [7:0]=CTRL, [15:8]=EDGE_DECISION
    reg [11:0] SWO_BIT_TIME_0;
    reg [11:0] SWO_BIT_TIME_1;
    reg [11:0] SWO_BIT_DECISION_HIGH;
    reg [11:0] SWO_BIT_DECISION_LOW;


    wire SWO_EN      = SWO_CR[0];        // 使能标志位
    wire SWO_MODE    = SWO_CR[1];        // 0: UART模式, 1: 曼彻斯特模式
    wire SWO_FIFO_RESET  = SWO_CR[2];    // FIFO 复位标志（写1复位，写0清除，读获取状态）
    wire [3:0] SWO_JITTER = SWO_CR[7:4]; // 4bit 抖动比较值
    wire [3:0] SWO_EDGE_DECISION = SWO_CR[11:8]; // 4bit 边沿判决时间

    assign swo_fifo_clear = SWO_FIFO_RESET;
    assign swo_active = SWO_EN;

    // AHB 写操作
    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            SWO_CR <= 16'd0;
            SWO_BIT_TIME_0 <= 16'd0;
            SWO_BIT_TIME_1 <= 16'd0;
            SWO_BIT_DECISION_HIGH <= 16'd0;
            SWO_BIT_DECISION_LOW <= 16'd0;
        end
        else begin
            if (ahb_write_en) begin
                case (ahb_addr[ADDRWIDTH-1:2]) /*synthesis parallel_case*/
                    SWO_CTRL_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0]) begin
                            if (SWO_EN) begin
                                SWO_CR[0] <= ahb_wdata[0];
                            end
                            else begin
                                SWO_CR[7:0] <= ahb_wdata[7:0];
                            end
                        end
                        if (ahb_byte_strobe[1] && !SWO_EN)
                            SWO_CR[15:8] <= ahb_wdata[15:8];
                    end
                    SWO_BIT_TIME_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0] && !SWO_EN)
                            SWO_BIT_TIME_0[7:0] <= ahb_wdata[7:0];
                        if (ahb_byte_strobe[1] && !SWO_EN)
                            SWO_BIT_TIME_0[11:8] <= ahb_wdata[11:8];
                        if (ahb_byte_strobe[2] && !SWO_EN)
                            SWO_BIT_TIME_1[7:0] <= ahb_wdata[23:16];
                        if (ahb_byte_strobe[3] && !SWO_EN)
                            SWO_BIT_TIME_1[11:8] <= ahb_wdata[27:24];
                    end
                    SWO_PARAM_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0] && !SWO_EN)
                            SWO_BIT_DECISION_HIGH[7:0] <= ahb_wdata[7:0];
                        if (ahb_byte_strobe[1] && !SWO_EN)
                            SWO_BIT_DECISION_HIGH[11:8] <= ahb_wdata[11:8];
                        if (ahb_byte_strobe[2] && !SWO_EN)
                            SWO_BIT_DECISION_LOW[7:0] <= ahb_wdata[23:16];
                        if (ahb_byte_strobe[3] && !SWO_EN)
                            SWO_BIT_DECISION_LOW[11:8] <= ahb_wdata[27:24];
                    end
                endcase
            end
        end
    end

    // AHB 读操作
    always @(*) begin : ahb_mem_read_ctrl
        if (ahb_read_en) begin
            case (ahb_addr[ADDRWIDTH-1:2]) /*synthesis parallel_case*/
                SWO_CTRL_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {3'd0, fifo_size, SWO_CR[15:4], 1'd0, swo_fifo_clear_done, SWO_CR[1:0]};
                SWO_BIT_TIME_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {4'd0, SWO_BIT_TIME_1, 4'd0, SWO_BIT_TIME_0};
                SWO_PARAM_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {4'd0, SWO_BIT_DECISION_LOW, 4'd0, SWO_BIT_DECISION_HIGH};
                default:
                    ahb_rdata = {32{1'bx}};
            endcase
        end
        else begin
            ahb_rdata = {32{1'bx}};
        end
    end

    reg swo_en_ff1;
    reg swo_en_ff2;

    wire resetn_200M = swo_en_ff2;
    always @(posedge clk_200M or negedge resetn) begin
        if (!resetn) begin
            swo_en_ff1 <= 1'd0;
            swo_en_ff2 <= 1'd0;

        end
        else begin
            swo_en_ff1 <= SWO_EN;
            swo_en_ff2 <= swo_en_ff1;

        end
    end


    // ========================================================================
    // 240M时钟域 - 边沿检测状态机
    // IDDR采样: Q0=上升沿, Q1=下降沿, {Q1,Q0}四种组合
    // ========================================================================
    localparam ST_LOW  = 1'd0;  // 确认低电平，等待上升沿
    localparam ST_HIGH = 1'd1;  // 确认高电平，等待下降沿


    reg [3:0] edge_cnt;
    reg swo_edge;                // 边沿脉冲
    reg swo_init;
    reg swo_state;               // 当前状态
    wire swo_rising = swo_state; // 边沿方向: 1=上升沿, 0=下降沿

    always @(posedge clk_200M) begin : edge_detect_fsm
        if (!resetn_200M) begin
            edge_cnt   <= 4'd0;
            swo_edge   <= 1'd0;
            swo_state  <= ST_LOW;
            swo_init <= 1'd0;
        end
        else begin
            swo_edge   <= 1'd0;

            if (!swo_init) begin
                // 根据模式同步当前电平状态，等待模式匹配的边沿触发状态机
                // 串口模式默认高电平，曼彻斯特模式默认低电平
                if (LOC_SWO_DDR_Q == {~SWO_MODE, ~SWO_MODE}) begin
                    swo_state <= (SWO_MODE == 1'b0) ? ST_HIGH : ST_LOW;
                    swo_init <= 1'd1;
                end
            end
            else begin
                case (swo_state)
                    ST_LOW: begin
                        case (LOC_SWO_DDR_Q)
                            2'b00:
                                edge_cnt <= (edge_cnt > 4'd1) ? edge_cnt - 4'd2 : 4'd0;
                            2'b11:
                                edge_cnt <= edge_cnt + 4'd2;
                            2'b10:
                                edge_cnt <= (edge_cnt == 4'd0) ? 4'd1 : edge_cnt;
                            2'b01:
                                edge_cnt <= (edge_cnt == 4'd0) ? 4'd0 : edge_cnt - 4'd1;
                        endcase
                        if (edge_cnt > SWO_EDGE_DECISION) begin
                            swo_edge <= 1'd1;
                            swo_state <= ST_HIGH;
                            edge_cnt <= 4'd0;
                        end
                    end
    
                    ST_HIGH: begin
                        case (~LOC_SWO_DDR_Q)
                            2'b00:
                                edge_cnt <= (edge_cnt > 4'd1) ? edge_cnt - 4'd2 : 4'd0;
                            2'b11:
                                edge_cnt <= edge_cnt + 4'd2;
                            2'b10:
                                edge_cnt <= (edge_cnt == 4'd0) ? 4'd1 : edge_cnt;
                            2'b01:
                                edge_cnt <= (edge_cnt == 4'd0) ? 4'd0 : edge_cnt - 4'd1;
                        endcase
                        if (edge_cnt > SWO_EDGE_DECISION) begin
                            swo_edge   <= 1'd1;
                            swo_state  <= ST_LOW;
                            edge_cnt   <= 4'd0;
                        end
                    end
                endcase
            end
        end
    end


    // ========================================================================
    // 240M时钟域 - 比特判定模块
    // 每周期累加高低电平数量，边沿脉冲来时清零
    // ========================================================================
    localparam [1:0] BIT_SM_IDLE = 2'd0;
    localparam [1:0] BIT_SM_BIT_RANGE1 = 2'd1;
    localparam [1:0] BIT_SM_BIT_RANGE2 = 2'd3;


    reg [11:0] high_sample_cnt; // 高电平采样计数
    reg [11:0] low_sample_cnt;  // 低电平采样计数
    reg [11:0] time_sample_cnt; // 时间采样计数（每周期+1）
    reg [11:0] bit_time; // 当前bit时间
    reg [3:0] jitter_cnt; // 抖动注入计数
    reg        bit_valid;       // bit判决有效
    reg        bit_level;       // bit判决电平
    reg [1:0]  bit_sm;
    reg [1:0] swo_input_ff1;
    reg [1:0] swo_input_bitcount_h;
    reg [1:0] swo_input_bitcount_l;

    wire [3:0] jitter_cnt_inv = {jitter_cnt[0], jitter_cnt[1], jitter_cnt[2], jitter_cnt[3]};

    wire decoder_wait_stop;

    always @(posedge clk_200M) begin : sample_counter
        if (!resetn_200M) begin
            bit_sm <= BIT_SM_IDLE;
            jitter_cnt <= 4'd0;
            high_sample_cnt <= 12'd0;
            low_sample_cnt  <= 12'd0;
            time_sample_cnt <= 12'd0;
            bit_valid       <= 1'd0;
            bit_level       <= 1'd0;
            bit_time <= 16'd0;
            swo_input_ff1 <= 2'd0;
            swo_input_bitcount_h <= 2'd0;
            swo_input_bitcount_l <= 2'd0;
        end
        else begin
            swo_input_ff1 <= LOC_SWO_DDR_Q;
            swo_input_bitcount_h <= bitcount_h(swo_input_ff1);
            swo_input_bitcount_l <= bitcount_l(swo_input_ff1);
            bit_time <= (jitter_cnt_inv < SWO_JITTER) ? SWO_BIT_TIME_1 : SWO_BIT_TIME_0;

            case (bit_sm)
                BIT_SM_IDLE: begin
                    // 触发边沿要和模式相匹配
                    if (swo_edge && swo_rising == SWO_MODE) begin
                        bit_sm <= BIT_SM_BIT_RANGE1;
                        high_sample_cnt <= swo_input_bitcount_h;
                        low_sample_cnt  <= swo_input_bitcount_l;
                        time_sample_cnt <= 12'd0;
                    end
                end
                BIT_SM_BIT_RANGE1: begin
                    high_sample_cnt <= high_sample_cnt + swo_input_bitcount_h;
                    low_sample_cnt <= low_sample_cnt + swo_input_bitcount_l;
                    time_sample_cnt <= time_sample_cnt + 2'd1;

                    if (time_sample_cnt == SWO_BIT_DECISION_LOW) begin
                        // 时间超过下阈值进入第二阶段
                        bit_sm <= BIT_SM_BIT_RANGE2;
                    end
                end
                BIT_SM_BIT_RANGE2: begin
                    if (swo_edge) begin
                        // 阈值时间内触发边沿，正常触发bit
                        time_sample_cnt <= 12'd0;
                        high_sample_cnt <= swo_input_bitcount_h;
                        low_sample_cnt  <= swo_input_bitcount_l;
                        bit_valid <= 1'd1;
                        bit_level <= (high_sample_cnt > low_sample_cnt) ? 1'd1 : 1'd0;
                        bit_sm <= BIT_SM_BIT_RANGE1;
                    end
                    else if (time_sample_cnt == SWO_BIT_DECISION_HIGH) begin
                        // bit超时，进入下一个bit，计时器减去1bit时间，高低电平计数清零
                        time_sample_cnt <= time_sample_cnt - bit_time;
                        high_sample_cnt <= swo_input_bitcount_h;
                        low_sample_cnt  <= swo_input_bitcount_l;
                        bit_valid <= 1'd1;
                        bit_level <= (high_sample_cnt > low_sample_cnt) ? 1'd1 : 1'd0;

                        // 解码器进入等待停止位状态并且发生bit超时说明线路进入空闲状态
                        bit_sm <= decoder_wait_stop ? BIT_SM_IDLE : BIT_SM_BIT_RANGE1;
                    end
                    else begin
                        high_sample_cnt <= high_sample_cnt + swo_input_bitcount_h;
                        low_sample_cnt <= low_sample_cnt + swo_input_bitcount_l;
                        time_sample_cnt <= time_sample_cnt + 2'd1;
                    end
                end
            endcase

            // 每次判断有效都对抖动注入计数器加一，并修改bit时间
            if (bit_valid) begin
                bit_valid <= 1'd0;
                jitter_cnt <= jitter_cnt + 1'd1;
            end

        end
    end

    // ========================================================================
    // 240M时钟域 - UART协议解码
    // bit_valid/bit_level → 起始位/数据位/停止位 → 字节输出
    // ========================================================================
    localparam U_IDLE = 2'd0;
    localparam U_DATA = 2'd1;
    localparam U_STOP = 2'd2;

    reg [1:0] uart_state;
    reg [2:0] uart_bit_cnt;
    reg [7:0] uart_shift;
    reg [7:0] uart_byte;
    reg       uart_valid;

    always @(posedge clk_200M) begin : uart_decode
        if (!resetn_200M) begin
            uart_state      <= U_IDLE;
            uart_bit_cnt    <= 3'd0;
            uart_shift      <= 8'd0;
            uart_byte       <= 8'd0;
            uart_valid      <= 1'd0;
        end
        else begin
            uart_valid <= 1'd0;

            if (bit_valid && SWO_MODE == 1'd0) begin
                case (uart_state)
                    U_IDLE: begin
                        if (bit_level == 1'd0) begin
                            uart_state   <= U_DATA;
                            uart_bit_cnt <= 3'd0;
                        end
                    end

                    U_DATA: begin
                        uart_shift   <= {bit_level, uart_shift[7:1]};
                        uart_bit_cnt <= uart_bit_cnt + 1'd1;
                        if (uart_bit_cnt == 3'd7) begin
                            uart_state <= U_STOP;
                        end
                    end

                    U_STOP: begin
                        if (bit_level == 1'd1) begin
                            uart_byte  <= uart_shift;
                            uart_valid <= 1'd1;
                        end
                        uart_state <= U_IDLE;
                    end
                endcase
            end
        end
    end

    // ========================================================================
    // 240M时钟域 - 曼彻斯特协议解码
    // 帧格式: 起始位(bit 1, 高→低) + 8bit数据(LSB first) + 停止位(连续两个低电平)
    // ========================================================================
    localparam M_IDLE = 2'd0;
    localparam M_DATA = 2'd1;
    localparam M_STOP_1 = 2'd3;
    localparam M_STOP_2 = 2'd2;

    reg [1:0]  manch_state;
    reg        manch_half;
    reg        manch_find_start;
    reg        manch_bit_ff;
    reg [2:0]  manch_bit_cnt;
    reg [7:0]  manch_shift;
    reg [7:0]  manch_byte;
    reg        manch_valid;
    reg        manch_error;

    always @(posedge clk_200M) begin : manch_decode
        if (!resetn_200M) begin
            manch_state       <= M_IDLE;
            manch_half        <= 1'd0;
            manch_find_start  <= 1'd0;
            manch_bit_ff      <= 1'd0;
            manch_bit_cnt     <= 3'd0;
            manch_shift       <= 8'd0;
            manch_byte        <= 8'd0;
            manch_valid       <= 1'd0;
            manch_error       <= 1'd0;
        end
        else begin
            manch_valid <= 1'd0;
            manch_error <= 1'd0;

            if (manch_state == M_IDLE) begin
                // 空闲状态复位其他寄存器
                manch_bit_cnt <= 3'd0;
                manch_half <= 1'd0;
            end

            if (bit_sm == BIT_SM_IDLE) begin
                manch_find_start <= 1'd0;
            end

            // 移位寄存器无条件持续工作
            if (bit_valid) begin
                manch_bit_ff <= bit_level;
            end

            if (bit_valid && SWO_MODE == 1'd1) begin
                case (manch_state)
                    M_IDLE: begin
                        manch_find_start <= manch_find_start ^ bit_level;
                        if (manch_find_start) begin
                            manch_find_start <= 1'd0;
                            // 第二位为0确认起始位，转移下一状态
                            manch_state <= bit_level ? M_IDLE : M_DATA;
                            manch_error <= bit_level;
                        end
                    end

                    M_DATA: begin
                        // 位有效翻转half标志
                        manch_half <= ~manch_half;
                        manch_bit_cnt <= manch_bit_cnt + manch_half;
                        if (manch_half) begin
                            manch_shift <= {manch_bit_ff, manch_shift[7:1]};
                            if (manch_bit_ff ^ bit_level) begin
                                // 有效电平
                                if (manch_bit_cnt == 3'd7) begin
                                    manch_state <= M_STOP_1;
                                end
                                else begin
                                    manch_state <= M_DATA;
                                end
                            end
                            else begin
                                // 错误电平
                                manch_error <= 1'd1;
                                manch_state <= M_IDLE;
                            end
                        end
                        else begin
                            manch_state <= M_DATA;
                        end
                    end

                    M_STOP_1: begin
                        if (bit_level == 1'd0) begin
                            manch_state <= M_STOP_2;
                            manch_byte <= manch_shift;
                        end
                        else begin
                            manch_state <= M_IDLE;
                        end
                        manch_error <= bit_level;
                    end
                    M_STOP_2: begin
                        manch_state <= M_IDLE;
                        manch_valid <= ~bit_level;
                        manch_error <= bit_level;
                    end
                endcase
            end
        end
    end

    assign decoder_wait_stop = (SWO_MODE == 1'd0) ? (uart_state == U_STOP) : (manch_state == M_STOP_2 || manch_error);

    // ========================================================================
    // CDC: clk_200M → clk  选通 + 跨时钟域
    // 在200M域根据SWO_MODE选择UART/Manch输出，通过toggle同步到clk域
    // ========================================================================
    reg [7:0] swo_byte_200M;
    reg       swo_toggle_200M;

    always @(posedge clk_200M) begin : swo_out_mux
        if (!resetn_200M) begin
            swo_byte_200M   <= 8'd0;
            swo_toggle_200M <= 1'd0;
        end
        else begin
            if (SWO_MODE == 1'd0) begin
                if (uart_valid) begin
                    swo_byte_200M   <= uart_byte;
                    swo_toggle_200M <= ~swo_toggle_200M;
                end
            end
            else begin
                if (manch_valid) begin
                    swo_byte_200M   <= manch_byte;
                    swo_toggle_200M <= ~swo_toggle_200M;
                end
            end
        end
    end

    // clk域同步与捕获
    reg swo_toggle_sync1;
    reg swo_toggle_sync2;
    reg swo_toggle_sync3;

    always @(posedge clk or negedge resetn) begin : swo_cdc_capture
        if (!resetn) begin
            swo_toggle_sync1 <= 1'd0;
            swo_toggle_sync2 <= 1'd0;
            swo_toggle_sync3 <= 1'd0;
            swo_byte         <= 8'd0;
            swo_valid        <= 1'd0;
        end
        else begin
            swo_toggle_sync1 <= swo_toggle_200M;
            swo_toggle_sync2 <= swo_toggle_sync1;
            swo_toggle_sync3 <= swo_toggle_sync2;
            swo_valid <= 1'd0;
            if (swo_toggle_sync2 != swo_toggle_sync3) begin
                swo_byte  <= swo_byte_200M;
                swo_valid <= 1'd1;
            end
        end
    end

    function [1:0] bitcount_h;
        input [1:0] D;
        begin
            case(D)
                2'b00:
                    bitcount_h = 2'd0;
                2'b01:
                    bitcount_h = 2'd1;
                2'b10:
                    bitcount_h = 2'd1;
                2'b11:
                    bitcount_h = 2'd2;
            endcase
        end
    endfunction

    function [1:0] bitcount_l;
        input [1:0] D;
        begin
            case(D)
                2'b00:
                    bitcount_l = 2'd2;
                2'b01:
                    bitcount_l = 2'd1;
                2'b10:
                    bitcount_l = 2'd1;
                2'b11:
                    bitcount_l = 2'd0;
            endcase
        end
    endfunction
endmodule
