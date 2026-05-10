`timescale 1 ns / 10 ps

module DAP_SWO_tb();

    // ========================================================================
    // 时钟与复位
    // ========================================================================
    reg clk;
    reg clk_200M;
    reg resetn;

    // clk: 50MHz (20ns)
    always begin
        #10 clk <= ~clk;
    end

    // clk_200M: 240MHz (~4.167ns)
    always begin
        #2.08333 clk_200M <= ~clk_200M;
    end

    // ========================================================================
    // DUT 接口
    // ========================================================================
    reg              ahb_write_en;
    reg              ahb_read_en;
    reg  [11:0]      ahb_addr;
    wire [31:0]      ahb_rdata;
    reg  [31:0]      ahb_wdata;
    reg  [3:0]       ahb_byte_strobe;
    reg  [31:0]      ahb_rdata_reg;
    reg  [1:0]       LOC_SWO_DDR_Q;
    wire [7:0]       swo_byte;
    wire             swo_valid;

    // ========================================================================
    // 参数配置
    // ========================================================================
    // 寄存器地址:  CTRL=0, BIT_TIME=4, PARAM=8
    localparam ADDR_CTRL     = 12'd0;
    localparam ADDR_BIT_TIME = 12'd4;
    localparam ADDR_PARAM    = 12'd8;

    // ---- 刺激生成用的240MHz周期数 ----
    // UART:  10周期/bit = 24Mbps (240MHz / 10)
    // Manch: 5周期/half-bit = 48Mbps
    localparam UART_CYCLES       = 16'd2;
    localparam MANCH_HALF_CYCLES = 16'd5;

    // ---- 寄存器写入值 (采样计数单位, 每240M周期2个采样) ----
    // UART 参数
    localparam UART_BIT_TIME     = 16'd1;   // 10周期 × 2采样/周期
    localparam UART_BIT_DEC_LOW  = 16'd0;   // 半bit附近开始检测边沿
    localparam UART_BIT_DEC_HIGH = 16'd1;   // 略大于bit_time, 超时阈值
    localparam UART_EDGE_DEC     = 4'd0;     // 边沿判决: 2个稳定采样

    // Manchester 参数
    localparam MANCH_BIT_TIME    = 16'd5;   // 5周期 × 2采样/周期
    localparam MANCH_BIT_DEC_LOW = 16'd6;
    localparam MANCH_BIT_DEC_HIGH = 16'd14;
    localparam MANCH_EDGE_DEC    = 4'd2;

    // ========================================================================
    // DUT 例化
    // ========================================================================
    DAP_SWO #(
        .ADDRWIDTH(12),
        .BASE_ADDR(0)
    ) u_dut (
        .clk            (clk),
        .clk_200M       (clk_200M),
        .resetn         (resetn),
        .ahb_write_en   (ahb_write_en),
        .ahb_read_en    (ahb_read_en),
        .ahb_addr       (ahb_addr),
        .ahb_rdata      (ahb_rdata),
        .ahb_wdata      (ahb_wdata),
        .ahb_byte_strobe(ahb_byte_strobe),
        .LOC_SWO_DDR_Q  (LOC_SWO_DDR_Q),
        .swo_byte  (swo_byte),
        .swo_valid (swo_valid)
    );

    // ========================================================================
    // 辅助任务
    // ========================================================================

    // AHB 写操作
    task ahb_write;
        input [11:0] addr;
        input [31:0] data;
        input [3:0]  strb;
        begin
            @(posedge clk);
            ahb_write_en <= 1;
            ahb_addr     <= addr;
            ahb_wdata    <= data;
            ahb_byte_strobe <= strb;
            @(posedge clk);
            ahb_write_en <= 0;
            ahb_addr     <= 0;
            ahb_wdata    <= 0;
            ahb_byte_strobe <= 0;
        end
    endtask

    // AHB 读操作
    task ahb_read;
        input [11:0] addr;
        begin
            @(posedge clk);
            ahb_read_en <= 1;
            ahb_addr    <= addr;
            @(posedge clk);
            ahb_read_en <= 0;
            ahb_addr    <= 0;
            ahb_rdata_reg <= ahb_rdata;
        end
    endtask

    // 配置UART模式
    task config_uart;
        begin
            ahb_write(ADDR_CTRL, 32'd0, 4'b0011);
            // BIT_TIME = {BIT_TIME_1, BIT_TIME_0}
            ahb_write(ADDR_BIT_TIME, {UART_BIT_TIME, UART_BIT_TIME}, 4'b1111);
            // PARAM = {BIT_DEC_LOW, BIT_DEC_HIGH}
            ahb_write(ADDR_PARAM, {UART_BIT_DEC_LOW, UART_BIT_DEC_HIGH}, 4'b1111);
            // CR: [15:8]=EDGE_DEC, [6:2]=JITTER, [1]=MODE(0=UART), [0]=EN(1)
            ahb_write(ADDR_CTRL, {16'd0, UART_EDGE_DEC, 2'd0, 4'd0, 1'b0, 1'b1}, 4'b0011);
        end
    endtask

    // 配置Manchester模式
    task config_manch;
        begin
            ahb_write(ADDR_CTRL, 32'd0, 4'b0011);
            // CR: [15:8]=EDGE_DEC, [6:2]=JITTER, [1]=MODE(1=Manch), [0]=EN(1)
            ahb_write(ADDR_BIT_TIME, {MANCH_BIT_TIME, MANCH_BIT_TIME}, 4'b1111);
            ahb_write(ADDR_PARAM, {MANCH_BIT_DEC_LOW, MANCH_BIT_DEC_HIGH}, 4'b1111);
            ahb_write(ADDR_CTRL, {16'd0, MANCH_EDGE_DEC, 2'd0, 4'd0, 1'b1, 1'b1}, 4'b0011);
        end
    endtask

    // 禁用SWO
    task disable_swo;
        begin
            ahb_write(ADDR_CTRL, 32'd0, 4'b0011);
        end
    endtask

    // 产生稳定的DDR信号 (两个样本电平相同)
    // level=1: 2'b11, level=0: 2'b00
    task ddr_stable;
        input level;
        begin
            LOC_SWO_DDR_Q <= level ? 2'b11 : 2'b00;
        end
    endtask

    // 在240M时钟域驱动UART bit
    // level=0/1, 持续 cycles 个240M周期
    task drive_uart_bit;
        input level;
        input [15:0] cycles;
        integer i;
        begin
            for (i = 0; i < cycles; i = i + 1) begin
                @(posedge clk_200M);
                ddr_stable(level);
            end
        end
    endtask

    // 发送UART字节: start(0) + 8bit LSB first + stop(1)
    // cycles_per_bit: 每个bit的240M周期数
    task send_uart_byte;
        input [7:0] byte_data;
        input [15:0] cycles_per_bit;
        integer b;
        begin
            // 确保line先保持idle (HIGH) 至少一个bit周期，让decoder进入IDLE
            drive_uart_bit(1, cycles_per_bit);

            // 起始位 (LOW)
            drive_uart_bit(0, cycles_per_bit);

            // 8数据位 LSB first
            for (b = 0; b < 8; b = b + 1) begin
                drive_uart_bit(byte_data[b], cycles_per_bit);
            end

            // 停止位 (HIGH)
            drive_uart_bit(1, cycles_per_bit);
            // 起始位 (LOW)
            drive_uart_bit(0, cycles_per_bit);

            // 8数据位 LSB first
            for (b = 0; b < 8; b = b + 1) begin
                drive_uart_bit(byte_data[b], cycles_per_bit);
            end

            // 停止位 (HIGH)
            drive_uart_bit(1, cycles_per_bit);

            // 继续保持idle
            drive_uart_bit(1, cycles_per_bit);
        end
    endtask

    // 发送Manchester帧
    // 起始位=1 (HIGH→LOW), 8bit数据, 停止位=连续两个LOW半bit
    // half_cycles: 每个半bit的240M周期数
    task send_manch_frame;
        input [7:0] byte_data;
        input [15:0] half_cycles;
        integer b;
        begin
            // 确保IDLE (先保持HIGH)
            drive_uart_bit(0, half_cycles * 2);

            // 起始位=1: HIGH(前半) → LOW(后半)
            drive_uart_bit(1, half_cycles);   // 前半HIGH
            drive_uart_bit(0, half_cycles);   // 后半LOW

            // 8数据位 LSB first: 0=low→high, 1=high→low
            for (b = 0; b < 8; b = b + 1) begin
                if (byte_data[b] == 1'd0) begin
                    // bit 0: low→high
                    drive_uart_bit(0, half_cycles);  // 前半LOW
                    drive_uart_bit(1, half_cycles);  // 后半HIGH
                end
                else begin
                    // bit 1: high→low
                    drive_uart_bit(1, half_cycles);  // 前半HIGH
                    drive_uart_bit(0, half_cycles);  // 后半LOW
                end
            end

            // 停止位: 连续两个LOW半bit
            drive_uart_bit(0, half_cycles);   // 第1个LOW
            drive_uart_bit(0, half_cycles);   // 第2个LOW

            // 回IDLE
            // drive_uart_bit(0, half_cycles);
        end
    endtask

    // 检查输出 (clk域)
    task check_output;
        input [7:0] exp_byte;
        begin
            fork
                begin
                    repeat (5000) @(posedge clk);
                    $display("ERROR: timeout waiting for byte 0x%02h", exp_byte);
                end
                begin
                    @(posedge clk);
                    while (!swo_valid) @(posedge clk);
                    if (swo_byte == exp_byte) begin
                        $display("PASS: received 0x%02h", swo_byte);
                    end
                    else begin
                        $display("FAIL: expected 0x%02h, got 0x%02h", exp_byte, swo_byte);
                    end
                end
            join_any
            disable fork;
        end
    endtask

    // ========================================================================
    // 主测试序列
    // ========================================================================
    integer test_pass;
    integer test_fail;

    initial begin
        $dumpfile("test.vcd");
        $dumpvars(0, DAP_SWO_tb);

        // 初始化
        clk             = 0;
        clk_200M        = 0;
        resetn          = 1;
        ahb_write_en    = 0;
        ahb_read_en     = 0;
        ahb_addr        = 0;
        ahb_wdata       = 0;
        ahb_byte_strobe = 0;
        LOC_SWO_DDR_Q   = 2'b11;  // idle HIGH
        test_pass       = 0;
        test_fail       = 0;

        // 复位
        #5 resetn = 0;
        #20 resetn = 1;
        #50;

        $display("========================================");
        $display("  DAP_SWO Testbench");
        $display("========================================");

        // ---------------------------------------------------------------
        // TEST 1: AHB 读写测试
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 1: AHB Read/Write ---");

        // 写CTRL寄存器 (offset 0, bytes 0-1)
        ahb_write(ADDR_CTRL, 32'h0000_1234, 4'b0011);
        #50;
        ahb_read(ADDR_CTRL);
        #50;
        $display("  CTRL readback: 0x%08h (expected low 16 bits = 0x1234)", ahb_rdata_reg);

        // 写BIT_TIME寄存器 (offset 4, all bytes)
        ahb_write(ADDR_BIT_TIME, 32'hABCD_5678, 4'b1111);
        #50;
        ahb_read(ADDR_BIT_TIME);
        #50;
        $display("  BIT_TIME readback: 0x%08h (expected = 0xABCD5678)", ahb_rdata_reg);

        // 写PARAM寄存器 (offset 8, all bytes)
        ahb_write(ADDR_PARAM, 32'hDEAD_BEEF, 4'b1111);
        #50;
        ahb_read(ADDR_PARAM);
        #50;
        $display("  PARAM readback: 0x%08h (expected = 0xDEADBEEF)", ahb_rdata_reg);

        // 复位
        #5 resetn = 0;
        #20 resetn = 1;
        #50;

        // ---------------------------------------------------------------
        // TEST 2: UART模式 - 初始化并发送0xA5
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 2: UART mode, init and send 0xA5 (b10100101) ---");

        config_uart();
        #200;

        send_uart_byte(8'hA5, UART_CYCLES);
        check_output(8'hA5);

        // ---------------------------------------------------------------
        // TEST 3: UART模式 - 发送0xFF (全1，测试连续无跳变位)
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 3: UART mode, byte 0xFF (all ones, no transition) ---");

        send_uart_byte(8'hFF, UART_CYCLES);
        check_output(8'hFF);

        // ---------------------------------------------------------------
        // TEST 4: UART模式 - 连续两个字节
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 4: UART mode, consecutive bytes 0x33 0xCC ---");

        send_uart_byte(8'h33, UART_CYCLES);
        check_output(8'h33);

        send_uart_byte(8'hCC, UART_CYCLES);
        check_output(8'hCC);

        // ---------------------------------------------------------------
        // TEST 5: 禁用后重新启用UART
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 5: Disable/Re-enable UART ---");

        disable_swo();
        #500;
        config_uart();
        #200;

        send_uart_byte(8'h55, UART_CYCLES);
        check_output(8'h55);

        // ---------------------------------------------------------------
        // TEST 6: Manchester模式 - 初始化并发送字节
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 6: Manchester mode, init and send 0x5A ---");

        config_manch();
        #200;

        send_manch_frame(8'h5A, MANCH_HALF_CYCLES);
        check_output(8'h5A);

        // ---------------------------------------------------------------
        // TEST 7: Manchester模式 - 另一个字节
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 7: Manchester mode, byte 0xE7 ---");

        send_manch_frame(8'hE7, MANCH_HALF_CYCLES);
        check_output(8'hE7);

        // ---------------------------------------------------------------
        // TEST 8: 模式切换 UART→Manch→UART
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 8: Mode switch UART→Manch→UART ---");

        config_uart();
        #200;
        send_uart_byte(8'h7E, UART_CYCLES);
        check_output(8'h7E);

        config_manch();
        #200;
        send_manch_frame(8'h3C, MANCH_HALF_CYCLES);
        check_output(8'h3C);

        config_uart();
        #200;
        send_uart_byte(8'h81, UART_CYCLES);
        check_output(8'h81);

        // ---------------------------------------------------------------
        // 测试完成
        // ---------------------------------------------------------------
        $display("");
        $display("========================================");
        $display("  All tests completed");
        $display("========================================");

        #500;
        $finish;
    end

endmodule
