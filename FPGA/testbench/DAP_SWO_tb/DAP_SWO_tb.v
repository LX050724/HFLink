`timescale 1 ns / 10 ps

module DAP_SWO_tb();

    // ========================================================================
    // 时钟与复位
    // ========================================================================
    reg clk;
    reg clk_200M;
    reg resetn;

    // clk: 60MHz (16.666ns)
    always begin
        #16.666 clk <= ~clk;
    end

    // clk_200M: 200MHz
    always begin
        #2.5 clk_200M <= ~clk_200M;
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
    reg              swo_pin_in;      // 模拟物理SWO引脚（单端输入）
    wire [1:0]       LOC_SWO_DDR_Q;   // IDDR输出: [1]=Q1, [0]=Q0
    wire [7:0]       swo_byte;
    wire             swo_valid;

    // ========================================================================
    // 参数配置
    // ========================================================================
    // 寄存器地址:  CTRL=0, BIT_TIME=4, PARAM=8
    localparam ADDR_CTRL     = 12'd0;
    localparam ADDR_BIT_TIME = 12'd4;
    localparam ADDR_PARAM    = 12'd8;

    // ---- 刺激生成用的200MHz周期数 ----
    localparam UART_CYCLES       = 12'd10;
    localparam MANCH_HALF_CYCLES = 12'd10;

    // ---- 寄存器写入值 (采样计数单位, 每200M周期2个采样) ----
    // UART 参数
    localparam [11:0] UART_BIT_TIME_0     = UART_CYCLES - 1;   // bit：1+1周期
    localparam [11:0] UART_BIT_TIME_1     = UART_CYCLES;   // bit：1+1周期
    localparam [11:0] UART_BIT_DEC_LOW  =  UART_CYCLES - 3;   // 死区：0+1周期
    localparam [11:0] UART_BIT_DEC_HIGH =  UART_CYCLES + 3;   // 超时：0+1周期
    localparam [11:0] UART_EDGE_DEC     = 4'd1;    // 边沿判决：>0采样

    // Manchester 参数
    localparam [11:0] MANCH_BIT_TIME    = MANCH_HALF_CYCLES - 1;
    localparam [11:0] MANCH_BIT_DEC_LOW = MANCH_HALF_CYCLES - 3;
    localparam [11:0] MANCH_BIT_DEC_HIGH = MANCH_HALF_CYCLES + 3;
    localparam [11:0] MANCH_EDGE_DEC    = 4'd1;

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
    // IDDR 仿真模型例化 (Gowin IDDR 原语)
    // 上升沿采样 → Q0, 下降沿采样 → Q1
    // ========================================================================
    IDDR u_iddr (
        .CLK(clk_200M),
        .D  (swo_pin_in),
        .Q0 (LOC_SWO_DDR_Q[0]),
        .Q1 (LOC_SWO_DDR_Q[1])
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
            ahb_write(ADDR_BIT_TIME, {4'd0, UART_BIT_TIME_1, 4'd0, UART_BIT_TIME_0}, 4'b1111);
            // PARAM = {BIT_DEC_LOW, BIT_DEC_HIGH}
            ahb_write(ADDR_PARAM, {4'd0, UART_BIT_DEC_LOW, 4'd0, UART_BIT_DEC_HIGH}, 4'b1111);
            // CR: [15:8]=EDGE_DEC, [6:2]=JITTER, [1]=MODE(0=UART), [0]=EN(1)
            ahb_write(ADDR_CTRL, {16'd0, UART_EDGE_DEC, 2'd0, 4'd0, 1'b0, 1'b1}, 4'b1111);
        end
    endtask

    // 配置Manchester模式
    task config_manch;
        begin
            ahb_write(ADDR_CTRL, 32'd0, 4'b0011);
            // CR: [15:8]=EDGE_DEC, [6:2]=JITTER, [1]=MODE(1=Manch), [0]=EN(1)
            ahb_write(ADDR_BIT_TIME, {4'd0, MANCH_BIT_TIME, 4'd0, MANCH_BIT_TIME}, 4'b1111);
            ahb_write(ADDR_PARAM, {4'd0, MANCH_BIT_DEC_LOW, 4'd0, MANCH_BIT_DEC_HIGH}, 4'b1111);
            ahb_write(ADDR_CTRL, {16'd0, MANCH_EDGE_DEC, 2'd0, 4'd0, 1'b1, 1'b1}, 4'b1111);
        end
    endtask

    // 禁用SWO
    task disable_swo;
        begin
            ahb_write(ADDR_CTRL, 32'd0, 4'b0011);
        end
    endtask

    // 产生稳定的DDR信号 (IDDR双沿采样，两个样本电平相同)
    // 驱动单端引脚 swo_pin_in，由IDDR在双沿采样
    task ddr_stable;
        input level;
        begin
            swo_pin_in <= level;
        end
    endtask

    // 在200M时钟域驱动UART bit
    // level=0/1, 持续 cycles 个200M周期
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
    // cycles_per_bit: 每个bit的200M周期数
    task send_uart_byte;
        input [7:0] byte_data;
        input [15:0] cycles_per_bit;
        integer b;
        begin
            // 起始位 (LOW)
            drive_uart_bit(0, cycles_per_bit);

            // 8数据位 LSB first
            for (b = 0; b < 8; b = b + 1) begin
                drive_uart_bit(byte_data[b], cycles_per_bit);
            end

            // 停止位 (HIGH)
            drive_uart_bit(1, cycles_per_bit);
        end
    endtask

    // 发送Manchester帧
    // 起始位=1 (HIGH→LOW), 8bit数据, 停止位=连续两个LOW半bit
    // half_cycles: 每个半bit的200M周期数
    task send_manch_frame;
        input [7:0] byte_data;
        input [15:0] half_cycles;
        integer b;
        begin
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
            drive_uart_bit(0, half_cycles * 2);
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
        swo_pin_in      = 1;   // idle HIGH
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


        send_uart_byte(8'h19, UART_CYCLES);
        #10
        send_uart_byte(8'h70, UART_CYCLES);
        #10
        send_uart_byte(8'h05, UART_CYCLES);
        #10
        send_uart_byte(8'h19, UART_CYCLES);
        #10
        send_uart_byte(8'h70, UART_CYCLES);
        #10
        send_uart_byte(8'h05, UART_CYCLES);
        send_uart_byte(8'h19, UART_CYCLES);
        send_uart_byte(8'h70, UART_CYCLES);
        send_uart_byte(8'h05, UART_CYCLES);
        send_uart_byte(8'h19, UART_CYCLES);
        send_uart_byte(8'h70, UART_CYCLES);
        send_uart_byte(8'h05, UART_CYCLES);
        send_uart_byte(8'h19, UART_CYCLES);
        send_uart_byte(8'h70, UART_CYCLES);
        send_uart_byte(8'h05, UART_CYCLES);

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
        // TEST 5B: 空闲期注入1bit宽LOW脉冲（模拟虚假起始位），然后正常帧
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 5B: UART idle LOW pulse (1 bit wide) + 0x3C ---");

        // 长空闲
        drive_uart_bit(1, 16'd10);
        // 注入1个bit宽度的LOW脉冲
        drive_uart_bit(0, UART_CYCLES);
        // 恢复空闲
        drive_uart_bit(1, 16'd10);
        // 发送正常帧
        send_uart_byte(8'h3C, UART_CYCLES);
        check_output(8'h3C);

        // ---------------------------------------------------------------
        // TEST 5C: 空闲期注入短LOW毛刺（1周期），然后正常帧
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 5C: UART idle glitch (1 cycle LOW) + 0x69 ---");

        // 长空闲
        drive_uart_bit(1, 16'd10);
        // 注入1周期LOW毛刺
        ddr_stable(0);
        @(posedge clk_200M);
        ddr_stable(1);
        // 恢复空闲
        drive_uart_bit(1, 16'd10);
        // 发送正常帧
        send_uart_byte(8'h69, UART_CYCLES);
        check_output(8'h69);

        // ---------------------------------------------------------------
        // TEST 5D: 空闲期连续噪声脉冲，长空闲恢复后正常帧
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 5D: UART idle noise burst, long recovery + 0xC3 ---");

        // 空闲
        drive_uart_bit(1, 16'd5);
        // 连续多个噪声脉冲
        drive_uart_bit(0, UART_CYCLES);
        drive_uart_bit(1, UART_CYCLES);
        drive_uart_bit(0, 16'd1);
        @(posedge clk_200M);
        ddr_stable(1);
        drive_uart_bit(0, UART_CYCLES);
        // 长空闲恢复
        drive_uart_bit(1, 16'd30);
        // 发送正常帧
        send_uart_byte(8'hC3, UART_CYCLES);
        check_output(8'hC3);

        // ---------------------------------------------------------------
        // TEST 6: Manchester 基本收发
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 6: Manchester init and send 0x5A ---");
        ddr_stable(1'd0);

        config_manch();
        #200;

        send_manch_frame(8'h5A, MANCH_HALF_CYCLES);
        check_output(8'h5A);

        // ---------------------------------------------------------------
        // TEST 7: Manchester 另一个字节
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 7: Manchester send 0xE7 ---");

        send_manch_frame(8'hE7, MANCH_HALF_CYCLES);
        check_output(8'hE7);

        // ---------------------------------------------------------------
        // TEST 8: Manchester 帧前注入HIGH脉冲干扰
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 8: Manchester pre-frame HIGH pulse + 0xE8 ---");

        drive_uart_bit(1, MANCH_HALF_CYCLES);
        drive_uart_bit(0, 32);
        send_manch_frame(8'hE8, MANCH_HALF_CYCLES);
        check_output(8'hE8);

        // ---------------------------------------------------------------
        // TEST 9: Manchester 帧前注入高低电平干扰
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 9: Manchester pre-frame HIGH+LOW burst + 0xE9 ---");

        drive_uart_bit(1, 32);
        drive_uart_bit(0, 32);
        send_manch_frame(8'hE9, MANCH_HALF_CYCLES);
        check_output(8'hE9);

        // ---------------------------------------------------------------
        // TEST 10: Manchester 连续两帧
        // ---------------------------------------------------------------
        $display("");
        $display("--- TEST 10: Manchester back-to-back 0x55 0xAA ---");
        begin
            fork
                begin
                    send_manch_frame(8'h55, MANCH_HALF_CYCLES);
                    send_manch_frame(8'hAA, MANCH_HALF_CYCLES);
                end
                begin
                   check_output(8'h55);
                   check_output(8'hAA);
                end
            join
            disable fork;
        end


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
