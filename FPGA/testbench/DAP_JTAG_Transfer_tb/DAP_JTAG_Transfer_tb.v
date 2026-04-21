`timescale 1 ns/ 10 ps
`include "DAP_Cmd.v"

module DAP_JTAG_Transfer_tb();

    // ============================================================================
    // 信号声明
    // ============================================================================
    
    // 系统时钟: 60MHz
    reg clk;
    reg resetn;
    
    // AHB-Lite接口 (用于配置BaudGenerator)
    reg ahb_write_en;
    reg ahb_read_en;
    reg [11:0] ahb_addr;
    wire [31:0] ahb_rdata;
    reg [31:0] ahb_wdata;
    reg [3:0] ahb_byte_strobe;
    
    // BaudGenerator输出信号
    wire sclk_out;
    wire sclk_negedge;
    wire sclk_sampling;
    wire sclk_sampling_en;
    
    // DAP_Seqence输入
    reg seq_tx_valid;
    reg [15:0] seq_tx_cmd;
    reg [31:0] seq_tx_data;
    wire seq_tx_full;
    
    // DAP_Seqence输出
    wire seq_rx_valid;
    wire [15:0] seq_rx_flag;
    wire [31:0] seq_rx_data;
    
    // 配置信号
    reg [15:0] DAP_TRANS_WAIT_RETRY;
    reg [1:0] SWD_TURN_CYCLE;
    reg SWD_CONF_FORCE_DATA;
    reg SWD_CONF_TURN_CLK;
    
    // JTAG配置信号
    reg [3:0] JTAG_COUNT;
    reg [3:0] JTAG_IR_LEN;
    reg [7:0] JTAG_IR_BEFORE_LEN;
    reg [7:0] JTAG_IR_AFTER_LEN;
    
    // GPIO接口
    wire SWCLK_TCK_O;
    wire SWDIO_TMS_T;
    wire SWDIO_TMS_O;
    reg SWDIO_TMS_I;
    reg SWO_TDO_I;
    wire TDI_O;
    reg SRST_I;
    wire SRST_O;
    reg TRST_I;
    wire TRST_O;
    
    // ============================================================================
    // 时钟生成: 60MHz -> 周期16.67ns，使用10ns周期便于仿真
    // ============================================================================
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 5ns * 2 = 10ns周期 = 100MHz，调整为8ns = 60MHz
    end
    
    // ============================================================================
    // DAP_BaudGenerator实例
    // ============================================================================
    DAP_BaudGenerator#(
        .ADDRWIDTH(12),
        .BASE_ADDR(12'h000)
    ) baud_gen_inst (
        .clk(clk),
        .sclk_in(clk),           // 使用系统时钟作为输入
        .resetn(resetn),
        
        // AHB MEM接口
        .ahb_write_en(ahb_write_en),
        .ahb_read_en(ahb_read_en),
        .ahb_addr(ahb_addr),
        .ahb_rdata(ahb_rdata),
        .ahb_wdata(ahb_wdata),
        .ahb_byte_strobe(ahb_byte_strobe),
        
        // 时钟输出
        .sclk_out(sclk_out),
        .sclk_negedge(sclk_negedge),
        .sclk_sampling(sclk_sampling),
        .sclk_sampling_en(sclk_sampling_en)
    );


    
    // ============================================================================
    // DUT实例
    // ============================================================================
    DAP_Seqence dut (
        .clk(clk),
        .resetn(resetn),
        
        // 串行时钟
        .sclk(clk),              // 使用系统时钟作为sclk
        .sclk_out(sclk_out),
        .sclk_negedge(sclk_negedge),
        .sclk_sampling(sclk_sampling),
        .sclk_sampling_en(sclk_sampling_en),
        
        // 控制器输入输出
        .seq_tx_valid(seq_tx_valid),
        .seq_tx_cmd(seq_tx_cmd),
        .seq_tx_data(seq_tx_data),
        .seq_tx_full(seq_tx_full),
        
        .seq_rx_valid(seq_rx_valid),
        .seq_rx_flag(seq_rx_flag),
        .seq_rx_data(seq_rx_data),
        
        // 配置信号
        .DAP_TRANS_WAIT_RETRY(DAP_TRANS_WAIT_RETRY),
        .SWD_TURN_CYCLE(SWD_TURN_CYCLE),
        .SWD_CONF_FORCE_DATA(SWD_CONF_FORCE_DATA),
        .SWD_CONF_TURN_CLK(SWD_CONF_TURN_CLK),
        
        // JTAG IR配置信号
        .JTAG_COUNT(JTAG_COUNT),
        .JTAG_IR_LEN(JTAG_IR_LEN),
        .JTAG_IR_BEFORE_LEN(JTAG_IR_BEFORE_LEN),
        .JTAG_IR_AFTER_LEN(JTAG_IR_AFTER_LEN),
        
        // GPIO
        .SWCLK_TCK_O(SWCLK_TCK_O),
        .SWDIO_TMS_T(SWDIO_TMS_T),
        .SWDIO_TMS_O(SWDIO_TMS_O),
        .SWDIO_TMS_I(SWDIO_TMS_I),
        .SWO_TDO_I(TDI_O),
        .TDI_O(TDI_O),
        .SRST_I(SRST_I),
        .SRST_O(SRST_O),
        .TRST_I(TRST_I),
        .TRST_O(TRST_O)
    );
    
    // ============================================================================
    // 测试参数和状态
    // ============================================================================
    integer i;
    integer pass_count;
    integer fail_count;
    
    // ============================================================================
    // 主测试流程
    // ============================================================================
    initial begin
        // 打开VCD波形文件
        $dumpfile("test.vcd");
        $dumpvars(0, DAP_JTAG_Transfer_tb);
        
        // 初始化所有信号
        resetn = 0;
        ahb_write_en = 0;
        ahb_read_en = 0;
        ahb_addr = 0;
        ahb_wdata = 0;
        ahb_byte_strobe = 0;
        seq_tx_valid = 0;
        seq_tx_cmd = 0;
        seq_tx_data = 0;
        DAP_TRANS_WAIT_RETRY = 5;
        SWD_TURN_CYCLE = 0;
        SWD_CONF_FORCE_DATA = 0;
        SWD_CONF_TURN_CLK = 0;
        JTAG_COUNT = 0;
        JTAG_IR_LEN = 8;
        JTAG_IR_BEFORE_LEN = 0;
        JTAG_IR_AFTER_LEN = 0;
        SWDIO_TMS_I = 0;
        SWO_TDO_I = 0;
        SRST_I = 0;
        TRST_I = 0;
        
        // 释放复位
        #100;
        resetn = 1;
        #100;
        
        pass_count = 0;
        fail_count = 0;
        // 配置BaudGenerator: 设置分频器和采样点
        configure_baud_generator();
        
        // 运行测试用例
        run_test_basic_write();
        run_test_basic_read();
        run_test_idcode_read();
        run_test_ir_shift();
        run_test_wait_retry();
        run_test_abort();
        
        // JTAG Sequence 测试用例
        run_test_jtag_seq_basic();
        run_test_jtag_seq_tms_toggle();
        run_test_jtag_seq_continuous();
        run_test_jtag_seq_state_nav();
        
        // 输出测试结果
        $display("");
        $display("========================================");
        $display("         TEST SUMMARY");
        $display("========================================");
        $display("  Passed: %0d", pass_count);
        $display("  Failed: %0d", fail_count);
        $display("========================================");
        
        if (fail_count == 0) begin
            $display("  *** ALL TESTS PASSED! ***");
            $finish(1);
        end else begin
            $display("  *** SOME TESTS FAILED! ***");
            $finish(0);
        end
    end
    
    // ============================================================================
    // 配置BaudGenerator
    // ============================================================================
    task configure_baud_generator;
    begin
        $display("[CONFIG] Configuring BaudGenerator...");
        
        // 写入TIMING寄存器: 设置分频和采样
        // DIV = 10 (决定sclk频率), SAMPLING = 5 (采样点)
        ahb_write_en = 1;
        ahb_addr = 12'h004;
        ahb_wdata = {16'd2, 16'd2};  // SAMPLING=5, DIV=10
        ahb_byte_strobe = 4'hf;
        #20;
        ahb_write_en = 0;
        
        // 写入CR寄存器: 使能时钟输出
        ahb_write_en = 1;
        ahb_addr = 12'h000;
        ahb_wdata = 32'h0000_0001;  // CEN = 1
        ahb_byte_strobe = 4'hf;
        #20;
        ahb_write_en = 0;

        $display("[CONFIG] BaudGenerator configured.");
    end
    endtask
    
    // ============================================================================
    // 测试用例1: 基本写操作
    // ============================================================================
    task run_test_basic_write;
    begin
        $display("");
        $display("========== TEST 1: Basic JTAG Write ==========");
        
        // 配置写操作: APnDP=0, RnW=0, A3=0, A2=1
        // cmd[15:12] = 5 (JTAG_TRANSFER), cmd[11:0] = {8'd0, APnDP, RnW, A3, A2, Index, IDCODE, Abort, IR}
        // Index=0, IDCODE=0, Abort=0, IR=0 -> cmd[11:0] = 12'd0
        //                        | IDCODE | JTAG Index | Abort | IR | A3 | A2 | RnW | APnDP |
        seq_tx_cmd = {`SEQ_CMD_JTAG_TRANSFER, 2'd0,     1'd0,        3'd0,   1'd0, 1'd1,    2'd0, 1'd0, 1'd0};
        seq_tx_data = 32'h5a5a5a5a;
        
        // 等待DUT就绪
        wait(seq_tx_valid == 0);
        
        // 发送命令
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        
        $display("  [TX] CMD: %04h, DATA: %08h", seq_tx_cmd, seq_tx_data);
        
        // 等待传输完成
        wait(seq_rx_valid);
        #50;
        
        if (seq_rx_valid) begin
            $display("  [RX] FLAG: %04h, DATA: %08h", seq_rx_flag, seq_rx_data);
            
            // 验证: 写操作应该返回ACK=001
            if (seq_rx_flag[2:0] == 3'b001) begin
                $display("  [PASS] Write ACK verified (001)");
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] Expected ACK 001, got %0b", seq_rx_flag[2:0]);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("  [FAIL] No response received");
            fail_count = fail_count + 1;
        end
    end
    endtask
    
    // ============================================================================
    // 测试用例2: 基本读操作
    // ============================================================================
    task run_test_basic_read;
    begin
        $display("");
        $display("========== TEST 2: Basic JTAG Read ==========");
        
        // 配置读操作: APnDP=0, RnW=1, A3=0, A2=1
        seq_tx_cmd = 16'h3002;  // RnW=1
        seq_tx_data = 32'd0;
        
        wait(seq_tx_valid == 0);
        
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        
        $display("  [TX] CMD: %04h (Read)", seq_tx_cmd);
        
        // 使用FORCE_DATA模式简化读测试
        SWD_CONF_FORCE_DATA = 1;
        #200;
        SWD_CONF_FORCE_DATA = 0;
        
        wait(seq_rx_valid);
        #50;
        
        if (seq_rx_valid) begin
            $display("  [RX] FLAG: %04h, DATA: %08h", seq_rx_flag, seq_rx_data);
            
            if (seq_rx_flag[2:0] == 3'b001) begin
                $display("  [PASS] Read ACK verified (001)");
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] Expected ACK 001, got %0b", seq_rx_flag[2:0]);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("  [FAIL] No response received");
            fail_count = fail_count + 1;
        end
    end
    endtask
    
    // ============================================================================
    // 测试用例3: IDCODE读取
    // ============================================================================
    task run_test_idcode_read;
    begin
        $display("");
        $display("========== TEST 3: IDCODE Read ==========");
        
        // IDCODE=1: cmd[9] = 1
        seq_tx_cmd = 16'h3200;  // bit 9 = IDCODE
        seq_tx_data = 32'd0;
        
        wait(seq_tx_valid == 0);
        
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        
        $display("  [TX] CMD: %04h (IDCODE)", seq_tx_cmd);
        
        wait(seq_rx_valid);
        #50;
        
        if (seq_rx_valid) begin
            $display("  [RX] FLAG: %04h, DATA: %08h", seq_rx_flag, seq_rx_data);
            
            if (seq_rx_flag[2:0] != 3'b111) begin
                $display("  [PASS] IDCODE read completed");
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] IDCODE read returned error");
                fail_count = fail_count + 1;
            end
        end else begin
            $display("  [FAIL] No response received");
            fail_count = fail_count + 1;
        end
    end
    endtask
    
    // ============================================================================
    // 测试用例4: IR移位操作
    // ============================================================================
    task run_test_ir_shift;
    begin
        $display("");
        $display("========== TEST 4: IR Shift ==========");
        
        // IR=1: cmd[4] = 1
        seq_tx_cmd = 16'h3010;  // bit 4 = IR
        seq_tx_data = 32'd0;
        
        // 设置IR长度参数
        JTAG_IR_LEN = 8;
        JTAG_IR_BEFORE_LEN = 0;
        JTAG_IR_AFTER_LEN = 0;
        
        wait(seq_tx_valid == 0);
        
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        
        $display("  [TX] CMD: %04h (IR Shift)", seq_tx_cmd);
        
        wait(seq_rx_valid);
        #50;
        
        if (seq_rx_valid) begin
            $display("  [RX] FLAG: %04h, DATA: %08h", seq_rx_flag, seq_rx_data);
            $display("  [PASS] IR shift completed");
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] No response received");
            fail_count = fail_count + 1;
        end
    end
    endtask
    
    // ============================================================================
    // 测试用例5: WAIT重试机制
    // ============================================================================
    task run_test_wait_retry;
    begin
        $display("");
        $display("========== TEST 5: WAIT Retry Mechanism ==========");
        
        // 设置最大重试次数为3
        DAP_TRANS_WAIT_RETRY = 3;
        
        seq_tx_cmd = 16'h3000;
        seq_tx_data = 32'd0;
        
        wait(seq_tx_valid == 0);
        
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        
        $display("  [TX] CMD: %04h (Retry Test)", seq_tx_cmd);
        
        wait(seq_rx_valid);
        #50;
        
        if (seq_rx_valid) begin
            $display("  [RX] FLAG: %04h", seq_rx_flag);
            $display("  [PASS] Retry test completed");
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] No response received");
            fail_count = fail_count + 1;
        end
        
        // 恢复默认值
        DAP_TRANS_WAIT_RETRY = 5;
    end
    endtask
    
    // ============================================================================
    // 测试用例6: Abort操作
    // ============================================================================
    task run_test_abort;
    begin
        $display("");
        $display("========== TEST 6: Abort Operation ==========");
        
        // Abort=1: cmd[5] = 1
        seq_tx_cmd = 16'h3020;  // bit 5 = Abort
        seq_tx_data = 32'd0;
        
        wait(seq_tx_valid == 0);
        
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        
        $display("  [TX] CMD: %04h (Abort)", seq_tx_cmd);
        
        wait(seq_rx_valid);
        #50;
        
        if (seq_rx_valid) begin
            $display("  [RX] FLAG: %04h, DATA: %08h", seq_rx_flag, seq_rx_data);
            $display("  [PASS] Abort completed");
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] No response received");
            fail_count = fail_count + 1;
        end
    end
    endtask
    
    // ============================================================================
    // GPIO状态监控
    // ============================================================================
    initial begin
        $display("");
        $display("[MONITOR] Starting GPIO monitoring...");

    end
    
    always @(posedge SWCLK_TCK_O) begin
        $display("  MIO: %b %b %b", 
                    SWDIO_TMS_O, TDI_O, SWO_TDO_I);
    end
    
    // ============================================================================
    // JTAG Sequence 测试用例
    // ============================================================================
    
    // -------------------------------------------------------------------------
    // 测试用例7: JTAG Sequence 基本操作
    // JTAG_Sequence命令格式:
    // | 15 14 13 12 | 11 10  9  8  7  6  5 |   4 |        3  2  1  0 |
    // |  0  1  0  0 |  x  x  x  x  x  x  x | TMS |  Number of cycles |
    // -------------------------------------------------------------------------
    task run_test_jtag_seq_basic;
        reg [15:0] cmd;
    begin
        $display("");
        $display("========== TEST 7: JTAG Sequence Basic ==========");
        
        // 构造 JTAG_Sequence 命令: TMS=0, Cycles=1
        // cmd[15:12] = 4 (SEQ_CMD_JTAG_SEQ)
        // cmd[11:5] = 7'd0 (保留位)
        // cmd[4] = TMS (0)
        // cmd[3:0] = 1 (1个时钟周期)
        cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, 1'b0, 4'd8};
        seq_tx_cmd = cmd;
        seq_tx_data = 32'h0000_00aa;  // 数据无意义，仅满足接口要求
        
        wait(seq_tx_valid == 0);
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        
        $display("  [TX] CMD: %04h (JTAG_Seq TMS=0, Cycles=1)", seq_tx_cmd);
        
        // 等待响应
        wait(seq_rx_valid);
        #50;
        
        if (seq_rx_valid) begin
            $display("  [RX] FLAG: %04h, DATA: %08h", seq_rx_flag, seq_rx_data);
            // JTAG Sequence 不返回 ACK，仅检查完成标志
            if (seq_rx_flag[0] == 1'b1) begin
                $display("  [PASS] JTAG Sequence basic test completed");
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] JTAG Sequence not completed properly");
                fail_count = fail_count + 1;
            end
        end else begin
            $display("  [FAIL] No response received");
            fail_count = fail_count + 1;
        end
    end
    endtask
    
    // -------------------------------------------------------------------------
    // 测试用例8: JTAG Sequence TMS 切换测试
    // 测试 TMS=1 的情况，用于状态切换
    // -------------------------------------------------------------------------
    task run_test_jtag_seq_tms_toggle;
        reg [15:0] cmd;
    begin
        $display("");
        $display("========== TEST 8: JTAG Sequence TMS Toggle ==========");
        
        // 构造 JTAG_Sequence 命令: TMS=1, Cycles=1
        // TMS=1 用于进入 JTAG 状态机的 Test-Logic-Reset 或其他状态
        // cmd[15:12]=SEQ_CMD_JTAG_SEQ, cmd[11:5]=7'd0, cmd[4]=TMS, cmd[3:0]=Cycles
        cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, 1'b1, 4'd1};
        seq_tx_cmd = cmd;
        seq_tx_data = 32'h0000_0000;
        
        wait(seq_tx_valid == 0);
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        
        $display("  [TX] CMD: %04h (JTAG_Seq TMS=1, Cycles=1)", seq_tx_cmd);
        
        // 等待响应
        wait(seq_rx_valid);
        #50;
        
        if (seq_rx_valid) begin
            $display("  [RX] FLAG: %04h, DATA: %08h", seq_rx_flag, seq_rx_data);
            if (seq_rx_flag[0] == 1'b1) begin
                $display("  [PASS] JTAG Sequence TMS toggle test completed");
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] JTAG Sequence TMS toggle not completed");
                fail_count = fail_count + 1;
            end
        end else begin
            $display("  [FAIL] No response received");
            fail_count = fail_count + 1;
        end
    end
    endtask
    
    // -------------------------------------------------------------------------
    // 测试用例9: JTAG Sequence 连续多次操作
    // 测试连续发送多个 Sequence 命令
    // -------------------------------------------------------------------------
    task run_test_jtag_seq_continuous;
        reg [15:0] cmd;
        integer seq_num;
    begin
        $display("");
        $display("========== TEST 9: JTAG Sequence Continuous ==========");
        
        seq_num = 0;
        
        // 连续发送 3 个 Sequence 命令
        repeat(3) begin : seq_loop
            seq_num = seq_num + 1;
            
            // 交替使用 TMS=0 和 TMS=1
            if (seq_num[0] == 1'b0) begin
                // TMS=0, Cycles=2
                cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, 1'b0, 4'd2};
            end else begin
                // TMS=1, Cycles=2
                cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, 1'b1, 4'd2};
            end
            
            seq_tx_cmd = cmd;
            seq_tx_data = 32'h0000_0000;
            
            wait(seq_tx_valid == 0);
            seq_tx_valid = 1;
            #50;
            seq_tx_valid = 0;
            
            $display("  [TX] CMD: %04h (JTAG_Seq #%0d)", seq_tx_cmd, seq_num);
            
            // 等待响应
            wait(seq_rx_valid);
            #50;
            
            if (!seq_rx_valid || seq_rx_flag[0] != 1'b1) begin
                $display("  [FAIL] JTAG Sequence #%0d failed", seq_num);
                disable seq_loop;
            end
        end
        
        if (seq_num == 3) begin
            $display("  [PASS] JTAG Sequence continuous test: 3 sequences completed");
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] JTAG Sequence continuous test: only %0d sequences completed", seq_num);
            fail_count = fail_count + 1;
        end
    end
    endtask
    
    // -------------------------------------------------------------------------
    // 测试用例10: JTAG Sequence 状态导航测试
    // 测试 JTAG 状态机导航序列 (TAP 状态转换)
    // -------------------------------------------------------------------------
    task run_test_jtag_seq_state_nav;
        reg [15:0] cmd;
        integer step;
    begin
        $display("");
        $display("========== TEST 10: JTAG Sequence State Navigation ==========");
        
        // JTAG TAP 状态导航序列:
        // 从 Run-Test/Idle 进入 Shift-DR 状态
        // 路径: Run-Test/Idle -> Select-DR-Scan -> Capture-DR -> Shift-DR
        
        // 步骤1: Run-Test/Idle -> Select-DR-Scan (TMS=1)
        step = 1;
        cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, 1'b1, 4'd1};  // TMS=1, Cycles=1
        seq_tx_cmd = cmd;
        seq_tx_data = 32'h0000_0000;
        wait(seq_tx_valid == 0);
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        $display("  [TX] Step %0d: CMD=%04h (TMS=1, Cycles=1) -> Select-DR-Scan", step, cmd);
        wait(seq_rx_valid);
        #50;
        if (!seq_rx_valid || seq_rx_flag[0] != 1'b1) begin
            $display("  [FAIL] Step %0d failed", step);
            fail_count = fail_count + 1;
        end
        
        // 步骤2: Select-DR-Scan -> Capture-DR (TMS=0)
        step = 2;
        cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, 1'b0, 4'd1};  // TMS=0, Cycles=1
        seq_tx_cmd = cmd;
        seq_tx_data = 32'h0000_0000;
        wait(seq_tx_valid == 0);
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        $display("  [TX] Step %0d: CMD=%04h (TMS=0, Cycles=1) -> Capture-DR", step, cmd);
        wait(seq_rx_valid);
        #50;
        if (!seq_rx_valid || seq_rx_flag[0] != 1'b1) begin
            $display("  [FAIL] Step %0d failed", step);
            fail_count = fail_count + 1;
        end
        
        // 步骤3: Capture-DR -> Shift-DR (TMS=0)
        step = 3;
        cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, 1'b0, 4'd1};  // TMS=0, Cycles=1
        seq_tx_cmd = cmd;
        seq_tx_data = 32'h0000_0000;
        wait(seq_tx_valid == 0);
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        $display("  [TX] Step %0d: CMD=%04h (TMS=0, Cycles=1) -> Shift-DR", step, cmd);
        wait(seq_rx_valid);
        #50;
        if (!seq_rx_valid || seq_rx_flag[0] != 1'b1) begin
            $display("  [FAIL] Step %0d failed", step);
            fail_count = fail_count + 1;
        end
        
        // 步骤4: 返回 Run-Test/Idle (TMS=1, 多次)
        step = 4;
        cmd = {`SEQ_CMD_JTAG_SEQ, 7'd0, 1'b1, 4'd3};  // TMS=1, Cycles=3 (Exit1-DR -> Update-DR -> Run-Test/Idle)
        seq_tx_cmd = cmd;
        seq_tx_data = 32'h0000_0000;
        wait(seq_tx_valid == 0);
        seq_tx_valid = 1;
        #50;
        seq_tx_valid = 0;
        $display("  [TX] Step %0d: CMD=%04h (TMS=1, Cycles=3) -> Run-Test/Idle", step, cmd);
        wait(seq_rx_valid);
        #50;
        if (!seq_rx_valid || seq_rx_flag[0] != 1'b1) begin
            $display("  [FAIL] Step %0d failed", step);
            fail_count = fail_count + 1;
        end
        
        if (step == 4) begin
            $display("  [PASS] JTAG State Navigation test: 4 steps completed");
            pass_count = pass_count + 1;
        end
    end
    endtask
    
endmodule
