`include "DAP_Cmd.v"

module DAP_SWJ #(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    )(
        input clk,
        input resetn,
        input us_tick,
        input [31:0] us_timer,
        input enable,

        // 串行时钟
        input sclk,
        input sclk_out,
        input sclk_pulse,
        input sclk_delay_pulse,

        // AHB MEM接口
        input ahb_write_en,
        input ahb_read_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        input dap_in_tvalid,
        output [`CMD_SWJ_RANGE] dap_in_tready,
        input [7:0] dap_in_tdata,

        output reg [9:0] ram_write_addr,
        output reg [7:0] ram_write_data,
        output reg ram_write_en,
        output reg [9:0] packet_len,

        input [`CMD_SWJ_RANGE] start,
        output reg [`CMD_SWJ_RANGE] done,

        output SWCLK_TCK_O,
        output SWDIO_TMS_T,
        output SWDIO_TMS_O,
        input SWDIO_TMS_I,
        input SWO_TDO_I,
        output TDI_O,
        // input RTCK_I,
        input SRST_I,
        output SRST_O,
        input TRST_I,
        output TRST_O,

        // swd模式信号
        output SWD_MODE
    );
    genvar gi;
    integer i;

    localparam [ADDRWIDTH-1:0] SWJ_CR_ADDR            = BASE_ADDR + 12'h000;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_SWD_CR_ADDR        = BASE_ADDR + 12'h004;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_CR_ADDR       = BASE_ADDR + 12'h008;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF0_ADDR = BASE_ADDR + 12'h00C;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF1_ADDR = BASE_ADDR + 12'h010;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF2_ADDR = BASE_ADDR + 12'h014;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF3_ADDR = BASE_ADDR + 12'h018;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF4_ADDR = BASE_ADDR + 12'h01C;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF5_ADDR = BASE_ADDR + 12'h020;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF6_ADDR = BASE_ADDR + 12'h024;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF7_ADDR = BASE_ADDR + 12'h028;  // RW

    reg [31:0] SWJ_CR;

    reg [8:0] SWJ_SWD_CR;
    reg [7:0] SWJ_JTAG_CR;
    reg [31:0] SWJ_JTAG_IR_CONF_REG [0:7];

    wire [7:0] SWD_CONF_TURN = SWJ_SWD_CR[7:0];
    wire SWD_CONF_FORCE_DATA = SWJ_SWD_CR[8];

    wire [7:0] JTAG_CR_COUNT = SWJ_JTAG_CR[7:0];
    wire [13:0] JTAG_IR_BEFORE_CONF [0:7];
    wire [13:0] JTAG_IR_AFTER_CONF [0:7];
    wire [3:0] JTAG_IR_LEN_CONF [0:7];

    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : jtag_ir_conf_loop
            assign JTAG_IR_BEFORE_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][18+:14];
            assign JTAG_IR_AFTER_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][4+:14];
            assign JTAG_IR_LEN_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][3:0];
        end
    endgenerate

    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            SWJ_CR <= 0;
            SWJ_SWD_CR <= 0;
            SWJ_JTAG_CR <= 0;
            for (i = 0; i < 8; i = i + 1) begin
                SWJ_JTAG_IR_CONF_REG[i] <= 32'd0;
            end
        end
        else begin
            if (ahb_write_en) begin
                case (ahb_addr[ADDRWIDTH-1:2])
                    SWJ_CR_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_CR);
                    SWJ_SWD_CR_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_SWD_CR);
                    SWJ_JTAG_CR_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_CR);
                    SWJ_JTAG_IR_CONF0_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[0]);
                    SWJ_JTAG_IR_CONF1_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[1]);
                    SWJ_JTAG_IR_CONF2_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[2]);
                    SWJ_JTAG_IR_CONF3_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[3]);
                    SWJ_JTAG_IR_CONF4_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[4]);
                    SWJ_JTAG_IR_CONF5_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[5]);
                    SWJ_JTAG_IR_CONF6_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[6]);
                    SWJ_JTAG_IR_CONF7_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[7]);
                endcase
            end
        end
    end

    always @(*) begin : ahb_mem_read_ctrl
        if (ahb_read_en) begin
            case (ahb_addr[ADDRWIDTH-1:2])
                SWJ_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_CR;
                SWJ_SWD_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_SWD_CR;
                SWJ_JTAG_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_CR;
                SWJ_JTAG_IR_CONF0_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[0];
                SWJ_JTAG_IR_CONF1_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[1];
                SWJ_JTAG_IR_CONF2_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[2];
                SWJ_JTAG_IR_CONF3_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[3];
                SWJ_JTAG_IR_CONF4_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[4];
                SWJ_JTAG_IR_CONF5_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[5];
                SWJ_JTAG_IR_CONF6_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[6];
                SWJ_JTAG_IR_CONF7_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = SWJ_JTAG_IR_CONF_REG[7];
                default:
                    ahb_rdata = {32{1'bx}};
            endcase
        end
        else begin
            ahb_rdata = {32{1'bx}};
        end
    end

    reg [15:0] seq_tx_cmd;
    reg [63:0] seq_tx_data;
    reg seq_tx_valid;
    wire seq_tx_full;

    wire seq_rx_valid;
    wire [15:0] seq_rx_flag;
    wire [63:0] seq_rx_data;

    DAP_Seqence dap_seqence_inst(
                    // 控制器时钟
                    .clk(clk),
                    .resetn(resetn),

                    // 串行时钟
                    .sclk(sclk),
                    .sclk_out(sclk_out),
                    .sclk_pulse(sclk_pulse),
                    .sclk_delay_pulse(sclk_delay_pulse),

                    // 控制器输入输出
                    .seq_tx_valid(seq_tx_valid),
                    .seq_tx_cmd(seq_tx_cmd),
                    .seq_tx_data(seq_tx_data),
                    .seq_tx_full(seq_tx_full),

                    .seq_rx_valid(seq_rx_valid),
                    .seq_rx_flag(seq_rx_flag),
                    .seq_rx_data(seq_rx_data),


                    // GPIO
                    .SWCLK_TCK_O(SWCLK_TCK_O),
                    .SWDIO_TMS_T(SWDIO_TMS_T),
                    .SWDIO_TMS_O(SWDIO_TMS_O),
                    .SWDIO_TMS_I(SWDIO_TMS_I),
                    .SWO_TDO_I(SWO_TDO_I),
                    .TDI_O(TDI_O),
                    // input RTCK_I,
                    .SRST_I(SRST_I),
                    .SRST_O(SRST_O),
                    .TRST_I(TRST_I),
                    .TRST_O(TRST_O)
                );

    reg [7:0] buf64_reg [0:7]; // 64位串转并缓存
    wire [63:0] buf64 = {
             buf64_reg[7],
             buf64_reg[6],
             buf64_reg[5],
             buf64_reg[4],
             buf64_reg[3],
             buf64_reg[2],
             buf64_reg[1],
             buf64_reg[0]
         };

    reg buf64_start;
    reg [6:0] buf64_rbit_len; // 设定的读取位数量
    reg buf64_finish_reg;
    wire buf64_finish = buf64_finish_reg && !buf64_start;

    // 写入索引
    wire [3:0] buf64_write_index = buf64_start ? 4'd0 : buf64_rbyte_num;
    reg [3:0] buf64_rbyte_num;
    wire [3:0] buf64_rbyte_num_next = buf64_write_index + 1'd1;
    wire buf64_tready = buf64_finish_reg == 1'd0 || buf64_start;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            buf64_reg[7] <= 8'd0;
            buf64_reg[6] <= 8'd0;
            buf64_reg[5] <= 8'd0;
            buf64_reg[4] <= 8'd0;
            buf64_reg[3] <= 8'd0;
            buf64_reg[2] <= 8'd0;
            buf64_reg[1] <= 8'd0;
            buf64_reg[0] <= 8'd0;
            buf64_finish_reg <= 1'd1;
            buf64_rbyte_num <= 4'd0;
        end
        else begin
            if (buf64_start) begin
                buf64_finish_reg <= 1'd0;
                buf64_rbyte_num <= 1'd0;
            end

            if (buf64_finish_reg == 1'd0 || buf64_start) begin
                if (dap_in_tvalid) begin
                    buf64_reg[buf64_write_index] <= dap_in_tdata;
                    if (buf64_rbyte_num_next >= ((buf64_rbit_len + 7) >> 3)) begin
                        buf64_finish_reg <= 1'd1;
                    end
                    buf64_rbyte_num <= buf64_rbyte_num_next;
                end
            end
        end
    end

    reg [1:0] seq_status_reg;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            seq_status_reg <= 2'd0;
        end
        else if (start) begin
            case (seq_status_reg)
                2'd0, 2'd2: begin // 未启动状态，传输完成状态
                    if (seq_tx_valid) begin
                        seq_status_reg <= 2'd1;
                    end
                end
                2'd1: begin // 正在传输状态
                    if (seq_rx_valid) begin
                        seq_status_reg <= 2'd2;
                    end
                end
            endcase
        end
        else begin
            seq_status_reg <= 2'd0;
        end
    end

    reg [1:0] seq_status;
    always @(*) begin
        if (start) begin
            case (seq_status_reg) /*synthesis full_case*/
                2'd0, 2'd2:
                    seq_status = seq_tx_valid ? 2'd1 : seq_status_reg;
                2'd1:
                    seq_status = seq_rx_valid ? 2'd2 : seq_status_reg;
            endcase
        end else begin
            seq_status = 2'd0;
        end
    end

    reg [8:0] swj_seq_bit_num;
    reg [1:0] swj_seq_sm;
    wire [8:0] swj_seq_bit_num_next = (swj_seq_bit_num >= 9'd64 ? (swj_seq_bit_num - 9'd64) : 9'd0);

    reg swd_seq_start;
    reg [7:0] swd_seq_recv_count;
    reg [7:0] swd_seq_send_count;
    reg [7:0] swd_seq_info;
    reg [1:0] swd_seq_recv_sm;
    reg [1:0] swd_seq_send_sm;
    reg [6:0] swd_seq_send_data_byte;
    reg [63:0] swd_seq_send_data;
    wire [7:0] swd_seq_info_next;
    assign swd_seq_info_next[7] = dap_in_tdata[7];
    assign swd_seq_info_next[6:0] = (dap_in_tdata[5:0] == 0) ? 7'd64 : {1'd0, dap_in_tdata[5:0]};

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            seq_tx_cmd <= 16'd0;
            seq_tx_data <= 64'd0;
            seq_tx_valid <= 1'd0;
            ram_write_en <= 1'd0;
            ram_write_data <= 8'd0;
            ram_write_addr <= 10'd0;
            done <= 0;
            packet_len <= 1'd0;

            swj_seq_bit_num <= 9'd0;
            swj_seq_sm <= 2'd0;

            swd_seq_start <= 1'd0;
            swd_seq_recv_count <= 8'd0;
            swd_seq_send_count <= 8'd0;
            swd_seq_info <= 8'd0;
            swd_seq_recv_sm <= 2'd0;
            swd_seq_send_sm <= 2'd0;
            swd_seq_send_count <= 7'd0;
            swd_seq_send_data_byte <= 7'd0;
            swd_seq_send_data <= 64'd0;
        end
        else begin
            buf64_start <= 1'd0;
            seq_tx_valid <= 1'd0;
            ram_write_en <= 1'd0;



            if (start[`CMD_SWJ_SEQUENCE_SHIFT]) begin
                case (swj_seq_sm)
                    0: begin
                        if (dap_in_tvalid) begin
                            if (dap_in_tdata == 0) begin
                                swj_seq_bit_num <= 9'd256;
                                buf64_rbit_len <= 7'd64;
                            end
                            else begin
                                swj_seq_bit_num <= dap_in_tdata;
                                buf64_rbit_len <= dap_in_tdata > 8'd64 ? 7'd64 : dap_in_tdata[6:0];
                            end
                            buf64_start <= 1'd1;
                            swj_seq_sm <= 1;
                        end
                    end

                    1: begin
                        if (buf64_finish) begin
                            seq_tx_cmd <= {`SEQ_CMD_SWD_SEQ, 5'd0, buf64_rbit_len};
                            seq_tx_data <= buf64;
                            seq_tx_valid <= 1'd1;

                            swj_seq_bit_num <= swj_seq_bit_num_next;
                            if (swj_seq_bit_num_next > 9'd64) begin
                                buf64_rbit_len <= 7'd64;
                                buf64_start <= 1'd1;
                            end
                            else if (swj_seq_bit_num_next != 0) begin
                                buf64_rbit_len <= swj_seq_bit_num_next[6:0];
                                buf64_start <= 1'd1;
                            end

                            swj_seq_sm <= 2;
                        end
                    end
                    2: begin
                        if (seq_rx_valid) begin
                            if (swj_seq_bit_num == 0) begin
                                swj_seq_sm <= 3;
                                ram_write_addr <= 10'd0;
                                ram_write_data <= 8'd0;
                                packet_len <= 1'd1;
                                ram_write_en <= 1'd1;
                                done[`CMD_SWJ_SEQUENCE_SHIFT] <= 1'd1;
                            end
                            else begin
                                swj_seq_sm <= 1;
                            end
                        end
                    end
                endcase
            end
            else begin
                done[`CMD_SWJ_SEQUENCE_SHIFT] <= 1'd0;
                swj_seq_sm <= 2'd0;
            end


            if (start[`CMD_SWD_SEQUENCE_SHIFT]) begin
                if (!swd_seq_start) begin
                    if (dap_in_tvalid) begin
                        swd_seq_start <= 1'd1;
                        swd_seq_recv_count <= dap_in_tdata;
                        swd_seq_send_count <= dap_in_tdata;
                        // 不存在失败情况，提前写入状态
                        ram_write_addr <= 10'd0;
                        ram_write_data <= 8'd0;
                        ram_write_en <= 1'd1;
                    end
                end
                else begin
                    case (swd_seq_recv_sm)
                        0: begin
                            if (dap_in_tvalid) begin // 接收seq info段
                                swd_seq_info <= swd_seq_info_next;
                                if (swd_seq_info_next[7] == 1'd0) begin // output
                                    buf64_rbit_len <= swd_seq_info_next[6:0];
                                    buf64_start <= 1'd1;
                                    swd_seq_recv_sm <= 2'd1;
                                end
                                else if (seq_status != 2'd1) begin // trans空闲触发
                                    swd_seq_recv_count <= swd_seq_recv_count - 8'd1;
                                    seq_tx_cmd <= {`SEQ_CMD_SWD_SEQ, 4'd0, swd_seq_info_next};
                                    seq_tx_data <= buf64;
                                    seq_tx_valid <= 1'd1;

                                    if (swd_seq_recv_count - 8'd1 == 0) begin
                                        swd_seq_recv_sm <= 2'd3; // 最后一个
                                    end
                                end
                                else begin // trans忙等待
                                    swd_seq_recv_sm <= 2'd1;
                                end
                            end
                        end

                        1: begin // 接收数据段
                            if (buf64_finish && seq_status != 2'd1) begin
                                swd_seq_recv_count <= swd_seq_recv_count - 8'd1;
                                seq_tx_cmd <= {`SEQ_CMD_SWD_SEQ, 4'd0, swd_seq_info};
                                seq_tx_data <= buf64;
                                seq_tx_valid <= 1'd1;

                                if (swd_seq_recv_count - 8'd1 == 0) begin
                                    swd_seq_recv_sm <= 2'd3; // 最后一个
                                end
                                else begin
                                    swd_seq_recv_sm <= 2'd2;
                                end
                            end
                        end

                        2: begin // TODO: 高云USB FIFO有BUG，一个周期的ready低电平无效
                            swd_seq_recv_sm <= 2'd0;
                        end
                    endcase


                    case (swd_seq_send_sm)
                        0: begin
                            if (seq_rx_valid) begin
                                if (seq_rx_flag[7] == 1'd1) begin // 有数据段
                                    swd_seq_send_data_byte <= (seq_rx_flag[6:0] + 3'd7) >> 3;
                                    swd_seq_send_data <= seq_rx_data;
                                    swd_seq_send_sm <= 2'd1;
                                end
                                else begin
                                    swd_seq_send_count <= swd_seq_send_count - 8'd1;
                                end
                            end
                        end
                        1: begin
                            if (swd_seq_send_data_byte) begin
                                swd_seq_send_data_byte <= swd_seq_send_data_byte - 1;
                                ram_write_addr <= ram_write_addr + 10'd1;
                                ram_write_data <= swd_seq_send_data[7:0];
                                ram_write_en <= 1'd1;
                                swd_seq_send_data <= {8'd0, swd_seq_send_data[63:8]};
                            end
                            else begin
                                swd_seq_send_count <= swd_seq_send_count - 8'd1;
                                swd_seq_send_sm <= 2'd0;
                            end
                        end
                    endcase

                    if (swd_seq_recv_count == 0 && swd_seq_send_count == 0) begin
                        packet_len <= ram_write_addr + 10'd1;
                        done[`CMD_SWD_SEQUENCE_SHIFT] <= 1'd1;
                    end
                end
            end
            else begin
                done[`CMD_SWD_SEQUENCE_SHIFT] <= 1'd0;
                swd_seq_start <= 1'd0;
                swd_seq_recv_sm <= 2'd0;
                swd_seq_send_sm <= 2'd0;
            end

            if (!start || done) begin
                ram_write_addr <= 10'd0;
                ram_write_data <= 8'd0;
                ram_write_en <= 1'd0;
            end
        end
    end

    assign dap_in_tready[`CMD_SWJ_SEQUENCE_SHIFT] = ((swj_seq_sm == 2'd0) || (swj_seq_sm == 2'd1)) || buf64_tready;
    assign dap_in_tready[`CMD_SWD_SEQUENCE_SHIFT] = (swd_seq_recv_sm == 2'd0) || buf64_tready;







    task AHB_WRITE_REG32;
        output [31:0] optreg;
        begin
            if (ahb_byte_strobe[0])
                optreg[ 0+:8] = ahb_wdata[ 0+:8];
            if (ahb_byte_strobe[1])
                optreg[ 8+:8] = ahb_wdata[ 8+:8];
            if (ahb_byte_strobe[2])
                optreg[16+:8] = ahb_wdata[16+:8];
            if (ahb_byte_strobe[3])
                optreg[24+:8] = ahb_wdata[24+:8];
        end
    endtask
endmodule
