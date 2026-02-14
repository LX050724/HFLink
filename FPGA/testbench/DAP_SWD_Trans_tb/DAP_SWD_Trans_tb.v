`timescale 1 ns/ 10 ps
`include "DAP_Cmd.v"

`define DELAY_TIME 3

module DAP_SWD_Trans_tb();
    reg clk;
    reg clk_x2;
    reg resetn;
    integer i;

    reg [7:0] test_data [0:4095];

    initial begin
        $dumpfile("test.vcd");
        $dumpvars(0, DAP_SWD_Trans_tb);
        $readmemh("test_data.txt", test_data);

        clk = 0;
        clk_x2 = 0;
        resetn = 1;
        #2
         resetn = 0;
        #8;
        resetn = 1;

        forever begin
            clk_x2 <= 1;
            #8.333333;
            clk_x2 <= 0;
            #8.333333;
            clk <= !clk;
        end
    end



    reg ahb_write_en;
    reg ahb_read_en;
    reg [11:0] ahb_addr;
    reg [31:0] ahb_wdata;
    reg [3:0] ahb_byte_strobe;
    reg [`CMD_REAL_NUM-1:0] start;
    wire [`CMD_REAL_NUM-1:0] done;
    wire read_en = dap_in_tready & start ? 1'd1 : 1'd0;


    reg [31:0] cnt;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            cnt <= 0;
            ahb_write_en <= 0;
            ahb_read_en <= 0;
            ahb_addr <= 0;
            ahb_wdata <= 0;
            ahb_byte_strobe <= 0;
            start <= 0;
        end
        else begin
            cnt <= cnt + 1;
            ahb_write_en <= 0;
            ahb_read_en <= 0;

            case (cnt)
                0: begin
                    ahb_write_en <= 1;
                    ahb_addr <= 12'h004;
                    ahb_wdata <= 32'h0005_0004;
                    ahb_byte_strobe <= 4'hf;
                end
                1: begin
                    ahb_write_en <= 1;
                    ahb_addr <= 12'h000;
                    ahb_wdata <= 32'h0000_0001;
                    ahb_byte_strobe <= 4'hf;
                end
                2: begin
                    start[`CMD_TRANSFER_BLOCK_SHIFT] <= 1'd1;
                end
                3: begin
                    if (done[`CMD_TRANSFER_BLOCK_SHIFT]) begin
                        start[`CMD_TRANSFER_BLOCK_SHIFT] <= 1'd0;
                    end
                    else begin
                        cnt <= 3;
                    end
                end
            endcase

            if (cnt == 32'h0001_0000) begin
                $finish(1);
            end
        end
    end

    reg [11:0] ram_address;
    reg dap_in_tvalid;
    wire [`CMD_REAL_NUM-1:0] dap_in_tready;
    wire [7:0] dap_in_tdata = test_data[ram_address];
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            ram_address <= 0;
            dap_in_tvalid <= 0;
        end
        else begin
            dap_in_tvalid <= 1;
            if (read_en)
                ram_address <= ram_address + 1;
        end
    end


    wire sclk_negedge;
    wire sclk_sampling;
    wire sclk_out;
    DAP_BaudGenerator baud_inst(
                          .clk(clk),
                          .sclk_in(clk_x2),
                          .resetn(resetn),

                          .ahb_write_en(ahb_write_en),
                          .ahb_read_en(ahb_read_en),
                          .ahb_addr(ahb_addr),
                          .ahb_rdata(),
                          .ahb_wdata(ahb_wdata),
                          .ahb_byte_strobe(ahb_byte_strobe),

                          .sclk_out(sclk_out),
                          .sclk_negedge(sclk_negedge),
                          .sclk_sampling(sclk_sampling)
                      );

    wire SWCLK_TCK_O;
    wire SWDIO_TMS_O;
    wire SWDIO_TMS_T;
    reg SWDIO_TMS_I;
    wire [9:0] ram_write_addr;
    wire [7:0] ram_write_data;
    wire ram_write_en;
    wire [9:0] packet_len;

    DAP_SWJ #(12, 12'h80)
            inst(
                .clk(clk),
                .resetn(resetn),
                .us_tick(),
                .us_timer(),
                .enable(1'd1),

                //串行时钟
                .sclk(clk_x2),
                .sclk_out(sclk_out),
                .sclk_negedge(sclk_negedge),
                .sclk_sampling(sclk_sampling),

                //AHBMEM接口
                .ahb_write_en(ahb_write_en),
                .ahb_read_en(ahb_read_en),
                .ahb_addr(ahb_addr),
                .ahb_rdata(),
                .ahb_wdata(ahb_wdata),
                .ahb_byte_strobe(ahb_byte_strobe),

                .dap_in_tvalid(dap_in_tvalid),
                .dap_in_tready(dap_in_tready[`CMD_SWJ_RANGE]),
                .dap_in_tdata(dap_in_tdata),

                .ram_write_addr(ram_write_addr),
                .ram_write_data(ram_write_data),
                .ram_write_en(ram_write_en),
                .packet_len(packet_len),

                .start(start[`CMD_SWJ_RANGE]),
                .done(done[`CMD_SWJ_RANGE]),

                .SWCLK_TCK_O(SWCLK_TCK_O),
                .SWDIO_TMS_T(SWDIO_TMS_T),
                .SWDIO_TMS_O(SWDIO_TMS_O),
                .SWDIO_TMS_I(SWDIO_TMS_I),
                .SWO_TDO_I(),
                .TDI_O(),
                //.RTCK_I,
                .SRST_I(),
                .SRST_O(),
                .TRST_I(),
                .TRST_O(),

                //swd模式信号
                .SWD_MODE()
            );


    reg [7:0] output_data [0:1023];

    always @(posedge clk) begin
        if (ram_write_en) begin
            output_data[ram_write_addr] <= ram_write_data;
            $display("write %08x %02x", ram_write_addr, ram_write_data);
        end

        if (done & start) begin
            #1;
            $display("packet_len: %d", packet_len);
            for (i = 0; i < packet_len; i = i + 1) begin
                $display("%08x: %02x", i, output_data[i]);
            end
            $finish(1);
        end
    end

    reg [7:0] swd_sm;
    reg APnDP;
    reg RnW;
    reg [3:0] Addr;
    reg Parity;
    reg [31:0] data;
    reg [7:0] data_cnt;
    reg DataParity;

    initial begin
        SWDIO_TMS_I = 0;
        swd_sm = 0;
        APnDP = 0;
        RnW = 0;
        Addr = 0;
        Parity = 0;
        data = 0;
        DataParity = 0;
    end

    wire swdio_i = SWDIO_TMS_T ? 1'd1 : SWDIO_TMS_O;

    always @(posedge SWCLK_TCK_O) begin
        case (swd_sm)
            0: begin
                DataParity <= 0;
                if (swdio_i) begin
                    swd_sm <= swd_sm + 1;
                end
            end
            1: begin
                APnDP <= swdio_i;
                swd_sm <= swd_sm + 1;
            end
            2: begin
                RnW <= swdio_i;
                swd_sm <= swd_sm + 1;
            end
            3: begin
                Addr[2] <= swdio_i;
                swd_sm <= swd_sm + 1;
            end
            4: begin
                Addr[3] <= swdio_i;
                swd_sm <= swd_sm + 1;
            end
            5: begin
                Parity <= swdio_i;
                swd_sm <= swd_sm + 1;
            end
            6, 7, 8: begin // stop, park, tm
                swd_sm <= swd_sm + 1;
            end
            9: begin // ack 0
                SWDIO_TMS_I <= #`DELAY_TIME 1'd1;
                swd_sm <= swd_sm + 1;
            end
            10: begin // ack 1
                SWDIO_TMS_I <= #`DELAY_TIME 1'd0;
                swd_sm <= swd_sm + 1;
            end
            11: begin // ack 2
                SWDIO_TMS_I <= #`DELAY_TIME 1'd0;
                data_cnt <= 0;
                swd_sm <= RnW ? 13 : 12;
            end
            12: begin // turn
                SWDIO_TMS_I <= #`DELAY_TIME 1'd0;
                swd_sm <= swd_sm + 1;
            end
            13: begin // data
                data_cnt <= data_cnt + 1;
                if (data_cnt == 31) begin
                    swd_sm <= 14;
                end
                if (RnW) begin
                    SWDIO_TMS_I <= #`DELAY_TIME data[data_cnt];
                end
                else begin
                    data[data_cnt] <= SWDIO_TMS_O;
                end
            end
            14: begin // p
                DataParity = (parity_32(data) == swdio_i);
                swd_sm <= 0;
            end
        endcase
    end

    function parity_32;
        input [31:0] data;
        begin
            parity_32 =
                data[ 0] ^ data[ 1] ^ data[ 2] ^ data[ 3] ^ data[ 4] ^ data[ 5] ^ data[ 6] ^ data[ 7] ^
                data[ 8] ^ data[ 9] ^ data[10] ^ data[11] ^ data[12] ^ data[13] ^ data[14] ^ data[15] ^
                data[16] ^ data[17] ^ data[18] ^ data[19] ^ data[20] ^ data[21] ^ data[22] ^ data[23] ^
                data[24] ^ data[25] ^ data[26] ^ data[27] ^ data[28] ^ data[29] ^ data[30] ^ data[31];
        end
    endfunction
endmodule

