`timescale 1 ns/ 10 ps
`include "DAP_Cmd.v"

module DAP_SWD_Trans_tb();
    reg clk;
    reg clk_x2;
    reg resetn;

    initial begin
        $dumpfile("test.vcd");
        $dumpvars(0, DAP_SWD_Trans_tb);

        clk = 0;
        clk_x2 = 0;
        resetn = 1;
        #2
         resetn = 0;
        #8;
        resetn = 1;

        forever begin
            clk_x2 <= 1;
            #1;
            clk_x2 <= 0;
            #1;
            clk <= !clk;
        end
    end


    reg ahb_write_en;
    reg ahb_read_en;
    reg [11:0] ahb_addr;
    wire [31:0] ahb_rdata;
    reg [31:0] ahb_wdata;
    reg [3:0] ahb_byte_strobe;

    reg seq_tx_valid;
    reg [15:0] seq_tx_cmd;
    reg [63:0] seq_tx_data;
    wire seq_tx_full;
    wire seq_rx_valid;
    wire [15:0] seq_rx_flag;
    wire [63:0] seq_rx_data;

    reg [31:0] cnt;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            cnt <= 0;
            ahb_write_en <= 0;
            ahb_read_en <= 0;
            ahb_addr <= 0;
            ahb_wdata <= 0;
            ahb_byte_strobe <= 0;
            seq_tx_valid <= 0;
            seq_tx_cmd <= 0;
            seq_tx_data <= 0;
        end
        else begin
            cnt <= cnt + 1;
            seq_tx_valid <= 0;
            ahb_write_en <= 0;
            ahb_read_en <= 0;

            case (cnt)
                0: begin
                    ahb_write_en <= 1;
                    ahb_addr <= 12'h004;
                    ahb_wdata <= 32'h0003_0000;
                    ahb_byte_strobe <= 4'hf;
                end
                1: begin
                    ahb_write_en <= 1;
                    ahb_addr <= 12'h000;
                    ahb_wdata <= 32'h0000_0001;
                    ahb_byte_strobe <= 4'hf;
                end

                10: begin
                    seq_tx_valid <= 1;
                    seq_tx_cmd <= {`SEQ_CMD_SWD_SEQ, 5'd0, 7'd64};
                    seq_tx_data <= 64'h123456789abcdef;
                end
            endcase

            if (seq_rx_valid || cnt == 32'h0001_0000) begin
                $finish(1);
            end
        end
    end


    wire sclk_pulse;
    wire sclk_delay_pulse;
    wire sclk_out;
    DAP_BaudGenerator baud_inst(
                          .clk(clk),
                          .sclk_in(clk_x2),
                          .resetn(resetn),

                          .ahb_write_en(ahb_write_en),
                          .ahb_read_en(ahb_read_en),
                          .ahb_addr(ahb_addr),
                          .ahb_rdata(ahb_rdata),
                          .ahb_wdata(ahb_wdata),
                          .ahb_byte_strobe(ahb_byte_strobe),

                          .sclk_out(sclk_out),
                          .sclk_pulse(sclk_pulse),
                          .sclk_delay_pulse(sclk_delay_pulse)
                      );


    DAP_Seqence inst(
                    // 控制器时钟
                    .clk(clk),
                    .resetn(resetn),

                    // 串行时钟
                    .sclk(clk_x2),
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
                    .SWCLK_TCK_O(),
                    .SWDIO_TMS_T(),
                    .SWDIO_TMS_O(),
                    .SWDIO_TMS_I(),
                    .SWO_TDO_I(),
                    .TDI_O(),
                    // input RTCK_I,
                    .SRST_I(),
                    .SRST_O(),
                    .TRST_I(),
                    .TRST_O()
                );

endmodule

