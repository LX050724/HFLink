module DAP_Delay_Worker(
        input  hclk,       // clock
        input  us_tick,
        input en,
        input start,
        
        input wire dap_in_tvalid,
        output wire dap_in_tready,
        input wire [7:0] dap_in_tdata,

        output wire dap_out_tvalid,
        output wire [7:0] dap_out_tdata,

        output done
    );


    reg [15:0] delay_time;
    reg delay_rx_tready;
    reg delay_tx_tvalid;
    reg delay_tx_tdata;
    reg [1:0] delay_sm;
    always @(posedge hclk) begin
        if (!en) begin
            delay_time <= 1'd0;
            delay_sm <= 2'd0;
            delay_tx_tvalid <= 1'd0;
            delay_tx_tdata <= 8'd0;
        end
        else begin
            if (start) begin
                case (delay_sm)
                    2'd0: // 收1
                        if (dap_in_tvalid) begin
                            delay_time[7:0] <= dap_in_tdata;
                            delay_sm <= 2'd1;
                        end
                    2'd1: // 收2
                        if (dap_in_tvalid) begin
                            delay_time[15:8] <= dap_in_tdata;
                            delay_sm <= 2'd2;
                        end
                    2'd2: // delay
                        if (us_tick) begin
                            if (delay_time != 16'd0) begin
                                delay_time <= delay_time - 16'd1;
                            end
                            else begin
                                // 发OK
                                delay_sm <= 2'd3;
                                delay_tx_tvalid <= 1'd1;
                                delay_tx_tdata <= 8'h00;
                            end
                        end
                    2'd3: begin
                        delay_tx_tvalid <=1'd0;
                        delay_tx_tdata <= 8'h00;
                    end
                endcase
            end
            else begin
                // 复位
                delay_sm <= 2'd0;
            end
        end
    end

    assign done = (delay_sm == 2'd3);
    assign dap_in_tready = en & (delay_sm == 2'd0 || delay_sm == 2'd1);
    assign dap_out_tvalid = delay_tx_tvalid;
    assign dap_out_tdata = delay_tx_tdata;
endmodule
