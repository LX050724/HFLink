`timescale 1 ns/ 10 ps

module DAP_USB_Reveiver_tb();
    reg clk;
    reg resetn;
    reg [3:0] endpt_sel;
    reg usb_rxact;
    reg [7:0] usb_rxdat;
    reg usb_rxval;
    reg usb_rxpktval;
    reg axis_tready;
    wire fifo_full;
    wire fifo_empty;
    wire axis_tvaild;
    wire [7:0] axis_tdata;
    integer i;

    initial begin
        $dumpfile("test.vcd");
        $dumpvars(0, DAP_USB_Reveiver_tb);

        clk = 0;
        resetn = 1;
        #2
         resetn = 0;
        #8;
        resetn = 1;

        forever begin
            clk <= !clk;
            #1;
        end
    end


    reg [5:0] cnt;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            cnt <= 0;
            usb_rxpktval <= 0;
            endpt_sel <= 0;
            usb_rxact <= 0;
            usb_rxdat <= 8'hxx;
            usb_rxval <= 0;
            axis_tready <= 0;
        end
        else begin
            usb_rxpktval <= 0;
            // axis_tready <= 0;
            cnt <= cnt + 1;
            casez (cnt)
                4, 5, 6, 7: begin
                    endpt_sel <= 2;
                end
                6'b001???: begin
                    usb_rxact <= 1;
                end
                6'b01????: begin
                    usb_rxdat <= cnt - 16;
                    usb_rxval <= 1;
                    $display("write data: %d", cnt - 16);
                end
                32: begin
                    usb_rxdat <= 8'hxx;
                    usb_rxval <= 0;
                end
                37:
                    usb_rxpktval <= 1;
                38:
                    usb_rxact <= 0;
                
                40: axis_tready <= 1;
                44: axis_tready <= 0;
                45: axis_tready <= 1;
                46: axis_tready <= 0;
                49: axis_tready <= 1;

            endcase

            if (axis_tready && axis_tvaild) begin
                $display("read data: %d", axis_tdata);
            end


            if (fifo_full) begin
                $finish(1);
            end

            if (cnt == 6'b111111)
                $finish(1);
        end
    end

    DAP_USB_Receiver inst(
                 .clk(clk),
                 .resetn(resetn),

                 .usb_endpt(endpt_sel),
                 .usb_rxval(usb_rxval),
                 .usb_rxdat(usb_rxdat),
                 .usb_rxpktval(usb_rxpktval),
                 .usb_rxact(usb_rxact),
                 .usb_rxrdy(usb_rxrdy),

                 .fifo_full(fifo_full),
                 .fifo_empty(fifo_empty),
                 
                 .axis_tready(axis_tready),
                 .axis_tvaild(axis_tvaild),
                 .axis_tdata(axis_tdata)

             );



endmodule

