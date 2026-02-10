module DAP_USB_Receiver #(
        parameter [3:0] P_ENDPOINT = 4'd2
    ) (
        input clk,
        input resetn,

        input [3:0] usb_endpt,
        input usb_rxval,
        input [7:0] usb_rxdat,
        input usb_rxpktval,
        input usb_rxact,
        output usb_rxrdy,

        output fifo_full,
        output fifo_empty,

        output reg [7:0] axis_tdata,
        output axis_tvaild,
        input axis_tready
    );
    wire ep_selected = (P_ENDPOINT == usb_endpt);
    wire usb_rx_active = usb_rxact & ep_selected;
    wire usb_rx_valid = usb_rxval & ep_selected;
    reg [7:0] ram [0:4095];

    reg [12:0] fifo_wptr;
    reg [12:0] fifo_wptr_tmp;
    reg [12:0] fifo_rptr;
    wire [12:0] fifo_rptr_next = fifo_rptr + 1'd1;


    wire [12:0] fifo_ext_wptr = fifo_wptr + 13'd512;
    assign fifo_full = (fifo_ext_wptr[12] ^ fifo_rptr[12]) && (fifo_ext_wptr[11:0] >= fifo_rptr[11:0]);
    assign fifo_empty = (fifo_wptr == fifo_rptr);
    assign usb_rxrdy = !fifo_full;
    
    reg usb_rx_active_store;
    
    wire fifo_write_en = usb_rx_valid;

    assign axis_tvaild = !fifo_empty;
    wire fifo_read_en = axis_tready && axis_tvaild;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            fifo_wptr <= 12'd0;
            fifo_wptr_tmp <= 12'd0;
            fifo_rptr <= 12'd0;
            usb_rx_active_store <= 1'd0;
            axis_tdata <= 8'd0;
        end
        else begin
            usb_rx_active_store <= usb_rx_active;

            if (usb_rx_active && !usb_rx_active_store) begin
                fifo_wptr_tmp <= fifo_wptr;
            end

            if (ep_selected && usb_rxpktval) begin
                fifo_wptr <= fifo_wptr_tmp;
            end

            if (usb_rx_valid) begin
                ram[fifo_wptr_tmp[11:0]] <= usb_rxdat;
                fifo_wptr_tmp <= fifo_wptr_tmp + 1'd1;
            end

            if (fifo_read_en) begin
                fifo_rptr <= fifo_rptr_next;
                axis_tdata <= ram[fifo_rptr_next[11:0]];
            end else begin
                axis_tdata <= ram[fifo_rptr[11:0]];
            end

        end
    end


endmodule
