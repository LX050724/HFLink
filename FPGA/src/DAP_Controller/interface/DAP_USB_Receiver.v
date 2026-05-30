`timescale 1 ns / 10 ps

module DAP_USB_Receiver #(
    parameter [3:0] P_ENDPOINT = 4'd2,
    parameter integer FIFO_DEPTH = 4096,
    parameter integer FIFO_BUFFER_LEN = 512
) (
    input clk,
    input resetn,

    input clear_data, // 清空FIFO，丢弃当前数据（传输中时延迟到传输完成后执行）
    output reg clear_done, // 清空完成信号（电平握手：清除后拉高，clear_data 撤销后拉低）


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

    localparam FIFO_ADDR_LEN = $clog2(FIFO_DEPTH);

    wire ep_selected = (P_ENDPOINT == usb_endpt);
    wire usb_rx_valid = usb_rxval & ep_selected;
    reg [7:0] ram[0:FIFO_DEPTH-1];

    reg [FIFO_ADDR_LEN:0] fifo_wptr;
    reg [FIFO_ADDR_LEN:0] fifo_wptr_shadow;
    reg [FIFO_ADDR_LEN:0] fifo_rptr;
    wire [FIFO_ADDR_LEN:0] fifo_rptr_next = (axis_tready && axis_tvaild) ? fifo_rptr + 1'd1 : fifo_rptr;


    wire [FIFO_ADDR_LEN:0] fifo_ext_wptr = fifo_wptr + FIFO_BUFFER_LEN;
    assign fifo_full = (fifo_ext_wptr[FIFO_ADDR_LEN] ^ fifo_rptr[FIFO_ADDR_LEN]) && (fifo_ext_wptr[FIFO_ADDR_LEN-1:0] >= fifo_rptr[FIFO_ADDR_LEN-1:0]);
    assign fifo_empty = (fifo_wptr == fifo_rptr);
    assign usb_rxrdy = !fifo_full && !clear_done;



    assign axis_tvaild = !fifo_empty;
    wire fifo_read_en = axis_tready && axis_tvaild;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            fifo_wptr <= {(FIFO_ADDR_LEN + 1) {1'd0}};
            fifo_wptr_shadow <= {(FIFO_ADDR_LEN + 1) {1'd0}};
            fifo_rptr <= {(FIFO_ADDR_LEN + 1) {1'd0}};
            axis_tdata <= 8'd0;
            clear_done <= 1'b0;
        end else begin
            if (ep_selected && usb_rxpktval) begin
                fifo_wptr <= fifo_wptr_shadow;
            end

            if (usb_rx_valid && !fifo_full) begin
                ram[fifo_wptr_shadow[FIFO_ADDR_LEN-1:0]] <= usb_rxdat;
                fifo_wptr_shadow <= fifo_wptr_shadow + 1'd1;
            end

            if (fifo_read_en) begin
                fifo_rptr  <= fifo_rptr_next;
                axis_tdata <= ram[fifo_rptr_next[FIFO_ADDR_LEN-1:0]];
            end else begin
                axis_tdata <= ram[fifo_rptr[FIFO_ADDR_LEN-1:0]];
            end

            if (clear_data) begin
                clear_done <= clear_done | ~ep_selected;
                fifo_wptr <= {(FIFO_ADDR_LEN + 1) {1'd0}};
                fifo_wptr_shadow <= {(FIFO_ADDR_LEN + 1) {1'd0}};
                fifo_rptr <= {(FIFO_ADDR_LEN + 1) {1'd0}};
            end else begin
                clear_done <= 1'b0;
                if (!ep_selected) begin
                    fifo_wptr_shadow <= fifo_wptr;
                end
            end
        end
    end


endmodule
