module DAP_USB_Transfer #(
        parameter [3:0] P_ENDPOINT = 4'd2,
        parameter FIFO_DEPTH = 4096,
        parameter [11:0] FIFO_BUFFER_LEN = 512
    ) (
        input clk,
        input resetn,

        input clear_data, // 清空FIFO，丢弃当前数据（传输中时延迟到传输完成后执行）
        output reg clear_done, // 清空完成信号（电平握手：清除后拉高，clear_data 撤销后拉低）

        input [7:0] in_tdata,
        input in_tvalid,
        output in_tready,

        output [$clog2(FIFO_DEPTH):0] fifo_size,

        // USB
        input [3:0] usb_endpt,
        input usb_txact,
        input usb_txpop,
        input usb_txpktfin,
        output usb_txcork,
        output reg [7:0] usb_txdata,
        output reg [11:0] usb_txlen
    );

    localparam FIFO_ADDR_LEN = $clog2(FIFO_DEPTH);

    reg [7:0] ram [0:FIFO_DEPTH-1];

    reg [FIFO_ADDR_LEN:0] wptr;
    reg [FIFO_ADDR_LEN:0] rptr;
    reg [FIFO_ADDR_LEN:0] rptr_shadow;
    wire [FIFO_ADDR_LEN:0] rptr_shadow_next = rptr_shadow + usb_txpop;

    // FIFO 空满检测
    wire fifo_empty = (wptr == rptr);
    wire fifo_full  = (wptr[FIFO_ADDR_LEN] != rptr[FIFO_ADDR_LEN]) &&
                      (wptr[FIFO_ADDR_LEN-1:0] == rptr[FIFO_ADDR_LEN-1:0]);

    assign in_tready = !fifo_full;
    assign fifo_size = (wptr - rptr);

    wire usb_ep_select = (usb_endpt == P_ENDPOINT);
    wire ram_read_en = (usb_ep_select && !fifo_empty);
    assign usb_txcork = (~ram_read_en) || clear_done; // 当没有数据可读时，保持cork状态

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            wptr        <= {(FIFO_ADDR_LEN + 1) {1'd0}};
            rptr        <= {(FIFO_ADDR_LEN + 1) {1'd0}};
            rptr_shadow <= {(FIFO_ADDR_LEN + 1) {1'd0}};
            usb_txdata  <= 8'd0;
            usb_txlen   <= 12'd0;
            clear_done  <= 1'b0;
        end
        else begin
            // ── AXIS 写入 ──
            if (in_tvalid && in_tready && !clear_data) begin
                ram[wptr[FIFO_ADDR_LEN-1:0]] <= in_tdata;
                wptr <= wptr + 1'd1;
            end

            // ── USB 读取 ──
            if (usb_ep_select && !fifo_empty) begin
                rptr_shadow <= rptr_shadow_next;
                usb_txdata  <= ram[rptr_shadow_next[FIFO_ADDR_LEN-1:0]];
            end

            if (!(usb_ep_select && usb_txact)) begin
                rptr_shadow <= rptr;
            end

            if (usb_ep_select && usb_txpktfin) begin
                rptr <= rptr_shadow;
            end

            if (!usb_ep_select) begin
                usb_txlen <= (fifo_size > FIFO_BUFFER_LEN) ? FIFO_BUFFER_LEN : fifo_size;

                // 端点未选中时：更新 txlen，处理清除
                if (clear_data) begin
                    clear_done  <= 1'b1;
                    wptr        <= {(FIFO_ADDR_LEN + 1) {1'd0}};
                    rptr        <= {(FIFO_ADDR_LEN + 1) {1'd0}};
                    rptr_shadow <= {(FIFO_ADDR_LEN + 1) {1'd0}};
                end
                else begin
                    clear_done <= 1'b0;
                end
            end
        end
    end

endmodule
