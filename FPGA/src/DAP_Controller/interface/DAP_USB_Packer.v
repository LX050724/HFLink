

module DAP_USB_Packer#(
        parameter [3:0] P_ENDPOINT = 1,
        parameter [3:0] MAX_PACKET_NUM = 8
    )(
        input clk,
        input resetn,

        // 数据包输入
        input [9:0] ram_write_addr, // 数据包相对地址，0起
        input [7:0] ram_write_data,
        input ram_write_en,
        input [9:0] packet_len, //
        input packet_finish, // 整包完成，触发发送，发送前必须通过group_finish更新所有数据
        input group_finish, // 分组完成，更新头指针位置并累加包总长
        output fifo_full,

        // USB
        input [3:0] usb_endpt,
        input usb_txact,
        input usb_txpop,
        input usb_txpktfin,
        output usb_txcork,
        output reg [7:0] usb_txdata,
        output [11:0] usb_txlen
    );
    reg [7:0] ram [0:4095];

    reg [8:0] txlen_queue [0:7];
    reg [3:0] wptr;
    reg [3:0] rptr;

    reg [8:0] total_packet_len; // 包总长
    reg [8:0] waddr_offset;     // 写偏移地址
    wire [8:0] waddr = waddr_offset + ram_write_addr; // 实际写入地址

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            waddr_offset <= 9'd0;
            total_packet_len <= 9'd0;
            wptr <= 4'd0;
        end
        else begin
            if (ram_write_en && !fifo_full) begin
                // 数据写入到当前扇区
                ram[{wptr[2:0], waddr}] <= ram_write_data;
            end

            if (packet_finish) begin
                // 总包打包完成复位变量
                waddr_offset <= 9'd0;
                total_packet_len <= 9'd0;
                // 移动扇区
                wptr <= wptr + 1'd1; 
                // 存储包长到队列
                txlen_queue[wptr[2:0]] <= total_packet_len;
            end
            else if (group_finish) begin
                // 分包打包完成计算偏移
                total_packet_len <= total_packet_len + packet_len;
                waddr_offset <= waddr_offset + packet_len;
            end

        end
    end

    assign fifo_full = (wptr[3] ^ rptr[3]) && (wptr[2:0] >= rptr[2:0]);
    wire fifo_empty = (wptr == rptr);

    wire usb_ep_select = (usb_endpt == P_ENDPOINT);
    wire ram_read_en = (usb_ep_select && !fifo_empty);

    assign usb_txlen = txlen_queue[rptr[2:0]];
    assign usb_txcork = ~ram_read_en;

    reg [8:0] ram_read_addr;
    wire [8:0] ram_read_addr_next = ram_read_addr + usb_txpop;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            rptr <= 4'd0;
            ram_read_addr <= 9'd0;
            usb_txdata <= 8'd0;
        end
        else begin

            if (ram_read_en) begin
                ram_read_addr <= ram_read_addr_next;
                usb_txdata <= ram[{rptr[2:0], ram_read_addr_next}];
            end

            if (!(usb_ep_select && usb_txact)) begin
                ram_read_addr <= 9'd0;
            end

            if (usb_ep_select && usb_txpktfin) begin
                rptr <= rptr + 1'd1;
            end
        end
    end

endmodule
