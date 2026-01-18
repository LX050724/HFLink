

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
        input packet_finish, // 整包完成，触发发送(高优先)
        input group_finish, // 分组完成，更新头指针位置并累加包总长
        output almost_full,

        // USB
        input [3:0] usb_endpt,
        input usb_txact,
        input usb_txpop,
        input usb_txpktfin,
        output usb_txcork,
        output [7:0] usb_txdata,
        output [11:0] usb_txlen
    );
    reg [7:0] ram [0:4095];
    integer i;


    wire ram_read_en;
    reg [7:0] ram_radata;

    reg [11:0] packet_head_addr; // 包头部地址
    reg [9:0] packet_total_len;
    wire [11:0] packet_tail_addr = packet_head_addr + packet_total_len + packet_len; // 计算的包末尾地址

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            packet_head_addr <= 12'd0;
        end
        else begin
            if (ram_write_en) begin
                // 写入数据
                ram[packet_head_addr + ram_write_addr] <= ram_write_data;
            end
            if (packet_finish) begin
                packet_total_len <= 10'd0;
                packet_head_addr <= {packet_tail_addr[11:4] + 12'd1, 4'd0};
            end 
            else if (group_finish) begin
                packet_total_len <= packet_total_len + packet_len;
                packet_head_addr <= packet_head_addr + packet_len;
            end
        end
    end

    reg [3:0] pack_queue_size;
    reg [9:0] pack_queue [0:MAX_PACKET_NUM-1];
    reg [11:0] read_addr;
    reg [11:0] read_addr_start;
    reg read_en;
    reg usb_tx_active_store;
    reg usb_txpktfin_store;

    wire usb_ep_select = (usb_endpt == P_ENDPOINT);

    assign ram_read_en = (usb_ep_select ? (pack_queue_size != 4'd0) : 1'd0);
    assign usb_txdata = ram_radata;
    assign usb_txlen = usb_ep_select ? pack_queue[0] : 12'd0;
    assign usb_txcork = ~ram_read_en;
    assign almost_full = pack_queue_size >= (MAX_PACKET_NUM - 1);

    wire usb_tx_active = (usb_txcork == 1'd0) && usb_txact;
    wire usb_tx_success = usb_tx_active_store & !usb_tx_active & usb_txpktfin_store;
    wire [11:0] next_read_addr = usb_txpop ? (read_addr + 1'd1) : read_addr;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            pack_queue_size <= 4'd0;
            read_addr <= 12'd0;
            read_addr_start <= 12'd0;
            usb_tx_active_store <= 1'd0;
            usb_txpktfin_store <= 1'd0;
            ram_radata <= 8'd0;
        end
        else begin
            usb_tx_active_store <= usb_tx_active;

            // 发送中
            if (usb_tx_active) begin
                read_addr <= next_read_addr;
                ram_radata <= ram[next_read_addr];

                if (usb_txpktfin)
                    usb_txpktfin_store <= 1'd1;
            end else begin
                ram_radata <= ram[read_addr];
            end

            // 发送结束计算地址
            if (usb_tx_active_store == 1'd1 && usb_tx_active == 1'd0) begin
                if (usb_txpktfin_store) begin
                    // 发送成功更新地址
                    read_addr_start <= {read_addr[11:4] + 1'd1, 4'd0};
                    read_addr <= {read_addr[11:4] + 1'd1, 4'd0};
                end
                else begin
                    // 发送失败还原地址
                    read_addr <= read_addr_start;
                end
            end

            // 更新队列
            case ({packet_finish, usb_tx_success})
                2'b00: begin // 无事发生
                end
                2'b01: begin // 弹出
                    pack_queue_size <= pack_queue_size - 1;
                    for (i = 0; i < (MAX_PACKET_NUM - 1); i=i+1) begin : shift_loop_1
                        pack_queue[i] <= pack_queue[i+1];
                    end
                    pack_queue[MAX_PACKET_NUM-1] <= 12'd0;
                end
                2'b10: begin // 压入
                    pack_queue_size <= pack_queue_size + 1;
                    pack_queue[pack_queue_size] <= packet_total_len + packet_len;
                end
                2'b11: begin // 同时弹出压入只移位队列
                    for (i = 0; i < (MAX_PACKET_NUM - 1); i=i+1) begin : shift_loop_2
                        pack_queue[i] <= pack_queue[i+1];
                    end
                    pack_queue[MAX_PACKET_NUM-1] <= 12'd0;
                    pack_queue[pack_queue_size] <= packet_total_len + packet_len;
                end
            endcase
        end
    end

endmodule
