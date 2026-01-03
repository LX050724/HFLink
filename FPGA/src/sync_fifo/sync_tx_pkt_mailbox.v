

module sync_tx_pkt_mailbox#(
        parameter [3:0] P_ENDPOINT = 1,
        parameter [3:0] MAX_PACKET_NUM = 8
    )(
        input clk,
        input resetn,

        // 输入
        input [7:0] i_tdata,
        input [11:0] i_tlen,
        input i_tvalid,
        output i_tready,

        // USB
        input [3:0] usb_endpt,
        input usb_txact,
        input usb_txpop,
        input usb_txpktfin,
        output usb_txcork,
        output [7:0] usb_txdata,
        output [11:0] usb_txlen
    );

    integer i;

    wire ram_write_en;
    wire [11:0] ram_waddr;
    wire [7:0] ram_wdata;

    wire ram_read_en;
    wire [11:0] ram_raddr;
    reg [7:0] ram_radata;

    reg [7:0] ram [0:4095];

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
        end else begin
            if (ram_write_en) begin
                ram[ram_waddr] <= ram_wdata;
            end
        end
    end

    reg [11:0] write_addr;
    reg i_tvalid_store;
    wire input_done = (i_tvalid == 1'd0) && (i_tvalid_store == 1'd1);
    assign ram_write_en = i_tvalid;
    assign ram_waddr = write_addr;
    assign ram_wdata = i_tdata;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            write_addr <= 12'd0;
            i_tvalid_store <= 1'd0;
        end
        else begin
            i_tvalid_store <= i_tvalid;
            if (i_tvalid) begin
                write_addr <= write_addr + 12'd1;
            end
            if (input_done) begin
                // 地址对齐
                write_addr <= {write_addr[11:4] + 12'd1, 4'd0};
            end
        end
    end


    reg [3:0] pack_queue_size;
    reg [11:0] pack_queue [0:MAX_PACKET_NUM-1];
    reg [11:0] read_addr;
    reg [11:0] read_addr_start;
    reg read_en;
    reg usb_tx_active_store;
    reg usb_txpktfin_store;
    
    wire usb_ep_select = (usb_endpt == P_ENDPOINT);

    assign ram_read_en = (usb_ep_select ? (pack_queue_size != 4'd0) : 1'd0);
    assign ram_raddr = read_addr;
    assign usb_txdata = ram_radata;
    assign usb_txlen = usb_ep_select ? pack_queue[0] : 12'd0;
    assign usb_txcork = ~ram_read_en;

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
            end

            // 发送结束计算地址
            if (usb_tx_active_store == 1'd1 && usb_tx_active == 1'd0) begin
                if (usb_txpktfin_store) begin
                    // 发送成功更新地址
                    read_addr_start <= {read_addr[11:4] + 1'd1, 4'd0};
                    read_addr <= {read_addr[11:4] + 1'd1, 4'd0};
                end else begin
                    // 发送失败还原地址
                    read_addr <= read_addr_start;
                end
            end

            // 更新队列
            case ({input_done, usb_tx_success})
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
                    pack_queue[pack_queue_size] <= i_tlen;
                end
                2'b11: begin // 同时弹出压入只移位队列
                    for (i = 0; i < (MAX_PACKET_NUM - 1); i=i+1) begin : shift_loop_2
                        pack_queue[i] <= pack_queue[i+1];
                    end
                    pack_queue[MAX_PACKET_NUM-1] <= 12'd0;
                    pack_queue[pack_queue_size] <= i_tlen;
                end
            endcase
        end
    end

endmodule
