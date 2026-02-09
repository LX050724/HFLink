`include "DAP_Cmd.v"

module DAP_Controller #(
        parameter ADDRWIDTH = 12,
        parameter [5:0] CLOCK_FREQ_M = 60,
        parameter GPIO_NUM = 4
    ) (
        input  wire                  dap_clk,
        input  wire                  hclk,       // clock
        input  wire                  hresetn,    // reset

        // AHB connection to master
        input  wire                  hsels,
        input  wire [ADDRWIDTH-1:0]  haddrs,
        input  wire [1:0]            htranss,
        input  wire [2:0]            hsizes,
        input  wire                  hwrites,
        input  wire                  hreadys,
        input  wire [31:0]           hwdatas,

        output reg                   hreadyouts,
        output wire                  hresps,
        output reg [31:0]            hrdatas,

        output wire intr,

        input [3:0] usb_endpt,
        input usb_txact,
        input usb_txpop,
        input usb_txpktfin,
        output usb_txcork,
        output [7:0] usb_txdata,
        output [11:0] usb_txlen,
        input [7:0] usb_rxdat,
        input usb_rxval,
        output usb_rxrdy,
        input usb_rxact,
        input usb_rxpktval,

        // 内部串口
        input LOC_UART_TX,
        output LOC_UART_RX,

        // SWD/JTAG连接器端口
        output EXT_SWCLK_TCK_O,
        output EXT_SWDIO_TMS_T,
        output EXT_SWDIO_TMS_O,
        input EXT_SWDIO_TMS_I,
        input EXT_SWO_TDO_I,
        output EXT_TDI_O,
        input EXT_RTCK_I,
        input EXT_SRST_I,
        output EXT_SRST_O,
        input EXT_TRST_I,
        output EXT_TRST_O,

        // 独立串口连接器
        output EXT_UART_TX,
        input EXT_UART_RX
    );

    // ----------------------------------------
    // Internal wires declarations
    wire                   trans_req= hreadys & hsels & htranss[1];
    // transfer request issued only in SEQ and NONSEQ status and slave is
    // selected and last transfer finish

    wire                   ahb_read_req  = trans_req & (~hwrites);// AHB read request
    wire                   ahb_write_req = trans_req &  hwrites;  // AHB write request
    wire                   update_read_req;    // To update the read enable register
    wire                   update_write_req;   // To update the write enable register

    reg  [ADDRWIDTH-1:0]   addr_reg;     // address signal, registered
    reg                    read_en_reg;  // read enable signal, registered
    reg                    write_en_reg; // write enable signal, registered

    reg  [3:0]             byte_strobe_reg; // registered output for byte strobe
    reg  [3:0]             byte_strobe_nxt; // next state for byte_strobe_reg
    //-----------------------------------------------------------
    // Module logic start
    //----------------------------------------------------------

    // Address signal registering, to make the address and data active at the same cycle
    always @(posedge hclk or negedge hresetn) begin
        if (~hresetn)
            addr_reg <= {(ADDRWIDTH){1'b0}}; //default address 0 is selected
        else if (trans_req)
            addr_reg <= haddrs[ADDRWIDTH-1:0];
    end


    // register read signal generation
    assign update_read_req = ahb_read_req | (read_en_reg & hreadys); // Update read enable control if
    //  1. When there is a valid read request
    //  2. When there is an active read, update it at the end of transfer (HREADY=1)

    always @(posedge hclk or negedge hresetn) begin
        if (~hresetn) begin
            read_en_reg <= 1'b0;
        end
        else if (update_read_req) begin
            read_en_reg  <= ahb_read_req;
        end
    end

    // register write signal generation
    assign update_write_req = ahb_write_req |( write_en_reg & hreadys);  // Update write enable control if
    //  1. When there is a valid write request
    //  2. When there is an active write, update it at the end of transfer (HREADY=1)

    always @(posedge hclk or negedge hresetn) begin
        if (~hresetn) begin
            write_en_reg <= 1'b0;
        end
        else if (update_write_req) begin
            write_en_reg  <= ahb_write_req;
        end
    end

    // byte strobe signal
    always @(hsizes or haddrs) begin
        if (hsizes == 3'b000)    //byte
        begin
            case(haddrs[1:0])
                2'b00:
                    byte_strobe_nxt = 4'b0001;
                2'b01:
                    byte_strobe_nxt = 4'b0010;
                2'b10:
                    byte_strobe_nxt = 4'b0100;
                2'b11:
                    byte_strobe_nxt = 4'b1000;
                default:
                    byte_strobe_nxt = 4'bxxxx;
            endcase
        end
        else if (hsizes == 3'b001) //half word
        begin
            if(haddrs[1]==1'b1)
                byte_strobe_nxt = 4'b1100;
            else
                byte_strobe_nxt = 4'b0011;
        end
        else // default 32 bits, word
        begin
            byte_strobe_nxt = 4'b1111;
        end
    end

    always @(posedge hclk or negedge hresetn) begin
        if (~hresetn)
            byte_strobe_reg <= {4{1'b0}};
        else if (update_read_req|update_write_req)
            // Update byte strobe registers if
            // 1. if there is a valid read/write transfer request
            // 2. if there is an on going transfer
            byte_strobe_reg  <= byte_strobe_nxt;
    end

    //-----------------------------------------------------------
    // Outputs
    //-----------------------------------------------------------
    // For simplify the timing, the master and slave signals are connected directly, execpt data bus.
    wire [ADDRWIDTH-1:0] addr = addr_reg[ADDRWIDTH-1:0];
    wire read_en     = read_en_reg;
    wire write_en    = write_en_reg;
    wire [31:0] wdata       = hwdatas;
    wire [3:0] byte_strobe = byte_strobe_reg;

    assign hresps      = 1'b0;  // OKAY response from slave
    //-----------------------------------------------------------
    //Module logic end
    //----------------------------------------------------------

    localparam [ADDRWIDTH-1:0] DAP_CR_ADDR               = 12'h000;
    localparam [ADDRWIDTH-1:0] DAP_TIME_ADDR             = 12'h004;
    localparam [ADDRWIDTH-1:0] DAP_SR_ADDR               = 12'h008;
    localparam [ADDRWIDTH-1:0] DAP_DR_ADDR               = 12'h00C;
    localparam [ADDRWIDTH-1:0] DAP_CURCMD_ADDR           = 12'h010;
    localparam [ADDRWIDTH-1:0] DAP_BAUD_CR_ADDR          = 12'h014;
    localparam [ADDRWIDTH-1:0] DAP_BAUD_TIMING_ADDR      = 12'h018;

    localparam [ADDRWIDTH-1:0] DAP_GPIO_CR_DELAY_ADDR    = 12'h020;
    localparam [ADDRWIDTH-1:0] DAP_GPIO_DELAY_ADDR       = 12'h024;
    localparam [ADDRWIDTH-1:0] DAP_GPIO_DI_ADDR          = 12'h028;
    localparam [ADDRWIDTH-1:0] DAP_GPIO_DO_ADDR          = 12'h02C;

    localparam [ADDRWIDTH-1:0] DAP_SWJ_CR_ADDR           = 12'h030;


    function addr_equ;
        input [ADDRWIDTH-1:0] addr;
        input [ADDRWIDTH-1:0] taddr;
        begin
            addr_equ = addr[ADDRWIDTH-1:2] == taddr[ADDRWIDTH-1:2];
        end
    endfunction


    reg [31:0] dap_ctrl_reg;
    wire DAP_CR_EN = dap_ctrl_reg[0];
    wire DAP_INT_EN = dap_ctrl_reg[31];
    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            dap_ctrl_reg <= 0;
        end
        else begin
            if (write_en && addr_equ(addr, DAP_CR_ADDR)) begin
                if (byte_strobe_reg[0])
                    dap_ctrl_reg[0 +: 8] <= hwdatas[0 +: 8];
                if (byte_strobe_reg[1])
                    dap_ctrl_reg[8 +: 8] <= hwdatas[8 +: 8];
                if (byte_strobe_reg[2])
                    dap_ctrl_reg[16 +: 8] <= hwdatas[16 +: 8];
                if (byte_strobe_reg[3])
                    dap_ctrl_reg[24 +: 8] <= hwdatas[24 +: 8];
            end
        end
    end

    reg [31:0] clk_timer; // 原始时钟时间戳定时器
    reg [31:0] us_timer;  // us定时器
    reg [ 5:0] us_timer_div;
    wire us_tick = us_timer_div == (CLOCK_FREQ_M - 6'd1);
    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            clk_timer <= 32'd0;
            us_timer <= 32'd0;
            us_timer_div <= 6'd0;
        end
        else begin
            clk_timer <= clk_timer + 1;
            if (us_tick) begin
                us_timer_div <= 6'd0;
                us_timer <= us_timer + 32'd1;
            end
            else begin
                us_timer_div <= us_timer_div + 6'd1;
            end

            // TIME寄存器写操作复位定时器
            if (write_en && addr_equ(addr, DAP_TIME_ADDR)) begin
                clk_timer <= 32'd0;
                us_timer <= 32'd0;
                us_timer_div <= 6'd0;
            end
        end
    end

    wire dap_in_tvalid;
    wire dap_in_tready;
    wire [7:0] dap_in_tdata;
    reg [11:0] ram_write_addr;
    reg [7:0] ram_write_data;
    reg ram_write_en;
    reg [11:0] packet_len;
    reg group_finish;
    reg packet_finish;
    wire packet_almost_full;

    DAP_USB_Receiver dap_usb_receiver_inst (
                         .clk(hclk),
                         .resetn(hresetn),

                         .usb_endpt(usb_endpt),
                         .usb_rxdat(usb_rxdat),
                         .usb_rxval(usb_rxval),
                         .usb_rxrdy(usb_rxrdy),
                         .usb_rxact(usb_rxact),
                         .usb_rxpktval(usb_rxpktval),

                         .axis_tvaild(dap_in_tvalid),
                         .axis_tready(dap_in_tready),
                         .axis_tdata(dap_in_tdata)
                     );

    DAP_USB_Packer dap_usb_packer_inst (
                       .clk(hclk),
                       .resetn(hresetn),

                       .usb_endpt(usb_endpt),
                       .usb_txact(usb_txact),
                       .usb_txpop(usb_txpop),
                       .usb_txpktfin(usb_txpktfin),
                       .usb_txcork(usb_txcork),
                       .usb_txdata(usb_txdata),
                       .usb_txlen(usb_txlen),

                       .ram_write_addr(ram_write_addr),
                       .ram_write_data(ram_write_data),
                       .ram_write_en(ram_write_en),
                       .packet_len(packet_len),
                       .group_finish(group_finish),
                       .packet_finish(packet_finish),
                       .almost_full(packet_almost_full)
                   );



    // 一阶段解码器，命令字转换独热码
    reg [`CMD_REG_WIDTH-1:0] cmd_decoder_reslut;
    always @(*) begin
        cmd_decoder_reslut = 12'd1;
        case ((dap_sm == 2'd0) ? dap_in_tdata : processing_cmd)
            // ID_DAP_Transfer
            8'h05:
                cmd_decoder_reslut = `CMD_TRANSFER;
            // ID_DAP_TransferBlock
            8'h06:
                cmd_decoder_reslut = `CMD_TRANSFER_BLOCK;
            // ID_DAP_TransferAbort
            8'h07:
                cmd_decoder_reslut = `CMD_TRANSFER_ABORT;
            // ID_DAP_WriteABORT
            8'h08:
                cmd_decoder_reslut = `CMD_WRITE_ABORT;
            // ID_DAP_Delay
            8'h09:
                cmd_decoder_reslut = `CMD_DELAY;
            // ID_DAP_SWJ_Pins
            8'h10:
                cmd_decoder_reslut = `CMD_SWJ_PINS;
            // ID_DAP_SWJ_Sequence
            8'h12:
                cmd_decoder_reslut = `CMD_SWJ_SEQUENCE;
            // ID_DAP_SWD_Sequence
            8'h1D:
                cmd_decoder_reslut = `CMD_SWD_SEQUENCE;
            // ID_DAP_JTAG_Sequence
            8'h14:
                cmd_decoder_reslut = `CMD_JTAG_SEQUENCE;
            // ID_DAP_JTAG_IDCODE
            8'h16:
                cmd_decoder_reslut = `CMD_JTAG_IDCODE;
            // ID_DAP_QueueCommands
            // ID_DAP_ExecuteCommands
            8'h7E, 8'h7F:
                cmd_decoder_reslut = `CMD_EXEC_CMD;
            // ID_DAP_Info
            // ID_DAP_HostStatus
            // ID_DAP_Connect
            // ID_DAP_Disconnect
            // ID_DAP_TransferConfigure
            // ID_DAP_ResetTarget
            // ID_DAP_SWJ_Clock
            // ID_DAP_SWD_Configure
            // ID_DAP_JTAG_Configure
            // ID_DAP_SWO_Transport
            // ID_DAP_SWO_Mode
            // ID_DAP_SWO_Baudrate
            // ID_DAP_SWO_Control
            // ID_DAP_SWO_Status
            // ID_DAP_SWO_ExtendedStatus
            // ID_DAP_SWO_Data
            // ID_DAP_UART_Transport
            // ID_DAP_UART_Configure
            // ID_DAP_UART_Control
            // ID_DAP_UART_Status
            // ID_DAP_UART_Transfer
            // ID_DAP_Vendor*
            default:
                cmd_decoder_reslut = `CMD_MCU_HELPER;
        endcase
    end

    reg [7:0] processing_cmd;   // 当前处理的命令
    reg [1:0] dap_sm;
    reg [7:0] num_cmd;
    wire fist_decoder_tready = (dap_sm == 2'd0 || dap_sm == 2'd1);    // 一阶段解码器读就绪
    // 根据状态选择解码信号


    // worker数据输出接口
    wire [9:0] worker_ram_write_addr [0:2];
    wire [7:0] worker_ram_write_data [0:2];
    wire worker_ram_write_en [0:2];
    wire [9:0] worker_packet_len [0:2];

    // worker启动标志，运行过程中全程拉高，done下一个周期拉低
    wire [`CMD_REAL_NUM-1:0] worker_start_flags = (dap_sm == 2'd2) ? cmd_decoder_reslut[`CMD_REAL_NUM-1:0] : 0;

    // worker完成标志
    wire [`CMD_REAL_NUM-1:0] worker_done_flags;

    // worker tready标志
    wire [`CMD_REAL_NUM-1:0] worker_dap_in_tready;

    // MCU部分操作信号
    wire AHB_WRITE_DR = write_en && addr_equ(addr, DAP_DR_ADDR) && byte_strobe[0];
    wire AHB_READ_DR = read_en && addr_equ(addr, DAP_DR_ADDR);
    wire AHB_CLEAR_FLAG = write_en && addr_equ(addr, DAP_SR_ADDR) && byte_strobe[3] && hwdatas[31];

    /*********************************************** MCU HELPER ***********************************************/
    reg [9:0] mcu_write_addr;
    reg mcu_helper_done;
    assign worker_ram_write_addr[0] = mcu_write_addr; // mcu地址自增
    assign worker_ram_write_data[0] = hwdatas[7:0]; // AHB数据线
    assign worker_ram_write_en[0] = AHB_WRITE_DR; // AHB写入DR信号作为写有效
    assign worker_packet_len[0] = mcu_write_addr; // MCU写入不支持随机访问，直接使用addr作为长度
    assign worker_done_flags[0] = mcu_helper_done; // MCU写入完成标志

    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            mcu_write_addr <= 10'd0;
            mcu_helper_done <= 1'd0;
        end
        else begin
            if (worker_start_flags[`CMD_MCU_HELPER_SHIFT]) begin
                if (AHB_WRITE_DR) begin
                    mcu_write_addr <= mcu_write_addr + 1'd1;
                end
                if (AHB_CLEAR_FLAG) begin
                    mcu_helper_done <= 1'd1;
                end
            end
            else begin
                mcu_helper_done <= 1'd0;
                mcu_write_addr <= 10'd0;
            end
        end
    end

    /************************************************** DELAY *************************************************/
    DAP_Delay DAP_Delay_inst (
                  .clk(hclk),
                  .resetn(hresetn),
                  .us_tick(us_tick),
                  .enable(DAP_CR_EN),

                  .start(worker_start_flags[`CMD_DELAY_SHIFT]),
                  .done(worker_done_flags[`CMD_DELAY_SHIFT]),

                  .dap_in_tvalid(dap_in_tvalid),
                  .dap_in_tready(worker_dap_in_tready[`CMD_DELAY_SHIFT]),
                  .dap_in_tdata(dap_in_tdata),

                  .ram_write_addr(worker_ram_write_addr[1]),
                  .ram_write_data(worker_ram_write_data[1]),
                  .ram_write_en(worker_ram_write_en[1]),
                  .packet_len(worker_packet_len[1])
              );

    /*************************************************** SWJ **************************************************/
    wire LOC_SWCLK_TCK_O;
    wire LOC_SWDIO_TMS_T;
    wire LOC_SWDIO_TMS_O;
    wire LOC_SWDIO_TMS_I;
    wire LOC_SWO_TDO_I;
    wire LOC_TDI_O;
    wire LOC_SRST_I;
    wire LOC_SRST_O;
    wire LOC_TRST_I;
    wire LOC_TRST_O;
    wire SWD_MODE;

    wire sclk_out;
    wire sclk_pulse;
    wire sclk_delay_pulse;

    wire [31:0] swj_hrdatas;
    DAP_SWJ #(
                .ADDRWIDTH(ADDRWIDTH),
                .BASE_ADDR(DAP_SWJ_CR_ADDR)
            ) DAP_SWJ_inst(
                .clk(hclk),
                .resetn(hresetn),
                .us_tick(us_tick),
                .us_timer(us_timer),
                .enable(DAP_CR_EN),

                .sclk(dap_clk),
                .sclk_out(sclk_out),
                .sclk_pulse(sclk_pulse),
                .sclk_delay_pulse(sclk_delay_pulse),

                .ahb_write_en(write_en),
                .ahb_read_en(read_en),
                .ahb_addr(addr),
                .ahb_rdata(swj_hrdatas),
                .ahb_wdata(hwdatas),
                .ahb_byte_strobe(byte_strobe_reg),

                .start(worker_start_flags[`CMD_SWJ_RANGE]),
                .done(worker_done_flags[`CMD_SWJ_RANGE]),

                .dap_in_tvalid(dap_in_tvalid),
                .dap_in_tready(worker_dap_in_tready[`CMD_SWJ_RANGE]),
                .dap_in_tdata(dap_in_tdata),

                .ram_write_addr(worker_ram_write_addr[2]),
                .ram_write_data(worker_ram_write_data[2]),
                .ram_write_en(worker_ram_write_en[2]),
                .packet_len(worker_packet_len[2]),

                .SWCLK_TCK_O(LOC_SWCLK_TCK_O),
                .SWDIO_TMS_T(LOC_SWDIO_TMS_T),
                .SWDIO_TMS_O(LOC_SWDIO_TMS_O),
                .SWDIO_TMS_I(LOC_SWDIO_TMS_I),
                .SWO_TDO_I(LOC_SWO_TDO_I),
                .TDI_O(LOC_TDI_O),
                .SRST_I(LOC_SRST_I),
                .SRST_O(LOC_SRST_O),
                .TRST_I(LOC_TRST_I),
                .TRST_O(LOC_TRST_O),
                .SWD_MODE(SWD_MODE)
            );


    /*********************************************** 一阶段状态机 ***********************************************/
    reg sm_group_finish;
    reg sm_package_finish;

    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn || !DAP_CR_EN) begin
            processing_cmd <= 8'd0;
            dap_sm <= 2'd0;
            num_cmd <= 8'd0;
            sm_package_finish <= 1'd0;
            sm_group_finish <= 1'd0;
        end
        else begin
            sm_package_finish <= 1'd0;
            sm_group_finish <= 1'd0;
            case (dap_sm)
                2'd0: begin // 读第一字节
                    // 读取命令
                    if (dap_in_tvalid) begin
                        processing_cmd <= dap_in_tdata;
                        if (cmd_decoder_reslut[`CMD_TRANSFER_ABORT_SHIFT])
                            dap_sm <= 2'd0;
                        else if (cmd_decoder_reslut[`CMD_EXEC_CMD_SHIFT])
                            dap_sm <= 2'd1;
                        else
                            dap_sm <= 2'd2;
                    end
                end
                2'd1: begin
                    // 读取numcmd
                    if (dap_in_tvalid) begin
                        num_cmd <= dap_in_tdata;
                        dap_sm <= 2'd0;
                    end
                end
                2'd2: begin
                    // 等待处理完成判断忙标志
                    if (cmd_decoder_reslut[`CMD_REAL_NUM-1:0] & worker_done_flags[`CMD_REAL_NUM-1:0]) begin
                        // 分组打包完成，状态转移
                        sm_group_finish <= 1'd1;
                        dap_sm <= 2'd3;
                    end
                end
                2'd3: begin
                    if (!packet_almost_full) begin
                        // 没有num_cmd或最后一个处理完
                        if (num_cmd == 8'd0 || num_cmd == 8'd1) begin
                            // 命令数量清零，整包打包完成
                            num_cmd <= 8'd0;
                            sm_package_finish <= 1'd1;
                        end
                        else begin
                            // 命令数量递减
                            num_cmd <= num_cmd - 1'd1;
                        end
                        dap_sm <= 2'd0;
                    end
                end
            endcase
        end
    end


    wire worker_tready = ((cmd_decoder_reslut[`CMD_REAL_NUM-1:1] & worker_dap_in_tready[`CMD_REAL_NUM-1:1]) ? 1'd1 : 1'd0);

    assign dap_in_tready = DAP_CR_EN & (fist_decoder_tready | worker_tready | AHB_READ_DR);

    wire DAP_IN_BYPASS_ACTIVE = DAP_CR_EN & dap_in_tvalid & !cmd_decoder_reslut[`CMD_TRANSFER_ABORT_SHIFT];

    // 输出管道路由
    always @(*) begin
        packet_finish = sm_package_finish;

        if (dap_sm == 2'd0) begin
            // DAP顶层控制器读取数据直接写入fifo, TransferAbort没有返回数据
            ram_write_addr = 10'd0;
            ram_write_data = dap_in_tdata;
            ram_write_en = DAP_IN_BYPASS_ACTIVE;
            packet_len = 10'd1;
            group_finish = DAP_IN_BYPASS_ACTIVE;
        end
        else if (dap_sm == 2'd1) begin
            ram_write_addr = 10'd0;
            ram_write_data = dap_in_tdata;
            ram_write_en = DAP_CR_EN & dap_in_tvalid;
            packet_len = 10'd1;
            group_finish = DAP_CR_EN & dap_in_tvalid;
        end
        else begin
            // 命令处理转接到各个worker
            casez (cmd_decoder_reslut)
                `CMD_MCU_HELPER: begin
                    ram_write_addr = worker_ram_write_addr[0];
                    ram_write_data = worker_ram_write_data[0];
                    ram_write_en = worker_ram_write_en[0];
                    packet_len = worker_packet_len[0];
                end
                `CMD_DELAY: begin
                    ram_write_addr = worker_ram_write_addr[1];
                    ram_write_data = worker_ram_write_data[1];
                    ram_write_en = worker_ram_write_en[1];
                    packet_len = worker_packet_len[1];
                end
                default: begin
                    ram_write_addr = worker_ram_write_addr[2];
                    ram_write_data = worker_ram_write_data[2];
                    ram_write_en = worker_ram_write_en[2];
                    packet_len = worker_packet_len[2];
                end
            endcase

            group_finish = sm_group_finish;
        end
    end


    assign intr = DAP_INT_EN & worker_start_flags[`CMD_MCU_HELPER_SHIFT];

    wire [31:0] baudgenerator_hrdatas;

    DAP_BaudGenerator #(
                          .ADDRWIDTH(ADDRWIDTH),
                          .BASE_ADDR(DAP_BAUD_CR_ADDR)
                      ) dap_baudgenerator_inst (
                          .clk(hclk),
                          .resetn(hresetn),
                          .sclk_in(dap_clk),
                          .ahb_write_en(write_en),
                          .ahb_read_en(read_en),
                          .ahb_addr(addr),
                          .ahb_rdata(baudgenerator_hrdatas),
                          .ahb_wdata(hwdatas),
                          .ahb_byte_strobe(byte_strobe_reg),

                          .sclk_out(sclk_out),
                          .sclk_pulse(sclk_pulse),
                          .sclk_delay_pulse(sclk_delay_pulse)
                      );

    wire [31:0] gpio_hrdatas;
    DAP_GPIO #(
                 .ADDRWIDTH(ADDRWIDTH),
                 .BASE_ADDR(DAP_GPIO_CR_DELAY_ADDR)
             ) dap_gpio_inst (
                 .clk(hclk),
                 .resetn(hresetn),

                 .ahb_write_en(write_en),
                 .ahb_read_en(read_en),
                 .ahb_addr(addr),
                 .ahb_rdata(gpio_hrdatas),
                 .ahb_wdata(hwdatas),
                 .ahb_byte_strobe(byte_strobe_reg),

                 .EXT_UART_TX(EXT_UART_TX),
                 .EXT_UART_RX(EXT_UART_RX),

                 .EXT_SWCLK_TCK_O(EXT_SWCLK_TCK_O),
                 .EXT_SWDIO_TMS_T(EXT_SWDIO_TMS_T),
                 .EXT_SWDIO_TMS_O(EXT_SWDIO_TMS_O),
                 .EXT_SWDIO_TMS_I(EXT_SWDIO_TMS_I),
                 .EXT_SWO_TDO_I(EXT_SWO_TDO_I),
                 .EXT_TDI_O(EXT_TDI_O),
                 .EXT_RTCK_I(EXT_RTCK_I),
                 .EXT_SRST_I(EXT_SRST_I),
                 .EXT_SRST_O(EXT_SRST_O),
                 .EXT_TRST_I(EXT_TRST_I),
                 .EXT_TRST_O(EXT_TRST_O),

                 .LOC_SWCLK_TCK_O(LOC_SWCLK_TCK_O),
                 .LOC_SWDIO_TMS_T(LOC_SWDIO_TMS_T),
                 .LOC_SWDIO_TMS_O(LOC_SWDIO_TMS_O),
                 .LOC_SWDIO_TMS_I(LOC_SWDIO_TMS_I),
                 .LOC_SWO_TDO_I(LOC_SWO_TDO_I),
                 .LOC_TDI_O(LOC_TDI_O),
                 .LOC_TRST_I(LOC_TRST_I),
                 .LOC_TRST_O(LOC_TRST_O),
                 .LOC_SRST_I(LOC_SRST_I),
                 .LOC_SRST_O(LOC_SRST_O),

                 .SWD_MODE(SWD_MODE),
                 .LOC_UART_TX(LOC_UART_TX),
                 .LOC_UART_RX(LOC_UART_RX)
             );


    always @(*) begin
        if (read_en) begin
            casez (addr[ADDRWIDTH-1:2])
                DAP_CR_ADDR[ADDRWIDTH-1:2]: begin
                    hrdatas = dap_ctrl_reg;
                    hreadyouts = 1'd1;
                end
                DAP_TIME_ADDR[ADDRWIDTH-1:2]: begin
                    hrdatas = clk_timer;
                    hreadyouts = 1'd1;
                end
                DAP_SR_ADDR[ADDRWIDTH-1:2]: begin
                    hrdatas = {worker_start_flags[`CMD_MCU_HELPER_SHIFT], 30'd0};
                    hreadyouts = 1'd1;
                end
                DAP_DR_ADDR[ADDRWIDTH-1:2]: begin
                    hrdatas = {23'd0, dap_in_tvalid, dap_in_tdata};
                    hreadyouts = 1'd1;
                end
                DAP_CURCMD_ADDR[ADDRWIDTH-1:2]: begin
                    hrdatas = {24'd0, processing_cmd};
                    hreadyouts = 1'd1;
                end
                // BAUD GENERATOR RANGE 14 18
                DAP_BAUD_CR_ADDR[ADDRWIDTH-1:2],
                DAP_BAUD_TIMING_ADDR[ADDRWIDTH-1:2]: begin
                    hrdatas = baudgenerator_hrdatas;
                    hreadyouts = 1'd1;
                end
                // GPIO ADDR RANGE 2x
                (10'b0000_0010_??): begin
                    hrdatas = gpio_hrdatas;
                    hreadyouts = 1'd1;
                end
                // SWJ ADDR RANGE 3x 4x 5x
                (10'b0000_0011_??), (10'b0000_010?_??): begin
                    hrdatas = swj_hrdatas;
                    hreadyouts = 1'd1;
                end
                default: begin
                    hrdatas = {32{1'bx}};
                    hreadyouts = 1'd1;
                end
            endcase
        end
        else begin
            hrdatas = {32{1'bx}};
            hreadyouts  = 1'b1;  // slave always ready
        end
    end

endmodule
