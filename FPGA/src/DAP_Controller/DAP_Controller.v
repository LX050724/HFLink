`include "DAP_Cmd.v"

module DAP_Controller #(
        parameter ADDRWIDTH = 12,
        parameter [5:0] CLOCK_FREQ_M = 60
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

        output wire                  hreadyouts,
        output wire                  hresps,
        output reg [31:0]            hrdatas,

        output wire intr,

        input wire dap_in_tvalid,
        output wire dap_in_tready,
        input wire [7:0] dap_in_tdata,

        output wire dap_out_tvalid,
        output wire [7:0] dap_out_tdata,

        output wire TCK_SWCLK,
        output wire TDI,
        output wire TDO,
        input wire TMS_SWDIO_I,
        output wire TMS_SWDIO_O,
        output wire TMS_SWDIO_T
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

    assign hreadyouts  = 1'b1;  // slave always ready
    assign hresps      = 1'b0;  // OKAY response from slave
    //-----------------------------------------------------------
    //Module logic end
    //----------------------------------------------------------

    localparam [ADDRWIDTH-1:0] DAP_CR_ADDR               = 12'h000;
    localparam [ADDRWIDTH-1:0] DAP_TIME_ADDR             = 12'h004;
    localparam [ADDRWIDTH-1:0] DAP_SR_ADDR               = 12'h008;
    localparam [ADDRWIDTH-1:0] DAP_DR_ADDR               = 12'h00C;
    localparam [ADDRWIDTH-1:0] DAP_CURCMD_ADDR           = 12'h010;

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

    reg [7:0] dap_out_fifo_wdata;
    wire [7:0] dap_out_fifo_rdata;
    reg dap_out_fifo_WrEn;
    reg dap_out_fifo_RdEn;
    wire dap_out_fifo_empty;

    fifo_sc_top dap_out_fifo_inst (
                    .Data(dap_out_fifo_wdata), //input [7:0] Data
                    .Clk(hclk), //input Clk
                    .WrEn(dap_out_fifo_WrEn), //input WrEn
                    .RdEn(dap_out_fifo_RdEn), //input RdEn
                    .Reset(!hresetn), //input Reset
                    .Wnum(), //output [11:0] Wnum
                    .Q(dap_out_tdata), //output [7:0] Q
                    .Empty(dap_out_fifo_empty), //output Empty
                    .Full() //output Full
                );


    // 一阶段解码器，命令字转换独热码
    reg [`CMD_REG_WIDTH-1:0] cmd_decoder_reslut;
    wire [7:0] decode_cmd;
    always @(*) begin
        cmd_decoder_reslut = 12'd1;
        casez (decode_cmd)
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
    reg mcu_helper_intr;        // mcu介入中断信号
    reg [1:0] dap_sm;
    reg [7:0] num_cmd;
    wire fist_decoder_tready = (dap_sm == 2'd0 || dap_sm == 2'd1);    // 一阶段解码器读就绪
    wire [`CMD_REG_WIDTH-1:1] worker_start_flags = (dap_sm == 2'd2) ? cmd_decoder_reslut[`CMD_REG_WIDTH-1:1] : `CMD_REG_WIDTH'd0;
    wire [`CMD_REG_WIDTH-1:1] worker_done_flags;
    wire [`CMD_REG_WIDTH-1:1] worker_dap_in_tready;
    wire [`CMD_REG_WIDTH-1:1] worker_dap_out_tready;
    wire [7:0] worker_dap_out_tdata [`CMD_REG_WIDTH-1:1];

    // 根据状态选择解码信号
    assign decode_cmd = (dap_sm == 2'd0) ? dap_in_tdata : processing_cmd;

    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn || !DAP_CR_EN) begin
            mcu_helper_intr <= 1'd0;
            processing_cmd <= 8'd0;
            dap_out_fifo_RdEn <= 1'd0;
            dap_sm <= 2'd0;
            num_cmd <= 8'd0;
        end
        else begin
            case (dap_sm)
                2'd0: begin // 读第一字节
                    // 读取命令
                    if (dap_in_tvalid) begin
                        processing_cmd <= dap_in_tdata;
                        if (cmd_decoder_reslut[`CMD_MCU_HELPER_SHIFT]) begin
                            mcu_helper_intr <= 1'd1;
                            dap_sm <= 2'd2;
                        end
                        else if (cmd_decoder_reslut[`CMD_EXEC_CMD_SHIFT]) begin
                            dap_sm <= 2'd1;
                        end
                        else begin
                            dap_sm <= 2'd2;
                        end
                    end
                end
                2'd1: begin
                    // 读取numcmd
                    if (dap_in_tvalid) begin
                        num_cmd <= dap_in_tdata;
                        dap_sm <= 2'd0;
                    end
                end
                2'd2: begin // 等待处理完成
                    // 判断忙标志
                    if (cmd_decoder_reslut[`CMD_MCU_HELPER_SHIFT]) begin
                        if (write_en && addr_equ(addr, DAP_SR_ADDR) && byte_strobe_reg[3] && hwdatas[31]) begin
                            mcu_helper_intr <= 1'd0;
                            dap_sm <= 2'd3;
                        end
                    end
                    else if (cmd_decoder_reslut[`CMD_REG_WIDTH-1:1] & worker_done_flags[`CMD_REG_WIDTH-1:1]) begin
                        dap_sm <= 2'd3;
                    end
                end
                2'd3: begin
                    // 没有num_cmd或最后一个处理完
                    if (num_cmd == 8'd0 || num_cmd == 8'd1) begin
                        num_cmd <= 8'd0;
                        // 输出fifo
                        if (dap_out_fifo_empty) begin
                            dap_out_fifo_RdEn <= 1'd0;
                            dap_sm <= 2'd0;
                        end
                        else begin
                            dap_out_fifo_RdEn <= 1'd1;
                        end
                    end
                    else begin
                        // 命令数量递减，反回读命令状态
                        num_cmd <= num_cmd - 8'd1;
                        dap_sm <= 2'd0;
                    end
                end
            endcase
        end
    end



    always @(*) begin
        if (read_en) begin
            case (addr[ADDRWIDTH-1:2])
                DAP_CR_ADDR[ADDRWIDTH-1:2]:
                    hrdatas = dap_ctrl_reg;
                DAP_TIME_ADDR[ADDRWIDTH-1:2]:
                    hrdatas = clk_timer;
                DAP_SR_ADDR[ADDRWIDTH-1:2]:
                    hrdatas = {mcu_helper_intr, 30'd0};
                DAP_DR_ADDR[ADDRWIDTH-1:2]:
                    hrdatas = {4{dap_in_tdata}};
                DAP_CURCMD_ADDR[ADDRWIDTH-1:2]:
                    hrdatas = {24'd0, processing_cmd};
                default:
                    hrdatas = {32{1'bx}};
            endcase
        end
        else begin
            hrdatas = {32{1'bx}};
        end
    end

    DAP_Delay_Worker DAP_Delay_Worker_inst (
                          .hclk(hclk),
                          .us_tick(us_tick),
                          .en(DAP_CR_EN),

                          .start(worker_start_flags[`CMD_DELAY_SHIFT]),
                          .done(worker_done_flags[`CMD_DELAY_SHIFT]),
                          .dap_in_tvalid(dap_in_tvalid),
                          .dap_in_tready(worker_dap_in_tready[`CMD_DELAY_SHIFT]),
                          .dap_in_tdata(dap_in_tdata),
                          .dap_out_tvalid(worker_dap_out_tready[`CMD_DELAY_SHIFT]),
                          .dap_out_tdata(worker_dap_out_tdata[`CMD_DELAY_SHIFT])
                      );

    assign dap_in_tready = DAP_CR_EN & (
               fist_decoder_tready | ((cmd_decoder_reslut[`CMD_REG_WIDTH-1:1] & worker_dap_in_tready[`CMD_REG_WIDTH-1:1]) ? 1'd1 : 1'd0));

    // 输出管道路由
    always @(*) begin
        if (write_en && addr_equ(addr, DAP_DR_ADDR) && byte_strobe[0]) begin
            // MCU写入优先级最高
            dap_out_fifo_WrEn = 1'd1;
            dap_out_fifo_wdata = hwdatas[7:0];
        end
        else if (fist_decoder_tready) begin
            // DAP顶层控制器读取数据直接写入fifo
            dap_out_fifo_WrEn = DAP_CR_EN & dap_in_tvalid;
            dap_out_fifo_wdata = dap_in_tdata;
        end
        else begin
            // 命令处理转接到各个worker
            casez (cmd_decoder_reslut)
                `CMD_DELAY: begin
                    dap_out_fifo_WrEn = worker_dap_out_tready[`CMD_DELAY_SHIFT];
                    dap_out_fifo_wdata  = worker_dap_out_tdata[`CMD_DELAY_SHIFT];
                end
                default: begin
                    dap_out_fifo_WrEn = 1'd0;
                    dap_out_fifo_wdata = 8'd0;
                end
            endcase
        end
    end


    assign intr = DAP_INT_EN & (mcu_helper_intr);
    assign dap_out_tvalid = dap_out_fifo_RdEn ? !dap_out_fifo_empty : 1'd0;

endmodule
