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

        output reg dat_out_tvalid,
        output reg [7:0] dap_out_tdata,

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


    wire delay_done;


    reg [7:0] processing_cmd;   // 当前处理的命令
    reg mcu_helper_intr;        // mcu介入中断信号
    reg fist_decoder_tready;    // 一阶段解码器读就绪

    // 一阶段解码器，命令字转换独热码
    reg [31:0] cmd_decoder_reslut;

    always @(*) begin
        cmd_decoder_reslut = 32'd0;
        casez (processing_cmd)
            8'b0000_00??, 8'b100?_????:
                cmd_decoder_reslut = `CMD_MCU_HELPER;
            8'h09:
                cmd_decoder_reslut = `CMD_DELAY;
        endcase
    end


    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn || !DAP_CR_EN) begin
            fist_decoder_tready <= 1'd1;
            mcu_helper_intr <= 1'd0;
            processing_cmd <= 8'd0;
        end
        else begin

            // 读取命令
            if (fist_decoder_tready && dap_in_tvalid) begin
                processing_cmd <= dap_in_tdata;
                fist_decoder_tready <= 1'd0;
                casez (cmd_decoder_reslut)
                    `CMD_MCU_HELPER:
                        mcu_helper_intr <= 1'd1;
                endcase
            end

            // 判断忙标志
            if (!fist_decoder_tready) begin
                casez (cmd_decoder_reslut)
                    `CMD_MCU_HELPER:
                        // SR.INT 写1清除中断标志
                        if (write_en && addr_equ(addr, DAP_SR_ADDR) && byte_strobe_reg[3]) begin
                            if (hwdatas[31]) begin
                                mcu_helper_intr <= 1'd0;
                                fist_decoder_tready <= 1'd1;
                            end
                        end
                    `CMD_DELAY:
                        if (delay_done)
                            fist_decoder_tready <= 1'd1;
                    default:
                        fist_decoder_tready <= 1'd1;
                endcase
            end
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

    wire write_dr_en = write_en && addr_equ(addr, DAP_DR_ADDR);



    wire delay_in_tready;
    wire delay_out_tvalid;
    wire [7:0] delay_out_tdata;
    DAP_Delay_Worker  DAP_Delay_Worker_inst (
                          .hclk(hclk),
                          .us_tick(us_tick),
                          .en(DAP_CR_EN),
                          .start(cmd_decoder_reslut[`CMD_DELAY_SHIFT]),
                          .dap_in_tvalid(dap_in_tvalid),
                          .dap_in_tready(delay_in_tready),
                          .dap_in_tdata(dap_in_tdata),
                          .dat_out_tvalid(delay_out_tvalid),
                          .dap_out_tdata(delay_out_tdata),
                          .done(delay_done)
                      );

    assign dap_in_tready = (DAP_CR_EN & fist_decoder_tready) | delay_in_tready;

    // 输出管道路由
    always @(*) begin
        if (write_dr_en) begin
            dat_out_tvalid = 1'd1;
            dap_out_tdata = hwdatas[7:0];
        end
        else begin
            casez (cmd_decoder_reslut)
                `CMD_DELAY: begin
                    dat_out_tvalid = delay_out_tvalid;
                    dap_out_tdata  = delay_out_tdata;
                end
                default: begin
                    dat_out_tvalid = 1'd0;
                    dap_out_tdata = 8'd0;
                end
            endcase
        end
    end

    assign intr = DAP_INT_EN & (mcu_helper_intr);

endmodule
