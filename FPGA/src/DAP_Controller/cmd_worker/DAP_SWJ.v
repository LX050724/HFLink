`include "DAP_Cmd.v"

module DAP_SWJ #(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    )(
        input clk,
        input resetn,
        input us_tick,
        input [31:0] us_timer,
        input enable,

        // AHB MEM接口
        input ahb_write_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        output [8:0] ram_write_addr,
        output [7:0] ram_write_data,
        output ram_write_en,
        output [8:0] packet_len,

        input [`CMD_REG_WIDTH-1:4] start,
        output [`CMD_REG_WIDTH-1:4] done,

        output SWCLK_TCK_O,
        output SWDIO_TMS_T,
        output SWDIO_TMS_O,
        input SWDIO_TMS_I,
        input SWO_TDO_I,
        output TDI_O,
        // input RTCK_I,
        input SRST_I,
        output SRST_O,
        input TRST_I,
        output TRST_O,

        // swd模式信号
        output SWD_MODE
    );
    genvar gi;
    integer i;

    localparam [ADDRWIDTH-1:0] SWJ_CR_ADDR            = BASE_ADDR + 12'h000;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_SWD_CR_ADDR        = BASE_ADDR + 12'h004;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_CR_ADDR       = BASE_ADDR + 12'h008;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF0_ADDR = BASE_ADDR + 12'h00C;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF1_ADDR = BASE_ADDR + 12'h010;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF2_ADDR = BASE_ADDR + 12'h014;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF3_ADDR = BASE_ADDR + 12'h018;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF4_ADDR = BASE_ADDR + 12'h01C;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF5_ADDR = BASE_ADDR + 12'h020;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF6_ADDR = BASE_ADDR + 12'h024;  // RW
    localparam [ADDRWIDTH-1:0] SWJ_JTAG_IR_CONF7_ADDR = BASE_ADDR + 12'h028;  // RW

    reg [31:0] SWJ_CR;

    reg [8:0] SWJ_SWD_CR;
    reg [7:0] SWJ_JTAG_CR;
    reg [31:0] SWJ_JTAG_IR_CONF_REG [0:7];

    wire [7:0] SWD_CONF_TURN = SWJ_SWD_CR[7:0];
    wire SWD_CONF_FORCE_DATA = SWJ_SWD_CR[8];

    wire [7:0] JTAG_CR_COUNT = SWJ_JTAG_CR[7:0];
    wire [13:0] JTAG_IR_BEFORE_CONF [0:7];
    wire [13:0] JTAG_IR_AFTER_CONF [0:7];
    wire [3:0] JTAG_IR_LEN_CONF [0:7];

    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : jtag_ir_conf_loop
            assign JTAG_IR_BEFORE_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][18+:14];
            assign JTAG_IR_AFTER_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][4+:14];
            assign JTAG_IR_LEN_CONF[gi] = SWJ_JTAG_IR_CONF_REG[gi][3:0];
        end
    endgenerate

    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            SWJ_CR <= 0;
            SWJ_SWD_CR <= 0;
            SWJ_JTAG_CR <= 0;
            for (i = 0; i < 8; i = i + 1) begin
                SWJ_JTAG_IR_CONF_REG[i] <= 32'd0;
            end
        end
        else begin
            if (ahb_wdata) begin
                case (ahb_addr[ADDRWIDTH-1:2])
                    SWJ_CR_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_CR);
                    SWJ_SWD_CR_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_SWD_CR);
                    SWJ_JTAG_CR_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_CR);
                    SWJ_JTAG_IR_CONF0_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[0]);
                    SWJ_JTAG_IR_CONF1_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[1]);
                    SWJ_JTAG_IR_CONF2_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[2]);
                    SWJ_JTAG_IR_CONF3_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[3]);
                    SWJ_JTAG_IR_CONF4_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[4]);
                    SWJ_JTAG_IR_CONF5_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[5]);
                    SWJ_JTAG_IR_CONF6_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[6]);
                    SWJ_JTAG_IR_CONF7_ADDR[ADDRWIDTH-1:2]:
                        AHB_WRITE_REG32(SWJ_JTAG_IR_CONF_REG[7]);
                endcase
            end
        end
    end

    always @(*) begin : ahb_mem_read_ctrl
        case (ahb_addr[ADDRWIDTH-1:2])
            SWJ_CR_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_CR;
            SWJ_SWD_CR_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_SWD_CR;
            SWJ_JTAG_CR_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_CR;
            SWJ_JTAG_IR_CONF0_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_IR_CONF_REG[0];
            SWJ_JTAG_IR_CONF1_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_IR_CONF_REG[1];
            SWJ_JTAG_IR_CONF2_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_IR_CONF_REG[2];
            SWJ_JTAG_IR_CONF3_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_IR_CONF_REG[3];
            SWJ_JTAG_IR_CONF4_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_IR_CONF_REG[4];
            SWJ_JTAG_IR_CONF5_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_IR_CONF_REG[5];
            SWJ_JTAG_IR_CONF6_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_IR_CONF_REG[6];
            SWJ_JTAG_IR_CONF7_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = SWJ_JTAG_IR_CONF_REG[7];
        endcase
    end

    task AHB_WRITE_REG32;
        output [31:0] optreg;
        if (ahb_byte_strobe[0])
            optreg[ 0+:8] <= ahb_wdata[ 0+:8];
        if (ahb_byte_strobe[1])
            optreg[ 8+:8] <= ahb_wdata[ 8+:8];
        if (ahb_byte_strobe[2])
            optreg[16+:8] <= ahb_wdata[16+:8];
        if (ahb_byte_strobe[3])
            optreg[24+:8] <= ahb_wdata[24+:8];
    endtask
endmodule
