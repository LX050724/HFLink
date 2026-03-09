module USB_DESC#(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    ) (
        input clk,
        input resetn,
        input usbrst,

        // AHB MEM接口
        input ahb_write_en,
        input ahb_read_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        output [7:0] txdat,
        output reg txval,
        output reg [11:0] txdat_len,
        input txpop,
        input txact,
        input [7:0] rxdat,
        input rxval,
        output reg rxrdy,
        input rxact,
        input setup,
        input [3:0] endpt_sel,

        input [15:0] descrom_raddr,
        input [7:0] desc_index,
        input [7:0] desc_type,
        output [7:0] descrom_rdata,
        output [15:0] desc_dev_addr,
        output [15:0] desc_dev_len,
        output [15:0] desc_qual_addr,
        output [15:0] desc_qual_len,
        output [15:0] desc_fscfg_addr,
        output [15:0] desc_fscfg_len,
        output [15:0] desc_hscfg_addr,
        output [15:0] desc_hscfg_len,
        output [15:0] desc_oscfg_addr,
        output [15:0] desc_hidrpt_addr,
        output [15:0] desc_hidrpt_len,
        output [15:0] desc_bos_addr,
        output [15:0] desc_bos_len,
        output [15:0] desc_strlang_addr,
        output [15:0] desc_strvendor_addr,
        output [15:0] desc_strvendor_len,
        output [15:0] desc_strproduct_addr,
        output [15:0] desc_strproduct_len,
        output [15:0] desc_strserial_addr,
        output [15:0] desc_strserial_len,
        output desc_have_strings,

        output reg intr
    );

    localparam [ADDRWIDTH-1:0] USB_SETUP_DATA_ADDR  = 12'h000 + BASE_ADDR; // 8B RO
    localparam [ADDRWIDTH-1:0] CDC_LINE_CODE_ADDR   = 12'h008 + BASE_ADDR; // 6+2B RO
    localparam [ADDRWIDTH-1:0] SERIAL_DESC_ADDR     = 12'h010 + BASE_ADDR; // 68B RW


    reg [7:0] desc_rom [0:503];

    assign desc_dev_addr = 16'd10;
    assign desc_dev_len = 16'd18;
    assign desc_qual_addr = 16'd0;
    assign desc_qual_len = 16'd10;
    assign desc_fscfg_addr = 16'd133;
    assign desc_fscfg_len = 16'd105;
    assign desc_hscfg_addr = 16'd28;
    assign desc_hscfg_len = 16'd105;
    assign desc_oscfg_addr = 16'd238;
    assign desc_hidrpt_addr = 16'd0;
    assign desc_hidrpt_len = 16'd0;
    assign desc_bos_addr = 16'd409;
    assign desc_bos_len = 16'd33;
    assign desc_strlang_addr = 16'd442;
    assign desc_strvendor_addr = 16'd446;
    assign desc_strvendor_len = 16'd14;
    assign desc_strproduct_addr = 16'd460;
    assign desc_strproduct_len = 16'd44;
    assign desc_strserial_addr = 16'h8000;
    assign desc_strserial_len = 16'd66;
    assign desc_have_strings = 1'd1;

    localparam [15:0] desc_winusb_addr = 16'd239;
    localparam [15:0] desc_winusb_len = 16'd170;


    integer i;
    genvar gi;
    reg [31:0] serial_str_ram_word [0:16];

    wire [7:0] serial_str_ram [0:67];

    generate
        for (gi = 0; gi < 68; gi=gi+1) begin
            assign serial_str_ram[gi] = serial_str_ram_word[gi/4][(gi%4)*8+:8];
        end
    endgenerate


    reg [7:0] txdat_reg;
    reg desc_mode;
    reg [15:0] desc_manual_raddr;
    reg [15:0] desc_other_str_addr;

    wire usb_rx_active = endpt_sel == 4'd0 && rxact;
    wire usb_tx_active = endpt_sel == 4'd0 && txact;
    wire rst = !resetn || usbrst;

    wire [15:0] desc_rom_raddr = desc_mode ? desc_manual_raddr : descrom_raddr;
    assign descrom_rdata = desc_rom_raddr[15] ? serial_str_ram[desc_rom_raddr[6:0]] : desc_rom[desc_rom_raddr];
    assign txdat = desc_mode ? descrom_rdata : txdat_reg;

    reg [7:0] setup_data [0:7];
    reg [7:0] line_coding [0:6];

    wire [7:0] bmRequestType = setup_data[0];
    wire [7:0] bRequest = setup_data[1];
    wire [15:0] wValue = {setup_data[3], setup_data[2]};
    wire [15:0] wIndex = {setup_data[5], setup_data[4]};
    wire [15:0] wLength = {setup_data[7], setup_data[6]};


    localparam [2:0] IDLE = 3'd0;
    localparam [2:0] PARSE_REQUEST = 3'd1;
    localparam [2:0] SEND_WINUSB_DESC = 3'd2;
    localparam [2:0] SEND_WINUSB_DESC_TURN = 3'd3;
    localparam [2:0] GET_LINE_CODING = 3'd4;
    localparam [2:0] SET_LINE_CODING = 3'd5;

    localparam [7:0] CDC_REQUEST_SET_LINE_CODING = 8'h20;
    localparam [7:0] CDC_REQUEST_GET_LINE_CODING = 8'h21;
    localparam [7:0] CDC_REQUEST_SET_CONTROL_LINE_STATE = 8'h22;
    localparam [7:0] CDC_REQUEST_SEND_BREAK = 8'h23;

    reg [2:0] setup_sm;
    reg [2:0] rx_byte_cnt;
    reg [5:0] tx_byte_cnt;
    reg [7:0] tx_num;


    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            setup_sm <= IDLE;
            rx_byte_cnt <= 0;
            tx_byte_cnt <= 0;
            txdat_reg <= 0;
            txdat_len <= 0;
            rxrdy <= 1'd0;
            txval <= 1'd0;
            desc_mode <= 0;
            desc_manual_raddr <= 0;
            desc_other_str_addr <= 0;
            intr <= 0;
            tx_num <= 0;

            for (i = 0; i < 8; i=i+1) begin
                setup_data[i] <= 8'd0;
            end
            for (i = 0; i < 6; i=i+1) begin
                line_coding[i] <= 8'd0;
            end
        end
        else begin
            if (ahb_write_en) begin
                case (ahb_addr[ADDRWIDTH-1:2])
                    CDC_LINE_CODE_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0]) begin
                            line_coding[0] <= ahb_wdata[7:0];
                        end
                        if (ahb_byte_strobe[0]) begin
                            line_coding[1] <= ahb_wdata[15:8];
                        end
                        if (ahb_byte_strobe[1]) begin
                            line_coding[2] <= ahb_wdata[23:16];
                        end
                        if (ahb_byte_strobe[2]) begin
                            line_coding[3] <= ahb_wdata[31:24];
                        end
                    end
                    CDC_LINE_CODE_ADDR[ADDRWIDTH-1:2]+1: begin
                        if (ahb_byte_strobe[0]) begin
                            line_coding[4] <= ahb_wdata[7:0];
                        end
                        if (ahb_byte_strobe[1]) begin
                            line_coding[5] <= ahb_wdata[15:8];
                        end
                        if (ahb_byte_strobe[2]) begin
                            line_coding[6] <= ahb_wdata[23:16];
                        end
                    end
                endcase
            end

            case (setup_sm)
                IDLE: begin
                    if (setup) begin
                        if (rxval) begin
                            setup_data[rx_byte_cnt] <= rxdat;
                            rx_byte_cnt <= rx_byte_cnt + 1;
                            if (rx_byte_cnt == 3'd7) begin
                                setup_sm <= PARSE_REQUEST;
                            end
                        end
                    end
                    else begin
                        rx_byte_cnt <= 3'd0;
                    end
                end
                PARSE_REQUEST: begin  // 判断请求类型
                    if (bmRequestType[6:5] == 2'd0 && bmRequestType[1:0] == 2'd0 && bRequest == 8'h06 && wValue[15:8] == 8'h03) begin
                        // TODO 支持更多字符串描述符
                        setup_sm <= IDLE;  // 字符串描述符请求
                    end
                    else if (bmRequestType[6:5] == 2'd1 && bmRequestType[1:0] == 2'd1 && wIndex[7:0] == 16'd1) begin
                        case (bRequest)
                            CDC_REQUEST_SET_LINE_CODING: begin
                                setup_sm <= SET_LINE_CODING;
                            end
                            CDC_REQUEST_GET_LINE_CODING: begin
                                setup_sm <= GET_LINE_CODING;
                            end
                            CDC_REQUEST_SET_CONTROL_LINE_STATE: begin
                                intr <= 1'd1;
                                setup_sm <= IDLE;
                            end
                            CDC_REQUEST_SEND_BREAK: begin
                                setup_sm <= IDLE;
                            end
                            default: begin
                                setup_sm <= IDLE;
                            end
                        endcase
                    end
                    else if (bmRequestType[6:5] == 2'd2 && bRequest == 8'h20 && wIndex == 16'h07) begin
                        // WINUSB供应商请求
                        tx_num <= desc_winusb_len;
                        desc_manual_raddr <= desc_winusb_addr;
                        desc_mode <= 1'd1;
                        setup_sm <= SEND_WINUSB_DESC;
                    end
                    else begin
                        setup_sm <= IDLE;  // 默认处理无效请求
                    end
                end
                SEND_WINUSB_DESC: begin
                    if (usb_tx_active) begin
                        if (txpop) begin
                            tx_byte_cnt <= tx_byte_cnt + 1;
                            desc_manual_raddr <= desc_manual_raddr + 1;
                        end

                        if (tx_byte_cnt == 6'd63) begin
                            txval <= 1'd0;
                            tx_num <= tx_num - 8'd64;
                            setup_sm <= SEND_WINUSB_DESC_TURN;
                        end

                        if (tx_byte_cnt + 1'd1 == tx_num) begin
                            txval <= 1'd0;
                            tx_num <= 8'd0;
                            desc_mode <= 0;
                            setup_sm <= IDLE;
                        end
                    end
                    else begin
                        txdat_len <= tx_num > 64 ? 64 : tx_num;
                        tx_byte_cnt <= 6'd0;
                        txval <= 1'd1;
                    end
                end
                SEND_WINUSB_DESC_TURN: begin
                    if (!usb_tx_active) begin
                        setup_sm <= SEND_WINUSB_DESC;
                    end
                end
                GET_LINE_CODING: begin
                    txdat_len <= 12'd7;
                    txval <= 1'd1;
                    if (usb_tx_active) begin
                        if (txpop) begin
                            tx_byte_cnt <= tx_byte_cnt + 1;
                            txdat_reg <= line_coding[tx_byte_cnt + 1];
                            if (tx_byte_cnt == 6'd6) begin
                                txval <= 1'd0;
                                setup_sm <= IDLE;
                            end
                        end
                    end
                    else begin
                        tx_byte_cnt <= 6'd0;
                        txdat_reg <= line_coding[0];
                    end
                end
                SET_LINE_CODING: begin
                    rxrdy <= 1'd1;
                    if (usb_rx_active) begin
                        if (rxval) begin
                            rx_byte_cnt <= rx_byte_cnt + 1;
                            line_coding[rx_byte_cnt] <= rxdat;
                            if (rx_byte_cnt == 3'd6) begin
                                intr <= 1'd1; // 接收linecoding后触发中断
                                setup_sm <= IDLE;
                            end
                        end
                    end
                    else begin
                        rx_byte_cnt <= 3'd0;
                    end
                end
                default: begin
                    setup_sm <= IDLE;  // 安全默认
                end
            endcase

            if (usbrst) begin
                setup_sm <= IDLE;
            end

            if (ahb_read_en && ahb_addr[ADDRWIDTH-1:2] == USB_SETUP_DATA_ADDR[ADDRWIDTH-1:2]) begin
                intr <= 1'd0;
            end
        end
    end

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            for (i = 0; i < 17; i=i+1) begin
                serial_str_ram_word[i] <= 32'd0;
            end
        end
        else begin
            if (ahb_write_en) begin
                if (ahb_addr[ADDRWIDTH-1:2] >= SERIAL_DESC_ADDR[ADDRWIDTH-1:2] && ahb_addr[ADDRWIDTH-1:2] < (SERIAL_DESC_ADDR[ADDRWIDTH-1:2] + 5'd17)) begin
                    if (ahb_byte_strobe[0]) begin
                        serial_str_ram_word[ahb_addr[ADDRWIDTH-1:2]-SERIAL_DESC_ADDR[ADDRWIDTH-1:2]][7:0] <= ahb_wdata[7:0];
                    end
                    if (ahb_byte_strobe[1]) begin
                        serial_str_ram_word[ahb_addr[ADDRWIDTH-1:2]-SERIAL_DESC_ADDR[ADDRWIDTH-1:2]][15:8] <= ahb_wdata[15:8];
                    end
                    if (ahb_byte_strobe[2]) begin
                        serial_str_ram_word[ahb_addr[ADDRWIDTH-1:2]-SERIAL_DESC_ADDR[ADDRWIDTH-1:2]][23:16] <= ahb_wdata[23:16];
                    end
                    if (ahb_byte_strobe[3]) begin
                        serial_str_ram_word[ahb_addr[ADDRWIDTH-1:2]-SERIAL_DESC_ADDR[ADDRWIDTH-1:2]][31:24] <= ahb_wdata[31:24];
                    end
                end
            end
        end
    end

    always @(*) begin
        if (ahb_read_en) begin
            case (ahb_addr[ADDRWIDTH-1:2])
                USB_SETUP_DATA_ADDR[ADDRWIDTH-1:2]: begin
                    ahb_rdata = {setup_data[3], setup_data[2], setup_data[1], setup_data[0]};
                end
                USB_SETUP_DATA_ADDR[ADDRWIDTH-1:2] + 1: begin
                    ahb_rdata = {setup_data[7], setup_data[6], setup_data[5], setup_data[4]};
                end
                CDC_LINE_CODE_ADDR[ADDRWIDTH-1:2]: begin
                    ahb_rdata = {line_coding[3], line_coding[2], line_coding[1], line_coding[0]};
                end
                CDC_LINE_CODE_ADDR[ADDRWIDTH-1:2] + 1: begin
                    ahb_rdata = {8'h00, line_coding[6], line_coding[5], line_coding[4]};
                end
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h00,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h01,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h02,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h03,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h04,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h05,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h06,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h07,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h08,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h09,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h0A,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h0B,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h0C,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h0D,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h0E,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h0F,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h10,
                SERIAL_DESC_ADDR[ADDRWIDTH-1:2]+5'h11 : begin
                    ahb_rdata = serial_str_ram_word[ahb_addr[ADDRWIDTH-1:2]-SERIAL_DESC_ADDR[ADDRWIDTH-1:2]];
                end
                default: begin
                    ahb_rdata = 32'hxxxx_xxxx;
                end
            endcase
        end
        else begin
            ahb_rdata = 32'hxxxx_xxxx;
        end
    end

    initial begin
        desc_rom[  0] = 8'h0a;
        desc_rom[  1] = 8'h06;
        desc_rom[  2] = 8'h10;
        desc_rom[  3] = 8'h02;
        desc_rom[  4] = 8'h00;
        desc_rom[  5] = 8'h00;
        desc_rom[  6] = 8'h00;
        desc_rom[  7] = 8'h40;
        desc_rom[  8] = 8'h01;
        desc_rom[  9] = 8'h00;
        desc_rom[ 10] = 8'h12;
        desc_rom[ 11] = 8'h01;
        desc_rom[ 12] = 8'h10;
        desc_rom[ 13] = 8'h02;
        desc_rom[ 14] = 8'hef;
        desc_rom[ 15] = 8'h02;
        desc_rom[ 16] = 8'h01;
        desc_rom[ 17] = 8'h40;
        desc_rom[ 18] = 8'h28;
        desc_rom[ 19] = 8'h0d;
        desc_rom[ 20] = 8'h04;
        desc_rom[ 21] = 8'h02;
        desc_rom[ 22] = 8'h00;
        desc_rom[ 23] = 8'h01;
        desc_rom[ 24] = 8'h01;
        desc_rom[ 25] = 8'h02;
        desc_rom[ 26] = 8'h03;
        desc_rom[ 27] = 8'h01;
        desc_rom[ 28] = 8'h09;
        desc_rom[ 29] = 8'h02;
        desc_rom[ 30] = 8'h69;
        desc_rom[ 31] = 8'h00;
        desc_rom[ 32] = 8'h03;
        desc_rom[ 33] = 8'h01;
        desc_rom[ 34] = 8'h00;
        desc_rom[ 35] = 8'h80;
        desc_rom[ 36] = 8'h96;
        desc_rom[ 37] = 8'h09;
        desc_rom[ 38] = 8'h04;
        desc_rom[ 39] = 8'h00;
        desc_rom[ 40] = 8'h00;
        desc_rom[ 41] = 8'h03;
        desc_rom[ 42] = 8'hff;
        desc_rom[ 43] = 8'h00;
        desc_rom[ 44] = 8'h00;
        desc_rom[ 45] = 8'h02;
        desc_rom[ 46] = 8'h07;
        desc_rom[ 47] = 8'h05;
        desc_rom[ 48] = 8'h02;
        desc_rom[ 49] = 8'h02;
        desc_rom[ 50] = 8'h00;
        desc_rom[ 51] = 8'h02;
        desc_rom[ 52] = 8'h00;
        desc_rom[ 53] = 8'h07;
        desc_rom[ 54] = 8'h05;
        desc_rom[ 55] = 8'h81;
        desc_rom[ 56] = 8'h02;
        desc_rom[ 57] = 8'h00;
        desc_rom[ 58] = 8'h02;
        desc_rom[ 59] = 8'h00;
        desc_rom[ 60] = 8'h07;
        desc_rom[ 61] = 8'h05;
        desc_rom[ 62] = 8'h86;
        desc_rom[ 63] = 8'h02;
        desc_rom[ 64] = 8'h00;
        desc_rom[ 65] = 8'h02;
        desc_rom[ 66] = 8'h00;
        desc_rom[ 67] = 8'h08;
        desc_rom[ 68] = 8'h0b;
        desc_rom[ 69] = 8'h01;
        desc_rom[ 70] = 8'h02;
        desc_rom[ 71] = 8'h02;
        desc_rom[ 72] = 8'h02;
        desc_rom[ 73] = 8'h00;
        desc_rom[ 74] = 8'h00;
        desc_rom[ 75] = 8'h09;
        desc_rom[ 76] = 8'h04;
        desc_rom[ 77] = 8'h01;
        desc_rom[ 78] = 8'h00;
        desc_rom[ 79] = 8'h01;
        desc_rom[ 80] = 8'h02;
        desc_rom[ 81] = 8'h02;
        desc_rom[ 82] = 8'h00;
        desc_rom[ 83] = 8'h00;
        desc_rom[ 84] = 8'h05;
        desc_rom[ 85] = 8'h24;
        desc_rom[ 86] = 8'h00;
        desc_rom[ 87] = 8'h10;
        desc_rom[ 88] = 8'h01;
        desc_rom[ 89] = 8'h05;
        desc_rom[ 90] = 8'h24;
        desc_rom[ 91] = 8'h01;
        desc_rom[ 92] = 8'h00;
        desc_rom[ 93] = 8'h02;
        desc_rom[ 94] = 8'h04;
        desc_rom[ 95] = 8'h24;
        desc_rom[ 96] = 8'h02;
        desc_rom[ 97] = 8'h02;
        desc_rom[ 98] = 8'h05;
        desc_rom[ 99] = 8'h24;
        desc_rom[100] = 8'h06;
        desc_rom[101] = 8'h01;
        desc_rom[102] = 8'h02;
        desc_rom[103] = 8'h07;
        desc_rom[104] = 8'h05;
        desc_rom[105] = 8'h85;
        desc_rom[106] = 8'h03;
        desc_rom[107] = 8'h08;
        desc_rom[108] = 8'h00;
        desc_rom[109] = 8'h0a;
        desc_rom[110] = 8'h09;
        desc_rom[111] = 8'h04;
        desc_rom[112] = 8'h02;
        desc_rom[113] = 8'h00;
        desc_rom[114] = 8'h02;
        desc_rom[115] = 8'h0a;
        desc_rom[116] = 8'h00;
        desc_rom[117] = 8'h00;
        desc_rom[118] = 8'h00;
        desc_rom[119] = 8'h07;
        desc_rom[120] = 8'h05;
        desc_rom[121] = 8'h04;
        desc_rom[122] = 8'h02;
        desc_rom[123] = 8'h00;
        desc_rom[124] = 8'h02;
        desc_rom[125] = 8'h00;
        desc_rom[126] = 8'h07;
        desc_rom[127] = 8'h05;
        desc_rom[128] = 8'h83;
        desc_rom[129] = 8'h02;
        desc_rom[130] = 8'h00;
        desc_rom[131] = 8'h02;
        desc_rom[132] = 8'h00;
        desc_rom[133] = 8'h09;
        desc_rom[134] = 8'h02;
        desc_rom[135] = 8'h69;
        desc_rom[136] = 8'h00;
        desc_rom[137] = 8'h03;
        desc_rom[138] = 8'h01;
        desc_rom[139] = 8'h00;
        desc_rom[140] = 8'h80;
        desc_rom[141] = 8'h96;
        desc_rom[142] = 8'h09;
        desc_rom[143] = 8'h04;
        desc_rom[144] = 8'h00;
        desc_rom[145] = 8'h00;
        desc_rom[146] = 8'h03;
        desc_rom[147] = 8'hff;
        desc_rom[148] = 8'h00;
        desc_rom[149] = 8'h00;
        desc_rom[150] = 8'h02;
        desc_rom[151] = 8'h07;
        desc_rom[152] = 8'h05;
        desc_rom[153] = 8'h02;
        desc_rom[154] = 8'h02;
        desc_rom[155] = 8'h40;
        desc_rom[156] = 8'h00;
        desc_rom[157] = 8'h00;
        desc_rom[158] = 8'h07;
        desc_rom[159] = 8'h05;
        desc_rom[160] = 8'h81;
        desc_rom[161] = 8'h02;
        desc_rom[162] = 8'h40;
        desc_rom[163] = 8'h00;
        desc_rom[164] = 8'h00;
        desc_rom[165] = 8'h07;
        desc_rom[166] = 8'h05;
        desc_rom[167] = 8'h86;
        desc_rom[168] = 8'h02;
        desc_rom[169] = 8'h40;
        desc_rom[170] = 8'h00;
        desc_rom[171] = 8'h00;
        desc_rom[172] = 8'h08;
        desc_rom[173] = 8'h0b;
        desc_rom[174] = 8'h01;
        desc_rom[175] = 8'h02;
        desc_rom[176] = 8'h02;
        desc_rom[177] = 8'h02;
        desc_rom[178] = 8'h00;
        desc_rom[179] = 8'h00;
        desc_rom[180] = 8'h09;
        desc_rom[181] = 8'h04;
        desc_rom[182] = 8'h01;
        desc_rom[183] = 8'h00;
        desc_rom[184] = 8'h01;
        desc_rom[185] = 8'h02;
        desc_rom[186] = 8'h02;
        desc_rom[187] = 8'h00;
        desc_rom[188] = 8'h00;
        desc_rom[189] = 8'h05;
        desc_rom[190] = 8'h24;
        desc_rom[191] = 8'h00;
        desc_rom[192] = 8'h10;
        desc_rom[193] = 8'h01;
        desc_rom[194] = 8'h05;
        desc_rom[195] = 8'h24;
        desc_rom[196] = 8'h01;
        desc_rom[197] = 8'h00;
        desc_rom[198] = 8'h02;
        desc_rom[199] = 8'h04;
        desc_rom[200] = 8'h24;
        desc_rom[201] = 8'h02;
        desc_rom[202] = 8'h02;
        desc_rom[203] = 8'h05;
        desc_rom[204] = 8'h24;
        desc_rom[205] = 8'h06;
        desc_rom[206] = 8'h01;
        desc_rom[207] = 8'h02;
        desc_rom[208] = 8'h07;
        desc_rom[209] = 8'h05;
        desc_rom[210] = 8'h85;
        desc_rom[211] = 8'h03;
        desc_rom[212] = 8'h08;
        desc_rom[213] = 8'h00;
        desc_rom[214] = 8'h0a;
        desc_rom[215] = 8'h09;
        desc_rom[216] = 8'h04;
        desc_rom[217] = 8'h02;
        desc_rom[218] = 8'h00;
        desc_rom[219] = 8'h02;
        desc_rom[220] = 8'h0a;
        desc_rom[221] = 8'h00;
        desc_rom[222] = 8'h00;
        desc_rom[223] = 8'h00;
        desc_rom[224] = 8'h07;
        desc_rom[225] = 8'h05;
        desc_rom[226] = 8'h04;
        desc_rom[227] = 8'h02;
        desc_rom[228] = 8'h40;
        desc_rom[229] = 8'h00;
        desc_rom[230] = 8'h00;
        desc_rom[231] = 8'h07;
        desc_rom[232] = 8'h05;
        desc_rom[233] = 8'h83;
        desc_rom[234] = 8'h02;
        desc_rom[235] = 8'h40;
        desc_rom[236] = 8'h00;
        desc_rom[237] = 8'h00;
        desc_rom[238] = 8'h07;
        desc_rom[239] = 8'h0a;
        desc_rom[240] = 8'h00;
        desc_rom[241] = 8'h00;
        desc_rom[242] = 8'h00;
        desc_rom[243] = 8'h00;
        desc_rom[244] = 8'h00;
        desc_rom[245] = 8'h03;
        desc_rom[246] = 8'h06;
        desc_rom[247] = 8'haa;
        desc_rom[248] = 8'h00;
        desc_rom[249] = 8'h08;
        desc_rom[250] = 8'h00;
        desc_rom[251] = 8'h02;
        desc_rom[252] = 8'h00;
        desc_rom[253] = 8'h00;
        desc_rom[254] = 8'h00;
        desc_rom[255] = 8'ha0;
        desc_rom[256] = 8'h00;
        desc_rom[257] = 8'h14;
        desc_rom[258] = 8'h00;
        desc_rom[259] = 8'h03;
        desc_rom[260] = 8'h00;
        desc_rom[261] = 8'h57;
        desc_rom[262] = 8'h49;
        desc_rom[263] = 8'h4e;
        desc_rom[264] = 8'h55;
        desc_rom[265] = 8'h53;
        desc_rom[266] = 8'h42;
        desc_rom[267] = 8'h00;
        desc_rom[268] = 8'h00;
        desc_rom[269] = 8'h00;
        desc_rom[270] = 8'h00;
        desc_rom[271] = 8'h00;
        desc_rom[272] = 8'h00;
        desc_rom[273] = 8'h00;
        desc_rom[274] = 8'h00;
        desc_rom[275] = 8'h00;
        desc_rom[276] = 8'h00;
        desc_rom[277] = 8'h84;
        desc_rom[278] = 8'h00;
        desc_rom[279] = 8'h04;
        desc_rom[280] = 8'h00;
        desc_rom[281] = 8'h07;
        desc_rom[282] = 8'h00;
        desc_rom[283] = 8'h2a;
        desc_rom[284] = 8'h00;
        desc_rom[285] = 8'h44;
        desc_rom[286] = 8'h00;
        desc_rom[287] = 8'h65;
        desc_rom[288] = 8'h00;
        desc_rom[289] = 8'h76;
        desc_rom[290] = 8'h00;
        desc_rom[291] = 8'h69;
        desc_rom[292] = 8'h00;
        desc_rom[293] = 8'h63;
        desc_rom[294] = 8'h00;
        desc_rom[295] = 8'h65;
        desc_rom[296] = 8'h00;
        desc_rom[297] = 8'h49;
        desc_rom[298] = 8'h00;
        desc_rom[299] = 8'h6e;
        desc_rom[300] = 8'h00;
        desc_rom[301] = 8'h74;
        desc_rom[302] = 8'h00;
        desc_rom[303] = 8'h65;
        desc_rom[304] = 8'h00;
        desc_rom[305] = 8'h72;
        desc_rom[306] = 8'h00;
        desc_rom[307] = 8'h66;
        desc_rom[308] = 8'h00;
        desc_rom[309] = 8'h61;
        desc_rom[310] = 8'h00;
        desc_rom[311] = 8'h63;
        desc_rom[312] = 8'h00;
        desc_rom[313] = 8'h65;
        desc_rom[314] = 8'h00;
        desc_rom[315] = 8'h47;
        desc_rom[316] = 8'h00;
        desc_rom[317] = 8'h55;
        desc_rom[318] = 8'h00;
        desc_rom[319] = 8'h49;
        desc_rom[320] = 8'h00;
        desc_rom[321] = 8'h44;
        desc_rom[322] = 8'h00;
        desc_rom[323] = 8'h73;
        desc_rom[324] = 8'h00;
        desc_rom[325] = 8'h00;
        desc_rom[326] = 8'h00;
        desc_rom[327] = 8'h50;
        desc_rom[328] = 8'h00;
        desc_rom[329] = 8'h7b;
        desc_rom[330] = 8'h00;
        desc_rom[331] = 8'h43;
        desc_rom[332] = 8'h00;
        desc_rom[333] = 8'h44;
        desc_rom[334] = 8'h00;
        desc_rom[335] = 8'h42;
        desc_rom[336] = 8'h00;
        desc_rom[337] = 8'h33;
        desc_rom[338] = 8'h00;
        desc_rom[339] = 8'h42;
        desc_rom[340] = 8'h00;
        desc_rom[341] = 8'h35;
        desc_rom[342] = 8'h00;
        desc_rom[343] = 8'h41;
        desc_rom[344] = 8'h00;
        desc_rom[345] = 8'h44;
        desc_rom[346] = 8'h00;
        desc_rom[347] = 8'h2d;
        desc_rom[348] = 8'h00;
        desc_rom[349] = 8'h32;
        desc_rom[350] = 8'h00;
        desc_rom[351] = 8'h39;
        desc_rom[352] = 8'h00;
        desc_rom[353] = 8'h33;
        desc_rom[354] = 8'h00;
        desc_rom[355] = 8'h42;
        desc_rom[356] = 8'h00;
        desc_rom[357] = 8'h2d;
        desc_rom[358] = 8'h00;
        desc_rom[359] = 8'h34;
        desc_rom[360] = 8'h00;
        desc_rom[361] = 8'h36;
        desc_rom[362] = 8'h00;
        desc_rom[363] = 8'h36;
        desc_rom[364] = 8'h00;
        desc_rom[365] = 8'h33;
        desc_rom[366] = 8'h00;
        desc_rom[367] = 8'h2d;
        desc_rom[368] = 8'h00;
        desc_rom[369] = 8'h41;
        desc_rom[370] = 8'h00;
        desc_rom[371] = 8'h41;
        desc_rom[372] = 8'h00;
        desc_rom[373] = 8'h33;
        desc_rom[374] = 8'h00;
        desc_rom[375] = 8'h36;
        desc_rom[376] = 8'h00;
        desc_rom[377] = 8'h2d;
        desc_rom[378] = 8'h00;
        desc_rom[379] = 8'h31;
        desc_rom[380] = 8'h00;
        desc_rom[381] = 8'h41;
        desc_rom[382] = 8'h00;
        desc_rom[383] = 8'h41;
        desc_rom[384] = 8'h00;
        desc_rom[385] = 8'h45;
        desc_rom[386] = 8'h00;
        desc_rom[387] = 8'h34;
        desc_rom[388] = 8'h00;
        desc_rom[389] = 8'h36;
        desc_rom[390] = 8'h00;
        desc_rom[391] = 8'h34;
        desc_rom[392] = 8'h00;
        desc_rom[393] = 8'h36;
        desc_rom[394] = 8'h00;
        desc_rom[395] = 8'h33;
        desc_rom[396] = 8'h00;
        desc_rom[397] = 8'h37;
        desc_rom[398] = 8'h00;
        desc_rom[399] = 8'h37;
        desc_rom[400] = 8'h00;
        desc_rom[401] = 8'h36;
        desc_rom[402] = 8'h00;
        desc_rom[403] = 8'h7d;
        desc_rom[404] = 8'h00;
        desc_rom[405] = 8'h00;
        desc_rom[406] = 8'h00;
        desc_rom[407] = 8'h00;
        desc_rom[408] = 8'h00;
        desc_rom[409] = 8'h05;
        desc_rom[410] = 8'h0f;
        desc_rom[411] = 8'h21;
        desc_rom[412] = 8'h00;
        desc_rom[413] = 8'h01;
        desc_rom[414] = 8'h1c;
        desc_rom[415] = 8'h10;
        desc_rom[416] = 8'h05;
        desc_rom[417] = 8'h00;
        desc_rom[418] = 8'hdf;
        desc_rom[419] = 8'h60;
        desc_rom[420] = 8'hdd;
        desc_rom[421] = 8'hd8;
        desc_rom[422] = 8'h89;
        desc_rom[423] = 8'h45;
        desc_rom[424] = 8'hc7;
        desc_rom[425] = 8'h4c;
        desc_rom[426] = 8'h9c;
        desc_rom[427] = 8'hd2;
        desc_rom[428] = 8'h65;
        desc_rom[429] = 8'h9d;
        desc_rom[430] = 8'h9e;
        desc_rom[431] = 8'h64;
        desc_rom[432] = 8'h8a;
        desc_rom[433] = 8'h9f;
        desc_rom[434] = 8'h00;
        desc_rom[435] = 8'h00;
        desc_rom[436] = 8'h03;
        desc_rom[437] = 8'h06;
        desc_rom[438] = 8'haa;
        desc_rom[439] = 8'h00;
        desc_rom[440] = 8'h20;
        desc_rom[441] = 8'h00;
        desc_rom[442] = 8'h04;
        desc_rom[443] = 8'h03;
        desc_rom[444] = 8'h09;
        desc_rom[445] = 8'h04;
        desc_rom[446] = 8'h0e;
        desc_rom[447] = 8'h03;
        desc_rom[448] = 8'h48;
        desc_rom[449] = 8'h00;
        desc_rom[450] = 8'h46;
        desc_rom[451] = 8'h00;
        desc_rom[452] = 8'h4c;
        desc_rom[453] = 8'h00;
        desc_rom[454] = 8'h69;
        desc_rom[455] = 8'h00;
        desc_rom[456] = 8'h6e;
        desc_rom[457] = 8'h00;
        desc_rom[458] = 8'h6b;
        desc_rom[459] = 8'h00;
        desc_rom[460] = 8'h2c;
        desc_rom[461] = 8'h03;
        desc_rom[462] = 8'h48;
        desc_rom[463] = 8'h00;
        desc_rom[464] = 8'h46;
        desc_rom[465] = 8'h00;
        desc_rom[466] = 8'h4c;
        desc_rom[467] = 8'h00;
        desc_rom[468] = 8'h69;
        desc_rom[469] = 8'h00;
        desc_rom[470] = 8'h6e;
        desc_rom[471] = 8'h00;
        desc_rom[472] = 8'h6b;
        desc_rom[473] = 8'h00;
        desc_rom[474] = 8'h20;
        desc_rom[475] = 8'h00;
        desc_rom[476] = 8'h43;
        desc_rom[477] = 8'h00;
        desc_rom[478] = 8'h4d;
        desc_rom[479] = 8'h00;
        desc_rom[480] = 8'h53;
        desc_rom[481] = 8'h00;
        desc_rom[482] = 8'h49;
        desc_rom[483] = 8'h00;
        desc_rom[484] = 8'h53;
        desc_rom[485] = 8'h00;
        desc_rom[486] = 8'h2d;
        desc_rom[487] = 8'h00;
        desc_rom[488] = 8'h44;
        desc_rom[489] = 8'h00;
        desc_rom[490] = 8'h41;
        desc_rom[491] = 8'h00;
        desc_rom[492] = 8'h50;
        desc_rom[493] = 8'h00;
        desc_rom[494] = 8'h20;
        desc_rom[495] = 8'h00;
        desc_rom[496] = 8'h50;
        desc_rom[497] = 8'h00;
        desc_rom[498] = 8'h6c;
        desc_rom[499] = 8'h00;
        desc_rom[500] = 8'h75;
        desc_rom[501] = 8'h00;
        desc_rom[502] = 8'h73;
        desc_rom[503] = 8'h00;
    end

endmodule
