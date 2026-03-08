module DAP_BaudGenerator#(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    )(
        input clk,
        input sclk_in,
        input resetn,

        // AHB MEM接口
        input ahb_write_en,
        input ahb_read_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        output reg sclk_out, // 输出到GPIO的时钟信号
        output reg sclk_negedge, // 控制器置位时刻参考
        output reg sclk_sampling // 控制器采样时刻参考
    );

    localparam [ADDRWIDTH-1:0] REG_CR_ADDR = BASE_ADDR + 0;  // RW
    localparam [ADDRWIDTH-1:0] REG_TIMING_ADDR = BASE_ADDR + 4; // RW

    reg [31:0] REG_CR;

    wire REG_CR_CEN = REG_CR[0];
    // wire [2:0] REG_DELAY_CHAIN = REG_CR[18:16];
    reg [15:0] REG_TIMING_DIV;
    reg [15:0] REG_TIMING_SIMPLING;


    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            REG_CR <= 32'd0;
            REG_TIMING_DIV <= 16'd0;
            REG_TIMING_SIMPLING <= 16'd0;
        end
        else begin
            if (ahb_write_en) begin
                case (ahb_addr[ADDRWIDTH-1:2])
                    REG_CR_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            REG_CR[ 0+:8] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            REG_CR[ 8+:8] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            REG_CR[16+:8] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            REG_CR[24+:8] <= ahb_wdata[24+:8];
                    end
                    REG_TIMING_ADDR[ADDRWIDTH-1:2]: begin
                        if (!REG_CR_CEN) begin
                            if (ahb_byte_strobe[0])
                                REG_TIMING_DIV[ 0+:8] <= ahb_wdata[ 0+:8];
                            if (ahb_byte_strobe[1])
                                REG_TIMING_DIV[ 8+:8] <= ahb_wdata[ 8+:8];
                            if (ahb_byte_strobe[2])
                                REG_TIMING_SIMPLING[ 0+:8] <= ahb_wdata[16+:8];
                            if (ahb_byte_strobe[3])
                                REG_TIMING_SIMPLING[ 8+:8] <= ahb_wdata[24+:8];
                        end
                    end
                endcase
            end
        end
    end

    always @(*) begin : ahb_mem_read_ctrl
        if (ahb_read_en) begin
            case (ahb_addr[ADDRWIDTH-1:2]) /*synthesis parallel_case*/
                REG_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = REG_CR;
                REG_TIMING_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {REG_TIMING_SIMPLING, REG_TIMING_DIV};
                default:
                    ahb_rdata = {32{1'bx}};
            endcase
        end
        else begin
            ahb_rdata = {32{1'bx}};
        end
    end


    reg cen_ff1;
    reg cen;

    reg [16:0] timer_cnt;
    // reg [6:0] delay_chain_reg;
    wire [16:0] div_count_next = timer_cnt + 1'd1;

    wire sclk_out_w = (timer_cnt >= REG_TIMING_DIV);
    wire sclk_negedge_w = cen && (div_count_next == (REG_TIMING_DIV * 2));
    wire sclk_sampling_w = cen && (div_count_next == REG_TIMING_SIMPLING);
    
    // wire [7:0] delay_chain = {delay_chain_reg, sclk_sampling_w};

    always @(posedge sclk_in or negedge resetn) begin
        if (!resetn) begin
            timer_cnt <= 16'd0;
            sclk_out <= 1'd0;
            sclk_negedge <= 1'd0;
            sclk_sampling <= 1'd0;
        end
        else begin
            cen_ff1 <= REG_CR_CEN;
            cen <= cen_ff1;

            if (cen) begin
                if (sclk_negedge_w) begin
                    timer_cnt <= 17'd0;
                end
                else begin
                    timer_cnt <= div_count_next;
                end

            end
            else begin
                timer_cnt <= 17'd0;
            end
            // delay_chain_reg <= delay_chain;
             
            sclk_out <= sclk_out_w;
            sclk_negedge <= sclk_negedge_w;
            sclk_sampling <= sclk_sampling_w;
        end
    end



endmodule
