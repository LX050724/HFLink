`define CMD_REG_WIDTH               14
`define CMD_REAL_NUM                (`CMD_REG_WIDTH-2)
`define CMD_SWJ_RANGE               `CMD_REAL_NUM-1:2

/* Controller处理命令 */
`define CMD_MCU_HELPER_SHIFT        0
`define CMD_MCU_HELPER              14'b??_????_????_???1

`define CMD_DELAY_SHIFT             1
`define CMD_DELAY                   14'b??_????_????_??1?
/* SWJ处理命令 */
`define CMD_TRANSFER_BLOCK_SHIFT    2
`define CMD_TRANSFER_BLOCK          14'b??_????_????_?1??

`define CMD_TRANSFER_SHIFT          3
`define CMD_TRANSFER                14'b??_????_????_1???

`define CMD_WRITE_ABORT_SHIFT       4
`define CMD_WRITE_ABORT             14'b??_????_???1_????

`define CMD_SWJ_PINS_SHIFT          5
`define CMD_SWJ_PINS                14'b??_????_??1?_????

`define CMD_SWJ_SEQUENCE_SHIFT      6
`define CMD_SWJ_SEQUENCE            14'b??_????_?1??_????

`define CMD_SWD_SEQUENCE_SHIFT      7
`define CMD_SWD_SEQUENCE            14'b??_????_1???_????

`define CMD_JTAG_SEQUENCE_SHIFT     8
`define CMD_JTAG_SEQUENCE           14'b??_???1_????_????

`define CMD_JTAG_IDCODE_SHIFT       9
`define CMD_JTAG_IDCODE             14'b??_??1?_????_????

`define CMD_RV_TRANSFER_BLOCK_SHIFT 10
`define CMD_RV_TRANSFER_BLOCK       14'b??_?1??_????_????

`define CMD_RV_TRANSFER_SHIFT       11
`define CMD_RV_TRANSFER             14'b??_1???_????_????
/* 不处理命令 */
`define CMD_TRANSFER_ABORT_SHIFT    12
`define CMD_TRANSFER_ABORT          14'b?1_????_????_????
/* 虚命令 */
`define CMD_EXEC_CMD_SHIFT          13
`define CMD_EXEC_CMD                14'b1?_????_????_????

/**
 * SWJ SEQ
 * | 15 14 13 | 12 11 10  9  8 | 7  6  5  4  3  2  1  0 |
 * |  0  0  0 |  0  0  0  0  0 |   Sequence Bit Count   |
 * Sequence Bit Count [1-64]
 */
`define SEQ_CMD_SWJ_SEQ             3'd0

 /**
 * SWD SEQ
 * | 15 14 13 | 12 11 10  9  8 |    7  | 6  5  4  3  2  1  0 |
 * |  0  0  1 |  0  0  0  0  0 | mode  |   Number of cycles  |
 * Number of cycles [1-64]
 */
`define SEQ_CMD_SWD_SEQ             3'd1
`define SEQ_CMD_SWD_TRANSFER        3'd2
`define SEQ_CMD_JTAG_SEQ            3'd3
`define SEQ_CMD_JTAG_READID         3'd4
`define SEQ_CMD_JTAG_ABORT          3'd5
`define SEQ_CMD_JTAG_IR             3'd6
`define SEQ_CMD_JTAG_TRANSFER       3'd7
