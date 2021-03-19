/*******************************************************************************
* File Name:  B_SPI_Master_v2_50.v
* Version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
*
* Description:
*  This file provides a base level model of the SPI Master component
*
* Note:
*  None
********************************************************************************
*                 Control and Status Register definitions
********************************************************************************
*
*  Control Register Definition
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+
*  |  Bit  |   7    |   6    |   5    |   4    |   3    |   2    |   1    |   0    |
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+
*  | Desc  | unused | unused | unused | unused |unused  |CPHA_SET|CPOL_SET| TX_EN  |
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+
*
*      tx_en        =>   0 = disable tx (for Bidirectional Mode only)
*                        1 = enable  tx (fof Bidirectional Mode only)
*
*      cpol_set     =>   0 = CPOL -> 0
*                        1 = CPOL -> 1
*
*      cpha_set     =>   0 == CPHA -> 0
*                        1 == CPHA -> 1
*
*  Tx interrupt Status Register Definition
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+
*  |  Bit  |   7    |   6    |   5    |   4    |   3    |   2    |   1    |   0    |
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+
*  | Desc |interrupt|unused  |unused  |spi_idle|bt_cpmlt|tx_f_n_f|tx_f_emp|spi_done|
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+

*  Rx interrupt Status Register Definition
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+
*  |  Bit  |   7    |   6    |   5    |   4    |   3    |   2    |   1    |   0    |
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+
*  | Desc |interrupt|rx_overr|rx_n_emp|rx_full |unused  | unused |unused  | unused |
*  +=======+--------+--------+--------+--------+--------+--------+--------+--------+
*
*    spi_done   =>  0 = spi transmission not done
*                   1 = spi transmission done
*
*    tx_f_e     =>  0 = TX FIFO not empty
*                   1 = TX FIFO empty
*
*    tx_f_n_f   =>  0 = TX_FIFO full
*                   1 = TX FIFO not full
*
*    rx_f_fll   =>  0 =  RX FIFO not full
*                   1 =  RX FIFO full
*
*    rx_f_n_e   =>  0 = RX FIFO empty
*                   1 = RX FIFO not empty
*
*    rx_f_over  =>  0 = RX FIFO not overrun
*                   1 = RX FIFO overrun
*
*    bt_cmplt   =>  0 = byte transfer is not complete
*                   1 = byte transfer complete
*
********************************************************************************
*                 Data Path register definitions
********************************************************************************
*  INSTANCE NAME:  DatapathName
*  DESCRIPTION:
*  REGISTER USAGE:
*    F0 => TX FIFO buffer
*    F1 => RX FIFO buffer
*    D0 => na
*    D1 => na
*    A0 => SPI Master TX value
*    A1 => SPI Master RX value
*
********************************************************************************
*               I*O Signals:
********************************************************************************
*  IO SIGNALS:
*
*    reset        input     component reset input
*    clock        input     component clock input
*    miso         input     SPI MISO input
*    sclk         output    SPI SCLK output
*    ss           output    SPI SS output
*    tx_enable    output    tx enable output(is used for Bidirectional Mode only)
*    mosi         output    SPI MOSI output
*    interrupt    output    interrupt output
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

`include "cypress.v"
`ifdef B_SPI_Master_v2_50__V_ALREADY_INCLUDED
`else
`define B_SPI_Master_v2_50__V_ALREADY_INCLUDED

module B_SPI_Master_v2_50(
    input    wire reset,      /* System Reset                               */
    input    wire clock,      /* User Supplied clock = 2x bitrate of output */
    input    wire miso,       /* SPI MISO input                             */

    output    wire mosi,       /* SPI MOSI output              */
    output    reg  sclk,       /* SPI SCLK output              */
    output    reg  ss,         /* SPI SS output                */
    output  wire tx_interpt, /* Interrupt output             */
    output  wire rx_interpt, /* Interrupt output             */
    output  wire tx_enable   /* TX enable (for BidirectMode) */
);

    /* Status Register inputs from the PLD/DP's */
    wire [6:0] tx_status;
    wire [6:0] rx_status;
    /*  Master Out Slave In from the Datapath. Selects between mosi_dp8 and mosi_dp16 based on NUM_BITS */
    wire mosi_from_dp;

    /* One compare output of the counter which signals when to load received data into the FIFO */
    wire dpcounter_one;
    wire dpcounter_zero;

    /* 7-bit counter output used for a compare to one output */
    wire [6:0] count;
    wire miso_to_dp;

    /* MOSI FIFO Status outputs */
    wire dpMOSI_fifo_not_empty;
    wire nc1,nc2,nc3,nc4;
    reg [2:0] state;

    /* bit order: default is MSb first (i.e Shift Left and ShiftLeft in static configuration is = 0) */
    /* DO NOT CHANGE these two parameters.  They define constants */
    localparam SPIM_MSB_FIRST = 1'b0;
    localparam SPIM_LSB_FIRST = 1'b1;

    /* State Machine state names */
    localparam SPIM_STATE_IDLE                = 3'h0;
    localparam SPIM_STATE_LOAD_TX_DATA        = 3'h1;
    localparam SPIM_STATE_SEND_TX_DATA        = 3'h2;
    localparam SPIM_STATE_CAPT_RX_DATA        = 3'h3;
    localparam SPIM_STATE_SHFT_N_LD_TX_DATA   = 3'h4;
    localparam SPIM_STATE_SPI_DONE            = 3'h5;
    localparam SPIM_STATE_WAIT                = 3'h6;
    localparam SPIM_STATE_SEND_TX_DATA_2      = 3'h7;

    /* Status Register bits */
    localparam SPIM_STS_SPI_DONE_BIT          = 3'd0;
    localparam SPIM_STS_TX_FIFO_EMPTY_BIT     = 3'd1;
    localparam SPIM_STS_TX_FIFO_NOT_FULL_BIT  = 3'd2;
    localparam SPIM_STS_BYTE_COMPLETE_BIT     = 3'd3;
    localparam SPIM_STS_SPI_IDLE_BIT          = 3'd4;
    localparam SPIM_STS_RX_FIFO_FULL_BIT      = 3'd4;
    localparam SPIM_STS_RX_FIFO_NOT_EMPTY_BIT = 3'd5;
    localparam SPIM_STS_RX_FIFO_OVERRUN_BIT   = 3'd6;

    localparam CTRL_TX_PERMISSION  = 1'b0;
    localparam CTRL_CPOL_SET       = 1'h1;
    localparam CTRL_CPHA_SET       = 2'h2;

    /*******************************************************************************
    *User parameters used to define how the component is compiled
    ******************************************************************************/
    parameter [0:0] ShiftDir         = SPIM_MSB_FIRST;
    parameter [6:0] NumberOfDataBits = 7'd8; /* set to 2-16 bits only. Default is 8 bits */
    parameter       BidirectMode     = 1'b1;
    parameter       HighSpeedMode    = 0;

    parameter [0:0] ModeCPHA = 1'b0; /* Default is rising edge mode */
    parameter [0:0] ModePOL  = 1'b0; /* Default is rising edge mode */

    localparam [2:0] dpMsbVal   = (NumberOfDataBits % 8)-3'b1;

    localparam [7:0] dpMISOMask = (NumberOfDataBits == 8 || NumberOfDataBits == 16) ? 8'b1111_1111 :
                                  (NumberOfDataBits == 7 || NumberOfDataBits == 15) ? 8'b0111_1111 :
                                  (NumberOfDataBits == 6 || NumberOfDataBits == 14) ? 8'b0011_1111 :
                                  (NumberOfDataBits == 5 || NumberOfDataBits == 13) ? 8'b0001_1111 :
                                  (NumberOfDataBits == 4 || NumberOfDataBits == 12) ? 8'b0000_1111 :
                                  (NumberOfDataBits == 3 || NumberOfDataBits == 11) ? 8'b0000_0111 :
                                  (NumberOfDataBits == 2 || NumberOfDataBits == 10) ? 8'b0000_0011 :
                                  (NumberOfDataBits == 9) ? 8'b0000_0001 : 8'b1111_1111;

    localparam [1:0] dynShiftDir     = (ShiftDir == SPIM_MSB_FIRST) ? 2'd1 : 2'd2;
    localparam [1:0] dp16MSBSIChoice = (ShiftDir == SPIM_MSB_FIRST) ? `SC_SI_A_CHAIN : `SC_SI_A_ROUTE;
    localparam [1:0] dp16LSBSIChoice = (ShiftDir == SPIM_MSB_FIRST) ? `SC_SI_A_ROUTE : `SC_SI_A_CHAIN;

    //localparam f1_ld_src = (ModeCPHA == 1) ? `SC_FIFO1_ALU :`SC_FIFO1__A1;
    localparam f1_ld_src = `SC_FIFO1_ALU;
    localparam [6:0] BitCntPeriod = (NumberOfDataBits << 1) - 1;
    localparam SR8  = 8'd8;

    localparam dp8_cfg = {
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM0:          IDLE */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM1:          LOAD F0 to A0 */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM2:          Change Shift Out */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM3:          Capture Shift In */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM4:          LDSHIFT */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM5:         END */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM6:          Change Shift Out */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM7:         */
        dpMISOMask, 8'h00,    /*CFG9:           */
        8'hFF, 8'hFF,    /*CFG11-10:           */
        `SC_CMPB_A1_D1, `SC_CMPA_A1_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_ENBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
        `SC_SI_A_ROUTE, /*CFG13-12:           */
        `SC_A0_SRC_ACC, ShiftDir, 1'h0,
        1'h0, f1_ld_src, `SC_FIFO0_BUS,
        `SC_MSB_ENBL, dpMsbVal, `SC_MSB_NOCHN,
        `SC_FB_NOCHN, `SC_CMP1_NOCHN,
        `SC_CMP0_NOCHN, /*CFG15-14:           */
        3'h00, `SC_FIFO_SYNC_NONE, 6'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_AX,
        `SC_FIFO_LEVEL,`SC_FIFO_ASYNC,`SC_EXTCRC_DSBL,
        `SC_WRK16CAT_DSBL  /*CFG17-16:           */
    };

    localparam dp16_lsb_cfg = {
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM0:          IDLE */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM1:          LOAD F0 to A0 */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM2:          Change Shift Out */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM3:          Capture Shift In */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM4:          LDSHIFT */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM5:          END */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM6:          Change Shift Out */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM7:          */
        8'hFF, 8'h00, /*CFG9:          */
        8'hFF, 8'hFF, /*CFG11-10:          */
        `SC_CMPB_A1_D1, `SC_CMPA_A1_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
        dp16LSBSIChoice, /*CFG13-12:           */
        `SC_A0_SRC_ACC, ShiftDir, 1'h0,
        1'h0, f1_ld_src, `SC_FIFO0_BUS,
        `SC_MSB_DSBL, `SC_MSB_BIT7, `SC_MSB_NOCHN,
        `SC_FB_NOCHN, `SC_CMP1_NOCHN,
        `SC_CMP0_NOCHN, /*CFG15-14:           */
         3'h00, `SC_FIFO_SYNC_NONE, 6'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_AX,
        `SC_FIFO_LEVEL,`SC_FIFO_ASYNC,`SC_EXTCRC_DSBL,
        `SC_WRK16CAT_DSBL /*CFG17-16:           */
    };

    localparam dp16_msb_cfg = {
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM0:          IDLE */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM1:          LOAD F0 to A0 */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM2:          Change Shift Out */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM3:          Capture Shift In */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM4:          LDSHIFT */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM5:          END */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM6:          Change Shift Out */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        dynShiftDir, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /*CFGRAM7:          */
        dpMISOMask, 8'h00, /*CFG9:          */
        8'hFF, 8'hFF, /*CFG11-10:          */
        `SC_CMPB_A1_D1, `SC_CMPA_A1_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_ENBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
        dp16MSBSIChoice, /*CFG13-12:           */
        `SC_A0_SRC_ACC, ShiftDir, 1'h0,
        1'h0, f1_ld_src, `SC_FIFO0_BUS,
        `SC_MSB_ENBL, dpMsbVal, `SC_MSB_NOCHN,
        `SC_FB_NOCHN, `SC_CMP1_NOCHN,
        `SC_CMP0_NOCHN, /*CFG15-14:           */
         3'h00, `SC_FIFO_SYNC_NONE, 6'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_AX,
        `SC_FIFO_LEVEL,`SC_FIFO_ASYNC,`SC_EXTCRC_DSBL,
        `SC_WRK16CAT_DSBL /*CFG17-16:           */
    };

    /* Clock Enable primitive instantiation */
    cy_psoc3_udb_clock_enable_v1_0 #(.sync_mode(`TRUE))
    ClkEn (
        .clock_in(clock),
        .enable(1'b1),
        .clock_out(clk_fin)
    );

    wire dpMOSI_fifo_not_full;
    wire dpMOSI_fifo_empty;
    wire dpMISO_fifo_not_empty;
    wire dpMISO_fifo_full;
    wire miso_buf_overrun;
    wire tmp1;
    wire mosi_from_dpL;
    wire mosi_from_dpR;
    wire [7:0] control;
    wire cnt_tc;
    reg dpcounter_one_reg;
    reg ld_ident;
    reg load_cond;
    wire dyn_cpol_control;
    wire dyn_cpha_control;
	wire dyn_cpol_control_sync;
	wire dyn_cpha_control_sync;
    // wire load_rx_data = (HighSpeedMode == 1 && ModeCPHA == 1) ?
    //                     dpcounter_one_reg : dpcounter_one; 
    
    wire load_rx_data = (HighSpeedMode == 1 && dyn_cpha_control_sync == 1) ?
                         dpcounter_one_reg : dpcounter_one;     
                         
    //wire pol_supprt = (ModePOL == 1) ? 1'b1: 1'b0;
    
    wire pol_supprt = (dyn_cpol_control_sync == 1) ? 1'b1: 1'b0;
    
    wire pre_tx_enable;
    wire so_send;
    wire pre_mosi;
    wire mosi_after_ld;
    wire mosi_cpha_1, mosi_cpha_0 , mosi_fin;
    reg cnt_enable;
    reg byte_complete;
    reg mosi_pre_reg, mosi_reg;
    reg so_send_reg;
    reg mosi_from_dp_reg;
    reg mosi_hs_reg;
    reg is_spi_done;

    assign miso_to_dp      = miso;
    assign mosi_after_ld   = so_send | so_send_reg;
    assign mosi            = (HighSpeedMode == 1) ? mosi_fin : mosi_reg;
    //assign mosi_fin      = (ModeCPHA == 1) ? mosi_cpha_1 : mosi_cpha_0;
    
    assign mosi_fin        = (dyn_cpha_control_sync == 1) ? mosi_cpha_1 : mosi_cpha_0;
    
    assign so_send         = (state == SPIM_STATE_SEND_TX_DATA) ? mosi_from_dp : 1'b0;
    assign mosi_cpha_0     = (state == SPIM_STATE_SPI_DONE || ss) ? 1'b0 : mosi_hs_reg;
    assign mosi_cpha_1     =  ss ? 1'b0 : mosi_hs_reg;
    assign pre_mosi        = ((count[4:0] == BitCntPeriod) || (count[4:0] == (BitCntPeriod-1))) ?
                               mosi_pre_reg | mosi_reg : mosi_reg;

    assign dpcounter_zero  = (count[4:0] == 5'h0);
    assign dpcounter_one   = (count[4:0] == 5'h01);

    always@(posedge clk_fin)
    begin
        if(state == SPIM_STATE_SHFT_N_LD_TX_DATA)
        begin
            load_cond <= 1'b1;
        end
        else if(count[4:0] == 5'h0)
        begin
            load_cond <= 1'b0;
        end
        else
        begin
            load_cond <= load_cond;
        end
    end

    always@(posedge clk_fin)
    begin
        dpcounter_one_reg <= dpcounter_one;
        mosi_from_dp_reg  <= mosi_from_dp;
    end

    assign tx_status[SPIM_STS_SPI_DONE_BIT]          = (state == SPIM_STATE_SPI_DONE);
    assign tx_status[SPIM_STS_TX_FIFO_EMPTY_BIT]     = dpMOSI_fifo_empty;
    assign tx_status[SPIM_STS_TX_FIFO_NOT_FULL_BIT]  = dpMOSI_fifo_not_full;
    assign tx_status[SPIM_STS_BYTE_COMPLETE_BIT]     = dpcounter_one;
    assign tx_status[SPIM_STS_SPI_IDLE_BIT]          = (state == SPIM_STATE_IDLE);

    assign rx_status[SPIM_STS_RX_FIFO_FULL_BIT]      = dpMISO_fifo_full;
    assign rx_status[SPIM_STS_RX_FIFO_NOT_EMPTY_BIT] = dpMISO_fifo_not_empty;
    assign rx_status[SPIM_STS_RX_FIFO_OVERRUN_BIT]   = dpcounter_one & dpMISO_fifo_full;

    assign tx_status[6:5] = 2'h0;
    assign rx_status[3:0] = 4'h0;

    generate    
               
        cy_psoc3_control #(.cy_force_order(1), .cy_ctrl_mode_1(8'h00), .cy_ctrl_mode_0(8'h01))
            CtrlReg
            (
                /*  input         */ .clock(clk_fin),
                /* output [07:00] */ .control(control)
            );

        assign dyn_cpol_control = control[CTRL_CPOL_SET];
        assign dyn_cpha_control = control[CTRL_CPHA_SET];       
        assign tx_enable  = 1'b0;       
   
    endgenerate

    cy_psoc3_sync sync_1(
        .sc_out(dyn_cpol_control_sync),
        .sc_in(dyn_cpol_control),
        .clock(clk_fin)
    );
	cy_psoc3_sync sync_2(
        .sc_out(dyn_cpha_control_sync),
        .sc_in(dyn_cpha_control),
        .clock(clk_fin)
    );

    generate
    
    if(HighSpeedMode)
    begin   
        always@(posedge clk_fin) 
        begin
            if(dyn_cpha_control_sync)
            begin
                mosi_reg    <= mosi_pre_reg;
                so_send_reg <= so_send;
            end
        else
            begin
                mosi_reg <= mosi_pre_reg;
                so_send_reg <= 1'b0;
            end
        end

        /* State Logic */
        always@(posedge clk_fin)   
        begin
                        
            if(!reset)
    	    begin
    		    case(state)
    		        SPIM_STATE_IDLE: 
                    begin
                        if(dpMOSI_fifo_empty)
                        begin
                            state <= SPIM_STATE_IDLE;
                        end
                        else 
                        begin
                            state <= SPIM_STATE_LOAD_TX_DATA;
                        end
                    end

                    SPIM_STATE_LOAD_TX_DATA:
                    begin
     			        if(dyn_cpha_control_sync)
    		     	    begin
                            state <= SPIM_STATE_WAIT;
                        end
                        else
    			        begin
                            state <= SPIM_STATE_SEND_TX_DATA_2;
                        end
                    end
                                             
                    SPIM_STATE_WAIT: 
                    begin
                        if(dyn_cpha_control_sync)
                        begin 
                            state <= SPIM_STATE_SEND_TX_DATA;
                        end
                        else
                        begin
                            state <= SPIM_STATE_CAPT_RX_DATA;
                        end
                    end      
                                  
    	            SPIM_STATE_SEND_TX_DATA_2:
                    begin
                        state <= SPIM_STATE_WAIT;
                    end

                    SPIM_STATE_SEND_TX_DATA: 
                    begin
                        if(dyn_cpha_control_sync)
                        begin
                            if(count[4:0] != 5'h01)
                            begin
                                state <= SPIM_STATE_CAPT_RX_DATA;
                            end
                            else
                            begin
                                if(is_spi_done)
                                begin
                                   state <= SPIM_STATE_SPI_DONE;
                                end
                                else
                                begin
                                    state <= SPIM_STATE_CAPT_RX_DATA;
                                end
                            end
                        end
                        else
                        begin
                            state <= SPIM_STATE_CAPT_RX_DATA;
                        end
                    end

    		        SPIM_STATE_CAPT_RX_DATA: 
                    begin
                        if(dyn_cpha_control_sync)
                        begin
                            if(count[4:0] != 5'h04)
                            begin
                                state <= SPIM_STATE_SEND_TX_DATA;
                            end 
                            else
                            begin
                                if(!dpMOSI_fifo_empty)
                                begin
                                    state <= SPIM_STATE_SHFT_N_LD_TX_DATA;
                                end
                                else
                                begin
                                    is_spi_done <= 1'b1;
                                    state <= SPIM_STATE_SEND_TX_DATA;
                                end
                            end
                        end
                        else
                            begin
                                if(count[4:0] == 5'h06 && !dpMOSI_fifo_empty) 
                                begin
                                    state <= SPIM_STATE_SHFT_N_LD_TX_DATA;
                                end
                                else   
                                    if(count[4:0] != 5'h02)
                                    begin
                                        state <= SPIM_STATE_SEND_TX_DATA;
                                    end
                                    else 
                                    begin
                                        if(!ld_ident) 
                                        begin
                                            state <= SPIM_STATE_SPI_DONE;
                                        end 
                                        else
                                        begin
                                            state <= SPIM_STATE_SEND_TX_DATA;
                                        end
                                    end
                            end
                    end
    		
    		        SPIM_STATE_SHFT_N_LD_TX_DATA: 
                    begin
                        state <= SPIM_STATE_CAPT_RX_DATA;
                    end

                    SPIM_STATE_SPI_DONE: 
                    begin
                        if(dyn_cpha_control_sync)
                        begin
                            is_spi_done <= 1'b0;
                        end
                     
                        state <= SPIM_STATE_IDLE;
                    end
                                       
                    default: 
                    begin
                        state  <= SPIM_STATE_IDLE;
                    end
                    
                endcase
            end    
            else
                begin
                    state <= SPIM_STATE_IDLE;
                end
        end            

        /* Output Logic */
        always@(posedge clk_fin) 
        begin
            case(state)
                SPIM_STATE_IDLE: 
                begin
                    ss           <= 1'b1;
                    cnt_enable   <= 1'b0;
                    mosi_pre_reg <= 1'b0;
                    sclk         <= pol_supprt;
                end
                   
                SPIM_STATE_LOAD_TX_DATA: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        cnt_enable  <= 1'b1;
                        ss       <= 1'b0;
                        mosi_hs_reg <= mosi_from_dp;
                    end
                    else
                    begin
                        sclk     <= pol_supprt;
                        ss       <= 1'b0;
                    end
                end

                SPIM_STATE_WAIT: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        mosi_hs_reg  <= mosi_from_dp;
                        mosi_pre_reg <= mosi_from_dp;
                        sclk         <= ~pol_supprt;
                    end
                    else
                    begin
                        cnt_enable <= 1'b1;
                        sclk       <= ~pol_supprt;
                    end
                end

                SPIM_STATE_SEND_TX_DATA_2: 
                begin
                    mosi_pre_reg <= mosi_from_dp;
                    mosi_hs_reg  <= mosi_from_dp;
                end
  	        
                SPIM_STATE_SEND_TX_DATA: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        if(count[4:0]!= 5'h01)
                        begin
                            sclk         <= pol_supprt;
                            mosi_pre_reg <= mosi_from_dp;
                        end 
                        else
                        begin
                            sclk <= pol_supprt;
                            
                            if(!ld_ident)
                            begin
                                mosi_pre_reg <= mosi_from_dp;
                            end
                            
                            if(is_spi_done)
                                begin
                                    cnt_enable <= 1'b0;
                                end
                            end
                        end
                        else
                        begin
                            if(count[4:0]!=5'h01)
                            begin
                                mosi_pre_reg <= mosi_from_dp;
                            end
                            else 
                            begin
                                ld_ident <= 1'b0;
                            end
                            
                            sclk <= ~pol_supprt;
                        end
                    end

                SPIM_STATE_CAPT_RX_DATA:
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        mosi_hs_reg <= mosi_from_dp_reg;
                        sclk        <= ~pol_supprt;
                    end
                    else
                    begin
                        mosi_hs_reg <= mosi_from_dp_reg;
                    
                        if(count[4:0] == 5'h0 && !ld_ident) 
                        begin
                            sclk         <= pol_supprt;
                            mosi_pre_reg <= mosi_from_dp;
                        end 
                        else 
                            if(count[4:0] != 5'h02)
                            begin
                                sclk <= pol_supprt;
                            end
                            else 
                            begin
                                sclk <= pol_supprt;
                            
                                if(!ld_ident)
                                begin
                                    mosi_pre_reg <= 1'b0;
                                end
                            end
                    end
                end
                       
                SPIM_STATE_SHFT_N_LD_TX_DATA: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        ld_ident <= 1'b1;
                        sclk     <= pol_supprt;
                    end
                    else
                    begin
                        sclk         <= ~pol_supprt;
                        ld_ident     <= 1'b1;
                        mosi_pre_reg <= mosi_from_dp;
                    end
                    
                end
                    
                SPIM_STATE_SPI_DONE: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        mosi_pre_reg <= 1'b0;
                        cnt_enable   <= 1'b0;
                        sclk         <= pol_supprt;
                    end
                    else
                    begin
                        mosi_pre_reg <= 1'b0;
                        ss           <= 1'b1;
                        cnt_enable   <= 1'b0;
                        sclk         <= pol_supprt;
                    end
                end
                  
                default:
                begin
                    ss           <= 1'b1;
                    cnt_enable   <= 1'b0;
                    mosi_pre_reg <= 1'b0;
                    sclk         <= pol_supprt;
                    ld_ident     <= 1'b0;                        
                end
            endcase
        end
    end    
    else
    begin
        /* State Logic */
        always@(posedge clk_fin) 
        begin
            if(dyn_cpha_control_sync)
            begin
                mosi_pre_reg <= 1'b0;
                so_send_reg <= 1'b0;                
            end
            else
            begin
                mosi_pre_reg <= 1'b0;
                so_send_reg  <= 1'b0;
            end

            if(!reset) 
            begin
                case(state)
                    SPIM_STATE_IDLE: 
                    begin
                        if(dpMOSI_fifo_empty)
                        begin
                            state <= SPIM_STATE_IDLE;
                        end
                        else 
                        begin
                            state <= SPIM_STATE_LOAD_TX_DATA;
                        end
                    end

                    SPIM_STATE_LOAD_TX_DATA: 
                    begin
                        if(dyn_cpha_control_sync)
                        begin
    			            state <= SPIM_STATE_SEND_TX_DATA;
                        end
                        else
                        begin
    		                state <= SPIM_STATE_SEND_TX_DATA_2;
                        end
                    end

                    SPIM_STATE_SEND_TX_DATA_2: 
                    begin
                        state <= SPIM_STATE_CAPT_RX_DATA;
                    end 

                    SPIM_STATE_CAPT_RX_DATA:
                    begin
                        if(dyn_cpha_control_sync)
                        begin
                            if(count[4:0] != 5'h01)
                            begin
                                state <= SPIM_STATE_SEND_TX_DATA;
                            end 
                            else
                            begin
                                state <= SPIM_STATE_SPI_DONE;
                            end
                        end
                        else
                        begin
                            if(count[4:0] != 5'h05)
                            begin
                                state <= SPIM_STATE_SEND_TX_DATA;
                            end 
                            else 
                                if(!dpMOSI_fifo_empty)
                                begin
                                    state <= SPIM_STATE_SHFT_N_LD_TX_DATA;
                                end
                                else
                                begin
                                    state <= SPIM_STATE_SEND_TX_DATA;
                                end
                        end
                    end

                    SPIM_STATE_SEND_TX_DATA:
                    begin
                        if(dyn_cpha_control_sync)
                        begin
                            if(count[4:0] != 5'h02)
                            begin
                                state <= SPIM_STATE_CAPT_RX_DATA;
                            end 
                            else
                            begin
                                if(!dpMOSI_fifo_empty)
                                begin
                                    state <= SPIM_STATE_SHFT_N_LD_TX_DATA;
                                end 
                                else
                                begin
                                    state <= SPIM_STATE_CAPT_RX_DATA;
                                end
                            end
                        end
                        else
                        begin
                            if(count[4:0] != 5'h02)
                            begin
                                state <= SPIM_STATE_CAPT_RX_DATA;
                            end
                            else
                            begin
                                if(!ld_ident) 
                                begin
                                    state <= SPIM_STATE_SPI_DONE;
                                end 
                                else
                                begin
                                    state <= SPIM_STATE_CAPT_RX_DATA;
                                end
                            end       
                        end
                    end
     
                    
                    SPIM_STATE_SHFT_N_LD_TX_DATA:
                    begin
                        if(dyn_cpha_control_sync)
                        begin
                            state <= SPIM_STATE_SEND_TX_DATA;
                        end
                        else
                        begin
                            state <= SPIM_STATE_CAPT_RX_DATA;
                        end
                    end
                             
                    SPIM_STATE_SPI_DONE: 
                    begin
                        state <= SPIM_STATE_IDLE;
                    end

                    default: 
                    begin
                        state <= SPIM_STATE_IDLE;
                    end
                endcase
            end
            else
            begin
                state <= SPIM_STATE_IDLE;
            end   
        end     

        /* Output Logic */
        always@(posedge clk_fin) 
        begin
        
            if(dyn_cpha_control_sync)
            begin
                ld_ident <= 1'b0;
            end
            
            case(state)
                SPIM_STATE_IDLE: 
                begin
                    ss         <= 1'b1;
                    cnt_enable <= 1'b0;
                    mosi_reg   <= 1'b0;
                    sclk       <= pol_supprt;
                end 

                SPIM_STATE_LOAD_TX_DATA: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        cnt_enable <= 1'b1;
                        ss         <= 1'b0;
                        sclk       <= pol_supprt;
                    end
                    else
                    begin
                        ss       <= 1'b0;
                        sclk     <= pol_supprt;
                    end
                end

                SPIM_STATE_SEND_TX_DATA_2:
                begin
                    mosi_reg <= mosi_from_dp;
                end

                SPIM_STATE_CAPT_RX_DATA: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        if(count[4:0]!= 5'h01)
                        begin
                            sclk  <= pol_supprt;
                        end
                        else
                        begin
                            cnt_enable <= 1'b0;
                            sclk       <= pol_supprt;
                        end
                    end
                    else
                    begin
                        cnt_enable <= 1'b1;
                        sclk       <= ~pol_supprt;
                    end
                end

                SPIM_STATE_SEND_TX_DATA:
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        sclk     <= ~pol_supprt;
                        mosi_reg <= mosi_from_dp;
                    end
                    else
                    begin
                        if(count[4:0] != 5'h02)
                        begin
                            sclk     <= pol_supprt;
                            mosi_reg <= mosi_from_dp;
                        end
                        else 
                        begin
                            sclk <= pol_supprt;
                    
                            if(!ld_ident) 
                            begin
                                mosi_reg <= 1'b0;
                            end 
                            else
                            begin
                                ld_ident <= 1'b0;
                                mosi_reg <= mosi_from_dp;
                            end
                        end
                    end
                end

                SPIM_STATE_SHFT_N_LD_TX_DATA:
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        sclk  <= pol_supprt;
                    end
                    else
                    begin
                        sclk       <= pol_supprt;
                        ld_ident   <= 1'b1;
                        mosi_reg   <= mosi_from_dp;
                    end
                end

                SPIM_STATE_SPI_DONE: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        mosi_reg   <= 1'b0;
                        cnt_enable <= 1'b0;
                        sclk       <= pol_supprt;
                    end
                    else
                    begin
                        mosi_reg   <= 1'b0;
                        ss         <= 1'b1;
                        cnt_enable <= 1'b0;
                        sclk       <= pol_supprt;
                    end
                end
                       
                default: 
                begin
                    if(dyn_cpha_control_sync)
                    begin
                        ss         <= 1'b1;
                        cnt_enable <= 1'b0;
                        mosi_reg   <= 1'b0;
                        sclk       <= pol_supprt;
                    end
                    else
                    begin
                        ss         <= 1'b1;
                        mosi_reg   <= 1'b0;
                        cnt_enable <= 1'b0;
                        ld_ident   <= 1'b0;
                        sclk       <= pol_supprt;
                    end
                end
            endcase
        end
    end
    
    endgenerate

    /* Couter7 instantiation */
    cy_psoc3_count7 #(.cy_period(BitCntPeriod),.cy_route_ld(0),.cy_route_en(1))
    BitCounter(
        /* input          */ .clock(clk_fin),
        /* input          */ .reset(reset),
        /* input          */ .load(1'b0),
        /* input          */ .enable(cnt_enable),
        /* output [06:00] */ .count(count),
        /* output         */ .tc(cnt_tc)
    );

    /* Statusi Register instantiation */
    cy_psoc3_statusi #(.cy_force_order(1), .cy_md_select(7'h09), .cy_int_mask(7'h00))
    TxStsReg(
        /* input            */ .clock(clk_fin),
        /* input    [06:00] */ .status(tx_status),
        /* output           */ .interrupt(tx_interpt)
    );

    /* Statusi Register instantiation */
    cy_psoc3_statusi #(.cy_force_order(1), .cy_md_select(7'h40), .cy_int_mask(7'h00))
    RxStsReg(
        /* input            */ .clock(clk_fin),
        /* input    [06:00] */ .status(rx_status),
        /* output           */ .interrupt(rx_interpt)
    );

    generate
    if(NumberOfDataBits <= SR8) begin: sR8

    cy_psoc3_dp8 #(.cy_dpconfig_a(dp8_cfg))
    Dp(
        /*  input           */ .clk(clk_fin),
        /*  input           */ .reset(reset),
        /*  input   [02:00] */ .cs_addr(state),
        /*  input           */ .route_si(miso_to_dp),
        /*  input           */ .route_ci(1'b0),
        /*  input           */ .f0_load(1'b0),
        /*  input           */ .f1_load(load_rx_data),
        /*  input           */ .d0_load(1'b0),
        /*  input           */ .d1_load(1'b0),
        /*  output          */ .ce0(),
        /*  output          */ .cl0(),
        /*  output          */ .z0(),
        /*  output          */ .ff0(),
        /*  output          */ .ce1(),
        /*  output          */ .cl1(),
        /*  output          */ .z1(),
        /*  output          */ .ff1(),
        /*  output          */ .ov_msb(),
        /*  output          */ .co_msb(),
        /*  output          */ .cmsb(),
        /*  output          */ .so(mosi_from_dp),
        /*  output          */ .f0_bus_stat(dpMOSI_fifo_not_full),
        /*  output          */ .f0_blk_stat(dpMOSI_fifo_empty),
        /*  output          */ .f1_bus_stat(dpMISO_fifo_not_empty),
        /*  output          */ .f1_blk_stat(dpMISO_fifo_full)
        );

    end /* NumberOfDataBits <= SR8 */
    else begin : sR16    /* NumberOfDataBits > 8 */
    cy_psoc3_dp16 #(.cy_dpconfig_a(dp16_lsb_cfg), .cy_dpconfig_b(dp16_msb_cfg))
    Dp(
            /*  input           */ .clk(clk_fin),
            /*  input           */ .reset(reset),
            /*  input   [02:00] */ .cs_addr(state),
            /*  input           */ .route_si(miso_to_dp),
            /*  input           */ .route_ci(1'b0),
            /*  input           */ .f0_load(1'b0),
            /*  input           */ .f1_load(load_rx_data),
            /*  input           */ .d0_load(1'b0),
            /*  input           */ .d1_load(1'b0),
            /*  output          */ .ce0(),
            /*  output          */ .cl0(),
            /*  output          */ .z0(),
            /*  output          */ .ff0(),
            /*  output          */ .ce1(),
            /*  output          */ .cl1(),
            /*  output          */ .z1(),
            /*  output          */ .ff1(),
            /*  output          */ .ov_msb(),
            /*  output          */ .co_msb(),
            /*  output          */ .cmsb(),
            /*  output          */ .so({mosi_from_dpL,mosi_from_dpR}),
            /*  output          */ .f0_bus_stat({dpMOSI_fifo_not_full,  nc1}),
            /*  output          */ .f0_blk_stat({dpMOSI_fifo_empty,     nc2}),
            /*  output          */ .f1_bus_stat({dpMISO_fifo_not_empty, nc3}),
            /*  output          */ .f1_blk_stat({dpMISO_fifo_full,      nc4})
    );
        assign mosi_from_dp = (ShiftDir == SPIM_MSB_FIRST) ? mosi_from_dpL : mosi_from_dpR;

    end /* NumberOfDataBits <= sR16 */
    endgenerate

endmodule
`endif /* B_SPIM_v2_50__V_ALREADY_INCLUDED */
