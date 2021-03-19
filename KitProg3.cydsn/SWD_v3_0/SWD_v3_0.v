/*******************************************************************************
* File Name:  SWD_v3_0.v
*
* Description:
*  Verilog file for the Custom SWD Component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2020, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

`include "cypress.v"

module SWD_v3_0 (
    output  wire out_en,     /* digital output enable for SWDO pin */
    output  reg  parity,     /* calculated parity of data portion for write transaction */
    output  wire parity_en,  /* signal to switch between swdo/parity: 0-swdo, 1-parity */
    output  reg  swdck,      /* SWD clock output */
    output  reg  swdo,       /* SWD data output */
    input   wire clock,      /* component clock input = 2x bitrate of SWD clock output */
    input   wire data_ready, /* data ready pulse input */
    input   wire RnW,        /* read or write SWD transaction. 0-write, 1-read */
    input   wire swdi,       /* SWD data input */
    input   wire skip_phase  /* skip data phase when SWD preamble is not OK acked */
);

/* State machine states */
localparam STATE_IDLE     = 2'b00;
localparam STATE_PREAMBLE = 2'b01;
localparam STATE_DATA     = 2'b10;
localparam STATE_ACK      = 2'b11;

/* Counter periods */
localparam [6:0] PREAMBLE_COUNTER_PERIOD = 15;
localparam [6:0] DATA_COUNTER_PERIOD     = 62;
localparam [6:0] ACK_COUNTER_PERIOD      = 8;
localparam [6:0] PARITY_COUNTER_PERIOD   = 61;
localparam [6:0] IDLE_COUNTER_PERIOD     = 8;

/* Registers */
reg [1:0] state;             /* Datapath state machine */
reg [2:0] cs_addr_preamble;  /* Preamble datapath address */
reg [2:0] cs_addr_data;      /* Data segment datapath address */
reg  parity_state;           /* Parity of data phase state */
reg  data_ready_sync;        /* Data ready register, triggers start SWD transaction */
reg  data_counter_en;        /* Start clock count for data phase */
reg  idle_reset;             /* Reset counter of SWD dummy cycles */
reg  present_data_phase;     /* Dataphase enabler */

/* Wires */
wire [6:0] status;           /* Component statusi register source */
wire preamble_counter_en;    /* Start clock count for preamble phase */
wire preamble_counter_tc;    /* Preamble phase is over */
wire preamble_so;            /* Preamble shift output */
wire data_counter_tc;        /* Data phase is over */
wire [3:0] data_dp_so;       /* Data datapath shift output */
wire data_so;                /* LSB of data_dp_so */
wire ack_counter_en;         /* Start clock count for SWD ACK */
wire ack_counter_tc;         /* SWD ACK is over */
wire parity_counter_tc;      /* Data parity is over */
wire idle_counter_en;        /* Start clock count for SWD dummy cycles */
wire idle_counter_tc;        /* SWD dummy cycles is over */
wire load_data;              /* SWD data is loaded */
wire clk_fin;                /* swdck transformed to clock source */
wire parity_error;           /* Data parity error */
wire check_ack;              /* SWD ACK is OK */

wire idle_state;             /* Current state is Idle */
wire preamble_state;         /* Current state is Preamble */
wire ack_state;              /* Current state is Ack */
wire data_state;             /* Current state is Data */

/* Wire states assignment */
assign idle_state = (state == STATE_IDLE);
assign preamble_state = (state == STATE_PREAMBLE);
assign ack_state = (state == STATE_ACK);
assign data_state = (state == STATE_DATA);

assign parity_error = RnW & data_counter_tc & (parity ^ swdi);
assign preamble_counter_en = ~preamble_state;
assign ack_counter_en = ~ack_state;
assign idle_counter_en = ~idle_reset;
assign data_so = data_dp_so[0];
assign load_data = parity_counter_tc | data_counter_tc;
assign parity_en = parity_state | data_counter_tc;
assign out_en = (idle_state | preamble_state | (data_state | parity_state) & ~RnW) & present_data_phase | idle_reset;

/* Statusi register bit assignment */
assign status[0] = parity_state;
assign status[1] = parity_error;
assign status[6:2] = 4'b0;

/* Clock Enable primitive instantiation to create synchronized clock source from swdck for parity calculation purposes */
cy_psoc3_udb_clock_enable_v1_0 #(.sync_mode(`TRUE))
ClkEn (
    .clock_in(swdck),
    .enable(1'b1),
    .clock_out(clk_fin)
);

/* Registered states assignment */
always @(posedge clock)
begin
    data_ready_sync <= data_ready;
    parity_state <= data_counter_tc;
    data_counter_en <= (~data_state) & (~ack_counter_tc | ~RnW);
    cs_addr_preamble <= {data_ready, swdck, state[0]};
    cs_addr_data <= {data_ready, swdck, ack_counter_tc | data_state | RnW & ack_state};
end

/* State machine logic */
always @(posedge clock)
begin
    case (state)
        STATE_IDLE:
            begin
                state <= (data_ready_sync) ? STATE_PREAMBLE : STATE_IDLE;
            end
        STATE_PREAMBLE:
            begin
                state <= (preamble_counter_tc) ? STATE_ACK : STATE_PREAMBLE;
            end
        STATE_ACK:
            begin
                state <= (ack_counter_tc) ? STATE_DATA : STATE_ACK;
            end
        STATE_DATA:
            begin
                state <= (data_counter_tc) ? STATE_IDLE : STATE_DATA;
            end
        default:
            begin
                state <= STATE_IDLE;
            end
    endcase
end

/* Idle Reset logic */
always @(posedge clock)
begin
    if (parity_state)
    begin
        idle_reset <= 1'b1;
    end
    else
    begin
        idle_reset <= (idle_counter_tc) ? 1'b0 : idle_reset;
    end
end

/* Ignore data phase logic */
always @(posedge clock)
begin
    if (skip_phase & (~preamble_state))
    begin
        present_data_phase <= (ack_counter_tc) ? check_ack : present_data_phase;
    end
    else
    begin
        present_data_phase <= 1'b1;
    end
end

/* Parity calculation */
always @(posedge clk_fin)
begin
    if (ack_state & (~RnW | ~ack_counter_tc))
    begin
        parity <= 1'b0;
    end
    else
    begin
        if (data_state | (RnW & ack_counter_tc))
        begin
            parity <= parity ^ (~RnW & swdo | RnW & swdi);
        end
        else
        begin
            parity <= parity;
        end
    end
end

/* SWD data output logic */
always @(posedge clock)
begin
    if (swdck == 1'b1)
    begin
        swdo <= (preamble_state & preamble_so |
             (data_state | ack_counter_tc) & data_so |
             data_counter_tc & parity) & ~idle_reset & present_data_phase;
    end
    else
    begin
        swdo <= swdo;
    end
end

/* SWD Clock logic */
always @(posedge clock)
begin
    if (idle_state & ~parity_state & ~idle_reset | ack_state & ack_counter_tc & ~check_ack | ~present_data_phase)
    begin
        swdck <= 1'b1;
    end
    else
    begin
        swdck <= !swdck;
    end
end

/* Preamble counter using Count7 cell */
cy_psoc3_count7 #(.cy_period(PREAMBLE_COUNTER_PERIOD), .cy_route_ld(`TRUE), .cy_route_en(`TRUE))
PreambleCounter (
/* input */ .clock (clock), // Clock
/* input */ .reset(1'b0), // Reset
/* input */ .load(preamble_counter_en), // Load signal used if cy_route_ld = TRUE
/* input */ .enable(1'b1), // Enable signal used if cy_route_en = TRUE
/* output [6:0] */ .count(), // Counter value output
/* output */ .tc(preamble_counter_tc) // Terminal Count output
);

/* Data phase counter using Count7 cell */
cy_psoc3_count7 #(.cy_period(DATA_COUNTER_PERIOD), .cy_route_ld(`TRUE), .cy_route_en(`TRUE))
DataCounter (
/* input */ .clock (clock), // Clock
/* input */ .reset(1'b0), // Reset
/* input */ .load(data_counter_en), // Load signal used if cy_route_ld = TRUE
/* input */ .enable(1'b1), // Enable signal used if cy_route_en = TRUE
/* output [6:0] */ .count(), // Counter value output
/* output */ .tc(data_counter_tc) // Terminal Count output
);

/* Ack counter using Count7 cell */
cy_psoc3_count7 #(.cy_period(ACK_COUNTER_PERIOD), .cy_route_ld(`TRUE), .cy_route_en(`TRUE))
AckCounter (
/* input */ .clock (clock), // Clock
/* input */ .reset(1'b0), // Reset
/* input */ .load(ack_counter_en), // Load signal used if cy_route_ld = TRUE
/* input */ .enable(1'b1), // Enable signal used if cy_route_en = TRUE
/* output [6:0] */ .count(), // Counter value output
/* output */ .tc(ack_counter_tc) // Terminal Count output
);

/* Parity counter using Count7 cell */
cy_psoc3_count7 #(.cy_period(PARITY_COUNTER_PERIOD), .cy_route_ld(`TRUE), .cy_route_en(`TRUE))
ParityCounter (
/* input */ .clock (clock), // Clock
/* input */ .reset(1'b0), // Reset
/* input */ .load(data_counter_en), // Load signal used if cy_route_ld = TRUE
/* input */ .enable(1'b1), // Enable signal used if cy_route_en = TRUE
/* output [6:0] */ .count(), // Counter value output
/* output */ .tc(parity_counter_tc) // Terminal Count output
);

/* Idle counter using Count7 cell */
cy_psoc3_count7 #(.cy_period(IDLE_COUNTER_PERIOD), .cy_route_ld(`TRUE), .cy_route_en(`TRUE))
IdleCounter (
/* input */ .clock (clock), // Clock
/* input */ .reset(1'b0), // Reset
/* input */ .load(idle_counter_en), // Load signal used if cy_route_ld = TRUE
/* input */ .enable(1'b1), // Enable signal used if cy_route_en = TRUE
/* output [6:0] */ .count(), // Counter value output
/* output */ .tc(idle_counter_tc) // Terminal Count output
);

/* Statusi Register instantiation */
cy_psoc3_statusi #(.cy_force_order(1), .cy_md_select(7'h03), .cy_int_mask(7'h00))
StatusReg(
        /* input            */ .clock(clock),
        /* input    [06:00] */ .status(status),
        /* output           */ .interrupt()
);

cy_psoc3_dp32 #(.cy_dpconfig_a(
{
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM0:    Idle*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM1:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM2:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP___SR, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM3:     Shift A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM4:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM5:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM6:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM7:            */
    8'hFF, 8'h00,  /*CFG9:            */
    8'hFF, 8'hFF,  /*CFG11-10:            */
    `SC_CMPB_A1_D1, `SC_CMPA_A1_D1, `SC_CI_B_ARITH,
    `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
    `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
    `SC_SI_A_CHAIN, /*CFG13-12:            */
    `SC_A0_SRC_ACC, `SC_SHIFT_SR, 1'h0,
    1'h0, `SC_FIFO1__A0, `SC_FIFO0_BUS,
    `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
    `SC_FB_NOCHN, `SC_CMP1_NOCHN,
    `SC_CMP0_NOCHN, /*CFG15-14:            */
    10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_AX,
    `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_DSBL,
    `SC_WRK16CAT_DSBL /*CFG17-16:            */
}
), .cy_dpconfig_b(
{
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM0:    Idle*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM1:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM2:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP___SR, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM3:     Shift A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM4:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM5:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM6:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM7:            */
    8'hFF, 8'h00,  /*CFG9:            */
    8'hFF, 8'hFF,  /*CFG11-10:            */
    `SC_CMPB_A1_D1, `SC_CMPA_A1_D1, `SC_CI_B_ARITH,
    `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
    `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
    `SC_SI_A_CHAIN, /*CFG13-12:            */
    `SC_A0_SRC_ACC, `SC_SHIFT_SR, 1'h0,
    1'h0, `SC_FIFO1__A0, `SC_FIFO0_BUS,
    `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
    `SC_FB_NOCHN, `SC_CMP1_NOCHN,
    `SC_CMP0_NOCHN, /*CFG15-14:            */
    10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_AX,
    `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_DSBL,
    `SC_WRK16CAT_DSBL /*CFG17-16:            */
}
), .cy_dpconfig_c(
{
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM0:    Idle*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM1:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM2:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP___SR, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM3:     Shift A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM4:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM5:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM6:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM7:            */
    8'hFF, 8'h00,  /*CFG9:            */
    8'hFF, 8'hFF,  /*CFG11-10:            */
    `SC_CMPB_A1_D1, `SC_CMPA_A1_D1, `SC_CI_B_ARITH,
    `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
    `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
    `SC_SI_A_CHAIN, /*CFG13-12:            */
    `SC_A0_SRC_ACC, `SC_SHIFT_SR, 1'h0,
    1'h0, `SC_FIFO1__A0, `SC_FIFO0_BUS,
    `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
    `SC_FB_NOCHN, `SC_CMP1_NOCHN,
    `SC_CMP0_NOCHN, /*CFG15-14:            */
    10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_AX,
    `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_DSBL,
    `SC_WRK16CAT_DSBL /*CFG17-16:            */
}
), .cy_dpconfig_d(
{
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM0:    Idle*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM1:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM2:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP___SR, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM3:     Shift A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM4:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM5:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM6:    Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM7:            */
    8'hFF, 8'h00,  /*CFG9:            */
    8'hFF, 8'hFF,  /*CFG11-10:            */
    `SC_CMPB_A1_D1, `SC_CMPA_A1_D1, `SC_CI_B_ARITH,
    `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
    `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
    `SC_SI_A_ROUTE, /*CFG13-12:            */
    `SC_A0_SRC_ACC, `SC_SHIFT_SR, 1'h0,
    1'h0, `SC_FIFO1__A0, `SC_FIFO0_BUS,
    `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
    `SC_FB_NOCHN, `SC_CMP1_NOCHN,
    `SC_CMP0_NOCHN, /*CFG15-14:            */
    10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_AX,
    `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_DSBL,
    `SC_WRK16CAT_DSBL /*CFG17-16:            */
}
)) data_dp(
        /*  input                   */  .reset(1'b0),
        /*  input                   */  .clk(clock),
        /*  input   [02:00]         */  .cs_addr(cs_addr_data),
        /*  input                   */  .route_si(swdi),
        /*  input                   */  .route_ci(1'b0),
        /*  input                   */  .f0_load(1'b0),
        /*  input                   */  .f1_load(load_data),
        /*  input                   */  .d0_load(1'b0),
        /*  input                   */  .d1_load(1'b0),
        /*  output  [03:00]                  */  .ce0(),
        /*  output  [03:00]                  */  .cl0(),
        /*  output  [03:00]                  */  .z0(),
        /*  output  [03:00]                  */  .ff0(),
        /*  output  [03:00]                  */  .ce1(),
        /*  output  [03:00]                  */  .cl1(),
        /*  output  [03:00]                  */  .z1(),
        /*  output  [03:00]                  */  .ff1(),
        /*  output  [03:00]                  */  .ov_msb(),
        /*  output  [03:00]                  */  .co_msb(),
        /*  output  [03:00]                  */  .cmsb(),
        /*  output  [03:00]                  */  .so(data_dp_so),
        /*  output  [03:00]                  */  .f0_bus_stat(),
        /*  output  [03:00]                  */  .f0_blk_stat(),
        /*  output  [03:00]                  */  .f1_bus_stat(),
        /*  output  [03:00]                  */  .f1_blk_stat()
);
cy_psoc3_dp8 #(.cy_dpconfig_a(
{
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM0:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM1:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM2:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP___SR, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM3:     Shift A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP___SR, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM4:     Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM5:     */
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC___F0, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM6:     Load F0 > A0*/
    `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
    `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
    `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
    `CS_CMP_SEL_CFGA, /*CFGRAM7:     */
    8'hFF, 8'hFF,  /*CFG9:  */
    8'hFF, 8'h70,  /*CFG11-10:     */
    `SC_CMPB_A1_D1, `SC_CMPA_A0_D1, `SC_CI_B_ARITH,
    `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_ENBL,
    `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
    `SC_SI_A_ROUTE, /*CFG13-12:     */
    `SC_A0_SRC_ACC, `SC_SHIFT_SR, 1'h0,
    1'h0, `SC_FIFO1__A0, `SC_FIFO0_BUS,
    `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
    `SC_FB_NOCHN, `SC_CMP1_NOCHN,
    `SC_CMP0_NOCHN, /*CFG15-14:     */
    10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_AX,
    `SC_FIFO__EDGE,`SC_FIFO_ASYNC,`SC_EXTCRC_DSBL,
    `SC_WRK16CAT_DSBL /*CFG17-16:     */
}
)) preamble_dp(
        /*  input                   */  .reset(1'b0),
        /*  input                   */  .clk(clock),
        /*  input   [02:00]         */  .cs_addr(cs_addr_preamble),
        /*  input                   */  .route_si(swdi),
        /*  input                   */  .route_ci(1'b0),
        /*  input                   */  .f0_load(1'b0),
        /*  input                   */  .f1_load(data_state),
        /*  input                   */  .d0_load(1'b0),
        /*  input                   */  .d1_load(1'b0),
        /*  output                  */  .ce0(check_ack),
        /*  output                  */  .cl0(),
        /*  output                  */  .z0(),
        /*  output                  */  .ff0(),
        /*  output                  */  .ce1(),
        /*  output                  */  .cl1(),
        /*  output                  */  .z1(),
        /*  output                  */  .ff1(),
        /*  output                  */  .ov_msb(),
        /*  output                  */  .co_msb(),
        /*  output                  */  .cmsb(),
        /*  output                  */  .so(preamble_so),
        /*  output                  */  .f0_bus_stat(),
        /*  output                  */  .f0_blk_stat(),
        /*  output                  */  .f1_bus_stat(),
        /*  output                  */  .f1_blk_stat()
);

endmodule

