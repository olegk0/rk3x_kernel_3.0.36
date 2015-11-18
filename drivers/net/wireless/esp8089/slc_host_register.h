//Generated at 2012-10-23 20:11:08
/*
 *  Copyright (c) 2011 Espressif System
 *
 */

#ifndef SLC_HOST_REGISTER_H_INCLUDED
#define SLC_HOST_REGISTER_H_INCLUDED

/* #define REG_SLC_HOST_BASE  0x00000000 */
/* skip the token1, since reading it will clean the credit */
#define REG_SLC_HOST_BASE  0x00000000


#define SLC_HOST_PF                          (REG_SLC_HOST_BASE + 0x0)
#define SLC_HOST_TOKEN_RDATA                 (REG_SLC_HOST_BASE + 0x4)
#define SLC_HOST_RX_PF_EOF 0x0000000F
#define SLC_HOST_RX_PF_EOF_S                 28
#define SLC_HOST_TOKEN1 0x00000FFF
#define SLC_HOST_TOKEN1_S 16
#define SLC_HOST_RX_PF_VALID (BIT(15))
#define SLC_HOST_TOKEN0               0x00000FFF
#define SLC_HOST_TOKEN0_S 0

#define SLC_HOST_TOKEN0_MASK SLC_HOST_TOKEN0

#define SLC_HOST_INT_RAW                     (REG_SLC_HOST_BASE + 0x8)
#define SLC_HOST_EXT_BIT3_INT_RAW (BIT(22))
#define SLC_HOST_EXT_BIT2_INT_RAW (BIT(21))
#define SLC_HOST_EXT_BIT1_INT_RAW (BIT(20))
#define SLC_HOST_RXFIFO_NOT_EMPTY_INT_RAW (BIT(19))
#define SLC_HOST_RX_PF_VALID_INT_RAW (BIT(18))
#define SLC_HOST_TX_OVF_INT_RAW (BIT(17))
#define SLC_HOST_RX_UDF_INT_RAW (BIT(16))
#define SLC_HOST_TX_START_INT_RAW (BIT(15))
#define SLC_HOST_RX_START_INT_RAW (BIT(14))
#define SLC_HOST_RX_EOF_INT_RAW (BIT(13))
#define SLC_HOST_RX_SOF_INT_RAW (BIT(12))
#define SLC_HOST_TOKEN1_0TO1_INT_RAW (BIT(11))
#define SLC_HOST_TOKEN0_0TO1_INT_RAW (BIT(10))
#define SLC_HOST_TOKEN1_1TO0_INT_RAW (BIT(9))
#define SLC_HOST_TOKEN0_1TO0_INT_RAW (BIT(8))
#define SLC_HOST_TOHOST_BIT7_INT_RAW (BIT(7))
#define SLC_HOST_TOHOST_BIT6_INT_RAW (BIT(6))
#define SLC_HOST_TOHOST_BIT5_INT_RAW (BIT(5))
#define SLC_HOST_TOHOST_BIT4_INT_RAW (BIT(4))
#define SLC_HOST_TOHOST_BIT3_INT_RAW (BIT(3))
#define SLC_HOST_TOHOST_BIT2_INT_RAW (BIT(2))
#define SLC_HOST_TOHOST_BIT1_INT_RAW (BIT(1))
#define SLC_HOST_TOHOST_BIT0_INT_RAW (BIT(0))

#define SLC_HOST_STATE_W0                    (REG_SLC_HOST_BASE + 0xC)
#define SLC_HOST_STATE3 0x000000FF
#define SLC_HOST_STATE3_S 24
#define SLC_HOST_STATE2 0x000000FF
#define SLC_HOST_STATE2_S 16
#define SLC_HOST_STATE1 0x000000FF
#define SLC_HOST_STATE1_S 8
#define SLC_HOST_STATE0 0x000000FF
#define SLC_HOST_STATE0_S 0

#define SLC_HOST_STATE_W1                    (REG_SLC_HOST_BASE + 0x10)
#define SLC_HOST_STATE7 0x000000FF
#define SLC_HOST_STATE7_S 24
#define SLC_HOST_STATE6 0x000000FF
#define SLC_HOST_STATE6_S 16
#define SLC_HOST_STATE5 0x000000FF
#define SLC_HOST_STATE5_S 8
#define SLC_HOST_STATE4 0x000000FF
#define SLC_HOST_STATE4_S 0

#define SLC_HOST_CONF_W0                     (REG_SLC_HOST_BASE + 0x14)
#define SLC_HOST_CONF3 0x000000FF
#define SLC_HOST_CONF3_S 24
#define SLC_HOST_CONF2 0x000000FF
#define SLC_HOST_CONF2_S 16
#define SLC_HOST_CONF1 0x000000FF
#define SLC_HOST_CONF1_S 8
#define SLC_HOST_CONF0 0x000000FF
#define SLC_HOST_CONF0_S 0

#define SLC_HOST_CONF_W1                     (REG_SLC_HOST_BASE + 0x18)
#define SLC_HOST_CONF7 0x000000FF
#define SLC_HOST_CONF7_S 24
#define SLC_HOST_CONF6 0x000000FF
#define SLC_HOST_CONF6_S 16
#define SLC_HOST_CONF5 0x000000FF
#define SLC_HOST_CONF5_S 8
#define SLC_HOST_CONF4 0x000000FF
#define SLC_HOST_CONF4_S 0

#define SLC_HOST_INT_ST                      (REG_SLC_HOST_BASE + 0x1C)
#define SLC_HOST_RX_ST (BIT(23))
#define SLC_HOST_EXT_BIT3_INT_ST (BIT(22))
#define SLC_HOST_EXT_BIT2_INT_ST (BIT(21))
#define SLC_HOST_EXT_BIT1_INT_ST (BIT(20))
#define SLC_HOST_RXFIFO_NOT_EMPTY_INT_ST (BIT(19))
#define SLC_HOST_RX_PF_VALID_INT_ST (BIT(18))
#define SLC_HOST_TX_OVF_INT_ST (BIT(17))
#define SLC_HOST_RX_UDF_INT_ST (BIT(16))
#define SLC_HOST_TX_START_INT_ST (BIT(15))
#define SLC_HOST_RX_START_INT_ST (BIT(14))
#define SLC_HOST_RX_EOF_INT_ST (BIT(13))
#define SLC_HOST_RX_SOF_INT_ST (BIT(12))
#define SLC_HOST_TOKEN1_0TO1_INT_ST (BIT(11))
#define SLC_HOST_TOKEN0_0TO1_INT_ST (BIT(10))
#define SLC_HOST_TOKEN1_1TO0_INT_ST (BIT(9))
#define SLC_HOST_TOKEN0_1TO0_INT_ST (BIT(8))
#define SLC_HOST_TOHOST_BIT7_INT_ST (BIT(7))
#define SLC_HOST_TOHOST_BIT6_INT_ST (BIT(6))
#define SLC_HOST_TOHOST_BIT5_INT_ST (BIT(5))
#define SLC_HOST_TOHOST_BIT4_INT_ST (BIT(4))
#define SLC_HOST_TOHOST_BIT3_INT_ST (BIT(3))
#define SLC_HOST_TOHOST_BIT2_INT_ST (BIT(2))
#define SLC_HOST_TOHOST_BIT1_INT_ST (BIT(1))
#define SLC_HOST_TOHOST_BIT0_INT_ST (BIT(0))

#define SLC_HOST_CONF_W2                     (REG_SLC_HOST_BASE + 0x20)
#define SLC_HOST_CONF11 0x000000FF
#define SLC_HOST_CONF11_S 24
#define SLC_HOST_CONF10 0x000000FF
#define SLC_HOST_CONF10_S 16
#define SLC_HOST_CONF9 0x000000FF
#define SLC_HOST_CONF9_S 8
#define SLC_HOST_CONF8 0x000000FF
#define SLC_HOST_CONF8_S 0

#define SLC_HOST_CONF_W3                     (REG_SLC_HOST_BASE + 0x24)
#define SLC_HOST_CONF15 0x000000FF
#define SLC_HOST_CONF15_S 24
#define SLC_HOST_CONF14 0x000000FF
#define SLC_HOST_CONF14_S 16
#define SLC_HOST_CONF13 0x000000FF
#define SLC_HOST_CONF13_S 8
#define SLC_HOST_CONF12 0x000000FF
#define SLC_HOST_CONF12_S 0

#define SLC_HOST_GEN_TXDONE_INT  BIT(16)
#define SLC_HOST_GEN_RXDONE_INT  BIT(17)

#define SLC_HOST_CONF_W4                     (REG_SLC_HOST_BASE + 0x28)
#define SLC_HOST_CONF19 0x000000FF
#define SLC_HOST_CONF19_S 24
#define SLC_HOST_CONF18 0x000000FF
#define SLC_HOST_CONF18_S 16
#define SLC_HOST_CONF17 0x000000FF
#define SLC_HOST_CONF17_S 8
#define SLC_HOST_CONF16 0x000000FF
#define SLC_HOST_CONF16_S 0

#define SLC_HOST_TOKEN_WDATA                 (REG_SLC_HOST_BASE + 0x2C)
#define SLC_HOST_TOKEN1_WD 0x00000FFF
#define SLC_HOST_TOKEN1_WD_S 16
#define SLC_HOST_TOKEN0_WD 0x00000FFF
#define SLC_HOST_TOKEN0_WD_S 0

#define SLC_HOST_INT_CLR                     (REG_SLC_HOST_BASE + 0x30)
#define SLC_HOST_TOKEN1_WR (BIT(31))
#define SLC_HOST_TOKEN0_WR (BIT(30))
#define SLC_HOST_TOKEN1_DEC (BIT(29))
#define SLC_HOST_TOKEN0_DEC (BIT(28))
#define SLC_HOST_EXT_BIT3_INT_CLR (BIT(22))
#define SLC_HOST_EXT_BIT2_INT_CLR (BIT(21))
#define SLC_HOST_EXT_BIT1_INT_CLR (BIT(20))
#define SLC_HOST_EXT_BIT0_INT_CLR (BIT(19))
#define SLC_HOST_RX_PF_VALID_INT_CLR (BIT(18))
#define SLC_HOST_TX_OVF_INT_CLR (BIT(17))
#define SLC_HOST_RX_UDF_INT_CLR (BIT(16))
#define SLC_HOST_TX_START_INT_CLR (BIT(15))
#define SLC_HOST_RX_START_INT_CLR (BIT(14))
#define SLC_HOST_RX_EOF_INT_CLR (BIT(13))
#define SLC_HOST_RX_SOF_INT_CLR (BIT(12))
#define SLC_HOST_TOKEN1_0TO1_INT_CLR (BIT(11))
#define SLC_HOST_TOKEN0_0TO1_INT_CLR (BIT(10))
#define SLC_HOST_TOKEN1_1TO0_INT_CLR (BIT(9))
#define SLC_HOST_TOKEN0_1TO0_INT_CLR (BIT(8))
#define SLC_HOST_TOHOST_BIT7_INT_CLR (BIT(7))
#define SLC_HOST_TOHOST_BIT6_INT_CLR (BIT(6))
#define SLC_HOST_TOHOST_BIT5_INT_CLR (BIT(5))
#define SLC_HOST_TOHOST_BIT4_INT_CLR (BIT(4))
#define SLC_HOST_TOHOST_BIT3_INT_CLR (BIT(3))
#define SLC_HOST_TOHOST_BIT2_INT_CLR (BIT(2))
#define SLC_HOST_TOHOST_BIT1_INT_CLR (BIT(1))
#define SLC_HOST_TOHOST_BIT0_INT_CLR (BIT(0))

#define SLC_HOST_INT_ENA                     (REG_SLC_HOST_BASE + 0x34)
#define SLC_HOST_EXT_BIT3_INT_ENA (BIT(22))
#define SLC_HOST_EXT_BIT2_INT_ENA (BIT(21))
#define SLC_HOST_EXT_BIT1_INT_ENA (BIT(20))
#define SLC_HOST_EXT_BIT0_INT_ENA (BIT(19))
#define SLC_HOST_RX_PF_VALID_INT_ENA (BIT(18))
#define SLC_HOST_TX_OVF_INT_ENA (BIT(17))
#define SLC_HOST_RX_UDF_INT_ENA (BIT(16))
#define SLC_HOST_TX_START_INT_ENA (BIT(15))
#define SLC_HOST_RX_START_INT_ENA (BIT(14))
#define SLC_HOST_RX_EOF_INT_ENA (BIT(13))
#define SLC_HOST_RX_SOF_INT_ENA (BIT(12))
#define SLC_HOST_TOKEN1_0TO1_INT_ENA (BIT(11))
#define SLC_HOST_TOKEN0_0TO1_INT_ENA (BIT(10))
#define SLC_HOST_TOKEN1_1TO0_INT_ENA (BIT(9))
#define SLC_HOST_TOKEN0_1TO0_INT_ENA (BIT(8))
#define SLC_HOST_TOHOST_BIT7_INT_ENA (BIT(7))
#define SLC_HOST_TOHOST_BIT6_INT_ENA (BIT(6))
#define SLC_HOST_TOHOST_BIT5_INT_ENA (BIT(5))
#define SLC_HOST_TOHOST_BIT4_INT_ENA (BIT(4))
#define SLC_HOST_TOHOST_BIT3_INT_ENA (BIT(3))
#define SLC_HOST_TOHOST_BIT2_INT_ENA (BIT(2))
#define SLC_HOST_TOHOST_BIT1_INT_ENA (BIT(1))
#define SLC_HOST_TOHOST_BIT0_INT_ENA (BIT(0))

#define SLC_HOST_CONF_W5                     (REG_SLC_HOST_BASE + 0x3C)
#define SLC_HOST_CONF23 0x000000FF
#define SLC_HOST_CONF23_S 24
#define SLC_HOST_CONF22 0x000000FF
#define SLC_HOST_CONF22_S 16
#define SLC_HOST_CONF21 0x000000FF
#define SLC_HOST_CONF21_S 8
#define SLC_HOST_CONF20 0x000000FF
#define SLC_HOST_CONF20_S 0


#define SLC_HOST_DATE                         (REG_SLC_HOST_BASE + 0x78)
#define SLC_HOST_ID                           (REG_SLC_HOST_BASE + 0x7C)

#define SLC_ADDR_WINDOW_CLEAR_MASK   (~(0xf<<12))
#define SLC_FROM_HOST_ADDR_WINDOW  (0x1<<12)
#define SLC_TO_HOST_ADDR_WINDOW    (0x3<<12)

#define SLC_SET_FROM_HOST_ADDR_WINDOW(v)   do { \
        (v) &= 0xffff;    \
	(v) &= SLC_ADDR_WINDOW_CLEAR_MASK; \
	(v) |= SLC_FROM_HOST_ADDR_WINDOW; \
} while (0);

#define SLC_SET_TO_HOST_ADDR_WINDOW(v)   do { \
        (v) &= 0xffff;    \
	(v) &= SLC_ADDR_WINDOW_CLEAR_MASK; \
	(v) |= SLC_TO_HOST_ADDR_WINDOW; \
} while (0);


#endif // SLC_HOST_REGISTER_H_INCLUDED