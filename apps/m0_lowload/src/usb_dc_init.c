//
// This is the bare bones code to power up the USB subsystem so that
// it can be used once the system is booted.
//
// All code was taken from the bl_mcu_sdk, primarily from the file:
//
// drivers/lhal/src/bflb_usb_v2.c
// 

/**
 * @brief
 *
 * Copyright (c) 2021 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */

#include <stdint.h>

#define getreg32(a)               (*(volatile uint32_t *)(uintptr_t)(a))
#define putreg32(v, a)            (*(volatile uint32_t *)(uintptr_t)(a) = (v))


// ONES WE ACTUALLY NEED
#define BLFB_USB_BASE ((uint32_t)0x20072000)
#define BFLB_PDS_BASE ((uint32_t)0x2000e000)

#define USB_OTG_CSR_OFFSET          (0x80)  /* OTG_CSR */
#define USB_GLB_INT_OFFSET          (0xC4)  /* GLB_INT */

#define PDS_USB_CTL_OFFSET      (0x500) /* usb_ctl */
#define PDS_USB_PHY_CTRL_OFFSET (0x504) /* usb_phy_ctrl */

#define PDS_REG_USB_PHY_XTLSEL_SHIFT (2U)
#define PDS_REG_USB_PHY_XTLSEL_MASK  (0x3 << PDS_REG_USB_PHY_XTLSEL_SHIFT)

#define PDS_REG_PU_USB20_PSW         (1 << 6U)

#define PDS_REG_USB_PHY_PONRST       (1 << 0U)

#define PDS_REG_USB_SW_RST_N   (1 << 0U)

#define PDS_REG_USB_EXT_SUSP_N (1 << 1U)

#define PDS_REG_USB_IDDIG      (1 << 5U)

#define USB_A_BUS_REQ_HOV         (1 << 4U)
#define USB_A_BUS_DROP_HOV        (1 << 5U)

#define USB_MDEV_INT (1 << 0U)
#define USB_MOTG_INT (1 << 1U)
#define USB_MHC_INT  (1 << 2U)

// DC VARS!

#define USB_DEV_CTL_OFFSET          (0x100) /* DEV_CTL */
#define USB_PHY_TST_OFFSET          (0x114) /* PHY_TST */
#define USB_DEV_ADR_OFFSET          (0x104) /* DEV_ADR */
#define USB_DEV_SMT_OFFSET          (0x110) /* DEV_SMT */
#define USB_DEV_MIGR_OFFSET         (0x130) /* DEV_MIGR */
#define USB_DEV_MISG0_OFFSET        (0x134) /* DEV_MISG0 */
#define USB_DEV_MISG1_OFFSET        (0x138) /* DEV_MISG1 */
#define USB_DEV_MISG2_OFFSET        (0x13C) /* DEV_MISG2 */
#define USB_DEV_EPMAP0_OFFSET       (0x1A0) /* DEV_EPMAP0 */
#define USB_DEV_EPMAP1_OFFSET       (0x1A4) /* DEV_EPMAP1 */
#define USB_DEV_FMAP_OFFSET         (0x1A8) /* DEV_FMAP */
#define USB_DEV_MISG3_OFFSET        (0x32C) /* DEV_MISG3 */
#define USB_VDMA_CTRL_OFFSET        (0x330) /* VDMA_CTRL */

#define USB_SOF_TIMER_MASK_AFTER_RESET_HS (0x44C)
#define USB_SOF_TIMER_MASK_AFTER_RESET_FS (0x2710)

/* 0x100 : DEV_CTL */
#define USB_CAP_RMWAKUP        (1 << 0U)
#define USB_HALF_SPEED_HOV     (1 << 1U)
#define USB_GLINT_EN_HOV       (1 << 2U)
#define USB_GOSUSP             (1 << 3U)
#define USB_SFRST_HOV          (1 << 4U)
#define USB_CHIP_EN_HOV        (1 << 5U)
#define USB_HS_EN_HOV          (1 << 6U)
#define USB_SYSBUS_WIDTH_HOV   (1 << 7U)
#define USB_FORCE_FS           (1 << 9U)
#define USB_IDLE_DEGLITCH_HOV  (1 << 10U)
#define USB_LPM_BESL_MAX_SHIFT (12U)
#define USB_LPM_BESL_MAX_MASK  (0xf << USB_LPM_BESL_MAX_SHIFT)
#define USB_LPM_BESL_MIN_SHIFT (16U)
#define USB_LPM_BESL_MIN_MASK  (0xf << USB_LPM_BESL_MIN_SHIFT)
#define USB_LPM_BESL_SHIFT     (20U)
#define USB_LPM_BESL_MASK      (0xf << USB_LPM_BESL_SHIFT)
#define USB_LPM_EN             (1 << 25U)
#define USB_LPM_ACCEPT         (1 << 26U)

/* 0x100 : DEV_CTL */
#define USB_CAP_RMWAKUP        (1 << 0U)
#define USB_CHIP_EN_HOV        (1 << 5U)
#define USB_SFRST_HOV          (1 << 4U)

/* 0x104 : DEV_ADR */
#define USB_DEVADR_SHIFT (0U)
#define USB_DEVADR_MASK  (0x7f << USB_DEVADR_SHIFT)
#define USB_AFT_CONF     (1 << 7U)

/* 0x110 : DEV_SMT */
#define USB_SOFMT_SHIFT (0U)
#define USB_SOFMT_MASK  (0xffff << USB_SOFMT_SHIFT)


/* 0x114 : PHY_TST */
#define USB_UNPLUG     (1 << 0U)

/* 0x130 : DEV_MIGR */
#define USB_MINT_G0 (1 << 0U)
#define USB_MINT_G1 (1 << 1U)
#define USB_MINT_G2 (1 << 2U)
#define USB_MINT_G3 (1 << 3U)
#define USB_MINT_G4 (1 << 4U)

/* 0x134 : DEV_MISG0 */
#define USB_MCX_SETUP_INT    (1 << 0U)
#define USB_MCX_IN_INT       (1 << 1U)
#define USB_MCX_OUT_INT      (1 << 2U)
#define USB_MCX_COMFAIL_INT  (1 << 4U)
#define USB_MCX_COMABORT_INT (1 << 5U)

/* 0x138 : DEV_MISG1 */
#define USB_MF0_OUT_INT (1 << 0U)
#define USB_MF0_SPK_INT (1 << 1U)
#define USB_MF1_OUT_INT (1 << 2U)
#define USB_MF1_SPK_INT (1 << 3U)
#define USB_MF2_OUT_INT (1 << 4U)
#define USB_MF2_SPK_INT (1 << 5U)
#define USB_MF3_OUT_INT (1 << 6U)
#define USB_MF3_SPK_INT (1 << 7U)
#define USB_MF0_IN_INT  (1 << 16U)
#define USB_MF1_IN_INT  (1 << 17U)
#define USB_MF2_IN_INT  (1 << 18U)
#define USB_MF3_IN_INT  (1 << 19U)

/* 0x13C : DEV_MISG2 */
#define USB_MUSBRST_INT        (1 << 0U)
#define USB_MSUSP_INT          (1 << 1U)
#define USB_MRESM_INT          (1 << 2U)
#define USB_MSEQ_ERR_INT       (1 << 3U)
#define USB_MSEQ_ABORT_INT     (1 << 4U)
#define USB_MTX0BYTE_INT       (1 << 5U)
#define USB_MRX0BYTE_INT       (1 << 6U)
#define USB_MDMA_CMPLT_HOV     (1 << 7U)
#define USB_MDMA_ERROR_HOV     (1 << 8U)
#define USB_MDEV_IDLE_HOV      (1 << 9U)
#define USB_MDEV_WAKEUP_BYVBUS (1 << 10U)

/* 0x1A0 : DEV_EPMAP0 */
#define USB_FNO_IEP1_SHIFT (0U)
#define USB_FNO_IEP1_MASK  (0xf << USB_FNO_IEP1_SHIFT)
#define USB_FNO_OEP1_SHIFT (4U)
#define USB_FNO_OEP1_MASK  (0xf << USB_FNO_OEP1_SHIFT)
#define USB_FNO_IEP2_SHIFT (8U)
#define USB_FNO_IEP2_MASK  (0xf << USB_FNO_IEP2_SHIFT)
#define USB_FNO_OEP2_SHIFT (12U)
#define USB_FNO_OEP2_MASK  (0xf << USB_FNO_OEP2_SHIFT)
#define USB_FNO_IEP3_SHIFT (16U)
#define USB_FNO_IEP3_MASK  (0xf << USB_FNO_IEP3_SHIFT)
#define USB_FNO_OEP3_SHIFT (20U)
#define USB_FNO_OEP3_MASK  (0xf << USB_FNO_OEP3_SHIFT)
#define USB_FNO_IEP4_SHIFT (24U)
#define USB_FNO_IEP4_MASK  (0xf << USB_FNO_IEP4_SHIFT)
#define USB_FNO_OEP4_SHIFT (28U)
#define USB_FNO_OEP4_MASK  (0xf << USB_FNO_OEP4_SHIFT)

/* 0x1A4 : DEV_EPMAP1 */

/* 0x1A8 : DEV_FMAP */
#define USB_EPNO_FIFO0_SHIFT (0U)
#define USB_EPNO_FIFO0_MASK  (0xf << USB_EPNO_FIFO0_SHIFT)
#define USB_DIR_FIFO0_SHIFT  (4U)
#define USB_DIR_FIFO0_MASK   (0x3 << USB_DIR_FIFO0_SHIFT)
#define USB_EPNO_FIFO1_SHIFT (8U)
#define USB_EPNO_FIFO1_MASK  (0xf << USB_EPNO_FIFO1_SHIFT)
#define USB_DIR_FIFO1_SHIFT  (12U)
#define USB_DIR_FIFO1_MASK   (0x3 << USB_DIR_FIFO1_SHIFT)
#define USB_EPNO_FIFO2_SHIFT (16U)
#define USB_EPNO_FIFO2_MASK  (0xf << USB_EPNO_FIFO2_SHIFT)
#define USB_DIR_FIFO2_SHIFT  (20U)
#define USB_DIR_FIFO2_MASK   (0x3 << USB_DIR_FIFO2_SHIFT)
#define USB_EPNO_FIFO3_SHIFT (24U)
#define USB_EPNO_FIFO3_MASK  (0xf << USB_EPNO_FIFO3_SHIFT)
#define USB_DIR_FIFO3_SHIFT  (28U)
#define USB_DIR_FIFO3_MASK   (0x3 << USB_DIR_FIFO3_SHIFT)

/* 0x32C : DEV_MISG3 */
#define USB_MVDMA_CMPLT_CXF (1 << 0U)
#define USB_MVDMA_CMPLT_F0  (1 << 1U)
#define USB_MVDMA_CMPLT_F1  (1 << 2U)
#define USB_MVDMA_CMPLT_F2  (1 << 3U)
#define USB_MVDMA_CMPLT_F3  (1 << 4U)
#define USB_MVDMA_ERROR_CXF (1 << 16U)
#define USB_MVDMA_ERROR_F0  (1 << 17U)
#define USB_MVDMA_ERROR_F1  (1 << 18U)
#define USB_MVDMA_ERROR_F2  (1 << 19U)
#define USB_MVDMA_ERROR_F3  (1 << 20U)

/* 0x330 : VDMA_CTRL */
#define USB_VDMA_EN (1 << 0U)


//void USBD_IRQHandler(int irq, void *arg);

//extern void USBH_IRQHandler();

#define USB_DEV_ISG4_OFFSET         (0x338) /* DEV_ISG4 */
#define USB_DEV_ISG0_OFFSET         (0x144) /* DEV_ISG0 */
#define USB_DEV_ISG1_OFFSET         (0x148) /* DEV_ISG1 */
#define USB_DEV_ISG2_OFFSET         (0x14C) /* DEV_ISG2 */
#define USB_DEV_ISG3_OFFSET         (0x328) /* DEV_ISG3 */
#define USB_DEV_ISG4_OFFSET         (0x338) /* DEV_ISG4 */

static void bflb_usb_set_ep_fifomap(uint8_t ep_idx, uint8_t fifo)
{
    uint32_t regval;

    if (ep_idx < 5) {
        regval = getreg32(BLFB_USB_BASE + USB_DEV_EPMAP0_OFFSET);
        regval &= ~(0xff << ((ep_idx - 1) * 8));
        regval |= (fifo << ((ep_idx - 1) * 8));
        regval |= (fifo << ((ep_idx - 1) * 8 + 4));
        putreg32(regval, BLFB_USB_BASE + USB_DEV_EPMAP0_OFFSET);
    } else {
        regval = getreg32(BLFB_USB_BASE + USB_DEV_EPMAP1_OFFSET);
        regval &= ~(0xff << ((ep_idx - 4 - 1) * 8));
        regval |= (fifo << ((ep_idx - 4 - 1) * 8));
        regval |= (fifo << ((ep_idx - 4 - 1) * 8 + 4));
        putreg32(regval, BLFB_USB_BASE + USB_DEV_EPMAP1_OFFSET);
    }
}

static void bflb_usb_set_fifo_epmap(uint8_t fifo, uint8_t ep_idx, uint8_t dir)
{
    uint32_t regval;

    regval = getreg32(BLFB_USB_BASE + USB_DEV_FMAP_OFFSET);
    regval &= ~(0x3f << (fifo * 8));
    regval |= (ep_idx << (fifo * 8));
    regval |= (dir << (fifo * 8 + 4));
    putreg32(regval, BLFB_USB_BASE + USB_DEV_FMAP_OFFSET);
}

static void bflb_usb_source_group_int_clear(uint8_t group, uint32_t int_clear)
{
    switch (group) {
        case 0:
            putreg32(int_clear, BLFB_USB_BASE + USB_DEV_ISG0_OFFSET);
            break;
        case 1:
            putreg32(int_clear, BLFB_USB_BASE + USB_DEV_ISG1_OFFSET);
            break;
        case 2:
            putreg32(int_clear, BLFB_USB_BASE + USB_DEV_ISG2_OFFSET);
            break;
        case 3:
            putreg32(int_clear, BLFB_USB_BASE + USB_DEV_ISG3_OFFSET);
            break;
        case 4:
            putreg32(int_clear, BLFB_USB_BASE + USB_DEV_ISG4_OFFSET);
            break;

        default:
            break;
    }
}

static void bflb_usb_phy_init(void)
{
    uint32_t regval;

    /* USB_PHY_CTRL[3:2] reg_usb_phy_xtlsel=0                             */
    /* 2000e504 = 0x40; #100; USB_PHY_CTRL[6] reg_pu_usb20_psw=1 (VCC33A) */
    /* 2000e504 = 0x41; #500; USB_PHY_CTRL[0] reg_usb_phy_ponrst=1        */
    /* 2000e500 = 0x20; #100; USB_CTL[0] reg_usb_sw_rst_n=0               */
    /* 2000e500 = 0x22; #500; USB_CTL[1] reg_usb_ext_susp_n=1             */
    /* 2000e500 = 0x23; #100; USB_CTL[0] reg_usb_sw_rst_n=1               */
    /* #1.2ms; wait UCLK                                                  */
    /* wait(soc616_b0.usb_uclk);                                          */


    regval = getreg32(BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
    regval &= ~PDS_REG_USB_PHY_XTLSEL_MASK;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
    regval |= PDS_REG_PU_USB20_PSW;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);


    regval = getreg32(BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
    regval |= PDS_REG_USB_PHY_PONRST;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

    /* greater than 5T */
    bflb_mtimer_delay_us(1);

    
    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval &= ~PDS_REG_USB_SW_RST_N;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    /* greater than 5T */
    bflb_mtimer_delay_us(1);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval |= PDS_REG_USB_EXT_SUSP_N;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    /* wait UCLK 1.2ms */
    bflb_mtimer_delay_ms(3);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval |= PDS_REG_USB_SW_RST_N;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    bflb_mtimer_delay_ms(2);
}

int usb_dc_init(void)
{
    uint32_t regval;

    bflb_usb_phy_init();

    //    bflb_irq_attach(37, USBD_IRQHandler, NULL);
    //bflb_irq_enable(37);

    /* disable global irq */
    regval = getreg32(BLFB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval &= ~USB_GLINT_EN_HOV;
    putreg32(regval, BLFB_USB_BASE + USB_DEV_CTL_OFFSET);

    regval = getreg32(BLFB_USB_BASE + USB_PHY_TST_OFFSET);
    regval |= USB_UNPLUG;
    putreg32(regval, BLFB_USB_BASE + USB_PHY_TST_OFFSET);

    regval = getreg32(BLFB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval &= ~USB_CAP_RMWAKUP;
    regval |= USB_CHIP_EN_HOV;
    putreg32(regval, BLFB_USB_BASE + USB_DEV_CTL_OFFSET);

    regval = getreg32(BLFB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval |= USB_SFRST_HOV;
    putreg32(regval, BLFB_USB_BASE + USB_DEV_CTL_OFFSET);

    while (getreg32(BLFB_USB_BASE + USB_DEV_CTL_OFFSET) & USB_SFRST_HOV) {
    }

    regval = getreg32(BLFB_USB_BASE + USB_DEV_ADR_OFFSET);
    regval &= ~USB_AFT_CONF;
    putreg32(regval, BLFB_USB_BASE + USB_DEV_ADR_OFFSET);

    regval = getreg32(BLFB_USB_BASE + USB_DEV_SMT_OFFSET);
    regval &= ~USB_SOFMT_MASK;
#ifdef CONFIG_USB_HS
    regval |= USB_SOF_TIMER_MASK_AFTER_RESET_HS;
#else
    regval |= USB_SOF_TIMER_MASK_AFTER_RESET_FS;
#endif
    putreg32(regval, BLFB_USB_BASE + USB_DEV_SMT_OFFSET);

    /* enable setup irq in source group0 */
    regval = getreg32(BLFB_USB_BASE + USB_DEV_MISG0_OFFSET);
    regval &= ~USB_MCX_SETUP_INT;
    regval |= USB_MCX_IN_INT;
    regval |= (1 << 3);
    regval |= USB_MCX_OUT_INT;
    regval |= USB_MCX_IN_INT;
    regval |= USB_MCX_COMFAIL_INT;
    regval |= USB_MCX_COMABORT_INT;
    putreg32(regval, BLFB_USB_BASE + USB_DEV_MISG0_OFFSET);

    /* disable all fifo irq in source group1 */
    putreg32(0xffffffff, BLFB_USB_BASE + USB_DEV_MISG1_OFFSET);

    /* enable rst/tx0/rx0 irq in source group2 */
    regval = 0xffffffff;
    regval &= ~USB_MUSBRST_INT;
    regval &= ~USB_MSUSP_INT;
    regval &= ~USB_MRESM_INT;
    regval &= ~USB_MTX0BYTE_INT;
    regval &= ~USB_MRX0BYTE_INT;
    putreg32(regval, BLFB_USB_BASE + USB_DEV_MISG2_OFFSET);

    /* enable vdma cmplt and error irq in source group3 */
    regval = 0xffffffff;
    regval &= ~(USB_MVDMA_CMPLT_CXF |
                USB_MVDMA_CMPLT_F0 |
                USB_MVDMA_CMPLT_F1 |
                USB_MVDMA_CMPLT_F2 |
                USB_MVDMA_CMPLT_F3);
    // regval &= ~(USB_MVDMA_ERROR_CXF |
    //             USB_MVDMA_ERROR_F0 |
    //             USB_MVDMA_ERROR_F1 |
    //             USB_MVDMA_ERROR_F2 |
    //             USB_MVDMA_ERROR_F3);
    putreg32(regval, BLFB_USB_BASE + USB_DEV_MISG3_OFFSET);

    /* enable group irq */
    regval = getreg32(BLFB_USB_BASE + USB_DEV_MIGR_OFFSET);
    regval &= ~USB_MINT_G0;
    regval &= ~USB_MINT_G1;
    regval &= ~USB_MINT_G2;
    regval &= ~USB_MINT_G3;
    regval &= ~USB_MINT_G4;
    putreg32(regval, BLFB_USB_BASE + USB_DEV_MIGR_OFFSET);

    /* enable device irq */
    regval = getreg32(BLFB_USB_BASE + USB_GLB_INT_OFFSET);
    regval |= USB_MHC_INT;
    regval |= USB_MOTG_INT;
    regval &= ~USB_MDEV_INT;
    putreg32(regval, BLFB_USB_BASE + USB_GLB_INT_OFFSET);

    bflb_usb_source_group_int_clear(2, 0x3ff);
    bflb_usb_source_group_int_clear(3, 0xffffffff);

    for (uint8_t i = 1; i < 9; i++) {
        bflb_usb_set_ep_fifomap(i, 15);
        bflb_usb_set_fifo_epmap(i, 15, 0);
    }

    /* enable vdma */
    regval = getreg32(BLFB_USB_BASE + USB_VDMA_CTRL_OFFSET);
    regval |= USB_VDMA_EN;
    putreg32(regval, BLFB_USB_BASE + USB_VDMA_CTRL_OFFSET);

    regval = getreg32(BLFB_USB_BASE + USB_PHY_TST_OFFSET);
    regval &= ~USB_UNPLUG;
    putreg32(regval, BLFB_USB_BASE + USB_PHY_TST_OFFSET);

    /* enable global irq */
    regval = getreg32(BLFB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval |= USB_GLINT_EN_HOV;
    putreg32(regval, BLFB_USB_BASE + USB_DEV_CTL_OFFSET);

    return 0;
}
