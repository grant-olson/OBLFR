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

//void USBD_IRQHandler(int irq, void *arg);

//extern void USBH_IRQHandler();

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

void usb_hc_low_level_init(void)
{
    uint32_t regval;

    bflb_usb_phy_init();

    //    bflb_irq_attach(37, USBH_IRQHandler, NULL);
    //bflb_irq_enable(37);

    /* enable device-A for host */
    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval &= ~PDS_REG_USB_IDDIG;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    
    regval = getreg32(BLFB_USB_BASE + USB_OTG_CSR_OFFSET);
    regval |= USB_A_BUS_DROP_HOV;
    regval &= ~USB_A_BUS_REQ_HOV;
    putreg32(regval, BLFB_USB_BASE + USB_OTG_CSR_OFFSET);

    bflb_mtimer_delay_ms(10);

    /* enable vbus and bus control */
    regval = getreg32(BLFB_USB_BASE + USB_OTG_CSR_OFFSET);
    regval &= ~USB_A_BUS_DROP_HOV;
    regval |= USB_A_BUS_REQ_HOV;
    putreg32(regval, BLFB_USB_BASE + USB_OTG_CSR_OFFSET);

    regval = getreg32(BLFB_USB_BASE + USB_GLB_FEATURE_OFFSET);
    regval &= ~USB_DEV_ONLY;
    regval |= USB_HOST_ONLY;
    putreg32(regval, BLFB_USB_BASE + USB_GLB_FEATURE_OFFSET);

    regval = getreg32(BLFB_USB_BASE + USB_GLB_ISR_OFFSET);
    regval &= ~USB_DEV_INT;
    regval &= ~USB_OTG_INT;
    regval |= USB_HC_INT;
    putreg32(regval, BLFB_USB_BASE + USB_GLB_ISR_OFFSET);

    regval = getreg32(BLFB_USB_BASE + USB_GLB_INT_OFFSET);
    regval &= ~USB_MDEV_INT;
    regval &= ~USB_MOTG_INT;
    regval |= USB_MHC_INT;
    putreg32(regval, BLFB_USB_BASE + USB_GLB_INT_OFFSET);
}

int usb_dc_init(void)
{
    uint32_t regval;

    bflb_usb_phy_init();

    bflb_irq_attach(37, USBD_IRQHandler, NULL);
    bflb_irq_enable(37);

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
