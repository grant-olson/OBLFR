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
    regval &= ~PDS_REG_USB_IDDIG;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    
    /*    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval = getreg32(BLFB_USB_BASE + USB_OTG_CSR_OFFSET);
    regval |= USB_A_BUS_DROP_HOV;
    regval &= ~USB_A_BUS_REQ_HOV;
    putreg32(regval, BLFB_USB_BASE + USB_OTG_CSR_OFFSET);
    */

    bflb_mtimer_delay_ms(10);
    /* enable vbus and bus control */
    /*    regval = getreg32(BLFB_USB_BASE + USB_OTG_CSR_OFFSET);
    regval &= ~USB_A_BUS_DROP_HOV;
    regval |= USB_A_BUS_REQ_HOV;
    putreg32(regval, BLFB_USB_BASE + USB_OTG_CSR_OFFSET);
    */
    
    regval = getreg32(BLFB_USB_BASE + USB_GLB_INT_OFFSET);
    regval |= USB_MDEV_INT;
    regval &= ~USB_MOTG_INT;
    regval &= ~USB_MHC_INT;
    putreg32(regval, BLFB_USB_BASE + USB_GLB_INT_OFFSET);
}

