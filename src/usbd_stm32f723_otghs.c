/* This file is the part of the Lightweight USB device Stack for STM32 microcontrollers
 *
 * Copyright ©2016 Dmitry Filimonchuk <dmitrystu[at]gmail[dot]com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32.h"
#include "usb.h"

#if defined(USBD_STM32F723HS)

// ------------------------------------------
// BEGIN CUBE CODE
// ------------------------------------------

#if !defined  (HSE_VALUE) 
#define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/* This variable is updated in three ways:
   1) by calling CMSIS function SystemCoreClockUpdate()
   2) by calling HAL API function HAL_RCC_GetHCLKFreq()
   3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
   Note: If you use this function to configure the system clock; then there
   is no need to call the 2 first functions listed above, since SystemCoreClock
   variable is updated automatically.
*/
uint32_t SystemCoreClock = 16000000;

#define USBx_PCGCCTL    *(__IO uint32_t *)((uint32_t)USBx_BASE + USB_OTG_PCGCCTL_BASE)
#define USBx_DEVICE     ((USB_OTG_DeviceTypeDef *)(USBx_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_SPEED_HIGH                     0U
#define USBx_INEP(i)    ((USB_OTG_INEndpointTypeDef *)(USBx_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)(USBx_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))

#ifndef STM32F723xx
#error not STM32F723xx
#endif

#ifndef HSE_25MHZ
#error not HSE_25MHZ
#endif

void __libc_init_array(void) { }

void SysTick_Handler(void) { }  // TODO disable TICKINT not strictly necessary

void HAL_Delay(volatile uint32_t Delay)
{
  while(Delay) {
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
      Delay--;
    }
  }
}

void HAL_PCD_MspInit(void)
{
  /* Enable HS PHY Clock */
  //    __HAL_RCC_OTGPHYC_CLK_ENABLE();
  _BST(RCC->APB2ENR, RCC_APB2ENR_OTGPHYCEN);

  /* Enable USB HS Clocks */
  //    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
  _BST(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSEN);

  //    __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();
  _BST(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSULPIEN);

  /* Set USBHS Interrupt to the lowest priority */
  //    HAL_NVIC_SetPriority(OTG_HS_IRQn, 5, 0);
  uint32_t prioritygroup = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(OTG_HS_IRQn, NVIC_EncodePriority(prioritygroup, 5, 0));

  /* Enable USBHS Interrupt */
  //    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
  NVIC_EnableIRQ(OTG_HS_IRQn);
}

static void USB_HS_PHYCInit(void)
{
  /* Enable LDO */
  USB_HS_PHYC->USB_HS_PHYC_LDO |= USB_HS_PHYC_LDO_ENABLE;

  /* wait for LDO Ready */
  while ((USB_HS_PHYC->USB_HS_PHYC_LDO & USB_HS_PHYC_LDO_STATUS) == 0U);

  /* Controls PHY frequency operation selection */
  /* HSE = 25MHz */
  USB_HS_PHYC->USB_HS_PHYC_PLL = (0x5U << 1);

  /* Control the tuning interface of the High Speed PHY */
#if !defined  (USB_HS_PHYC_TUNE_VALUE)
#define USB_HS_PHYC_TUNE_VALUE        0x00000F13U /*!< Value of USB HS PHY Tune */
#endif /* USB_HS_PHYC_TUNE_VALUE */
  USB_HS_PHYC->USB_HS_PHYC_TUNE |= USB_HS_PHYC_TUNE_VALUE;

  /* Enable PLL internal PHY */
  USB_HS_PHYC->USB_HS_PHYC_PLL |= USB_HS_PHYC_PLL_PLLEN;

  /* 2ms Delay required to get internal phy clock stable */
  HAL_Delay(2U);
}

static void USB_CoreReset(USB_OTG_GlobalTypeDef *USBx)
{
  /* Wait for AHB master IDLE state. */
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  /* Core Soft Reset */
  USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);
}

void USB_CoreInit(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);

  /* Init The UTMI Interface */
  USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

  /* Select vbus source */
  USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);

  /* Select UTMI Interace */
  USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_ULPI_UTMI_SEL;
  USBx->GCCFG |= USB_OTG_GCCFG_PHYHSEN;

  /* Enables control of a High Speed USB PHY */
  USB_HS_PHYCInit();

  if (0) // TODO figure out external VBUS???
    {
      USBx->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVBUSD;
    }
  /* Reset after a PHY select  */
  USB_CoreReset(USBx);
}

void USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
  // else if (mode == USB_DEVICE_MODE)
  USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
  HAL_Delay(50U);
}

void USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num)
{
  USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (num << 6));
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);
}

void USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);
}

void USB_DevInit(USB_OTG_GlobalTypeDef *USBx)
{
  const int dev_endpoints = 9;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t i;
  for (i = 0U; i < 15U; i++)
    {
      USBx->DIEPTXF[i] = 0U;
    }

  /* VBUS Sensing setup */
  //  if (cfg.vbus_sensing_enable == 0U)
  //  {
  //    USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
  //
  //    /* Deactivate VBUS Sensing B */
  //    USBx->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
  //
  //    /* B-peripheral session valid override enable */
  //    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  //    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  //  }
  //  else
  //  {
  /* Enable HW VBUS sensing */
  USBx->GCCFG |= USB_OTG_GCCFG_VBDEN;
  //  }

  /* Restart the Phy Clock */
  USBx_PCGCCTL = 0U;

  /* Device mode configuration */
  //      /* Set Core speed to High speed mode */
  //      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH);
  USBx_DEVICE->DCFG |= USB_OTG_SPEED_HIGH;

  /* Flush the FIFOs */
  USB_FlushTxFifo(USBx, 0x10U); /* all Tx FIFOs */

  USB_FlushRxFifo(USBx);

  /* Clear all pending Device Interrupts */
  USBx_DEVICE->DIEPMSK = 0U;
  USBx_DEVICE->DOEPMSK = 0U;
  USBx_DEVICE->DAINTMSK = 0U;

  for (i = 0U; i < dev_endpoints; i++) {
    if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
      if (i == 0U) {
        USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
      } else {
        USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
      }
    } else {
      USBx_INEP(i)->DIEPCTL = 0U;
    }
    USBx_INEP(i)->DIEPTSIZ = 0U;
    USBx_INEP(i)->DIEPINT  = 0xFB7FU;
  }

  for (i = 0U; i < dev_endpoints; i++) {
    if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
      if (i == 0U) {
        USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
      } else {
        USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
      }
    } else {
      USBx_OUTEP(i)->DOEPCTL = 0U;
    }
    USBx_OUTEP(i)->DOEPTSIZ = 0U;
    USBx_OUTEP(i)->DOEPINT  = 0xFB7FU;
  }

  USBx_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

  /* Disable all interrupts. */
  USBx->GINTMSK = 0U;

  /* Clear any pending interrupts */
  USBx->GINTSTS = 0xBFFFFFFFU;

  /* Enable the common interrupts */
  //  if (cfg.dma_enable == 0U)
  //  {
  USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
  //  }

  /* Enable interrupts matching to the Device mode ONLY */
  USBx->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
    USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
    USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM |
    USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;

  //  if (cfg.Sof_enable != 0U)
  //  {
  //    USBx->GINTMSK |= USB_OTG_GINTMSK_SOFM;
  //  }

  //  if (cfg.vbus_sensing_enable == 1U)
  //  {
  USBx->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
  //  }
}

void USB_DevConnect(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
  HAL_Delay(3U);
}

void USB_DevDisconnect(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
  HAL_Delay(3U);
}

void init_usb(void)
{
  HAL_PCD_MspInit();
  USB_CoreInit(USB_OTG_HS);
  USB_SetCurrentMode(USB_OTG_HS);
  USB_DevInit(USB_OTG_HS);
  USB_DevDisconnect(USB_OTG_HS);

  //	HAL_PCDEx_SetRxFiFo(&hpcd, 0x200);
  //	HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x80);
  //	HAL_PCDEx_SetTxFiFo(&hpcd, 1, 0x174);
  USB_OTG_HS->GRXFSIZ = 0x200;
  uint32_t Tx_Offset = USB_OTG_HS->GRXFSIZ;
  USB_OTG_HS->DIEPTXF0_HNPTXFSIZ = ((uint32_t)0x80 << 16) | Tx_Offset;
  Tx_Offset += (USB_OTG_HS->DIEPTXF0_HNPTXFSIZ) >> 16;
  USB_OTG_HS->DIEPTXF[1 - 1U] = ((uint32_t)0x174 << 16) | Tx_Offset;

  USB_DevConnect(USB_OTG_HS);
  //	Enables the controller's Global Int in the AHB Config reg
  //	USB_OTG_HS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
}

// ------------------------------------------
// END CUBE CODE
// ------------------------------------------

#define MAX_EP          9 // per cubef7
#define MAX_RX_PACKET   128 // usbhs constant
#define MAX_CONTROL_EP  1
#define MAX_FIFO_SZ     1024  /*in 32-bit chunks */ // should be correct per 1228

#define RX_FIFO_SZ      (10 + (2 * (MAX_RX_PACKET / 4) + 1)) // TODO what's the math behind this?

#define STATUS_VAL(x)   (USBD_HW_ADDRFST | (x))

#define BWAIT for(volatile long i; 1;){ i++; }

static USB_OTG_GlobalTypeDef * const OTG  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
static USB_OTG_DeviceTypeDef * const OTGD = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
static volatile uint32_t * const OTGPCTL  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE);


inline static uint32_t* EPFIFO(uint32_t ep) {
  return (uint32_t*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep << 12));
}

inline static USB_OTG_INEndpointTypeDef* EPIN(uint32_t ep) {
  return (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep << 5));
}

inline static USB_OTG_OUTEndpointTypeDef* EPOUT(uint32_t ep) {
  return (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep << 5));
}

inline static void Flush_RX(void) {
  _BST(OTG->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
  _WBC(OTG->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
}

inline static void Flush_TX(uint8_t ep) {
  _BMD(OTG->GRSTCTL, USB_OTG_GRSTCTL_TXFNUM,
       _VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, ep) | USB_OTG_GRSTCTL_TXFFLSH);
  _WBC(OTG->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH);
}

static uint32_t getinfo(void) {
  if (!(RCC->AHB1ENR & RCC_AHB1ENR_OTGHSEN)) return STATUS_VAL(0);
  if (!(OTGD->DCTL & USB_OTG_DCTL_SDIS)) {
    if (_FLD2VAL(USB_OTG_DSTS_ENUMSPD, OTGD->DSTS) == 0x00) return STATUS_VAL(USBD_HW_ENABLED | USBD_HW_SPEED_HS);
    if (_FLD2VAL(USB_OTG_DSTS_ENUMSPD, OTGD->DSTS) == 0x03) return STATUS_VAL(USBD_HW_ENABLED | USBD_HW_SPEED_FS);
  }
  return STATUS_VAL(USBD_HW_ENABLED | USBD_HW_SPEED_NC);
}

// should be OK; TODO endpoint addressing correct???
static void ep_setstall(uint8_t ep, bool stall) {
  if (ep & 0x80) {
    ep &= 0x7F;
    uint32_t _t = EPIN(ep)->DIEPCTL;
    if (_t & USB_OTG_DIEPCTL_USBAEP) {
      if (stall) {
        _BST(_t, USB_OTG_DIEPCTL_STALL);
      } else {
        _BMD(_t, USB_OTG_DIEPCTL_STALL,
             USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_SNAK);
      }
      EPIN(ep)->DIEPCTL = _t;
    }
  } else {
    uint32_t _t = EPOUT(ep)->DOEPCTL;
    if (_t & USB_OTG_DOEPCTL_USBAEP) {
      if (stall) {
        _BST(_t, USB_OTG_DOEPCTL_STALL);
      } else {
        _BMD(_t, USB_OTG_DOEPCTL_STALL,
             USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_CNAK);
      }
      EPOUT(ep)->DOEPCTL = _t;
    }
  }
}

// OK
static bool ep_isstalled(uint8_t ep) {
  if (ep & 0x80) {
    ep &= 0x7F;
    return (EPIN(ep)->DIEPCTL & USB_OTG_DIEPCTL_STALL) ? true : false;
  } else {
    return (EPOUT(ep)->DOEPCTL & USB_OTG_DOEPCTL_STALL) ? true : false;
  }
}

static void enable(bool enable) {
  if (enable) {
    init_usb();
    /*         /\* enabling USB_OTG in RCC *\/ */
    /*         _BST(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSEN); */
    /*         _WBS(OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL); */

    /*         /\* configure OTG as device *\/ */
    /*         // TODO USB_OTG_GUSBCFG_PHYSEL should be 0? per 1212; trdt and tocal should be fine */
    /*         OTG->GUSBCFG = USB_OTG_GUSBCFG_FDMOD | */
    /*           _VAL2FLD(USB_OTG_GUSBCFG_TRDT, 0x09) | */
    /*           _VAL2FLD(USB_OTG_GUSBCFG_TOCAL, 0x01); */
    /*         /\* OTG->GUSBCFG = USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | *\/ */
    /*         /\*                _VAL2FLD(USB_OTG_GUSBCFG_TRDT, 0x09) | *\/ */
    /*         /\*                _VAL2FLD(USB_OTG_GUSBCFG_TOCAL, 0x01); *\/ */

    /*         // per 1231, don't think this matters for hs? just fs */
    /* /\* VBUS detect alternafe function AF12 for PB13 is missed in tech documentation *\/ */
    /* /\* #if defined(USBD_VBUS_DETECT) && defined(USBD_SOF_OUT) *\/ */
    /* /\*         OTG->GCCFG = USB_OTG_GCCFG_VBUSBSEN | USB_OTG_GCCFG_SOFOUTEN; *\/ */
    /* /\* #elif defined(USBD_VBUS_DETECT) *\/ */
    /* /\*         OTG->GCCFG = USB_OTG_GCCFG_VBUSBSEN; *\/ */
    /* /\* #elif defined(USBD_SOF_OUT) *\/ */
    /* /\*         OTG->GCCFG = USB_OTG_GCCFG_NOVBUSSENS | USB_OTG_GCCFG_SOFOUTEN; *\/ */
    /* /\* #else *\/ */
    /* /\*         OTG->GCCFG = USB_OTG_GCCFG_NOVBUSSENS; *\/ */
    /* /\* #endif *\/ */
    /*         OTG->GCCFG = USB_OTG_GCCFG_VBDEN; */

    /*         /\* BWAIT; *\/ */

    /*         /\* do core soft reset *\/ */
    /*         _BST(OTG->GRSTCTL, USB_OTG_GRSTCTL_CSRST); */
    /*         /\* _WBC(OTG->GRSTCTL, USB_OTG_GRSTCTL_CSRST); *\/ */
    /*         _WBS(OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL); */
    /*         /\* Setup USB FS speed *\/ */

    /*         // TODO per 1253, this sets to FS using internal phy; should be 00 for hs? */
    /*         /\* _BMD(OTGD->DCFG, USB_OTG_DCFG_DSPD, _VAL2FLD(USB_OTG_DCFG_DSPD, 0x03)); *\/ */
    /*         _BMD(OTGD->DCFG, USB_OTG_DCFG_DSPD, _VAL2FLD(USB_OTG_DCFG_DSPD, 0x00)); */

    /*         /\* start PHY clock *\/ */
    /*         *OTGPCTL = 0; */
    /*         /\* soft disconnect device *\/ */
    /*         _BST(OTGD->DCTL, USB_OTG_DCTL_SDIS); */
    /*         /\* setting max RX FIFO size *\/ */
    /*         OTG->GRXFSIZ = RX_FIFO_SZ; */
    /*         /\* setting up EP0 TX FIFO SZ as 64 byte *\/ */
    /*         // per 1228 this probably stays the same */
    /*         OTG->DIEPTXF0_HNPTXFSIZ = RX_FIFO_SZ | (0x10 << 16); */
    /*         /\* unmask EP interrupts *\/ */
    /*         OTGD->DIEPMSK = USB_OTG_DIEPMSK_XFRCM; */
    /*         /\* unmask core interrupts *\/ */
    /*         OTG->GINTMSK  = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | */
    /* #if !defined(USBD_SOF_DISABLED) */
    /*                         USB_OTG_GINTMSK_SOFM | */
    /* #endif */
    /*                         USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM | */
    /*                         USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_RXFLVLM | */
    /*                         USB_OTG_GINTMSK_MMISM | USB_OTG_GINTMSK_OTGINT | // TODO added these per 1225 */
    /*                         USB_OTG_GINTMSK_ESUSPM ; // TODO added per 1298 */
    /*         /\* clear pending interrupts *\/ */
    /*         OTG->GINTSTS = 0xFFFFFFFF; */
    /*         /\* unmask global interrupt *\/ */
    /*         _BST(OTG->GAHBCFG, USB_OTG_GAHBCFG_GINT); */
  } else {
    // disabling peripheral via RCC
    if (RCC->AHB1ENR & RCC_AHB1ENR_OTGHSEN) {
      _BST(RCC->AHB1RSTR, RCC_AHB1RSTR_OTGHRST);
      _BCL(RCC->AHB1RSTR, RCC_AHB1RSTR_OTGHRST);
      _BCL(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSEN);
    }
  }
}

static uint8_t connect(bool connect) {
  if (connect) {
    _BST(OTG->GCCFG, USB_OTG_GCCFG_PWRDWN);
    _BCL(OTGD->DCTL, USB_OTG_DCTL_SDIS);
  } else {
    _BST(OTGD->DCTL, USB_OTG_DCTL_SDIS);
    _BCL(OTG->GCCFG, USB_OTG_GCCFG_PWRDWN);
  }
  return usbd_lane_unk;
}

static void setaddr (uint8_t addr) {
  _BMD(OTGD->DCFG, USB_OTG_DCFG_DAD, addr << 4);
}

/**\brief Helper. Set up TX fifo
 * \param ep endpoint index
 * \param epsize required max packet size in bytes
 * \return true if TX fifo is successfully set
 */
static bool set_tx_fifo(uint8_t ep, uint16_t epsize) {
  uint32_t _fsa = OTG->DIEPTXF0_HNPTXFSIZ;
  /* calculating initial TX FIFO address. next from EP0 TX fifo */
  _fsa = 0xFFFF & (_fsa + (_fsa >> 16));
  /* looking for next free TX fifo address */
  for (int i = 0; i < (MAX_EP - 1); i++) {
    uint32_t _t = OTG->DIEPTXF[i];
    if ((_t & 0xFFFF) < 0x200) {
      _t = 0xFFFF & (_t + (_t >> 16));
      if (_t > _fsa) {
        _fsa = _t;
      }
    }
  }
  /* calculating requited TX fifo size */
  /* getting in 32 bit terms */
  epsize = (epsize + 0x03) >> 2;
  /* it must be 16 32-bit words minimum */
  if (epsize < 0x10) epsize = 0x10;
  /* checking for the available fifo */
  if ((_fsa + epsize) > MAX_FIFO_SZ) return false;
  /* programming fifo register */
  _fsa |= (epsize << 16);
  OTG->DIEPTXF[ep - 1] = _fsa;
  return true;
}

static bool ep_config(uint8_t ep, uint8_t eptype, uint16_t epsize) {
  if (ep == 0) {
    /* configuring control endpoint EP0 */
    // maximum packet size
    uint32_t mpsize;
    if (epsize <= 0x08) {
      epsize = 0x08;
      mpsize = 0x03;
    } else if (epsize <= 0x10) {
      epsize = 0x10;
      mpsize = 0x02;
    } else if (epsize <= 0x20) {
      epsize = 0x20;
      mpsize = 0x01;
    } else {
      epsize = 0x40;
      mpsize = 0x00;
    }
    /* EP0 TX FIFO size is setted on init level */
    /* enabling RX and TX interrupts from EP0 */
    // TODO how is this enabling rx/tx interrupts on both tx and rx, if DAINTMSK 31:16 is in and 15:0 is out?
    // parent datasheet says same spec as well but cfg still looks wrong
    OTGD->DAINTMSK |= 0x00010001;
    /* setting up EP0 TX and RX registers */
    /*EPIN(ep)->DIEPTSIZ  = epsize;*/
    EPIN(ep)->DIEPCTL = mpsize | USB_OTG_DIEPCTL_SNAK;
    /* 1 setup packet, 1 packets total */
    EPOUT(ep)->DOEPTSIZ = epsize | (1 << 29) | (1 << 19);
    EPOUT(ep)->DOEPCTL = mpsize | USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
    return true;
  }
  if (ep & 0x80) {
    ep &= 0x7F;
    USB_OTG_INEndpointTypeDef* epi = EPIN(ep);
    /* configuring TX endpoint */
    /* setting up TX fifo and size register */
    if ((eptype == USB_EPTYPE_ISOCHRONUS) ||
        (eptype == (USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF))) {
      if (!set_tx_fifo(ep, epsize << 1)) return false;
    } else {
      if (!set_tx_fifo(ep, epsize)) return false;
    }
    /* enabling EP TX interrupt */
    OTGD->DAINTMSK |= (0x0001UL << ep);
    /* setting up TX control register*/
    // TODO assuming that all this endpoint configuration is correct
    switch (eptype) {
    case USB_EPTYPE_ISOCHRONUS:
      epi->DIEPCTL = USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK |
        (0x01 << 18) | USB_OTG_DIEPCTL_USBAEP |
        USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
        (ep << 22) | epsize;
      break;
    case USB_EPTYPE_BULK:
    case USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF:
      epi->DIEPCTL = USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_USBAEP |
        (0x02 << 18) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
        (ep << 22) | epsize;
      break;
    default:
      epi->DIEPCTL = USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_USBAEP |
        (0x03 << 18) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
        (ep << 22) | epsize;
      break;
    }
  } else {
    /* configuring RX endpoint */
    USB_OTG_OUTEndpointTypeDef* epo = EPOUT(ep);
    /* setting up RX control register */
    switch (eptype) {
    case USB_EPTYPE_ISOCHRONUS:
      epo->DOEPCTL = USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_CNAK |
        USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP |
        (0x01 << 18) | epsize;
      break;
    case USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF:
    case USB_EPTYPE_BULK:
      epo->DOEPCTL = USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_CNAK |
        USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP |
        (0x02 << 18) | epsize;
      break;
    default:
      epo->DOEPCTL = USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_CNAK |
        USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP |
        (0x03 << 18) | epsize;
      break;
    }
  }
  return true;
}

static void ep_deconfig(uint8_t ep) {
  ep &= 0x7F;
  volatile USB_OTG_INEndpointTypeDef*  epi = EPIN(ep);
  volatile USB_OTG_OUTEndpointTypeDef* epo = EPOUT(ep);
  /* deconfiguring TX part */
  /* disable interrupt */
  // TODO why does this work??????
  OTGD->DAINTMSK &= ~(0x10001 << ep);
  /* decativating endpoint */
  _BCL(epi->DIEPCTL, USB_OTG_DIEPCTL_USBAEP);
  /* flushing FIFO */
  Flush_TX(ep);
  /* disabling endpoint */
  if ((epi->DIEPCTL & USB_OTG_DIEPCTL_EPENA) && (ep != 0)) {
    epi->DIEPCTL = USB_OTG_DIEPCTL_EPDIS;
  }
  /* clean EP interrupts */
  epi->DIEPINT = 0xFF;
  /* deconfiguring TX FIFO */
  if (ep > 0) {
    OTG->DIEPTXF[ep-1] = 0x02000200 + 0x200 * ep;
  }
  /* deconfigureing RX part */
  _BCL(epo->DOEPCTL, USB_OTG_DOEPCTL_USBAEP);
  if ((epo->DOEPCTL & USB_OTG_DOEPCTL_EPENA) && (ep != 0)) {
    epo->DOEPCTL = USB_OTG_DOEPCTL_EPDIS;
  }
  epo->DOEPINT = 0xFF;
}

static int32_t ep_read(uint8_t ep, void* buf, uint16_t blen) {
  uint32_t len, tmp;
  volatile uint32_t *fifo = EPFIFO(0);
  /* no data in RX FIFO */
  if (!(OTG->GINTSTS & USB_OTG_GINTSTS_RXFLVL)) return -1;
  ep &= 0x7F;
  if ((OTG->GRXSTSR & USB_OTG_GRXSTSP_EPNUM) != ep) return -1;
  /* pop data from fifo */
  len = _FLD2VAL(USB_OTG_GRXSTSP_BCNT, OTG->GRXSTSP);
  for (int idx = 0; idx < len; idx++) {
    if ((idx & 0x03) == 0x00) {
      tmp = *fifo;
    }
    if (idx < blen) {
      ((uint8_t*)buf)[idx] = tmp & 0xFF;
      tmp >>= 8;
    }
  }
  return (len < blen) ? len : blen;
}

static int32_t ep_write(uint8_t ep, void *buf, uint16_t blen) {
  uint32_t len, tmp;
  ep &= 0x7F;
  volatile uint32_t* fifo = EPFIFO(ep);
  USB_OTG_INEndpointTypeDef* epi = EPIN(ep);
  /* transfer data size in 32-bit words */
  len = (blen + 3) >> 2;
  /* no enough space in TX fifo */
  if (len > _FLD2VAL(USB_OTG_DTXFSTS_INEPTFSAV, epi->DTXFSTS)) return -1;
  if (ep != 0 && epi->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
    return -1;
  }
  _BMD(epi->DIEPTSIZ,
       USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_MULCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
       _VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_MULCNT, 1 ) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, blen));
  _BMD(epi->DIEPCTL, USB_OTG_DIEPCTL_STALL, USB_OTG_DOEPCTL_CNAK);
  _BST(epi->DIEPCTL, USB_OTG_DOEPCTL_EPENA);
  /* push data to FIFO */
  tmp = 0;
  for (int idx = 0; idx < blen; idx++) {
    tmp |= (uint32_t)((uint8_t*)buf)[idx] << ((idx & 0x03) << 3);
    if ((idx & 0x03) == 0x03 || (idx + 1) == blen) {
      *fifo = tmp;
      tmp = 0;
    }
  }
  return blen;
}

static uint16_t get_frame (void) {
  return _FLD2VAL(USB_OTG_DSTS_FNSOF, OTGD->DSTS);
}

static void evt_poll(usbd_device *dev, usbd_evt_callback callback) {
  uint32_t evt;
  uint32_t ep = 0;
  while (1) {
    uint32_t _t = OTG->GINTSTS;
    /* bus RESET event */
    if (_t & USB_OTG_GINTSTS_USBRST) {
      OTG->GINTSTS = USB_OTG_GINTSTS_USBRST;
      for (uint8_t i = 0; i < MAX_EP; i++ ) {
        ep_deconfig(i);
      }
      Flush_RX();
      continue;
    } else if (_t & USB_OTG_GINTSTS_ENUMDNE) {
      OTG->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
      evt = usbd_evt_reset;
    } else if (_t & USB_OTG_GINTSTS_IEPINT) {
      for (;; ep++) {
        USB_OTG_INEndpointTypeDef* epi = EPIN(ep);
        if (ep >= MAX_EP) return;
        if (epi->DIEPINT & USB_OTG_DIEPINT_XFRC) {
          _BST(epi->DIEPINT, USB_OTG_DIEPINT_XFRC);
          evt = usbd_evt_eptx;
          ep |= 0x80;
          break;
        }
      }
    } else if (_t & USB_OTG_GINTSTS_RXFLVL) {
      _t = OTG->GRXSTSR;
      ep = _t & USB_OTG_GRXSTSP_EPNUM;
      switch (_FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, _t)) {
      case 0x02:  /* OUT recieved */
        evt = usbd_evt_eprx;
        break;
      case 0x06:  /* SETUP recieved */
        /* flushing TX if something stuck in control endpoint */
        if (EPIN(ep)->DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT) {
          Flush_TX(ep);
        }
        evt = usbd_evt_epsetup;
        break;
      case 0x03:  /* OUT completed */
      case 0x04:  /* SETUP completed */
        _BST(EPOUT(ep)->DOEPCTL, USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
      default:
        /* pop GRXSTSP */
        OTG->GRXSTSP;
        continue;
      }
#if !defined(USBD_SOF_DISABLED)
    } else if (_t & USB_OTG_GINTSTS_SOF) {
      OTG->GINTSTS = USB_OTG_GINTSTS_SOF;
      evt = usbd_evt_sof;
#endif
    } else if (_t & USB_OTG_GINTSTS_USBSUSP) {
      evt = usbd_evt_susp;
      OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    } else if (_t & USB_OTG_GINTSTS_WKUINT) {
      OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;
      evt = usbd_evt_wkup;
    } else {
      /* no more supported events */
      return;
    }
    return callback(dev, evt, ep);
  }
}

// TODO ????? https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
static uint32_t fnv1a32_turn (uint32_t fnv, uint32_t data ) {
  for (int i = 0; i < 4 ; i++) {
    fnv ^= (data & 0xFF);
    fnv *= 16777619;
    data >>= 8;
  }
  return fnv;
}

static uint16_t get_serialno_desc(void *buffer) {
  struct  usb_string_descriptor *dsc = buffer;
  uint16_t *str = dsc->wString;
  uint32_t fnv = 2166136261;
  fnv = fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x00));
  fnv = fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x04));
  fnv = fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x08));
  for (int i = 28; i >= 0; i -= 4 ) {
    uint16_t c = (fnv >> i) & 0x0F;
    c += (c < 10) ? '0' : ('A' - 10);
    *str++ = c;
  }
  dsc->bDescriptorType = USB_DTYPE_STRING;
  dsc->bLength = 18;
  return 18;
}

__attribute__((externally_visible)) const struct usbd_driver usbd_otghs = {
  getinfo,
  enable,
  connect,
  setaddr,
  ep_config,
  ep_deconfig,
  ep_read,
  ep_write,
  ep_setstall,
  ep_isstalled,
  evt_poll,
  get_frame,
  get_serialno_desc,
};

#endif //USBD_STM32F723HS
