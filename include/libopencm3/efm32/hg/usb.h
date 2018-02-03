/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
 * Copyright (C) 2018 Seb Holzapfel <schnommus@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBOPENCM3_EFM32_USB_H
#define LIBOPENCM3_EFM32_USB_H

#include <libopencm3/cm3/common.h>
#include <libopencm3/usb/usbd.h>

#define USB_CTRL	    MMIO32(USB_BASE + 0x000)
#define USB_STATUS	    MMIO32(USB_BASE + 0x004)
#define USB_IF		    MMIO32(USB_BASE + 0x008)
#define USB_IFS		    MMIO32(USB_BASE + 0x00C)
#define USB_IFC		    MMIO32(USB_BASE + 0x010)
#define USB_IEN		    MMIO32(USB_BASE + 0x014)
#define USB_ROUTE	    MMIO32(USB_BASE + 0x018)

/* USB_CTRL */
/* Bits 31:26 - Reserved */
#define USB_CTRL_BIASPROGEM23_MASK (0x3 << 24)
/* Bits 23:22 - Reserved */
#define USB_CTRL_BIASPROGEM01_MASK (0x3 << 20)
/* Bits 19:18 - Reserved */
#define USB_CTRL_VREGOSEN	   (1 << 17)
#define USB_CTRL_VREGDIS	   (1 << 16)
/* Bits 15:10 - Reserved */
#define USB_CTRL_LEMIDLEEN	   (1 << 9)
/* Bit 8 - Reserved */
#define USB_CTRL_LEMPHYCTRL	   (1 << 7)
/* Bit 6 - Reserved */
#define USB_CTRL_LEMOSCCTRL_MASK   (0x3 << 4)
#define USB_CTRL_LEMOSCCTRL_NONE   (0x0 << 4)
#define USB_CTRL_LEMOSCCTRL_GATE   (0x1 << 4)
/* Bits 3:2 - Reserved */
#define USB_CTRL_DMPUAP		   (1 << 1)
/* Bit 0 - Reserved */

/* USB_ROUTE */
/* Bits 31:3 - Reserved */
#define USB_ROUTE_DMPUPEN	(1 << 2)
/* Bit 1 - Reserved */
#define USB_ROUTE_PHYPEN	(1 << 0)

/* Core Global Control and Status Registers */
#define USB_OTG_BASE		(USB_BASE + 0x3C000)
#define USB_GAHBCFG		MMIO32(USB_OTG_BASE + 0x008)
#define USB_GUSBCFG		MMIO32(USB_OTG_BASE + 0x00C)
#define USB_GRSTCTL		MMIO32(USB_OTG_BASE + 0x010)
#define USB_GINTSTS		MMIO32(USB_OTG_BASE + 0x014)
#define USB_GINTMSK		MMIO32(USB_OTG_BASE + 0x018)
#define USB_GRXSTSR		MMIO32(USB_OTG_BASE + 0x01C)
#define USB_GRXSTSP		MMIO32(USB_OTG_BASE + 0x020)
#define USB_GRXFSIZ		MMIO32(USB_OTG_BASE + 0x024)
#define USB_GNPTXFSIZ		MMIO32(USB_OTG_BASE + 0x028)
#define USB_GDFIFOCFG		MMIO32(USB_OTG_BASE + 0x05C)
#define USB_DIEPTXF(x)		\
	MMIO32(USB_OTG_BASE + 0x104 + (4 * ((x) - 1)))

/* Device-mode Control and Status Registers */
#define USB_DCFG		MMIO32(USB_OTG_BASE + 0x800)
#define USB_DCTL		MMIO32(USB_OTG_BASE + 0x804)
#define USB_DSTS		MMIO32(USB_OTG_BASE + 0x808)
#define USB_DIEPMSK		MMIO32(USB_OTG_BASE + 0x810)
#define USB_DOEPMSK		MMIO32(USB_OTG_BASE + 0x814)
#define USB_DAINT		MMIO32(USB_OTG_BASE + 0x818)
#define USB_DAINTMSK		MMIO32(USB_OTG_BASE + 0x81C)
#define USB_DIEPEMPMSK		MMIO32(USB_OTG_BASE + 0x834)

#define USB_DIEPx_CTL(x)	\
	MMIO32(USB_OTG_BASE + 0x900 + ((x) * 0x20))
#define USB_DIEPx_INT(x)	\
	MMIO32(USB_OTG_BASE + 0x908 + ((x) * 0x20))
#define USB_DIEPx_TSIZ(x)	\
	MMIO32(USB_OTG_BASE + 0x910 + ((x) * 0x20))
#define USB_DIEP0CTL		USB_DIEPx_CTL(0)
#define USB_DIEP0TSIZ		USB_DIEPx_TSIZ(0)
#define USB_DIEP0INT		USB_DIEPx_INT(0)

#define USB_DOEPx_CTL(x)	\
	MMIO32(USB_OTG_BASE + 0xB00 + ((x) * 0x20))
#define USB_DOEPx_INT(x)	\
	MMIO32(USB_OTG_BASE + 0xB08 + ((x) * 0x20))
#define USB_DOEPx_TSIZ(x)	\
	MMIO32(USB_OTG_BASE + 0xB10 + ((x) * 0x20))
#define USB_DOEP0CTL		USB_DOEPx_CTL(0)
#define USB_DOEP0TSIZ		USB_DOEPx_TSIZ(0)
#define USB_DOEP0INT		USB_DOEPx_INT(0)

/* Power and clock gating control and status register */
#define USB_PCGCCTL	    MMIO32(USB_OTG_BASE + 0xE00)

/* Data FIFO */
#define USB_FIFOxD(x)		\
	(&MMIO32(USB_OTG_BASE + (((x) + 1) << 12)))

/* Global CSRs */
/* Power & clock gating control register (USB_PCGCCTL) */
/* Bits 31:7 - Reserved */
#define USB_PCGCCTL_PHYSLEEP		(1 << 6)
/* Bits 5:4 - Reserved */
#define USB_PCGCCTL_RSTPDWNMODULE	(1 << 3)
#define USB_PCGCCTL_PWRCLMP		(1 << 2)
#define USB_PCGCCTL_GATEHCLK		(1 << 1)
#define USB_PCGCCTL_STOPPCLK		(1 << 0)

/* AHB configuration register (USB_GAHBCFG) */
#define USB_GAHBCFG_AHBSINGLE		(1 << 23)
#define USB_GAHBCFG_NOTIALLDMAWRIT	(1 << 22)
#define USB_GAHBCFG_REMMEMSUPP		(1 << 21)
/* Bits 20:8 - Reserved */
#define USB_GAHBCFG_NPTXFEMPLVL		(1 << 7)
/* Bit 6 - Reserved */
#define USB_GAHBCFG_DMAEN		(1 << 5)
#define USB_GAHBCFG_HBSTLEN_MASK	(0xf << 1)
#define USB_GAHBCFG_HBSTLEN_SINGLE	(0x0 << 1)
#define USB_GAHBCFG_GLBLINTRMSK		(1 << 0)

/* USB configuration register (USB_GUSBCFG) */
#define USB_GUSBCFG_CORRUPTTXPKT	(1 << 31)
/* Bits 30:29 - Reserved */
#define USB_GUSBCFG_TXENDDELAY		(1 << 28)
/* Bits 27:23 - Reserved */
#define USB_GUSBCFG_TERMSELDLPULSE	(1 << 22)
/* Bits 21:14 - Reserved */
#define USB_GUSBCFG_USBTRDTIM_MASK	(0xf << 10)
/* Bits 9:6 - Reserved */
#define USB_GUSBCFG_FSINTF		(1 << 5)
/* Bits 4:3 - Reserved */
#define USB_GUSBCFG_TOUTCAL_MASK	(0x7 << 0)

		/* reset register (USB_GRSTCTL) */
#define USB_GRSTCTL_AHBIDLE	   (1 << 31)
#define USB_GRSTCTL_DMAREQ	   (1 << 30)
		/* Bits 29:11 - Reserved */
#define USB_GRSTCTL_TXFNUM_MASK    (0x1f << 6)
#define USB_GRSTCTL_TXFFLSH	   (1 << 5)
#define USB_GRSTCTL_RXFFLSH	   (1 << 4)
		/* Bits 3:2 - Reserved */
#define USB_GRSTCTL_PIUFSSFTRST    (1 << 1)
#define USB_GRSTCTL_CSFTRST	   (1 << 0)

		/* interrupt status register (USB_GINTSTS) */
#define USB_GINTSTS_WKUPINTMSK	   (1 << 31)
		/* Bits 30:24 - Reserved */
#define USB_GINTSTS_RESETDET	   (1 << 23)
#define USB_GINTSTS_FETSUSP	   (1 << 22)
#define USB_GINTSTS_INCOMPLP	   (1 << 21)
#define USB_GINTSTS_INCOMPISOIN    (1 << 20)
#define USB_GINTSTS_OEPINT	   (1 << 19)
#define USB_GINTSTS_IEPINT	   (1 << 18)
/* Bits 17:16 - Reserved */
#define USB_GINTSTS_EOPF	   (1 << 15)
#define USB_GINTSTS_ISOOUTDROP	   (1 << 14)
#define USB_GINTSTS_ENUMDONE	   (1 << 13)
#define USB_GINTSTS_USBRST	   (1 << 12)
#define USB_GINTSTS_USBSUSP	   (1 << 11)
#define USB_GINTSTS_ERLYSUSP	   (1 << 10)
/* Bits 9:8 - Reserved */
#define USB_GINTSTS_GOUTNAKEFF	   (1 << 7)
#define USB_GINTSTS_GINNAKEFF	   (1 << 6)
/* Bit 5 - Reserved */
#define USB_GINTSTS_RXFLVL	   (1 << 4)
#define USB_GINTSTS_SOF		   (1 << 3)
/* Bits 2:1 - Reserved */
#define USB_GINTSTS_CURMOD	   (1 << 0)

/* interrupt mask register (USB_GINTMSK) */
#define USB_GINTMSK_WKUPINTMSK	   (1 << 31)
/* Bits 30:24 - Reserved */
#define USB_GINTMSK_RESETDETMSK    (1 << 23)
#define USB_GINTMSK_FETSUSPMSK	   (1 << 22)
#define USB_GINTMSK_INCOMPLPMSK    (1 << 21)
#define USB_GINTMSK_INCOMPISOINMSK (1 << 20)
#define USB_GINTMSK_OEPINTMSK	   (1 << 19)
#define USB_GINTMSK_IEPINTMSK	   (1 << 18)
/* Bits 17:16 - Reserved */
#define USB_GINTMSK_EOPFMSK	   (1 << 15)
#define USB_GINTMSK_ISOOUTDROPMSK  (1 << 14)
#define USB_GINTMSK_ENUMDONEMSK    (1 << 13)
#define USB_GINTMSK_USBRSTMSK	   (1 << 12)
#define USB_GINTMSK_USBSUSPMSK	   (1 << 11)
#define USB_GINTMSK_ERLYSUSPMSK    (1 << 10)
/* Bits 9:8 - Reserved */
#define USB_GINTMSK_GOUTNAKEFFMSK  (1 << 7)
#define USB_GINTMSK_GINNAKEFFMSK   (1 << 6)
/* Bit 5 - Reserved */
#define USB_GINTMSK_RXFLVLMSK	   (1 << 4)
#define USB_GINTMSK_SOFMSK	   (1 << 3)
/* Bit 2 - Reserved */
#define USB_GINTMSK_MODEMISMSK	   (1 << 1)
/* Bit 0 - Reserved */

/* Receive Status Pop Register (USB_GRXSTSP) */
/* Bits 31:25 - Reserved */
#define USB_GRXSTSP_FN_MASK	      (0xf << 21)
#define USB_GRXSTSP_PKTSTS_MASK       (0xf << 17)
#define USB_GRXSTSP_PKTSTS_GOUTNAK    (0x1 << 17)
#define USB_GRXSTSP_PKTSTS_PKTRCV     (0x2 << 17)
#define USB_GRXSTSP_PKTSTS_XFERCOMPL  (0x3 << 17)
#define USB_GRXSTSP_PKTSTS_SETUPCOMPL (0x4 << 17)
#define USB_GRXSTSP_PKTSTS_SETUPRCV   (0x6 << 17)
#define USB_GRXSTSP_DPID_MASK	      (0x3 << 15)
#define USB_GRXSTSP_DPID_DATA0	      (0x0 << 15)
#define USB_GRXSTSP_DPID_DATA1	      (0x1 << 15)
#define USB_GRXSTSP_DPID_DATA2	      (0x2 << 15)
#define USB_GRXSTSP_DPID_MDATA	      (0x3 << 15)
#define USB_GRXSTSP_BCNT_MASK	      (0x7ff << 4)
#define USB_GRXSTSP_CHEPNUM_MASK      (0xf << 0)

/* Device-mode CSRs */
/* device control register (USB_DCTL) */
/* Bits 31:17 - Reserved */
#define USB_DCTL_NAKONBBLE	   (1 << 16)
#define USB_DCTL_IGNRFRMNUM	   (1 << 15)
/* Bits 14:12 - Reserved */
#define USB_DCTL_PWRONPRGDONE	   (1 << 11)
#define USB_DCTL_CGOUTNAK	   (1 << 10)
#define USB_DCTL_SGOUTNAK	   (1 << 9)
#define USB_DCTL_CGNPINNAK	   (1 << 8)
#define USB_DCTL_SGNPINNAK	   (1 << 7)
#define USB_DCTL_TSTCTL_MASK	   (0x7 << 4)
#define USB_DCTL_GOUTNAKSTS	   (1 << 3)
#define USB_DCTL_GNPINNAKSTS	   (1 << 2)
#define USB_DCTL_SFTDISCON	   (1 << 1)
#define USB_DCTL_RMTWKUPSIG	   (1 << 0)

#define DCTL_WO_BITMASK \
	(USB_DCTL_CGOUTNAK  | USB_DCTL_SGOUTNAK | \
	 USB_DCTL_CGNPINNAK | USB_DCTL_SGNPINNAK)

/* device configuration register (USB_DCFG) */
#define USB_DCFG_RESVALID_MASK	   (0x3f << 26)
/* Bits 25:16 - Reserved */
#define USB_DCFG_ERRATICINTMSK	   (1 << 15)
/* Bits 14:13 - Reserved */
#define USB_DCFG_PERFRINT_MASK	   (0x3 << 11)
#define USB_DCFG_DEVADDR_MASK	   (0x7f << 4)
#define USB_DCFG_ENA32KHZSUSP	   (1 << 3)
#define USB_DCFG_NZSTSOUTHSHK	   (1 << 2)
#define USB_DCFG_DEVSPD_MASK	   (0x3 << 0)
#define USB_DCFG_DEVSPD_FS	   (0x3 << 0)
#define USB_DCFG_DEVSPD_LS	   (0x2 << 0)

/* Device IN Endpoint Common Interrupt Mask Register (USB_DIEPMSK) */
/* Bits 31:14 - Reserved */
#define USB_DIEPMSK_NAKMSK	   (1 << 13)
/* Bits 12:9 - Reserved */
#define USB_DIEPMSK_TXFIFOUNDRNMSK (1 << 8)
/* Bit 7 - Reserved */
#define USB_DIEPMSK_INEPNAKEFFMSK  (1 << 6)
/* Bit 5 - Reserved */
#define USB_DIEPMSK_INTKNTXFEMPMSK (1 << 4)
#define USB_DIEPMSK_TIMEOUTMSK	   (1 << 3)
#define USB_DIEPMSK_AHBERRMSK	   (1 << 2)
#define USB_DIEPMSK_EPDISBLDMSK    (1 << 1)
#define USB_DIEPMSK_XFERCOMPLMSK   (1 << 0)

/* Device OUT Endpoint Common Interrupt Mask Register (USB_DOEPMSK) */
/* Bits 31:14 - Reserved */
#define USB_DOEPMSK_NAKMSK	   (1 << 13)
#define USB_DOEPMSK_BBLEERRMSK	   (1 << 12)
/* Bits 11:9 - Reserved */
#define USB_DOEPMSK_OUTPKTERRMSK   (1 << 8)
/* Bit 7 - Reserved */
#define USB_DOEPMSK_BACK2BACKSETUP (1 << 6)
#define USB_DOEPMSK_STSPHSERCVDMSK (1 << 5)
#define USB_DOEPMSK_OUTTKNEPDISMSK (1 << 4)
#define USB_DOEPMSK_SETUPMSK	   (1 << 3)
#define USB_DOEPMSK_AHBERRMSK	   (1 << 2)
#define USB_DOEPMSK_EPDISBLDMSK    (1 << 1)
#define USB_DOEPMSK_XFERCOMPLMSK   (1 << 0)

/* Device Control IN Endpoint 0 Control Register (USB_DIEP0CTL) */
#define USB_DIEP0CTL_EPENA	   (1 << 31)
#define USB_DIEP0CTL_EPDIS	   (1 << 30)
#define USB_DIEP0CTL_SETD1PIDOF    (1 << 29)
#define USB_DIEP0CTL_SETD0PIDEF    (1 << 28)
#define USB_DIEP0CTL_SNAK	   (1 << 27)
#define USB_DIEP0CTL_CNAK	   (1 << 26)
#define USB_DIEP0CTL_TXFNUM_MASK   (0xf << 22)
#define USB_DIEP0CTL_STALL	   (1 << 21)
/* Bit 20 - Reserved */
#define USB_DIEP0CTL_EPTYPE_MASK   (0x3 << 18)
#define USB_DIEP0CTL_NAKSTS	   (1 << 17)
/* Bit 16 - Reserved */
#define USB_DIEP0CTL_USBACTEP	   (1 << 15)
/* Bits 14:2 - Reserved */
#define USB_DIEP0CTL_MPS_MASK	   (0x3 << 0)
#define USB_DIEP0CTL_MPSIZ_64	   (0x0 << 0)
#define USB_DIEP0CTL_MPSIZ_32	   (0x1 << 0)
#define USB_DIEP0CTL_MPSIZ_16	   (0x2 << 0)
#define USB_DIEP0CTL_MPSIZ_8	   (0x3 << 0)

/* Device Control OUT Endpoint 0 Control Register (USB_DOEP0CTL) */
#define USB_DOEP0CTL_EPENA	   (1 << 31)
#define USB_DOEP0CTL_EPDIS	   (1 << 30)
#define USB_DOEP0CTL_SETD1PIDOF    (1 << 29)
#define USB_DOEP0CTL_SETD0PIDEF    (1 << 28)
#define USB_DOEP0CTL_SNAK	   (1 << 27)
#define USB_DOEP0CTL_CNAK	   (1 << 26)
/* Bits 25:22 - Reserved */
#define USB_DOEP0CTL_STALL	   (1 << 21)
#define USB_DOEP0CTL_SNP	   (1 << 20)
#define USB_DOEP0CTL_EPTYPE_MASK   (0x3 << 18)
#define USB_DOEP0CTL_NAKSTS	   (1 << 17)
/* Bit 16 - Reserved */
#define USB_DOEP0CTL_USBACTEP	   (1 << 15)
/* Bits 14:2 - Reserved */
#define USB_DOEP0CTL_MPS_MASK	   (0x3 << 0)
#define USB_DOEP0CTL_MPSIZ_64	   (0x0 << 0)
#define USB_DOEP0CTL_MPSIZ_32	   (0x1 << 0)
#define USB_DOEP0CTL_MPSIZ_16	   (0x2 << 0)
#define USB_DOEP0CTL_MPSIZ_8	   (0x3 << 0)

/* Device IN Endpoint Interrupt Register (USB_DIEPINTx) */
/* Bits 31:14 - Reserved */
#define USB_DIEP_INT_NAKINTRPT	   (1 << 13)
#define USB_DIEP_INT_BBLEERR	   (1 << 12)
#define USB_DIEP_INT_PKTDRPSTS	   (1 << 11)
/* Bits 10:8 - Reserved */
#define USB_DIEP_INT_TXFEMP	   (1 << 7)
#define USB_DIEP_INT_INEPNAKEFF    (1 << 6)
/* Bit 5 - Reserved */
#define USB_DIEP_INT_INTKNTXFEMP   (1 << 4)
#define USB_DIEP_INT_TIMEOUT	   (1 << 3)
#define USB_DIEP_INT_AHBERR	   (1 << 2)
#define USB_DIEP_INT_EPDISBLD	   (1 << 1)
#define USB_DIEP_INT_XFERCOMPL	   (1 << 0)

/* Device OUT Endpoint Interrupt Register (USB_DOEPINTx) */
/* Bits 31:16 - Reserved */
#define USB_DOEP_INT_STUPPKTRCVD	(1 << 15)
/* Bit 14 - Reserved */
#define USB_DOEP_INT_NAKINTRPT		(1 << 13)
#define USB_DOEP_INT_BBLEERR		(1 << 12)
#define USB_DOEP_INT_PKTDRPSTS		(1 << 11)
/* Bits 10:7 - Reserved */
#define USB_DOEP_INT_BACK2BACKSETUP	(1 << 6)
#define USB_DOEP_INT_STSPHSERCVD	(1 << 5)
#define USB_DOEP_INT_OUTTKNEPDIS	(1 << 4)
#define USB_DOEP_INT_SETUP		(1 << 3)
#define USB_DOEP_INT_AHBERR		(1 << 2)
#define USB_DOEP_INT_EPDISBLD		(1 << 1)
#define USB_DOEP_INT_XFERCOMPL		(1 << 0)

/* Device OUT Endpoint 0 Transfer Size Register (USB_DOEP0TSIZ) */
/* Bit 31 - Reserved */
#define USB_DIEP0TSIZ_SUPCNT_MASK	(0x3 << 29)
#define USB_DIEP0TSIZ_STUPCNT_1		(0x1 << 29)
#define USB_DIEP0TSIZ_STUPCNT_2		(0x2 << 29)
#define USB_DIEP0TSIZ_STUPCNT_3		(0x3 << 29)
/* Bits 28:20 - Reserved */
#define USB_DIEP0TSIZ_PKTCNT		(1 << 19)
/* Bits 18:7 - Reserved */
#define USB_DIEP0TSIZ_XFERSIZE_MASK	(0x7f << 0)

/* Device All Endpoints Interrupt Mask Register (USB_DAINTMSK) */
/* Bits 31:20 - Reserved */
#define USB_DAINTMSK_OUTEPMSK3		(1 << 19)
#define USB_DAINTMSK_OUTEPMSK2		(1 << 18)
#define USB_DAINTMSK_OUTEPMSK1		(1 << 17)
#define USB_DAINTMSK_OUTEPMSK0		(1 << 16)
/* Bits 15:4 - Reserved */
#define USB_DAINTMSK_INEPMSK3		(1 << 3)
#define USB_DAINTMSK_INEPMSK2		(1 << 2)
#define USB_DAINTMSK_INEPMSK1		(1 << 1)
#define USB_DAINTMSK_INEPMSK0		(1 << 0)

#endif

