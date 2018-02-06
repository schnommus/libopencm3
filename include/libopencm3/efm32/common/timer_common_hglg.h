/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
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

/** @cond */
#if defined(LIBOPENCM3_TIMER_H)
/** @endcond */
#ifndef LIBOPENCM3_EFM32_TIMER_COMMON_HGLG_H
#define LIBOPENCM3_EFM32_TIMER_COMMON_HGLG_H

#include <libopencm3/efm32/memorymap.h>
#include <libopencm3/cm3/common.h>

#define TIMER_CTRL(base)		MMIO32((base) + 0x000)
#define TIMER_CMD(base)			MMIO32((base) + 0x004)
#define TIMER_STATUS(base)		MMIO32((base) + 0x008)
#define TIMER_IEN(base)			MMIO32((base) + 0x00C)
#define TIMER_IF(base)			MMIO32((base) + 0x010)
#define TIMER_IFS(base)			MMIO32((base) + 0x014)
#define TIMER_IFC(base)			MMIO32((base) + 0x018)
#define TIMER_TOP(base)			MMIO32((base) + 0x01C)
#define TIMER_TOPB(base)		MMIO32((base) + 0x020)
#define TIMER_CNT(base)			MMIO32((base) + 0x024)
#define TIMER_ROUTE(base)		MMIO32((base) + 0x028)

#define TIMER_CCx_CTRL(base, x)		MMIO32((base) + 0x030 + (0x10 * (x)))
#define TIMER_CCx_CCV(base, x)		MMIO32((base) + 0x034 + (0x10 * (x)))
#define TIMER_CCx_CCVP(base, x)		MMIO32((base) + 0x038 + (0x10 * (x)))
#define TIMER_CCx_CCVB(base, x)		MMIO32((base) + 0x03C + (0x10 * (x)))

#define TIMER_CC0_CTRL(base)		TIMER_CCx_CTRL(base, 0)
#define TIMER_CC0_CCV(base)		TIMER_CCx_CCV(base, 0)
#define TIMER_CC0_CCVP(base)		TIMER_CCx_CCVP(base, 0)
#define TIMER_CC0_CCVB(base)		TIMER_CCx_CCVB(base, 0)

#define TIMER_CC1_CTRL(base)		TIMER_CCx_CTRL(base, 1)
#define TIMER_CC1_CCV(base)		TIMER_CCx_CCV(base, 1)
#define TIMER_CC1_CCVP(base)		TIMER_CCx_CCVP(base, 1)
#define TIMER_CC1_CCVB(base)		TIMER_CCx_CCVB(base, 1)

#define TIMER_CC2_CTRL(base)		TIMER_CCx_CTRL(base, 2)
#define TIMER_CC2_CCV(base)		TIMER_CCx_CCV(base, 2)
#define TIMER_CC2_CCVP(base)		TIMER_CCx_CCVP(base, 2)
#define TIMER_CC2_CCVB(base)		TIMER_CCx_CCVB(base, 2)

#define TIMER_DTCTRL(base)		MMIO32((base) + 0x070)
#define TIMER_DTTIME(base)		MMIO32((base) + 0x074)
#define TIMER_DTFC(base)		MMIO32((base) + 0x078)
#define TIMER_DTOGEN(base)		MMIO32((base) + 0x07C)
#define TIMER_DTFAULT(base)		MMIO32((base) + 0x080)
#define TIMER_DTFAULTC(base)		MMIO32((base) + 0x084)
#define TIMER_DTLOCK(base)		MMIO32((base) + 0x088)

/* TIMER_CTRL */
#define TIMER_CTRL_RSSCOIST		(1 << 29)
#define TIMER_CTRL_ATI			(1 << 28)

#define TIMER_CTRL_PRESC_SHIFT		(24)
#define TIMER_CTRL_PRESC_MASK		(0xF << TIMER_CTRL_PRESC_SHIFT)
#define TIMER_CTRL_PRESC(v)			\
	(((v) << TIMER_CTRL_PRESC_SHIFT) & TIMER_CTRL_PRESC_MASK)
#define TIMER_CTRL_PRESC_DIV1		TIMER_CTRL_PRESC(0)
#define TIMER_CTRL_PRESC_DIV2		TIMER_CTRL_PRESC(1)
#define TIMER_CTRL_PRESC_DIV4		TIMER_CTRL_PRESC(2)
#define TIMER_CTRL_PRESC_DIV8		TIMER_CTRL_PRESC(3)
#define TIMER_CTRL_PRESC_DIV16		TIMER_CTRL_PRESC(4)
#define TIMER_CTRL_PRESC_DIV32		TIMER_CTRL_PRESC(5)
#define TIMER_CTRL_PRESC_DIV64		TIMER_CTRL_PRESC(6)
#define TIMER_CTRL_PRESC_DIV128		TIMER_CTRL_PRESC(7)
#define TIMER_CTRL_PRESC_DIV256		TIMER_CTRL_PRESC(8)
#define TIMER_CTRL_PRESC_DIV512		TIMER_CTRL_PRESC(9)
#define TIMER_CTRL_PRESC_DIV1024	TIMER_CTRL_PRESC(10)
#define TIMER_CTRL_PRESC_NODIV		TIMER_CTRL_PRESC_DIV1

#define TIMER_CTRL_CLKSEL_SHIFT		(16)
#define TIMER_CTRL_CLKSEL_MASK		(0x3 << TIMER_CTRL_CLKSEL_SHIFT)
#define TIMER_CTRL_CLKSEL(v)		\
	(((v) << TIMER_CTRL_CLKSEL_SHIFT) & TIMER_CTRL_CLKSEL_MASK)
#define TIMER_CTRL_CLKSEL_PRESCHFPERCLK	TIMER_CTRL_CLKSEL(0)
#define TIMER_CTRL_CLKSEL_CC1		TIMER_CTRL_CLKSEL(1)
#define TIMER_CTRL_CLKSEL_TIMEROUF	TIMER_CTRL_CLKSEL(2)

#define TIMER_CTRL_X2CNT		(1 << 13)

#define TIMER_CTRL_FALLA_SHIFT		(10)
#define TIMER_CTRL_FALLA_MASK		(0x3 << TIMER_CTRL_FALLA_SHIFT)
#define TIMER_CTRL_FALLA(v)		\
	(((v) << TIMER_CTRL_FALLA_SHIFT) & TIMER_CTRL_FALLA_MASK)
#define TIMER_CTRL_FALLA_NONE		TIMER_CTRL_FALLA(0)
#define TIMER_CTRL_FALLA_START		TIMER_CTRL_FALLA(1)
#define TIMER_CTRL_FALLA_STOP		TIMER_CTRL_FALLA(2)
#define TIMER_CTRL_FALLA_RELOADSTART	TIMER_CTRL_FALLA(3)

#define TIMER_CTRL_RISEA_SHIFT		(8)
#define TIMER_CTRL_RISEA_MASK		(0x3 << TIMER_CTRL_RISEA_SHIFT)
#define TIMER_CTRL_RISEA(v)		\
	(((v) << TIMER_CTRL_RISEA_SHIFT) & TIMER_CTRL_RISEA_MASK)
#define TIMER_CTRL_RISEA_NONE		TIMER_CTRL_RISEA(0)
#define TIMER_CTRL_RISEA_START		TIMER_CTRL_RISEA(1)
#define TIMER_CTRL_RISEA_STOP		TIMER_CTRL_RISEA(2)
#define TIMER_CTRL_RISEA_RELOADSTART	TIMER_CTRL_RISEA(3)

/* TIMER_CTRL_DMACLRACT bit is strangely documented.
 * set this bit,
 * in case you are doing one DMA transfer on every timer trigger event.
 * if this bit is not set, strange behaviour is seen.
*/

#define TIMER_CTRL_DMACLRACT		(1 << 7)
#define TIMER_CTRL_DEBUGRUN		(1 << 6)
#define TIMER_CTRL_QDM			(1 << 5)
#define TIMER_CTRL_OSMEN		(1 << 4)
#define TIMER_CTRL_SYNC			(1 << 3)

#define TIMER_CTRL_MODE_SHIFT		(0)
#define TIMER_CTRL_MODE_MASK		(0x3 << TIMER_CTRL_MODE_SHIFT)
#define TIMER_CTRL_MODE(v)		\
	(((v) << TIMER_CTRL_MODE_SHIFT) & TIMER_CTRL_MODE_MASK)
#define TIMER_CTRL_MODE_UP		TIMER_CTRL_MODE(0)
#define TIMER_CTRL_MODE_DOWN		TIMER_CTRL_MODE(1)
#define TIMER_CTRL_MODE_UPDOWN		TIMER_CTRL_MODE(2)
#define TIMER_CTRL_MODE_QDEC		TIMER_CTRL_MODE(3)

/* TIMER_CMD */
#define TIMER_CMD_STOP			(1 << 1)
#define TIMER_CMD_START			(1 << 0)

/* TIMER_STATUS */
#define TIMER_STATUS_CCPOLx(x)		(1 << ((x) + 24))
#define TIMER_STATUS_CCPOL2		TIMER_STATUS_CCPOLx(2)
#define TIMER_STATUS_CCPOL1		TIMER_STATUS_CCPOLx(1)
#define TIMER_STATUS_CCPOL0		TIMER_STATUS_CCPOLx(0)

#define TIMER_STATUS_ICVx(x)		(1 << ((x) + 16))
#define TIMER_STATUS_ICV2		TIMER_STATUS_ICVx(2)
#define TIMER_STATUS_ICV1		TIMER_STATUS_ICVx(1)
#define TIMER_STATUS_ICV0		TIMER_STATUS_ICVx(0)

#define TIMER_STATUS_CCVBVx(x)		(1 << ((x) + 8))
#define TIMER_STATUS_CCVBV2		TIMER_STATUS_CCVBVx(2)
#define TIMER_STATUS_CCVBV1		TIMER_STATUS_CCVBVx(1)
#define TIMER_STATUS_CCVBV0		TIMER_STATUS_CCVBVx(0)

#define TIMER_STATUS_TOPBV		(1 << 2)
#define TIMER_STATUS_DIR		(1 << 1)
#define TIMER_STATUS_RUNNING		(1 << 0)

/* TIMER_IEN */
#define TIMER_IEN_ICBOFx(x)		(1 << ((x) + 8))
#define TIMER_IEN_ICBOF0		TIMER_IEN_ICBOFx(0)
#define TIMER_IEN_ICBOF1		TIMER_IEN_ICBOFx(1)
#define TIMER_IEN_ICBOF2		TIMER_IEN_ICBOFx(2)

#define TIMER_IEN_CCx(x)		(1 << ((x) + 4))
#define TIMER_IEN_CC0			TIMER_IEN_CCx(0)
#define TIMER_IEN_CC1			TIMER_IEN_CCx(1)
#define TIMER_IEN_CC2			TIMER_IEN_CCx(2)

#define TIMER_IEN_UF			(1 << 1)
#define TIMER_IEN_OF			(1 << 0)


/* TIMER_IF */
#define TIMER_IF_ICBOFx(x)		(1 << ((x) + 8))
#define TIMER_IF_ICBOF0			TIMER_IF_ICBOFx(0)
#define TIMER_IF_ICBOF1			TIMER_IF_ICBOFx(1)
#define TIMER_IF_ICBOF2			TIMER_IF_ICBOFx(2)

#define TIMER_IF_CCx(x)			(1 << ((x) + 4))
#define TIMER_IF_CC0			TIMER_IF_CCx(0)
#define TIMER_IF_CC1			TIMER_IF_CCx(1)
#define TIMER_IF_CC2			TIMER_IF_CCx(2)

#define TIMER_IF_UF			(1 << 1)
#define TIMER_IF_OF			(1 << 0)

/* TIMER_IFS */
#define TIMER_IFS_ICBOFx(x)		(1 << ((x) + 8))
#define TIMER_IFS_ICBOF0		TIMER_IFS_ICBOFx(0)
#define TIMER_IFS_ICBOF1		TIMER_IFS_ICBOFx(1)
#define TIMER_IFS_ICBOF2		TIMER_IFS_ICBOFx(2)

#define TIMER_IFS_CCx(x)		(1 << ((x) + 4))
#define TIMER_IFS_CC0			TIMER_IFS_CCx(0)
#define TIMER_IFS_CC1			TIMER_IFS_CCx(1)
#define TIMER_IFS_CC2			TIMER_IFS_CCx(2)

#define TIMER_IFS_UF			(1 << 1)
#define TIMER_IFS_OF			(1 << 0)


/* TIMER_IFC */
#define TIMER_IFC_ICBOFx(x)		(1 << ((x) + 8))
#define TIMER_IFC_ICBOF0		TIMER_IFC_ICBOFx(0)
#define TIMER_IFC_ICBOF1		TIMER_IFC_ICBOFx(1)
#define TIMER_IFC_ICBOF2		TIMER_IFC_ICBOFx(2)

#define TIMER_IFC_CCx(x)		(1 << ((x) + 4))
#define TIMER_IFC_CC0			TIMER_IFC_CCx(0)
#define TIMER_IFC_CC1			TIMER_IFC_CCx(1)
#define TIMER_IFC_CC2			TIMER_IFC_CCx(2)

#define TIMER_IFC_UF			(1 << 1)
#define TIMER_IFC_OF			(1 << 0)

/* TIMER_ROUTE */
#define TIMER_ROUTE_LOCATION_SHIFT	(16)
#define TIMER_ROUTE_LOCATION_MASK	(0x7 << TIMER_ROUTE_LOCATION_SHIFT)
#define TIMER_ROUTE_LOCATION(v)		\
	(((v) << TIMER_ROUTE_LOCATION_SHIFT) & TIMER_ROUTE_LOCATION_MASK)
#define TIMER_ROUTE_LOCATION_LOCx(x)	TIMER_ROUTE_LOCATION(x)
#define TIMER_ROUTE_LOCATION_LOC0	TIMER_ROUTE_LOCATION_LOCx(0)
#define TIMER_ROUTE_LOCATION_LOC1	TIMER_ROUTE_LOCATION_LOCx(1)
#define TIMER_ROUTE_LOCATION_LOC2	TIMER_ROUTE_LOCATION_LOCx(2)
#define TIMER_ROUTE_LOCATION_LOC3	TIMER_ROUTE_LOCATION_LOCx(3)
#define TIMER_ROUTE_LOCATION_LOC4	TIMER_ROUTE_LOCATION_LOCx(4)
#define TIMER_ROUTE_LOCATION_LOC5	TIMER_ROUTE_LOCATION_LOCx(5)

#define TIMER_ROUTE_CDTIxPEN(x)		(1 << (8 + (x)))
#define TIMER_ROUTE_CDTI0PEN		TIMER_ROUTE_CDTIxPEN(0)
#define TIMER_ROUTE_CDTI1PEN		TIMER_ROUTE_CDTIxPEN(1)
#define TIMER_ROUTE_CDTI2PEN		TIMER_ROUTE_CDTIxPEN(2)

#define TIMER_ROUTE_CCxPEN(x)		(1 << (0 + (x)))
#define TIMER_ROUTE_CC0PEN		TIMER_ROUTE_CCxPEN(0)
#define TIMER_ROUTE_CC1PEN		TIMER_ROUTE_CCxPEN(1)
#define TIMER_ROUTE_CC2PEN		TIMER_ROUTE_CCxPEN(2)

/* TIMER_CCx_CTRL */
#define TIMER_CC_CTRL_ICEVCTRL_SHIFT	(26)
#define TIMER_CC_CTRL_ICEVCTRL_MASK	(0x3 << TIMER_CC_CTRL_ICEVCTRL_SHIFT)
#define TIMER_CC_CTRL_ICEVCTRL(v)	\
	(((v) << TIMER_CC_CTRL_ICEVCTRL_SHIFT) & TIMER_CC_CTRL_ICEVCTRL_MASK)
#define TIMER_CC_CTRL_ICEVCTRL_EVERYEDGE	TIMER_CC_CTRL_ICEVCTRL(0)
#define TIMER_CC_CTRL_ICEVCTRL_EVERYSECONDEDGE	TIMER_CC_CTRL_ICEVCTRL(1)
#define TIMER_CC_CTRL_ICEVCTRL_RISING		TIMER_CC_CTRL_ICEVCTRL(2)
#define TIMER_CC_CTRL_ICEVCTRL_FALLING		TIMER_CC_CTRL_ICEVCTRL(3)

#define TIMER_CC_CTRL_ICEVCTRL_EVERY_EDGE	\
	TIMER_CC_CTRL_ICEVCTRL_EVERYEDGE
#define TIMER_CC_CTRL_ICEVCTRL_EVERY_SECOND_EDGE	\
	TIMER_CC_CTRL_ICEVCTRL_EVERYSECONDEDGE

#define TIMER_CC_CTRL_ICEDGE_SHIFT	(24)
#define TIMER_CC_CTRL_ICEDGE_MASK	(0x3 << TIMER_CC_CTRL_ICEDGE_SHIFT)
#define TIMER_CC_CTRL_ICEDGE(v)		\
	(((v) << TIMER_CC_CTRL_ICEDGE_SHIFT) & TIMER_CC_CTRL_ICEDGE_MASK)
#define TIMER_CC_CTRL_ICEDGE_RISING	TIMER_CC_CTRL_ICEDGE(0)
#define TIMER_CC_CTRL_ICEDGE_FALLING	TIMER_CC_CTRL_ICEDGE(1)
#define TIMER_CC_CTRL_ICEDGE_BOTH	TIMER_CC_CTRL_ICEDGE(2)
#define TIMER_CC_CTRL_ICEDGE_NONE	TIMER_CC_CTRL_ICEDGE(3)

#define TIMER_CC_CTRL_FILT		(1 << 21)
#define TIMER_CC_CTRL_INSEL		(1 << 21)


#define TIMER_CC_CTRL_PRSSEL_SHIFT	(16)
#define TIMER_CC_CTRL_PRSSEL_MASK	(0xF << TIMER_CC_CTRL_PRSSEL_SHIFT)
#define TIMER_CC_CTRL_PRSSEL(v)		\
	(((v) << TIMER_CC_CTRL_PRSSEL_SHIFT) & TIMER_CC_CTRL_PRSSEL_MASK)
#define TIMER_CC_CTRL_PRSSEL_PRSCHx(x)	TIMER_CC_CTRL_PRSSEL(x)
#define TIMER_CC_CTRL_PRSSEL_PRSCH0	TIMER_CC_CTRL_PRSSEL_PRSCHx(0)
#define TIMER_CC_CTRL_PRSSEL_PRSCH1	TIMER_CC_CTRL_PRSSEL_PRSCHx(1)
#define TIMER_CC_CTRL_PRSSEL_PRSCH2	TIMER_CC_CTRL_PRSSEL_PRSCHx(2)
#define TIMER_CC_CTRL_PRSSEL_PRSCH3	TIMER_CC_CTRL_PRSSEL_PRSCHx(3)
#define TIMER_CC_CTRL_PRSSEL_PRSCH4	TIMER_CC_CTRL_PRSSEL_PRSCHx(4)
#define TIMER_CC_CTRL_PRSSEL_PRSCH5	TIMER_CC_CTRL_PRSSEL_PRSCHx(5)
#define TIMER_CC_CTRL_PRSSEL_PRSCH6	TIMER_CC_CTRL_PRSSEL_PRSCHx(6)
#define TIMER_CC_CTRL_PRSSEL_PRSCH7	TIMER_CC_CTRL_PRSSEL_PRSCHx(7)
#define TIMER_CC_CTRL_PRSSEL_PRSCH8	TIMER_CC_CTRL_PRSSEL_PRSCHx(8)
#define TIMER_CC_CTRL_PRSSEL_PRSCH9	TIMER_CC_CTRL_PRSSEL_PRSCHx(9)
#define TIMER_CC_CTRL_PRSSEL_PRSCH10	TIMER_CC_CTRL_PRSSEL_PRSCHx(10)
#define TIMER_CC_CTRL_PRSSEL_PRSCH11	TIMER_CC_CTRL_PRSSEL_PRSCHx(11)

#define TIMER_CC_CTRL_CUFOA_SHIFT	(12)
#define TIMER_CC_CTRL_CUFOA_MASK	(0x3 << TIMER_CC_CTRL_CUFOA_SHIFT)
#define TIMER_CC_CTRL_CUFOA(v)		\
	(((v) << TIMER_CC_CTRL_CUFOA_SHIFT) & TIMER_CC_CTRL_CUFOA_MASK)
#define TIMER_CC_CTRL_CUFOA_NONE	TIMER_CC_CTRL_CUFOA(0)
#define TIMER_CC_CTRL_CUFOA_TOGGLE	TIMER_CC_CTRL_CUFOA(1)
#define TIMER_CC_CTRL_CUFOA_CLEAR	TIMER_CC_CTRL_CUFOA(2)
#define TIMER_CC_CTRL_CUFOA_SET		TIMER_CC_CTRL_CUFOA(3)

#define TIMER_CC_CTRL_COFOA_SHIFT	(10)
#define TIMER_CC_CTRL_COFOA_MASK	(0x3 << TIMER_CC_CTRL_COFOA_SHIFT)
#define TIMER_CC_CTRL_COFOA(v)		\
	(((v) << TIMER_CC_CTRL_COFOA_SHIFT) & TIMER_CC_CTRL_COFOA_MASK)
#define TIMER_CC_CTRL_COFOA_NONE	TIMER_CC_CTRL_COFOA(0)
#define TIMER_CC_CTRL_COFOA_TOGGLE	TIMER_CC_CTRL_COFOA(1)
#define TIMER_CC_CTRL_COFOA_CLEAR	TIMER_CC_CTRL_COFOA(2)
#define TIMER_CC_CTRL_COFOA_SET		TIMER_CC_CTRL_COFOA(3)

#define TIMER_CC_CTRL_CMOA_SHIFT	(8)
#define TIMER_CC_CTRL_CMOA_MASK		(0x3 << TIMER_CC_CTRL_CMOA_SHIFT)
#define TIMER_CC_CTRL_CMOA(v)		\
	(((v) << TIMER_CC_CTRL_CMOA_SHIFT) & TIMER_CC_CTRL_CMOA_MASK)
#define TIMER_CC_CTRL_CMOA_NONE		TIMER_CC_CTRL_CMOA(0)
#define TIMER_CC_CTRL_CMOA_TOGGLE	TIMER_CC_CTRL_CMOA(1)
#define TIMER_CC_CTRL_CMOA_CLEAR	TIMER_CC_CTRL_CMOA(2)
#define TIMER_CC_CTRL_CMOA_SET		TIMER_CC_CTRL_CMOA(3)

#define TIMER_CC_CTRL_COIST		(1 << 4)
#define TIMER_CC_CTRL_OUTINV		(1 << 2)

#define TIMER_CC_CTRL_MODE_SHIFT	(0)
#define TIMER_CC_CTRL_MODE_MASK		(0x3 << TIMER_CC_CTRL_MODE_SHIFT)
#define TIMER_CC_CTRL_MODE(v)		\
	(((v) << TIMER_CC_CTRL_MODE_SHIFT) & TIMER_CC_CTRL_MODE_MASK)
#define TIMER_CC_CTRL_MODE_OFF		TIMER_CC_CTRL_MODE(0)
#define TIMER_CC_CTRL_MODE_INPUTCAPTURE		TIMER_CC_CTRL_MODE(1)
#define TIMER_CC_CTRL_MODE_OUTPUTCOMPARE	TIMER_CC_CTRL_MODE(2)
#define TIMER_CC_CTRL_MODE_PWM		TIMER_CC_CTRL_MODE(3)

#define TIMER_CC_CTRL_MODE_INPUT_CAPTURE	\
	TIMER_CC_CTRL_MODE_INPUTCAPTURE
#define TIMER_CC_CTRL_MODE_OUTPUT_CAPTURE	\
	TIMER_CC_CTRL_MODE_OUTPUTCAPTURE

/* TIMER_DTCTRL */
#define TIMER_DTCTRL_DTPRSEN		(1 << 24)

#define TIMER_DTCTRL_DTPRSSEL_SHIFT	(4)
#define TIMER_DTCTRL_DTPRSSEL_MASK	(0xF << TIMER_DTCTRL_DTPRSSEL_SHIFT)
#define TIMER_DTCTRL_DTPRSSEL(v)	\
	(((v) << TIMER_DTCTRL_DTPRSSEL_SHIFT) & TIMER_DTCTRL_DTPRSSEL_MASK)
#define TIMER_DTCTRL_DTPRSSEL_PRSCHx(x)	TIMER_DTCTRL_DTPRSSEL(x)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH0	TIMER_DTCTRL_DTPRSSEL_PRSCHx(0)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH1	TIMER_DTCTRL_DTPRSSEL_PRSCHx(1)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH2	TIMER_DTCTRL_DTPRSSEL_PRSCHx(2)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH3	TIMER_DTCTRL_DTPRSSEL_PRSCHx(3)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH4	TIMER_DTCTRL_DTPRSSEL_PRSCHx(4)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH5	TIMER_DTCTRL_DTPRSSEL_PRSCHx(5)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH6	TIMER_DTCTRL_DTPRSSEL_PRSCHx(6)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH7	TIMER_DTCTRL_DTPRSSEL_PRSCHx(7)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH8	TIMER_DTCTRL_DTPRSSEL_PRSCHx(8)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH9	TIMER_DTCTRL_DTPRSSEL_PRSCHx(9)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH10	TIMER_DTCTRL_DTPRSSEL_PRSCHx(10)
#define TIMER_DTCTRL_DTPRSSEL_PRSCH11	TIMER_DTCTRL_DTPRSSEL_PRSCHx(11)

#define TIMER_DTCTRL_DTCINV		(1 << 3)
#define TIMER_DTCTRL_DTIPOL		(1 << 2)
#define TIMER_DTCTRL_DTDAS		(1 << 1)
#define TIMER_DTCTRL_DTEN		(1 << 0)

/* TIMER_DTTIME */
#define TIMER_DTTIME_DTFALLT_SHIFT	(16)
#define TIMER_DTTIME_DTFALLT_MASK	(0x3F << TIMER_DTTIME_DTFALLT_SHIFT)
#define TIMER_DTTIME_DTFALLT(v)		\
	(((v) << TIMER_DTTIME_DTFALLT_SHIFT) & TIMER_DTTIME_DTFALLT_MASK)

#define TIMER_DTTIME_DTRISET_SHIFT	(8)
#define TIMER_DTTIME_DTRISET_MASK	(0x3F << TIMER_DTTIME_DTRISET_SHIFT)
#define TIMER_DTTIME_DTRISET(v)		\
	(((v) << TIMER_DTTIME_DTRISET_SHIFT) & TIMER_DTTIME_DTRISET_MASK)


#define TIMER_DTTIME_DTPRESC_SHIFT	(0)
#define TIMER_DTTIME_DTPRESC_MASK	(0xF << TIMER_DTTIME_DTPRESC_SHIFT)
#define TIMER_DTTIME_DTPRESC(v)		\
	(((v) << TIMER_DTTIME_DTPRESC_SHIFT) & TIMER_DTTIME_DTPRESC_MASK)
#define TIMER_DTTIME_DTPRESC_DIV1	TIMER_DTTIME_DTPRESC(0)
#define TIMER_DTTIME_DTPRESC_DIV2	TIMER_DTTIME_DTPRESC(1)
#define TIMER_DTTIME_DTPRESC_DIV4	TIMER_DTTIME_DTPRESC(2)
#define TIMER_DTTIME_DTPRESC_DIV8	TIMER_DTTIME_DTPRESC(3)
#define TIMER_DTTIME_DTPRESC_DIV16	TIMER_DTTIME_DTPRESC(4)
#define TIMER_DTTIME_DTPRESC_DIV32	TIMER_DTTIME_DTPRESC(5)
#define TIMER_DTTIME_DTPRESC_DIV64	TIMER_DTTIME_DTPRESC(6)
#define TIMER_DTTIME_DTPRESC_DIV128	TIMER_DTTIME_DTPRESC(7)
#define TIMER_DTTIME_DTPRESC_DIV256	TIMER_DTTIME_DTPRESC(8)
#define TIMER_DTTIME_DTPRESC_DIV512	TIMER_DTTIME_DTPRESC(8)
#define TIMER_DTTIME_DTPRESC_DIV1024	TIMER_DTTIME_DTPRESC(10)
#define TIMER_DTTIME_DTPRESC_NODIV	TIMER_DTTIME_DTPRESC_DIV1

/* TIMER_DTFC */
#define TIMER_DTFC_DTLOCKUPFEN		(1 << 27)
#define TIMER_DTFC_DTDBGFEN		(1 << 26)
#define TIMER_DTFC_DTPRS1FEN		(1 << 25)
#define TIMER_DTFC_DTPRS0FEN		(1 << 24)

#define TIMER_DTFC_DTFA_SHIFT		(16)
#define TIMER_DTFC_DTFA_MASK		(0x3 << TIMER_DTFC_DTFA_SHIFT)
#define TIMER_DTFC_DTFA(v)		\
	(((v) << TIMER_DTFC_DTFA_SHIFT) & TIMER_DTFC_DTFA_MASK)
#define TIMER_DTFC_DTFA_NONE		TIMER_DTFC_DTFA(0)
#define TIMER_DTFC_DTFA_INACTIVE	TIMER_DTFC_DTFA(1)
#define TIMER_DTFC_DTFA_CLEAR		TIMER_DTFC_DTFA(2)
#define TIMER_DTFC_DTFA_TRISTATE	TIMER_DTFC_DTFA(3)

#define TIMER_DTFC_DTPRS1FSEL_SHIFT	(8)
#define TIMER_DTFC_DTPRS1FSEL_MASK	(0x3 << TIMER_DTFC_DTPRS1FSEL_SHIFT)
#define TIMER_DTFC_DTPRS1FSEL(v)	\
	(((v) << TIMER_DTFC_DTPRS1FSEL_SHIFT) & TIMER_DTFC_DTPRS1FSEL_MASK)
#define TIMER_DTFC_DTPRS1FSEL_PRSCHx(x)	TIMER_DTFC_DTPRS1FSEL(x)
#define TIMER_DTFC_DTPRS1FSEL_PRSCH0	TIMER_DTFC_DTPRS1FSEL_PRSCHx(0)
#define TIMER_DTFC_DTPRS1FSEL_PRSCH1	TIMER_DTFC_DTPRS1FSEL_PRSCHx(1)
#define TIMER_DTFC_DTPRS1FSEL_PRSCH2	TIMER_DTFC_DTPRS1FSEL_PRSCHx(2)
#define TIMER_DTFC_DTPRS1FSEL_PRSCH3	TIMER_DTFC_DTPRS1FSEL_PRSCHx(3)
#define TIMER_DTFC_DTPRS1FSEL_PRSCH4	TIMER_DTFC_DTPRS1FSEL_PRSCHx(4)
#define TIMER_DTFC_DTPRS1FSEL_PRSCH5	TIMER_DTFC_DTPRS1FSEL_PRSCHx(5)
#define TIMER_DTFC_DTPRS1FSEL_PRSCH6	TIMER_DTFC_DTPRS1FSEL_PRSCHx(6)
#define TIMER_DTFC_DTPRS1FSEL_PRSCH7	TIMER_DTFC_DTPRS1FSEL_PRSCHx(7)

#define TIMER_DTFC_DTPRS0FSEL_SHIFT	(8)
#define TIMER_DTFC_DTPRS0FSEL_MASK	(0x3 << TIMER_DTFC_DTPRS0FSEL_SHIFT)
#define TIMER_DTFC_DTPRS0FSEL(v)	\
	(((v) << TIMER_DTFC_DTPRS0FSEL_SHIFT) & TIMER_DTFC_DTPRS0FSEL_MASK)
#define TIMER_DTFC_DTPRS0FSEL_PRSCHx(x)	TIMER_DTFC_DTPRS0FSEL(x)
#define TIMER_DTFC_DTPRS0FSEL_PRSCH0	TIMER_DTFC_DTPRS0FSEL_PRSCHx(0)
#define TIMER_DTFC_DTPRS0FSEL_PRSCH1	TIMER_DTFC_DTPRS0FSEL_PRSCHx(1)
#define TIMER_DTFC_DTPRS0FSEL_PRSCH2	TIMER_DTFC_DTPRS0FSEL_PRSCHx(2)
#define TIMER_DTFC_DTPRS0FSEL_PRSCH3	TIMER_DTFC_DTPRS0FSEL_PRSCHx(3)
#define TIMER_DTFC_DTPRS0FSEL_PRSCH4	TIMER_DTFC_DTPRS0FSEL_PRSCHx(4)
#define TIMER_DTFC_DTPRS0FSEL_PRSCH5	TIMER_DTFC_DTPRS0FSEL_PRSCHx(5)
#define TIMER_DTFC_DTPRS0FSEL_PRSCH6	TIMER_DTFC_DTPRS0FSEL_PRSCHx(6)
#define TIMER_DTFC_DTPRS0FSEL_PRSCH7	TIMER_DTFC_DTPRS0FSEL_PRSCHx(7)

/* TIMER_DTOGEN */
#define TIMER_DTOGEN_DTOGCDTI2EN	(1 << 5)
#define TIMER_DTOGEN_DTOGCDTI1EN	(1 << 4)
#define TIMER_DTOGEN_DTOGCDTI0EN	(1 << 3)
#define TIMER_DTOGEN_DTOGCC2EN		(1 << 2)
#define TIMER_DTOGEN_DTOGCC1EN		(1 << 1)
#define TIMER_DTOGEN_DTOGCC0EN		(1 << 0)

/* TIMER_DTFAULT */
#define TIMER_DTFAULT_DTLOCKUPF		(1 << 3)
#define TIMER_DTFAULT_DTDBGF		(1 << 2)
#define TIMER_DTFAULT_DTPRS1F		(1 << 1)
#define TIMER_DTFAULT_DTPRS0F		(1 << 0)

/* TIMER_DTFAULTC */
#define TIMER_DTFAULTC_TLOCKUPFC	(1 << 3)
#define TIMER_DTFAULTC_DTDBGFC		(1 << 2)
#define TIMER_DTFAULTC_DTPRS1FC		(1 << 1)
#define TIMER_DTFAULTC_DTPRS0FC		(1 << 0)

/* TIMER_DTLOCK */
#define TIMER_DTLOCK_LOCKKEY_SHIFT	(0)
#define TIMER_DTLOCK_LOCKKEY_MASK	(0xFFFF << TIMER_DTLOCK_LOCKKEY_SHIFT)
#define TIMER_DTLOCK_LOCKKEY_UNLOCKED	(0x0000 << TIMER_DTLOCK_LOCKKEY_SHIFT)
#define TIMER_DTLOCK_LOCKKEY_LOCKED	(0x0001 << TIMER_DTLOCK_LOCKKEY_SHIFT)
#define TIMER_DTLOCK_LOCKKEY_LOCK	(0x0000 << TIMER_DTLOCK_LOCKKEY_SHIFT)
#define TIMER_DTLOCK_LOCKKEY_UNLOCK	(0xCE80 << TIMER_DTLOCK_LOCKKEY_SHIFT)

/* TIMER0 */
#define TIMER0				TIMER0_BASE
#define TIMER0_CTRL			TIMER_CTRL(TIMER0)
#define TIMER0_CMD			TIMER_CMD(TIMER0)
#define TIMER0_STATUS			TIMER_STATUS(TIMER0)
#define TIMER0_IEN			TIMER_IEN(TIMER0)
#define TIMER0_IF			TIMER_IF(TIMER0)
#define TIMER0_IFS			TIMER_IFS(TIMER0)
#define TIMER0_IFC			TIMER_IFC(TIMER0)
#define TIMER0_TOP			TIMER_TOP(TIMER0)
#define TIMER0_TOPB			TIMER_TOPB(TIMER0)
#define TIMER0_CNT			TIMER_CNT(TIMER0)
#define TIMER0_ROUTE			TIMER_ROUTE(TIMER0)

#define TIMER0_CC0_CTRL			TIMER_CC0_CTRL(TIMER0)
#define TIMER0_CC0_CCV			TIMER_CC0_CCV(TIMER0)
#define TIMER0_CC0_CCVP			TIMER_CC0_CCVP(TIMER0)
#define TIMER0_CC0_CCVB			TIMER_CC0_CCVB(TIMER0)

#define TIMER0_CC1_CTRL			TIMER_CC1_CTRL(TIMER0)
#define TIMER0_CC1_CCV			TIMER_CC1_CCV(TIMER0)
#define TIMER0_CC1_CCVP			TIMER_CC1_CCVP(TIMER0)
#define TIMER0_CC1_CCVB			TIMER_CC1_CCVB(TIMER0)

#define TIMER0_CC2_CTRL			TIMER_CC2_CTRL(TIMER0)
#define TIMER0_CC2_CCV			TIMER_CC2_CCV(TIMER0)
#define TIMER0_CC2_CCVP			TIMER_CC2_CCVP(TIMER0)
#define TIMER0_CC2_CCVB			TIMER_CC2_CCVB(TIMER0)

#define TIMER0_DTCTRL			TIMER_DTCTRL(TIMER0)
#define TIMER0_DTTIME			TIMER_DTTIME(TIMER0)
#define TIMER0_DTFC			TIMER_DTFC(TIMER0)
#define TIMER0_DTOGEN			TIMER_DTOGEN(TIMER0)
#define TIMER0_DTFAULT			TIMER_DTFAULT(TIMER0)
#define TIMER0_DTFAULTC			TIMER_DTFAULTC(TIMER0)
#define TIMER0_DTLOCK			TIMER_DTLOCK(TIMER0)

/* TIMER1 */
#define TIMER1				TIMER1_BASE
#define TIMER1_CTRL			TIMER_CTRL(TIMER1)
#define TIMER1_CMD			TIMER_CMD(TIMER1)
#define TIMER1_STATUS			TIMER_STATUS(TIMER1)
#define TIMER1_IEN			TIMER_IEN(TIMER1)
#define TIMER1_IF			TIMER_IF(TIMER1)
#define TIMER1_IFS			TIMER_IFS(TIMER1)
#define TIMER1_IFC			TIMER_IFC(TIMER1)
#define TIMER1_TOP			TIMER_TOP(TIMER1)
#define TIMER1_TOPB			TIMER_TOPB(TIMER1)
#define TIMER1_CNT			TIMER_CNT(TIMER1)
#define TIMER1_ROUTE			TIMER_ROUTE(TIMER1)

#define TIMER1_CC0_CTRL			TIMER_CC0_CTRL(TIMER1)
#define TIMER1_CC0_CCV			TIMER_CC0_CCV(TIMER1)
#define TIMER1_CC0_CCVP			TIMER_CC0_CCVP(TIMER1)
#define TIMER1_CC0_CCVB			TIMER_CC0_CCVB(TIMER1)

#define TIMER1_CC1_CTRL			TIMER_CC1_CTRL(TIMER1)
#define TIMER1_CC1_CCV			TIMER_CC1_CCV(TIMER1)
#define TIMER1_CC1_CCVP			TIMER_CC1_CCVP(TIMER1)
#define TIMER1_CC1_CCVB			TIMER_CC1_CCVB(TIMER1)

#define TIMER1_CC2_CTRL			TIMER_CC2_CTRL(TIMER1)
#define TIMER1_CC2_CCV			TIMER_CC2_CCV(TIMER1)
#define TIMER1_CC2_CCVP			TIMER_CC2_CCVP(TIMER1)
#define TIMER1_CC2_CCVB			TIMER_CC2_CCVB(TIMER1)

/* TIMER2 */
#define TIMER2				TIMER2_BASE
#define TIMER2_CTRL			TIMER_CTRL(TIMER2)
#define TIMER2_CMD			TIMER_CMD(TIMER2)
#define TIMER2_STATUS			TIMER_STATUS(TIMER2)
#define TIMER2_IEN			TIMER_IEN(TIMER2)
#define TIMER2_IF			TIMER_IF(TIMER2)
#define TIMER2_IFS			TIMER_IFS(TIMER2)
#define TIMER2_IFC			TIMER_IFC(TIMER2)
#define TIMER2_TOP			TIMER_TOP(TIMER2)
#define TIMER2_TOPB			TIMER_TOPB(TIMER2)
#define TIMER2_CNT			TIMER_CNT(TIMER2)
#define TIMER2_ROUTE			TIMER_ROUTE(TIMER2)

#define TIMER2_CC0_CTRL			TIMER_CC0_CTRL(TIMER2)
#define TIMER2_CC0_CCV			TIMER_CC0_CCV(TIMER2)
#define TIMER2_CC0_CCVP			TIMER_CC0_CCVP(TIMER2)
#define TIMER2_CC0_CCVB			TIMER_CC0_CCVB(TIMER2)

#define TIMER2_CC1_CTRL			TIMER_CC1_CTRL(TIMER2)
#define TIMER2_CC1_CCV			TIMER_CC1_CCV(TIMER2)
#define TIMER2_CC1_CCVP			TIMER_CC1_CCVP(TIMER2)
#define TIMER2_CC1_CCVB			TIMER_CC1_CCVB(TIMER2)

#define TIMER2_CC2_CTRL			TIMER_CC2_CTRL(TIMER2)
#define TIMER2_CC2_CCV			TIMER_CC2_CCV(TIMER2)
#define TIMER2_CC2_CCVP			TIMER_CC2_CCVP(TIMER2)
#define TIMER2_CC2_CCVB			TIMER_CC2_CCVB(TIMER2)

/* TIMER3 */
#define TIMER3				TIMER3_BASE
#define TIMER3_CTRL			TIMER_CTRL(TIMER3)
#define TIMER3_CMD			TIMER_CMD(TIMER3)
#define TIMER3_STATUS			TIMER_STATUS(TIMER3)
#define TIMER3_IEN			TIMER_IEN(TIMER3)
#define TIMER3_IF			TIMER_IF(TIMER3)
#define TIMER3_IFS			TIMER_IFS(TIMER3)
#define TIMER3_IFC			TIMER_IFC(TIMER3)
#define TIMER3_TOP			TIMER_TOP(TIMER3)
#define TIMER3_TOPB			TIMER_TOPB(TIMER3)
#define TIMER3_CNT			TIMER_CNT(TIMER3)
#define TIMER3_ROUTE			TIMER_ROUTE(TIMER3)

#define TIMER3_CC0_CTRL			TIMER_CC0_CTRL(TIMER3)
#define TIMER3_CC0_CCV			TIMER_CC0_CCV(TIMER3)
#define TIMER3_CC0_CCVP			TIMER_CC0_CCVP(TIMER3)
#define TIMER3_CC0_CCVB			TIMER_CC0_CCVB(TIMER3)

#define TIMER3_CC1_CTRL			TIMER_CC1_CTRL(TIMER3)
#define TIMER3_CC1_CCV			TIMER_CC1_CCV(TIMER3)
#define TIMER3_CC1_CCVP			TIMER_CC1_CCVP(TIMER3)
#define TIMER3_CC1_CCVB			TIMER_CC1_CCVB(TIMER3)

#define TIMER3_CC2_CTRL			TIMER_CC2_CTRL(TIMER3)
#define TIMER3_CC2_CCV			TIMER_CC2_CCV(TIMER3)
#define TIMER3_CC2_CCVP			TIMER_CC2_CCVP(TIMER3)
#define TIMER3_CC2_CCVB			TIMER_CC2_CCVB(TIMER3)

/** @defgroup timer_ch Timer Channel Number
@ingroup timer_defines

@{*/
enum tim_ch {
	TIM_CH0 = 0,
	TIM_CH1,
	TIM_CH2
};
/**@}*/

BEGIN_DECLS

void timer_start(uint32_t timer);
void timer_stop(uint32_t timer);

void timer_set_clock_prescaler(uint32_t timer, uint32_t prescaler);

void timer_set_top(uint32_t timer, uint32_t top);

/* TODO: interrupt {enable, disable, read-flags} */

/* TODO: for channel (output value, input value)
 *  enable channel
 * set location, set output mode */

END_DECLS

#endif
/** @cond */
#else
#warning "timer_common_hglg.h should not be included explicitly, only via timer.h"
#endif
/** @endcond */
