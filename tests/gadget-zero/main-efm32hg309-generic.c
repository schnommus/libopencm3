/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>
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

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdio.h>
#include "usb-gadget0.h"

/* no trace on cm0 #define ER_DEBUG */
#ifdef ER_DEBUG
#define ER_DPRINTF(fmt, ...) \
	do { printf(fmt, ## __VA_ARGS__); } while (0)
#else
#define ER_DPRINTF(fmt, ...) \
	do { } while (0)
#endif

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7

const uint32_t ahb_frequency = 14000000;

usbd_device *usbd_dev = NULL;

#include "trace.h"
void trace_send_blocking8(int stimulus_port, char c)
{
	(void)stimulus_port;
	(void)c;
}

void usb_isr(void)
{
	//gadget0_run(usbd_dev);
	usbd_poll(usbd_dev);
	gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
}

int main(void)
{
	/* Disable the watchdog that the bootloader started. */
	WDOG_CTRL = 0;

	/* Set up both LEDs as outputs */
	cmu_periph_clock_enable(CMU_GPIO);
	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
	gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);
	gpio_set(LED_RED_PORT, LED_RED_PIN);

	usbd_dev = gadget0_init(&efm32hg_usb_driver,
			"efm32hg309-generic");

	/* Enable USB IRQs */
	nvic_enable_irq(NVIC_USB_IRQ);

	ER_DPRINTF("bootup complete\n");
	gpio_clear(LED_RED_PORT, LED_RED_PIN);

	while (1) {
		/* usb_isr will execute gadget0_run */
	}
}

