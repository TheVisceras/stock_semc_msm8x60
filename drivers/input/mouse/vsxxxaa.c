/*
 * Driver for	DEC VSXXX-AA mouse (hockey-puck mouse, ball or two rollers)
 *		DEC VSXXX-GA mouse (rectangular mouse, with ball)
 *		DEC VSXXX-AB tablet (digitizer with hair cross or stylus)
 *
 * Copyright (C) 2003-2004 by Jan-Benedict Glaw <jbglaw@lug-owl.de>
 *
 * The packet format was initially taken from a patch to GPM which is (C) 2001
 * by	Karsten Merker <merker@linuxtag.org>
 * and	Maciej W. Rozycki <macro@ds2.pg.gda.pl>
 * Later on, I had access to the device's documentation (referenced below).
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*
 * Building an adaptor to DE9 / DB25 RS232
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * DISCLAIMER: Use this description AT YOUR OWN RISK! I'll not pay for
 * anything if you break your mouse, your computer or whatever!
 *
 * In theory, this mouse is a simple RS232 device. In practice, it has got
 * a quite uncommon plug and the requirement to additionally get a power
 * supply at +5V and -12V.
 *
 * If you look at the socket/jack (_not_ at the plug), we use this pin
 * numbering:
 *    _______
 *   / 7 6 5 \
 *  | 4 --- 3 |
 *   \  2 1  /
 *    -------
 *
 *	DEC socket	DE9	DB25	Note
 *	1 (GND)		5	7	-
 *	2 (RxD)		2	3	-
 *	3 (TxD)		3	2	-
 *	4 (-12V)	-	-	Somewhere from the PSU. At ATX, it's
 *					the thin blue wire at pin 12 of the
 *					ATX power connector. Only required for
 *					VSXXX-AA/-GA mice.
 *	5 (+5V)		-	-	PSU (red wires of ATX power connector
 *					on pin 4, 6, 19 or 20) or HDD power
 *					connector (also red wire).
 *	6 (+12V)	-	-	HDD power connector, yellow wire. Only
 *					required for VSXXX-AB digitizer.
 *	7 (dev. avail.)	-	-	The mouse shorts this one to pin 1.
 *					This way, the host computer can detect
 *					the mouse. To use it with the adaptor,
 *					simply don't connect this pin.
 *
 * So to get a working adaptor, you need to connect the mouse with three
 * wires to a RS232 port and two or three additional wires for +5V, +12V and
 * -12V to the PSU.
 *
 * Flow specification for the link is 4800, 8o1.
 *
 * The mice and tablet are described in "VCB02 Video Subsystem - Technical
 * Manual", DEC EK-104AA-TM-001. You'll find it at MANX, a search engine
 * specific for DEC documentation. Try
 * http://www.vt100.net/manx/details?pn=EK-104AA-TM-001;id=21;cp=1
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>

#define DRIVER_DESC "Driver for DEC VSXXX-AA and -GA mice and VSXXX-AB tablet"

MODULE_AUTHOR("Jan-Benedict Glaw <jbglaw@lug-owl.de>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#undef VSXXXAA_DEBUG
#ifdef VSXXXAA_DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) do {} while (0)
#endif

#define VSXXXAA_INTRO_MASK	0x80
#define VSXXXAA_INTRO_HEAD	0x80
#define IS_HDR_BYTE(x)			\
	(((x) & VSXXXAA_INTRO_MASK) == VSXXXAA_INTRO_HEAD)

#define VSXXXAA_PACKET_MASK	0xe0
#define VSXXXAA_PACKET_REL	0x80
#define VSXXXAA_PACKET_ABS	0xc0
#define VSXXXAA_PACKET_POR	0xa0
#define MATCH_PACKET_TYPE(data, type)	\
	(((data) & VSXXXAA_PACKET_MASK) == (type))



struct vsxxxaa {
	struct input_dev *dev;
	struct serio *serio;
#define BUFLEN 15 /* At least 5 is needed for a full tablet packet */
	unsigned char buf[BUFLEN];
	unsigned char count;
	unsigned char version;
	unsigned char country;
	unsigned char type;
	char name[64];
	char phys[32];
};

static void vsxxxaa_drop_bytes(struct vsxxxaa *mouse, int num)
{
	if (num >= mouse->count) {
		mouse->count = 0;
	} else {
		memmove(mouse->buf, mouse->buf + num - 1, BUFLEN - num);
		mouse->count -= num;
	}
}

static void vsxxxaa_queue_byte(struct vsxxxaa *mouse, unsigned char byte)
{
	if (mouse->count == BUFLEN) {
		printk(KERN_ERR "%s on %s: Dropping a byte of full buffer.\n",
			mouse->name, mouse->phys);
		vsxxxaa_drop_bytes(mouse, 1);
	}

	DBG(KERN_INFO "Queueing byte 0x%02x\n", byte);

	mouse->buf[mouse->count++] = byte;
}

static void vsxxxaa_detection_done(struct vsxxxaa *mouse)
{
	switch (mouse->type) {
	case 0x02:
		strlcpy(mouse->name, "DEC VSXXX-AA/-GA mouse",
			sizeof(mouse->name));
		break;

	case 0x04:
		strlcpy(mouse->name, "DEC VSXXX-AB digitizer",
			sizeof(mouse->name));
		break;

	default:
		snprintf(mouse->name, sizeof(mouse->name),
			 "unknown DEC pointer device (type = 0x%02x)",
			 mouse->type);
		break;
	}

	printk(KERN_INFO
		"Found %s version 0x%02x from country 0x%02x on port %s\n",
		mouse->name, mouse->version, mouse->country, mouse->phys);
}

/*
 * Returns number of bytes to be dropped, 0 if packet is okay.
 */
static int vsxxxaa_check_packet(struct vsxxxaa *mouse, int packet_len)
{
	int i;

	/* First byte must be a header byte */
	if (!IS_HDR_BYTE(mouse->buf[0])) {
		DBG("vsck: len=%d, 1st=0x%02x\n", packet_len, mouse->buf[0]);
		return 1;
	}

	/* Check all following bytes */
	for (i = 1; i < packet_len; i++) {
		if (IS_HDR_BYTE(mouse->buf[i])) {
			printk(KERN_ERR
				"Need to drop %d bytes of a broken packet.\n",
				i - 1);
			DBG(KERN_INFO "check: len=%d, b[%d]=0x%02x\n",
			    packet_len, i, mouse->buf[i]);
			return i - 1;
		}
	}

	return 0;
}

static inline int vsxxxaa_smells_like_packet(struct vsxxxaa *mouse,
					     unsigned char type, size_t len)
{
	return mouse->count >= len && MATCH_PACKET_TYPE(mouse->buf[0], type);
}

static void vsxxxaa_handle_REL_packet(struct vsxxxaa *mouse)
{
	struct input_dev *dev = mouse->dev;
	unsigned char *buf = mouse->buf;
	int left, middle, right;
	int dx, dy;

	/*
	 * Check for normal stream packets. This is three bytes,
	 * with the first byte's 3 MSB set to 100.
	 *
	 * [0]:	1	0	0	SignX	SignY	Left	Middle	Right
	 * [1]: 0	dx	dx	dx	dx	dx	dx	dx
	 * [2]:	0	dy	dy	dy	dy	dy	dy	dy
	 */

	/*
	 * Low 7 bit of byte 1 are abs(dx), bit 7 is
	 * 0, bit 4 of byte 0 is direction.
	 */
	dx = buf[1] & 0x7f;
	dx *= ((buf[0] >> 4) & 0x01) ? 1 : -1;

	/*
	 * Low 7 bit of byte 2 are abs(dy), bit 7 is
	 * 0, bit 3 of byte 0 is direction.
	 */
	dy = buf[2] & 0x7f;
	dy *= ((buf[0] >> 3) & 0x01) ? -1 : 1;

	/*
	 * Get button state. It's the low three bits
	 * (for three buttons) of byte 0.
	 */
	left	= buf[0] & 0x04;
	middle	= buf[0] & 0x02;
	right	= buf[0] & 0x01;

	vsxxxaa_drop_bytes(mouse, 3);

	DBG(KERN_INFO "%s on %s: dx=%d, dy=%d, buttons=%s%s%s\n",
	    mouse->name, mouse->phys, dx, dy,
	    left ? "L" : "l", middle ? "M" : "m", right ? "R" : "r");

	/*
	 * Report what we've found so far...
	 */
	input_report_key(dev, BTN_LEFT, left);
	input_report_key(dev, BTN_MIDDLE, middle);
	input_report_key(dev, BTN_RIGHT, right);
	input_report_key(dev, BTN_TOUCH, 0);
	input_report_rel(dev, REL_X, dx);
	input_report_rel(dev, REL_Y, dy);
	input_sync(dev);
}

static void vsxxxaa_handle_ABS_packet(struct vsxxxaa *mouse)
{
	struct input_dev *dev = mouse->dev;
	unsigned char *buf = mouse->buf;
	int left, middle, right, touch;
	int x, y;

	/*
	 * Tablet position / button packet
	 *
	 * [0]:	1	1	0	B4	B3	B2	B1	Pr
	 * [1]:	0	0	X5	X4	X3	X2	X1	X0
	 * [2]:	0	0	X11	X10	X9	X8	X7	X6
	 * [3]:	0	0	Y5	Y4	Y3	Y2	Y1	Y0
	 * [4]:	0	0	Y11	Y10	Y9	Y8	Y7	Y6
	 */

	/*
	 * Get X/Y position. Y axis needs to be inverted since VSXXX-AB
	 * counts down->top while monitor counts top->bottom.
	 */
	x = ((buf[2] & 0x3f) << 6) | (buf[1] & 0x3f);
	y = ((buf[4] & 0x3f) << 6) | (buf[3] & 0x3f);
	y = 1023 - y;

	/*
	 * Get button state. It's bits <4..1> of byte 0.
	 */
	left	= buf[0] & 0x02;
	middle	= buf[0] & 0x04;
	right	= buf[0] & 0x08;
	touch	= buf[0] & 0x10;

	vsxxxaa_drop_bytes(mouse, 5);

	DBG(KERN_INFO "