/*
 * Copyright (C) 2011 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/netdevice.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/ip.h>

#include "modem_prj.h"
#include "modem_utils.h"

#define CMD_SUSPEND	((unsigned short)(0x00CA))
#define CMD_RESUME	((unsigned short)(0x00CB))

/* dump2hex
 * dump data to hex as fast as possible.
 * the length of @buf must be greater than "@len * 3"
 * it need 3 bytes per one data byte to print.
 */
static inline int dump2hex(char *buf, const char *data, size_t len)
{
	static const char *hex = "0123456789abcdef";
	char *dest = buf;
	int i;

	for (i = 0; i < len; i++) {
		*dest++ = hex[(data[i] >> 4) & 0xf];
		*dest++ = hex[data[i] & 0xf];
		*dest++ = ' ';
	}
	if (likely(len > 0))
		dest--; /* last space will be overwrited with null */

	*dest = '\0';

	return dest - buf;
}

/* print buffer as hex string */
int pr_buffer(const char *tag, const char *data, size_t data_len,
							size_t max_len)
{
	size_t len = min(data_len, max_len);
	unsigned char hexstr[len ? len * 3 : 1]; /* 1 <= sizeof <= max_len*3 */
	dump2hex(hexstr, data, len);

	/* don't change this printk to mif_debug for print this as level7 */
	return printk(KERN_DEBUG "%s(%u): %s%s\n", tag, data_len, hexstr,
			len == data_len ? "" : " ...");
}

/* flow control CMfrom CP, it use in serial devices */
int link_rx_flowctl_cmd(struct link_device *ld, const char *data, size_t len)
{
	struct mif_common *commons = &ld->mc->commons;
	unsigned short *cmd, *end = (unsigned short *)(data + len);

	mif_debug("flow control cmd: size=%d\n", len);

	for (cmd = (unsigned short *)data; cmd < end; cmd++) {
		switch (*cmd) {
		case CMD_SUSPEND:
			iodevs_for_each(commons, iodev_netif_stop, 0);
			ld->raw_tx_suspended = true;
			mif_info("flowctl CMD_SUSPEND(%04X)\n", *cmd);
			break;

		case CMD_RESUME:
			iodevs_for_each(commons, iodev_netif_wake, 0);
			ld->raw_tx_suspended = false;
			complete_all(&ld->raw_tx_resumed_by_cp);
			mif_info("flowctl CMD_RESUME(%04X)\n", *cmd);
			break;

		default:
			mif_err("flowctl BACMD: %04X\n", *cmd);
			break;
		}
	}

	return 0;
}

struct io_device *get_iod_with_channel(struct mif_common *commons,
					unsigned channel)
{
	struct rb_node *n = commons->iodevs_tree_chan.rb_node;
	struct io_device *iodev;
	while (n) {
		iodev = rb_entry(n, struct io_device, node_chan);
		if (channel < iodev->id)
			n = n->rb_left;
		else if (channel > iodev->id)
			n = n->rb_right;
		else
			return iodev;
	}
	return NULL;
}

struct io_device *get_iod_with_format(struct mif_common *commons,
			enum dev_format format)
{
	struct rb_node *n = commons->iodevs_tree_fmt.rb_node;
	struct io_device *iodev;
	while (n) {
		iodev = rb_entry(n, struct io_device, node_fmt);
		if (format < iodev->format)
			n = n->rb_left;
		else if (format > iodev->format)
			n = n->rb_right;
		else
			return iodev;
	}
	return NULL;
}

struct io_device *insert_iod_with_channel(struct mif_common *commons,
		unsigned channel, struct io_device *iod)
{
	struct rb_node **p = &commons->iodevs_tree_chan.rb_node;
	struct rb_node *parent = NULL;
	struct io_device *iodev;
	while (*p) {
		parent = *p;
		iodev = rb_entry(parent, struct io_device, node_chan);
		if (channel < iodev->id)
			p = &(*p)->rb_left;
		else if (channel > iodev->id)
			p = &(*p)->rb_right;
		else
			return iodev;
	}
	rb_link_node(&iod->node_chan, parent, p);
	rb_insert_color(&iod->node_chan, &commons->iodevs_tree_chan);
	return NULL;
}

struct io_device *insert_iod_with_format(struct mif_common *commons,
		enum dev_format format, struct io_device *iod)
{
	struct rb_node **p = &commons->iodevs_tree_fmt.rb_node;
	struct rb_node *parent = NULL;
	struct io_device *iodev;
	while (*p) {
		parent = *p;
		iodev = rb_entry(parent, struct io_device, node_fmt);
		if (format < iodev->format)
			p = &(*p)->rb_left;
		else if (format > iodev->format)
			p = &(*p)->rb_right;
		else
			return iodev;
	}
	rb_link_node(&iod->node_fmt, parent, p);
	rb_insert_color(&iod->node_fmt, &commons->iodevs_tree_fmt);
	return NULL;
}

void iodevs_for_each(struct mif_common *commons, action_fn action, void *args)
{
	struct io_device *iod;
	struct rb_node *node = rb_first(&commons->iodevs_tree_chan);
	for (; node; node = rb_next(node)) {
		iod = rb_entry(node, struct io_device, node_chan);
		action(iod, args);
	}
}

void iodev_netif_wake(struct io_device *iod, void *args)
{
	if (iod->io_typ == IODEV_NET && iod->ndev) {
		netif_wake_queue(iod->ndev);
		mif_info("%s\n", iod->name);
	}
}

void iodev_netif_stop(struct io_device *iod, void *args)
{
	if (iod->io_typ == IODEV_NET && iod->ndev) {
		netif_stop_queue(iod->ndev);
		mif_info("%s\n", iod->name);
	}
}

static void iodev_set_tx_link(struct io_device *iod, void *args)
{
	struct link_device *ld = (struct link_device *)args;
	if (iod->io_typ == IODEV_NET && IS_CONNECTED(iod, ld)) {
		set_current_link(iod, ld);
		mif_err("%s -> %s\n", iod->name, ld->name);
	}
}

void rawdevs_set_tx_link(struct mif_common *commons, enum modem_link link_type)
{
	struct link_device *ld = find_linkdev(commons, link_type);
	if (ld)
		iodevs_for_each(commons, iodev_set_tx_link, ld);
}

void mif_print_data(char *buf, int len)
{
	int   words = len >> 4;
	int   residue = len - (words << 4);
	int   i = 0;
	char *b = NULL;
	char  last[80];
	char  tb[8];

	/* Make the last line, if ((len % 16) > 0) */
	if (residue > 0) {
		memset(last, 0, sizeof(last));
		memset(tb, 0, sizeof(tb));
		b = buf + (words << 4);

		sprintf(last, "%04X: ", (words << 4));
		for (i = 0; i < residue; i++) {
			sprintf(tb, "%02x ", b[i]);
			strcat(last, tb);
			if ((i & 0x3) == 0x3) {
				sprintf(tb, " ");
				strcat(last, tb);
			}
		}
	}

	for (i = 0; i < words; i++) {
		b = buf + (i << 4);
		mif_err("%04X: "
			"%02x %02x %02x %02x  %02x %02x %02x %02x  "
			"%02x %02x %02x %02x  %02x %02x %02x %02x\n",
			(i << 4),
			b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
			b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
	}

	/* Print the last line */
	if (residue > 0)
		mif_err("%s\n", last);
}

void print_sipc4_hdlc_fmt_frame(const u8 *psrc)
{
	u8  *hf = NULL;					/* HDLC Frame	*/
	struct fmt_hdr *hh;             /* HDLC Header  */
	struct sipc_fmt_hdr *fh;        /* IPC Header   */
	u16 hh_len = sizeof(struct fmt_hdr);
	u16 fh_len = sizeof(struct sipc_fmt_hdr);
	u8  *data = NULL;
	int  data_len = 0;

	/* Actual HDLC header starts from after START flag (0x7F) */
	hf = (u8 *)(psrc + 1);

	/* Point HDLC header and IPC header */
	hh = (struct fmt_hdr *)(hf);
	fh = (struct sipc_fmt_hdr *)(hf + hh_len);

	/* Point IPC data */
	data     = hf + (hh_len + fh_len);
	data_len = hh->len - (hh_len + fh_len);

	mif_err("--------------------HDLC & FMT HEADER----------------------\n");

	mif_err("HDLC: length %d, control 0x%02x\n", hh->len, hh->control);

	mif_err("(M)0x%02X, (S)0x%02X, (T)0x%02X, mseq %d, aseq %d, len %d\n",
		fh->main_cmd, fh->sub_cmd, fh->cmd_type,
		fh->msg_seq, fh->ack_seq, fh->len);

	mif_err("-----------------------IPC FMT DATA------------------------\n");

	if (data_len > 0) {
		if (data_len > 64)
			data_len = 64;
		mif_print_data(data, data_len);
	}

	mif_err("-----------------------------------------------------------\n");
}

void print_sipc4_fmt_frame(const u8 *psrc)
{
	struct sipc_fmt_hdr *fh = (struct sipc_fmt_hdr *)psrc;
	u16 fh_len = sizeof(struct sipc_fmt_hdr);
	u8  *data = NULL;
	int  data_len = 0;

	/* Point IPC data */
	data     = (u8 *)(psrc + fh_len);
	data_len = fh->len - fh_len;

	mif_err("----------------------IPC FMT HEADER-----------------------\n");

	mif_err("(M)0x%02X, (S)0x%02X, (T)0x%02X, mseq:%d, aseq:%d, len:%d\n",
		fh->main_cmd, fh->sub_cmd, fh->cmd_type,
		fh->msg_seq, fh->ack_seq, fh->len);

	mif_err("-----------------------IPC FMT DATA------------------------\n");

	if (data_len > 0)
		mif_print_data(data, data_len);

	mif_err("-----------------------------------------------------------\n");
}
