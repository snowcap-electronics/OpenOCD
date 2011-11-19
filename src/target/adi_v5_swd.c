/***************************************************************************
 *
 *   Copyright (C) 2010 by David Brownell
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the
 *   Free Software Foundation, Inc.,
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 ***************************************************************************/

/**
 * @file
 * Utilities to support ARM "Serial Wire Debug" (SWD), a low pin-count debug
 * link protocol used in cases where JTAG is not wanted.  This is coupled to
 * recent versions of ARM's "CoreSight" debug framework.  This specific code
 * is a transport level interface, with "target/arm_adi_v5.[hc]" code
 * understanding operation semantics, shared with the JTAG transport.
 *
 * Single-DAP support only.
 *
 * for details, see "ARM IHI 0031A"
 * ARM Debug Interface v5 Architecture Specification
 * especially section 5.3 for SWD protocol
 *
 * On many chips (most current Cortex-M3 parts) SWD is a run-time alternative
 * to JTAG.  Boards may support one or both.  There are also SWD-only chips,
 * (using SW-DP not SWJ-DP).
 *
 * Even boards that also support JTAG can benefit from SWD support, because
 * usually there's no way to access the SWO trace view mechanism in JTAG mode.
 * That is, trace access may require SWD support.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>

#include <transport/transport.h>
#include <jtag/interface.h>

#include <jtag/swd.h>

static int swd_queue_dp_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	/* REVISIT status return vs ack ... */
	return swd->read_reg(swd_cmd(true,  false, reg), data);
}

static int swd_queue_idcode_read(struct adiv5_dap *dap,
		uint8_t *ack, uint32_t *data)
{
	int status = swd_queue_dp_read(dap, DP_IDCODE, data);
	if (status < 0)
		return status;
	*ack = status;
	/* ?? */
	return ERROR_OK;
}

static int (swd_queue_dp_write)(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	/* REVISIT status return vs ack ... */
	return swd->write_reg(swd_cmd(false,  false, reg), data);
}


static int (swd_queue_ap_read)(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	/* REVISIT  APSEL ... */
	/* REVISIT status return ... */
	return swd->read_reg(swd_cmd(true,  true, reg), data);
}

static int (swd_queue_ap_write)(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	/* REVISIT  APSEL ... */
	/* REVISIT status return ... */
	return swd->write_reg(swd_cmd(false,  true, reg), data);
}

static int (swd_queue_ap_abort)(struct adiv5_dap *dap, uint8_t *ack)
{
	return ERROR_FAIL;
}

/** Executes all queued DAP operations. */
static int swd_run(struct adiv5_dap *dap)
{
	/* for now the SWD interface hard-wires a zero-size queue.  */

	/* FIXME but we still need to check and scrub
	 * any hardware errors ...
	 */
	return ERROR_OK;
}

const struct dap_ops swd_dap_ops = {
	.is_swd = true,

	.queue_idcode_read = swd_queue_idcode_read,
	.queue_dp_read = swd_queue_dp_read,
	.queue_dp_write = swd_queue_dp_write,
	.queue_ap_read = swd_queue_ap_read,
	.queue_ap_write = swd_queue_ap_write,
	.queue_ap_abort = swd_queue_ap_abort,
	.run = swd_run,
};

/***************************************************************************
 *
 * DPACC and APACC scanchain access through JTAG-DP (or SWJ-DP)
 *
***************************************************************************/

/**
 * Scan DPACC or APACC using target ordered uint8_t buffers.  No endianness
 * conversions are performed.  See section 4.4.3 of the ADIv5 spec, which
 * discusses operations which access these registers.
 *
 * Note that only one scan is performed.  If RnW is set, a separate scan
 * will be needed to collect the data which was read; the "invalue" collects
 * the posted result of a preceding operation, not the current one.
 *
 * @param dap the DAP
 * @param instr SWD_DP_APACC (AP access) or SWD_DP_DPACC (DP access)
 * @param reg_addr two significant bits; A[3:2]; for APACC access, the
 *	SELECT register has more addressing bits.
 * @param RnW false iff outvalue will be written to the DP or AP
 * @param outvalue points to a 32-bit (little-endian) integer
 * @param invalue NULL, or points to a 32-bit (little-endian) integer
 * @param ack points to where the three bit SWD_ACK_* code will be stored
 */

/* FIXME don't export ... this is a temporary workaround for the
 * mem_ap_read_buf_u32() mess, until it's no longer JTAG-specific.
 */
int adi_swd_dp_scan(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint8_t *outvalue, uint8_t *invalue, uint8_t *ack)
{
	if (RnW == DPAP_READ)
	{
		swd_add_transact_in(instr, 1, reg_addr, (uint32_t *)invalue, ack);
	}
	else
	{
		swd_add_transact_out(instr, 0, reg_addr, *(uint32_t *)outvalue, ack);
	}

	return ERROR_OK;
}

/**
 * Scan DPACC or APACC out and in from host ordered uint32_t buffers.
 * This is exactly like adi_swd_dp_scan(), except that endianness
 * conversions are performed (so the types of invalue and outvalue
 * must be different).
 */
static int adi_swd_dp_scan_u32(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint32_t outvalue, uint32_t *invalue, uint8_t *ack)
{
	if (RnW == DPAP_READ)
	{
		swd_add_transact_in(instr, 1, reg_addr, invalue, ack);
	}
	else
	{
		swd_add_transact_out(instr, 0, reg_addr, outvalue, ack);
	}

	return ERROR_OK;
}

/**
 * Utility to write AP registers.
 */
static inline int adi_swd_ap_write_check(struct adiv5_dap *dap,
		uint8_t reg_addr, uint8_t *outvalue)
{
	return adi_swd_dp_scan(dap, SWD_DP_APACC, reg_addr, DPAP_WRITE,
			outvalue, NULL, NULL);
}

static int adi_swd_scan_inout_check_u32(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint32_t outvalue, uint32_t *invalue)
{
	int retval;

	/* Issue the read or write */
	retval = adi_swd_dp_scan_u32(dap, instr, reg_addr,
			RnW, outvalue, invalue, NULL);
	if (retval != ERROR_OK)
		return retval;

	/* For reads,  collect posted value; RDBUFF has no other effect.
	 * Assumes read gets acked with OK/FAULT, and CTRL_STAT says "OK".
	 */
	if ((RnW == DPAP_READ) && (invalue != NULL) && (instr == SWD_DP_APACC))
		retval = adi_swd_dp_scan_u32(dap, SWD_DP_DPACC,
				DP_RDBUFF, DPAP_READ, 0, invalue, &dap->ack);
	return retval;
}

static int swddp_transaction_endcheck(struct adiv5_dap *dap)
{
	int retval;
	uint32_t ctrlstat;

	/* too expensive to call keep_alive() here */

#if 0
	/* Danger!!!! BROKEN!!!! */
	adi_swd_scan_inout_check_u32(dap, SWD_DP_DPACC,
			DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
	/* Danger!!!! BROKEN!!!! Why will jtag_execute_queue() fail here????
	R956 introduced the check on return value here and now Michael Schwingen reports
	that this code no longer works....

	https://lists.berlios.de/pipermail/openocd-development/2008-September/003107.html
	*/
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("BUG: Why does this fail the first time????");
	}
	/* Why??? second time it works??? */
#endif

	/* Post CTRL/STAT read; discard any previous posted read value
	 * but collect its ACK status.
	 */
	retval = adi_swd_scan_inout_check_u32(dap, SWD_DP_DPACC,
			DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
	if (retval != ERROR_OK)
		return retval;
	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;

	dap->ack = dap->ack & 0x7;

	/* common code path avoids calling timeval_ms() */
	if (dap->ack != SWD_ACK_OK)
	{
		long long then = timeval_ms();

		while (dap->ack != SWD_ACK_OK)
		{
			if (dap->ack == SWD_ACK_WAIT)
			{
				if ((timeval_ms()-then) > 1000)
				{
					LOG_WARNING("Timeout (1000ms) waiting "
						"for ACK=OK/FAULT "
						"in swd-DP transaction");
					return ERROR_JTAG_DEVICE_ERROR;
				}
			}
			else
			{
				LOG_WARNING("Invalid ACK %#x "
						"in swd-DP transaction",
						dap->ack);
				return ERROR_JTAG_DEVICE_ERROR;
			}

			retval = adi_swd_scan_inout_check_u32(dap, SWD_DP_DPACC,
					DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if (retval != ERROR_OK)
				return retval;
			if ((retval = dap_run(dap)) != ERROR_OK)
				return retval;
			dap->ack = dap->ack & 0x7;
		}
	}

	/* REVISIT also STICKYCMP, for pushed comparisons (nyet used) */

	/* Check for STICKYERR and STICKYORUN */
	if (ctrlstat & (SSTICKYORUN | SSTICKYERR))
	{
		LOG_DEBUG("swd-dp: CTRL/STAT error, 0x%" PRIx32, ctrlstat);
		/* Check power to debug regions */
		if ((ctrlstat & 0xf0000000) != 0xf0000000)
		{
			retval = ahbap_debugport_init(dap);
			if (retval != ERROR_OK)
				return retval;
		}
		else
		{
			uint32_t mem_ap_csw, mem_ap_tar;

			/* Maybe print information about last intended
			 * MEM-AP access; but not if autoincrementing.
			 * *Real* CSW and TAR values are always shown.
			 */
			if (dap->ap_tar_value != (uint32_t) -1)
				LOG_DEBUG("MEM-AP Cached values: "
					"ap_bank 0x%" PRIx32
					", ap_csw 0x%" PRIx32
					", ap_tar 0x%" PRIx32,
					dap->ap_bank_value,
					dap->ap_csw_value,
					dap->ap_tar_value);

			if (ctrlstat & SSTICKYORUN)
				LOG_ERROR("SWD-DP OVERRUN - check clock, "
					"memaccess, or reduce swd speed");

			if (ctrlstat & SSTICKYERR)
				LOG_ERROR("SWD-DP STICKY ERROR");

			/* Clear Sticky Error Bits */
			retval = adi_swd_scan_inout_check_u32(dap, SWD_DP_DPACC,
					DP_ABORT, DPAP_WRITE,
					dap->dp_ctrl_stat | ORUNERRCLR
						| STKERRCLR, NULL);
			if (retval != ERROR_OK)
				return retval;
			retval = adi_swd_scan_inout_check_u32(dap, SWD_DP_DPACC,
					DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if (retval != ERROR_OK)
				return retval;
			if ((retval = dap_run(dap)) != ERROR_OK)
				return retval;

			LOG_DEBUG("swd-dp: CTRL/STAT 0x%" PRIx32, ctrlstat);

			retval = dap_queue_ap_read(dap,
					AP_REG_CSW, &mem_ap_csw);
			if (retval != ERROR_OK)
				return retval;

			retval = dap_queue_ap_read(dap,
					AP_REG_TAR, &mem_ap_tar);
			if (retval != ERROR_OK)
				return retval;

			if ((retval = dap_run(dap)) != ERROR_OK)
				return retval;
			LOG_ERROR("MEM_AP_CSW 0x%" PRIx32 ", MEM_AP_TAR 0x%"
					PRIx32, mem_ap_csw, mem_ap_tar);

		}
		if ((retval = dap_run(dap)) != ERROR_OK)
			return retval;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/*--------------------------------------------------------------------------*/

static int swd_idcode_q_read(struct adiv5_dap *dap,
		uint8_t *ack, uint32_t *data)
{
	return adi_swd_scan_inout_check_u32(dap, SWD_DP_DPACC,
			DP_IDCODE, DPAP_READ, 0, data);
}

static int swd_dp_q_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	return adi_swd_scan_inout_check_u32(dap, SWD_DP_DPACC,
			reg, DPAP_READ, 0, data);
}

static int swd_dp_q_write(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	return adi_swd_scan_inout_check_u32(dap, SWD_DP_DPACC,
			reg, DPAP_WRITE, data, NULL);
}

/** Select the AP register bank matching bits 7:4 of reg. */
static int swd_ap_q_bankselect(struct adiv5_dap *dap, unsigned reg)
{
	uint32_t select_ap_bank = reg & 0x000000F0;

	if (select_ap_bank == dap->ap_bank_value)
		return ERROR_OK;
	dap->ap_bank_value = select_ap_bank;

	select_ap_bank |= dap->apsel;

	return swd_dp_q_write(dap, DP_SELECT, select_ap_bank);
}

static int swd_ap_q_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	int retval = swd_ap_q_bankselect(dap, reg);

	if (retval != ERROR_OK)
		return retval;

	return adi_swd_scan_inout_check_u32(dap, SWD_DP_APACC, reg,
			DPAP_READ, 0, data);
}

static int swd_ap_q_write(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	uint8_t out_value_buf[4];

	int retval = swd_ap_q_bankselect(dap, reg);
	if (retval != ERROR_OK)
		return retval;

	buf_set_u32(out_value_buf, 0, 32, data);

	return adi_swd_ap_write_check(dap, reg, out_value_buf);
}

static int swd_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	return ERROR_OK;
}

static int swd_dp_run(struct adiv5_dap *dap)
{
	dap->ack = SWD_ACK_OK;
	return swddp_transaction_endcheck(dap);
}

/* FIXME don't export ... just initialize as
 * part of DAP setup
*/
const struct dap_ops swd_dp_ops = {
	.queue_idcode_read =	swd_idcode_q_read,
	.queue_dp_read =	swd_dp_q_read,
	.queue_dp_write =	swd_dp_q_write,
	.queue_ap_read =	swd_ap_q_read,
	.queue_ap_write =	swd_ap_q_write,
	.queue_ap_abort =	swd_ap_q_abort,
	.queue_dp_scan =	adi_swd_dp_scan,
	.run =			swd_dp_run,
};


/*
 * This represents the bits which must be sent out on TMS/SWDIO to
 * switch a DAP implemented using an SWJ-DP module into SWD mode.
 * These bits are stored (and transmitted) LSB-first.
 *
 * See the DAP-Lite specification, section 2.2.5 for information
 * about making the debug link select SWD or JTAG.  (Similar info
 * is in a few other ARM documents.)
 */
static const uint8_t jtag2swd_bitseq[] = {
	/* More than 50 TCK/SWCLK cycles with TMS/SWDIO high,
	 * putting both JTAG and SWD logic into reset state.
	 */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* Switching sequence enables SWD and disables JTAG
	 * NOTE: bits in the DP's IDCODE may expose the need for
	 * an old/obsolete/deprecated sequence (0xb6 0xed).
	 */
	0x9e, 0xe7,
	/* More than 50 TCK/SWCLK cycles with TMS/SWDIO high,
	 * putting both JTAG and SWD logic into reset state.
	 */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f,
};

/**
 * Put the debug link into SWD mode, if the target supports it.
 * The link's initial mode may be either JTAG (for example,
 * with SWJ-DP after reset) or SWD.
 *
 * @param target Enters SWD mode (if possible).
 *
 * Note that targets using the JTAG-DP do not support SWD, and that
 * some targets which could otherwise support it may have have been
 * configured to disable SWD signaling
 *
 * @return ERROR_OK or else a fault code.
 */
int dap_to_swd(struct target *target)
{
//	struct arm *arm = target_to_arm(target);
	int retval;

	LOG_DEBUG("Enter SWD mode");

	/* REVISIT it's ugly to need to make calls to a "jtag"
	 * subsystem if the link may not be in JTAG mode...
	 */

	swd_add_sequence((uint8_t*)jtag2swd_bitseq, sizeof(jtag2swd_bitseq) * 8);
	retval = jtag_execute_queue();

	/* set up the DAP's ops vector for SWD mode. */
//	arm->dap->ops = &swd_dap_ops;

	return retval;
}



COMMAND_HANDLER(handle_swd_wcr)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;
	uint32_t wcr;
	unsigned trn, scale = 0;

	switch (CMD_ARGC) {
	/* no-args: just dump state */
	case 0:
		/*retval = swd_queue_dp_read(dap, DP_WCR, &wcr); */
		retval = dap_queue_dp_read(dap, DP_WCR, &wcr);
		if (retval == ERROR_OK)
			dap->ops->run(dap);
		if (retval != ERROR_OK) {
			LOG_ERROR("can't read WCR?");
			return retval;
		}

		command_print(CMD_CTX,
			"turnaround=%d, prescale=%d",
			WCR_TO_TRN(wcr),
			WCR_TO_PRESCALE(wcr));
	return ERROR_OK;

	case 2:		/* TRN and prescale */
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], scale);
		if (scale > 7) {
			LOG_ERROR("prescale %d is too big", scale);
			return ERROR_FAIL;
		}
		/* FALL THROUGH */

	case 1:		/* TRN only */
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], trn);
		if (trn < 1 || trn > 4) {
			LOG_ERROR("turnaround %d is invalid", trn);
			return ERROR_FAIL;
		}

		wcr = ((trn - 1) << 8) | scale;
		/* FIXME
		 * write WCR ...
		 * then, re-init adapter with new TRN
		 */
		LOG_ERROR("can't yet modify WCR");
		return ERROR_FAIL;

	default:	/* too many arguments */
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
}

static const struct command_registration swd_commands[] = {
	{
		/*
		 * Set up SWD and JTAG targets identically, unless/until
		 * infrastructure improves ...  meanwhile, ignore all
		 * JTAG-specific stuff like IR length for SWD.
		 *
		 * REVISIT can we verify "just one SWD DAP" here/early?
		 */
		.name = "newdap",
		.jim_handler = jim_jtag_newtap,
		.mode = COMMAND_CONFIG,
		.help = "declare a new SWD DAP"
	},
	{
		.name = "wcr",
		.handler = handle_swd_wcr,
		.mode = COMMAND_ANY,
		.help = "display or update DAP's WCR register",
		.usage = "turnaround (1..4), prescale (0..7)",
	},

	/* REVISIT -- add a command for SWV trace on/off */
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration swd_handlers[] = {
	{
		.name = "swd",
		.mode = COMMAND_ANY,
		.help = "SWD command group",
		.chain = swd_commands,
	},
	COMMAND_REGISTRATION_DONE
};

static int swd_select(struct command_context *ctx)
{
//	struct target *target = get_current_target(ctx);
	int retval;

	retval = register_commands(ctx, NULL, swd_handlers);

	if (retval != ERROR_OK)
		return retval;

	retval = jtag_register_commands(ctx);

	if (retval != ERROR_OK)
		return retval;
#if 0
	 /* be sure driver is in SWD mode; start
	  * with hardware default TRN (1), it can be changed later
	  */
	if (!swd || !swd->read_reg || !swd->write_reg || !swd->init) {
		LOG_DEBUG("no SWD driver?");
		return ERROR_FAIL;
	}

	 retval = swd->init(1);
	if (retval != ERROR_OK) {
		LOG_DEBUG("can't init SWD driver");
		return retval;
	}

	/* force DAP into SWD mode (not JTAG) */
	retval = dap_to_swd(target);
#endif
	return retval;
}

static int swd_init(struct command_context *ctx)
{
#if 0
	struct target *target = get_current_target(ctx);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;
	uint32_t idcode;
	int status;

	/* FIXME validate transport config ... is the
	 * configured DAP present (check IDCODE)?
	 * Is *only* one DAP configured?
	 *
	 * MUST READ IDCODE
	 */

 /* Note, debugport_init() does setup too */

	uint8_t ack;

	status = swd_queue_idcode_read(dap, &ack, &idcode);

	if (status == ERROR_OK)
		LOG_INFO("SWD IDCODE %#8.8x", idcode);

	return status;

#endif
	int retval;
	jtag_add_reset(0, 0);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;
	return ERROR_OK;
}

static struct transport swd_transport = {
	.name = "swd",
	.select = swd_select,
	.init = swd_init,
};

static void swd_constructor(void) __attribute__((constructor));
static void swd_constructor(void)
{
	transport_register(&swd_transport);
}

/** Returns true if the current debug session
 * is using SWD as its transport.
 */
bool transport_is_swd(void)
{
	return get_current_transport() == &swd_transport;
}
