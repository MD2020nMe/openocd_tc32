/***************************************************************************
 *   Copyright (C) 2016  by MD2020                                         *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *                                                                         *
 *   MD >> name is misleading.   this is not a JTAG.  just the template    *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"


#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/swd.h>
//#include <usb.h>
#include "usb_common.h"
#include "libusb1_common.h"
#include <libusb.h>

// Telink T32 JTag Dongle
#define USB_VID    0x248a
#define USB_PID    0x8266

// print debug buffer
#define _DEBUG_USB_COMMS_ 1

#define TLINK_EPT_BULK_OUT		0x05
#define TLINK_EPT_BULK_IN		0x88
#define TLINK_USB_INTERFACE 	1

#define TLINK_USB_TIMEOUT        1000

#define TLINK_IN_BUFFER_SIZE     (4*1024)
#define TLINK_OUT_BUFFER_SIZE    (4*1024)

/* USB command request codes. */
#define CMD_GET_VERSION          0x00
#define CMD_SELECT_DPIMPL        0x10
#define CMD_SET_TCK_FREQUENCY    0x11
#define CMD_GET_TCK_FREQUENCY    0x12
#define CMD_MEASURE_MAX_TCK_FREQ 0x15
#define CMD_MEASURE_RTCK_RESPONSE 0x16
#define CMD_TAP_SHIFT            0x17
#define CMD_SET_TAPHW_STATE      0x20
#define CMD_GET_TAPHW_STATE      0x21
#define CMD_TGPWR_SETUP          0x22

/* Global USB buffers */
static uint8_t usb_in_buffer[TLINK_IN_BUFFER_SIZE];
static uint8_t usb_out_buffer[TLINK_OUT_BUFFER_SIZE];

static bool swd_mode;


/* Queue command functions */
static void tlink_end_state(tap_state_t state);
static void tlink_state_move(void);
static void tlink_path_move(int num_states, tap_state_t *path);
static void tlink_runtest(int num_cycles);
static void tlink_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command);
static void tlink_reset(int trst, int srst);
/* static void tlink_simple_command(uint8_t command); */
static int tlink_get_status(void);

/* tap buffer functions */
static void tlink_tap_init(void);
static int  tlink_tap_execute(void);
static void tlink_tap_ensure_space(int scans, int bits);
static void tlink_tap_append_step(int tms, int tdi);
static void tlink_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command);

/* TLINK lowlevel functions */
struct tlink {
    //struct usb_dev_handle *usb_handle;
    struct jtag_libusb_device_handle *usb_handle;
};

static struct tlink *tlink_usb_open(void);
static void tlink_usb_close(struct tlink *tlink);
static int tlink_usb_message(struct tlink *tlink, int out_length, int in_length);
static int tlink_usb_write(struct tlink *tlink, int out_length);
static int tlink_usb_read(struct tlink *tlink, int exp_in_length);

/* helper functions */
static int tlink_get_version_info(void);

static void tlink_reset_buffer(uint8_t *buffer, int length);

#ifdef _DEBUG_USB_COMMS_
static void tlink_debug_buffer(uint8_t *buffer, int length);
static void tlink_loopback_test(struct tlink * tlink);
#else
static inline void tlink_debug_buffer(uint8_t *buffer, int length) {}
static inline void tlink_loopback_test(struct tlink * tl) {}
#endif

static struct tlink *tlink_handle;


static int tlink_swd_init(void)
{
	LOG_INFO("Tlink SWD mode enabled");
	swd_mode = true;

	return ERROR_OK;
}

static int tlink_swd_frequency(int_least32_t hz)
{
	//LOG_DEBUG("SWD delay: %d, retry count: %d", delay, retry_count);

	LOG_DEBUG("SWD line reset");
	return hz;
}

static int tlink_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void tlink_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
}

static void tlink_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
    usb_out_buffer[0] = value;
    usb_out_buffer[1] = ap_delay_clk;
	tlink_usb_write(tlink_handle, 4); 
}

/**************************************************************************
 * External interface implementation */

static int tlink_execute_queue(void)
{
    struct jtag_command *cmd = jtag_command_queue;
    int scan_size;
    enum scan_type type;
    uint8_t *buffer;

    while (cmd != NULL) {
        switch (cmd->type) {
            case JTAG_RUNTEST:
                DEBUG_JTAG_IO("runtest %i cycles, end in %i",
                        cmd->cmd.runtest->num_cycles, \
                        cmd->cmd.runtest->end_state);

                tlink_end_state(cmd->cmd.runtest->end_state);
                tlink_runtest(cmd->cmd.runtest->num_cycles);
                break;

            case JTAG_TLR_RESET:
                DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

                tlink_end_state(cmd->cmd.statemove->end_state);
                tlink_state_move();
                break;

            case JTAG_PATHMOVE:
                DEBUG_JTAG_IO("pathmove: %i states, end in %i", \
                        cmd->cmd.pathmove->num_states, \
                        cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

                tlink_path_move(cmd->cmd.pathmove->num_states,
                        cmd->cmd.pathmove->path);
                break;

            case JTAG_SCAN:
                DEBUG_JTAG_IO("scan end in %i", cmd->cmd.scan->end_state);

                tlink_end_state(cmd->cmd.scan->end_state);

                scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
                DEBUG_JTAG_IO("scan input, length = %d", scan_size);

#ifdef _DEBUG_USB_COMMS_
                tlink_debug_buffer(buffer, (scan_size + 7) / 8);
#endif
                type = jtag_scan_type(cmd->cmd.scan);
                tlink_scan(cmd->cmd.scan->ir_scan,
                        type, buffer,
                        scan_size, cmd->cmd.scan);
                break;

            case JTAG_RESET:
                DEBUG_JTAG_IO("reset trst: %i srst %i",
                        cmd->cmd.reset->trst,
                        cmd->cmd.reset->srst);

                tlink_tap_execute();

                if (cmd->cmd.reset->trst == 1)
                    tap_set_state(TAP_RESET);
                tlink_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
                break;

            case JTAG_SLEEP:
                DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
                tlink_tap_execute();
                jtag_sleep(cmd->cmd.sleep->us);
                break;

            default:
                LOG_ERROR("BUG: unknown JTAG command type encountered");
                exit(-1);
        }
        cmd = cmd->next;
    }

    return tlink_tap_execute();
}

static int tlink_swd_run_queue(void)
{
	return tlink_execute_queue();
}


/* Sets speed in kHz. */
static int tlink_speed(int speed)
{
    int result;
    int speed_real;

	LOG_INFO("Requesting speed %dkHz", speed);

    usb_out_buffer[0] = CMD_SET_TCK_FREQUENCY;
    buf_set_u32(usb_out_buffer + 1, 0, 32, speed*1000);

    //result = tlink_usb_message(tlink_handle, 5, 4);
    result = tlink_usb_write(tlink_handle, 5);

    if (result < 0) {
        LOG_ERROR("TLINK setting speed failed (%d)", result);
        return ERROR_JTAG_DEVICE_ERROR;
    }

    usb_out_buffer[0] = CMD_GET_TCK_FREQUENCY;
    //result = tlink_usb_message(tlink_handle, 1, 4);
    result = tlink_usb_read(tlink_handle, 5);
    speed_real = 32; // (int)buf_get_u32(usb_in_buffer, 0, 32) / 1000;
    if (result < 0) {
        LOG_ERROR("TLINK getting speed failed (%d)", result);
        return ERROR_JTAG_DEVICE_ERROR;
    } else
        LOG_INFO("Requested speed %dkHz, emulator reported %dkHz.", speed, speed_real);

    return ERROR_OK;
}

static int tlink_khz(int khz, int *jtag_speed)
{
	if (khz == 0)
		*jtag_speed = 0;
	else
		*jtag_speed = khz;

    return ERROR_OK;
}

static int tlink_speed_div(int speed, int *khz)
{
	if (speed == 0)
		*khz = 0;
	else
		*khz = 64000/speed;

    return ERROR_OK;
}

static int tlink_init(void)
{
    tlink_handle = tlink_usb_open();

    if (tlink_handle == 0) {
        LOG_ERROR(
            "Cannot find Telink Debugger!");
        return ERROR_JTAG_INIT_FAILED;
    }

    /* Initial JTAG speed (for reset and initialization): 32 kHz */
    tlink_speed(32);

    LOG_INFO("Telink Interface ready");

    tlink_reset(0, 0);
    tlink_tap_init();

	tlink_loopback_test(tlink_handle);

    return ERROR_OK;
}

static int tlink_quit(void)
{
    tlink_usb_close(tlink_handle);
    return ERROR_OK;
}

/**************************************************************************
 * Queue command implementations */

static void tlink_end_state(tap_state_t state)
{
    if (tap_is_state_stable(state))
        tap_set_end_state(state);
    else {
        LOG_ERROR("BUG: %i is not a valid end state", state);
        exit(-1);
    }
}

/* Goes to the end state. */
static void tlink_state_move(void)
{
    int i;
    int tms = 0;
    uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
    int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

    for (i = 0; i < tms_count; i++) {
        tms = (tms_scan >> i) & 1;
        tlink_tap_append_step(tms, 0);
    }

    tap_set_state(tap_get_end_state());
}

static void tlink_path_move(int num_states, tap_state_t *path)
{
    int i;

    for (i = 0; i < num_states; i++) {
        /*
         * TODO: The TLINK hardware delays TDI with 3 TCK cycles when in RTCK mode.
         * Either handle that here, or update the documentation with examples
         * how to fix that in the configuration files.
         */
        if (path[i] == tap_state_transition(tap_get_state(), false))
            tlink_tap_append_step(0, 0);
        else if (path[i] == tap_state_transition(tap_get_state(), true))
            tlink_tap_append_step(1, 0);
        else {
            LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
                tap_state_name(tap_get_state()), tap_state_name(path[i]));
            exit(-1);
        }

        tap_set_state(path[i]);
    }

    tap_set_end_state(tap_get_state());
}

static void tlink_runtest(int num_cycles)
{
    int i;

    tap_state_t saved_end_state = tap_get_end_state();

    /* only do a state_move when we're not already in IDLE */
    if (tap_get_state() != TAP_IDLE) {
        tlink_end_state(TAP_IDLE);
        tlink_state_move();
    }

    /* execute num_cycles */
    for (i = 0; i < num_cycles; i++)
        tlink_tap_append_step(0, 0);

    /* finish in end_state */
    tlink_end_state(saved_end_state);
    if (tap_get_state() != tap_get_end_state())
        tlink_state_move();
}

static void tlink_scan(bool ir_scan,
    enum scan_type type,
    uint8_t *buffer,
    int scan_size,
    struct scan_command *command)
{
    tap_state_t saved_end_state;

    tlink_tap_ensure_space(1, scan_size + 8);

    saved_end_state = tap_get_end_state();

    /* Move to appropriate scan state */
    tlink_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

    /* Only move if we're not already there */
    if (tap_get_state() != tap_get_end_state())
        tlink_state_move();

    tlink_end_state(saved_end_state);

    /* Scan */
    tlink_tap_append_scan(scan_size, buffer, command);

    /* We are in Exit1, go to Pause */
    tlink_tap_append_step(0, 0);

    tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

    if (tap_get_state() != tap_get_end_state())
        tlink_state_move();
}

static void tlink_reset(int trst, int srst)
{
    const uint8_t trst_mask = (1u << 5);
    const uint8_t srst_mask = (1u << 6);
    uint8_t val = 0;
    uint8_t outp_en = 0;
    uint8_t change_mask = 0;
    int result;

    LOG_DEBUG("trst: %i, srst: %i", trst, srst);

    if (srst == 0) {
        val |= srst_mask;
        outp_en &= ~srst_mask;        /* tristate */
        change_mask |= srst_mask;
    } else if (srst == 1) {
        val &= ~srst_mask;
        outp_en |= srst_mask;
        change_mask |= srst_mask;
    }

    if (trst == 0) {
        val |= trst_mask;
        outp_en &= ~trst_mask;        /* tristate */
        change_mask |= trst_mask;
    } else if (trst == 1) {
        val &= ~trst_mask;
        outp_en |= trst_mask;
        change_mask |= trst_mask;
    }

    usb_out_buffer[0] = CMD_SET_TAPHW_STATE;
    usb_out_buffer[1] = val;
    usb_out_buffer[2] = outp_en;
    usb_out_buffer[3] = change_mask;

    result = tlink_usb_write(tlink_handle, 4);

    if (result != 4)
        LOG_ERROR("TLINK TRST/SRST pin set failed failed (%d)", result);

    LOG_DEBUG("Telink TRST/SRST write succeeded");

	// reset buffer
    usb_out_buffer[0] = 0;
    usb_out_buffer[1] = 0;
    usb_out_buffer[2] = 0;
    usb_out_buffer[3] = 0;


    result = tlink_usb_read(tlink_handle, 4);

    if (result != 4)
        LOG_ERROR("TLINK read back error (%d)", result);
	else
    	LOG_DEBUG("Telink TRST/SRST write succeeded");
}

static int tlink_get_status(void)
{
    int result;

        LOG_INFO("Get USB status");

    usb_out_buffer[0] = CMD_GET_TAPHW_STATE;
    result = tlink_usb_write(tlink_handle, 1);

    if (result == 0) {
        unsigned int u_tg = buf_get_u32(usb_in_buffer, 0, 16);
        LOG_INFO(
            "U_tg = %d mV, U_aux = %d mV, U_tgpwr = %d mV, I_tgpwr = %d mA, D1 = %d, Target power %s %s",
            (int)(buf_get_u32(usb_in_buffer + 0, 0, 16)),
            (int)(buf_get_u32(usb_in_buffer + 2, 0, 16)),
            (int)(buf_get_u32(usb_in_buffer + 4, 0, 16)),
            (int)(buf_get_u32(usb_in_buffer + 6, 0, 16)),
            usb_in_buffer[9],
            usb_in_buffer[11] ? "OVERCURRENT" : "OK",
            usb_in_buffer[10] ? "enabled" : "disabled");

        if (u_tg < 1500)
            LOG_ERROR("Vref too low. Check Target Power");
    } else
        LOG_ERROR("TLINK command CMD_GET_TAPHW_STATE failed (%d)", result);

    return ERROR_OK;
}

static int tlink_get_version_info(void)
{
    int result;
    char sn[16];
    char auxinfo[257];

    LOG_INFO("Get version info");

    /* query hardware version */
    usb_out_buffer[0] = CMD_GET_VERSION;
    result = tlink_usb_message(tlink_handle, 1, 4 + 15 + 256);

    if (result != 0) {
        LOG_ERROR("TLINK command CMD_GET_VERSION failed (%d)", result);
        return ERROR_JTAG_DEVICE_ERROR;
    }

    memcpy(sn, usb_in_buffer + 4, 15);
    sn[15] = '\0';
    memcpy(auxinfo, usb_in_buffer + 4+15, 256);
    auxinfo[256] = '\0';

    LOG_INFO(
        "TLINK firmware version %d.%d, hardware revision %c, SN=%s, Additional info: %s",    \
        usb_in_buffer[1],
        usb_in_buffer[0], \
        isgraph(usb_in_buffer[2]) ? usb_in_buffer[2] : 'X', \
        sn,
        auxinfo);

    if (1 != usb_in_buffer[1] || 6 != usb_in_buffer[0])
        LOG_WARNING(
            "TLINK firmware version %d.%d is untested with this version of OpenOCD. You might experience unexpected behavior.",
            usb_in_buffer[1],
            usb_in_buffer[0]);
    return ERROR_OK;
}

COMMAND_HANDLER(tlink_handle_tlink_info_command)
{
    if (tlink_get_version_info() == ERROR_OK) {
        /* attempt to get status */
        tlink_get_status();
    }

    return ERROR_OK;
}

static const struct command_registration tlink_command_handlers[] = {
    {
        .name = "tlink_info",
        .handler = &tlink_handle_tlink_info_command,
        .mode = COMMAND_EXEC,
        .help = "query tlink info",
                .usage = "Telink T-Link Jtag interface"
    },
    COMMAND_REGISTRATION_DONE
};

static const struct swd_driver tlink_swd_driver = {
	.init = tlink_swd_init,
	.frequency = tlink_swd_frequency,
	.switch_seq = tlink_swd_switch_seq,
	.read_reg = tlink_swd_read_reg,
	.write_reg = tlink_swd_write_reg,
	.run = tlink_swd_run_queue,
};

struct jtag_interface tlink_interface = {
    .name = "tlink",
    .commands = tlink_command_handlers,
	.swd = &tlink_swd_driver,
    .transports = jtag_only,
    .execute_queue = tlink_execute_queue,
    .speed = tlink_speed,
    .speed_div = tlink_speed_div,
    .khz = tlink_khz,
    .init = tlink_init,
    .quit = tlink_quit,
};

/**************************************************************************
 * TLINK tap functions */

/* 2048 is the max value we can use here */
#define TLINK_TAP_BUFFER_SIZE 2048

static int tap_length;
static uint8_t tms_buffer[TLINK_TAP_BUFFER_SIZE];
static uint8_t tdi_buffer[TLINK_TAP_BUFFER_SIZE];
//static uint8_t tdo_buffer[TLINK_TAP_BUFFER_SIZE];

struct pending_scan_result {
    int first;    /* First bit position in tdo_buffer to read */
    int length;    /* Number of bits to read */
    struct scan_command *command;    /* Corresponding scan command */
    uint8_t *buffer;
};

#define MAX_PENDING_SCAN_RESULTS 256

static int pending_scan_results_length;
static struct pending_scan_result pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

static int last_tms;

static void tlink_tap_init(void)
{
    tap_length = 0;
    pending_scan_results_length = 0;
}

static void tlink_tap_ensure_space(int scans, int bits)
{
    int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
    int available_bits = TLINK_TAP_BUFFER_SIZE * 8 - tap_length;

    if (scans > available_scans || bits > available_bits)
        tlink_tap_execute();
}

static void tlink_tap_append_step(int tms, int tdi)
{
    last_tms = tms;
    int index_local = tap_length / 8;

    if (index_local < TLINK_TAP_BUFFER_SIZE) {
        int bit_index = tap_length % 8;
        uint8_t bit = 1 << bit_index;

        if (tms)
            tms_buffer[index_local] |= bit;
        else
            tms_buffer[index_local] &= ~bit;

        if (tdi)
            tdi_buffer[index_local] |= bit;
        else
            tdi_buffer[index_local] &= ~bit;

        tap_length++;
    } else
        LOG_ERROR("tlink_tap_append_step, overflow");
}

void tlink_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command)
{
    struct pending_scan_result *pending_scan_result =
        &pending_scan_results_buffer[pending_scan_results_length];
    int i;

    pending_scan_result->first = tap_length;
    pending_scan_result->length = length;
    pending_scan_result->command = command;
    pending_scan_result->buffer = buffer;

    for (i = 0; i < length; i++)
        tlink_tap_append_step((i < length-1 ? 0 : 1), (buffer[i/8] >> (i%8)) & 1);
    pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
static int tlink_tap_execute(void)
{
#if 0 // We dont do TAP yet

    int byte_length;
    int tms_offset;
    int tdi_offset;
    int i;
    int result;

        LOG_INFO("Tap execute");

    if (tap_length > 0) {
        /* Pad last byte so that tap_length is divisible by 8 */
        while (tap_length % 8 != 0) {
            /* More of the last TMS value keeps us in the same state,
             * analogous to free-running JTAG interfaces. */
            tlink_tap_append_step(last_tms, 0);
        }

        byte_length = tap_length / 8;

        usb_out_buffer[0] = CMD_TAP_SHIFT;
        buf_set_u32(usb_out_buffer + 1, 0, 16, byte_length);

        tms_offset = 3;
        for (i = 0; i < byte_length; i++)
            usb_out_buffer[tms_offset + i] = flip_u32(tms_buffer[i], 8);

        tdi_offset = tms_offset + byte_length;
        for (i = 0; i < byte_length; i++)
            usb_out_buffer[tdi_offset + i] = flip_u32(tdi_buffer[i], 8);

        result = tlink_usb_message(tlink_handle,
                3 + 2 * byte_length,
                byte_length + 4);

        if (result == 0) {
            int stat_local;

            stat_local = (int)buf_get_u32(usb_in_buffer + byte_length, 0, 32);
            if (stat_local) {
                LOG_ERROR(
                    "tlink_tap_execute, emulator returned error code %d for a CMD_TAP_SHIFT command",
                    stat_local);
                return ERROR_JTAG_QUEUE_FAILED;
            }

            for (i = 0; i < byte_length; i++)
                tdo_buffer[i] = flip_u32(usb_in_buffer[i], 8);

            for (i = 0; i < pending_scan_results_length; i++) {
                struct pending_scan_result *pending_scan_result =
                    &pending_scan_results_buffer[i];
                uint8_t *buffer = pending_scan_result->buffer;
                int length = pending_scan_result->length;
                int first = pending_scan_result->first;
                struct scan_command *command = pending_scan_result->command;

                /* Copy to buffer */
                buf_set_buf(tdo_buffer, first, buffer, 0, length);

                DEBUG_JTAG_IO("pending scan result, length = %d", length);

#ifdef _DEBUG_USB_COMMS_
                tlink_debug_buffer(buffer, byte_length);
#endif

                if (jtag_read_buffer(buffer, command) != ERROR_OK) {
                    tlink_tap_init();
                    return ERROR_JTAG_QUEUE_FAILED;
                }

                if (pending_scan_result->buffer != NULL)
                    free(pending_scan_result->buffer);
            }
        } else {
            LOG_ERROR("tlink_tap_execute, wrong result %d, expected %d",
                result,
                byte_length);
            return ERROR_JTAG_QUEUE_FAILED;
        }

        tlink_tap_init();
    }
#endif // if 0

    return ERROR_OK;
}

/****************************************************************************
 * JLink USB low-level functions */

static struct tlink *tlink_usb_open()
{
    //usb_init();

    const uint16_t vids[] = { USB_VID, 0 };
    const uint16_t pids[] = { USB_PID, 0 };
    //struct usb_dev_handle *dev;
	jtag_libusb_device_handle * dev = NULL;
    //if (jtag_usb_open(vids, pids, &dev) != ERROR_OK)
    if (jtag_libusb_open(vids, pids, NULL, &dev) != ERROR_OK)
        return NULL;

    struct tlink *result = malloc(sizeof(struct tlink));
    result->usb_handle = dev;

#if 0
    /* usb_set_configuration required under win32 */
    usb_set_configuration(dev, dev->config[0].bConfigurationValue);
#endif
    LOG_DEBUG("Opening Interface %d", TLINK_USB_INTERFACE);
    if (libusb_claim_interface(dev, TLINK_USB_INTERFACE) != ERROR_OK)
    {
    	LOG_ERROR("Opening Interface failed, Please Insert Telink Debugger");
		jtag_libusb_close(dev);
		return 0;
    }

#if 0
    /*
     * This makes problems under Mac OS X. And is not needed
     * under Windows. Hopefully this will not break a linux build
     */
    usb_set_altinterface(dev, 0);
#endif
    return result;
}

static void tlink_usb_close(struct tlink *tlink)
{
    //usb_close(tlink->usb_handle);
    jtag_libusb_close(tlink->usb_handle);
    free(tlink);
}

/* Send a message and receive the reply. */
static int tlink_usb_message(struct tlink *tlink, int out_length, int in_length)
{
    int result;

    result = tlink_usb_write(tlink, out_length);

    if (result == out_length) {
        result = tlink_usb_read(tlink, in_length);
        if (result != in_length) {
            LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)",
                in_length,
                result);
            return -1;
        }
    } else {
        LOG_ERROR("usb_message failed (requested=%d, result=%d)", out_length, result);
        return -1;
    }
    return 0;
}

/* Write data from out_buffer to USB. */
static int tlink_usb_write(struct tlink *tlink, int out_length)
{
    int result;

    if (out_length > TLINK_OUT_BUFFER_SIZE) {
        LOG_ERROR("tlink_write illegal out_length=%d (max=%d)", out_length, TLINK_OUT_BUFFER_SIZE);
        return -1;
    }

    //result = usb_bulk_write(tlink->usb_handle, TLINK_EPT_BULK_OUT, \
    //        (char *)usb_out_buffer, out_length, TLINK_USB_TIMEOUT);
    result = jtag_libusb_bulk_write(tlink->usb_handle, TLINK_EPT_BULK_OUT, \
            (char *)usb_out_buffer, out_length, TLINK_USB_TIMEOUT);

    //DEBUG_JTAG_IO("tlink_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
    tlink_debug_buffer(usb_out_buffer, out_length);
#endif
    return result;
}



/* Read data from USB into in_buffer. */
static int tlink_usb_read(struct tlink *tlink, int exp_in_length)
{
    //int result = usb_bulk_read(tlink->usb_handle, TLINK_EPT_BULK_IN, \
    //        (char *)usb_in_buffer, exp_in_length, TLINK_USB_TIMEOUT);
    int result = jtag_libusb_bulk_read(tlink->usb_handle, TLINK_EPT_BULK_IN, \
            (char *)usb_in_buffer, exp_in_length, TLINK_USB_TIMEOUT);

LOG_INFO("tlink usb read %d bytes", result);

    DEBUG_JTAG_IO("tlink_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
    tlink_debug_buffer(usb_in_buffer, result);
#endif
    return result;
}

static inline void tlink_reset_buffer(uint8_t *buffer, int length)
{
     memset(buffer, 0, length);
}

#ifdef _DEBUG_USB_COMMS_
static void tlink_debug_buffer(uint8_t *buffer, int length)
{
#define BYTES_PER_LINE  16
    char line[81];
    char s[4];
    int i;
    int j;

    for (i = 0; i < length; i += BYTES_PER_LINE) {
        snprintf(line, 5, "%04x", i);
        for (j = i; j < i + BYTES_PER_LINE && j < length; j++) {
            snprintf(s, 4, " %02x", buffer[j]);
            strcat(line, s);
        }
        LOG_DEBUG("%s", line);

        /* Prevent GDB timeout (writing to log might take some time) */
        keep_alive();
    }
}

static void tlink_loopback_test(struct tlink * tl)
{
    char line[] = "Telink Debugger Loopback Test";
    int len = strlen(line);

    memcpy(usb_out_buffer, line, len) ;
    tlink_usb_write(tl, len);
    keep_alive();
	tlink_reset_buffer(usb_out_buffer, len + 1);

	LOG_DEBUG("Checking buffer");
 	//tlink_debug_buffer((uint8_t *)line, len);
 	tlink_debug_buffer(usb_out_buffer, len);
	LOG_DEBUG("Reading it back");
    tlink_usb_read(tl, len);

}
#endif // DEBUG_USB_COMMS


