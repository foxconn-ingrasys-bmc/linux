/********************************************************************************
*
* File Name:    jtag_drv_mod2.c
* Description:  JTAG driver additions
* Copyright (c) 2016-2017 Intel Corporation.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
*********************************************************************************/
/********************************************************************************
* File Name     : driver/char/asped/ast_jtag.c
* Author         : Ryan Chen
* Description   : AST JTAG driver
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#ifdef INTEL_JTAG_ADDITIONS
#include "jtag.h"
#include "jtag_ioctl.h"
#include "ast_jtag.h"

extern volatile u8		*ast_jtag_v_add;

#else

#include <plat/ast-scu.h>
#include <plat/regs-scu.h>
#include <plat/regs-jtag.h>

#include <mach/hardware.h>
#include <mach/platform.h>
#endif

#include "jtag_drv.h"


#define JTAG_DEV_MAJOR    252

//#define AST_JTAG_DEBUG

#ifdef AST_JTAG_DEBUG
#define JTAG_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define JTAG_DBUG(fmt, args...)
#endif

#define JTAG_MSG(fmt, args...) printk(fmt, ## args)

#ifndef INTEL_JTAG_ADDITIONS
static u32 ast_scu_base = IO_ADDRESS(AST_SCU_BASE);
static DEFINE_SPINLOCK(jtag_state_lock);
#endif

/*************************************************************************************/
struct ast_jtag_info {
    void __iomem        *reg_base;
    u8                  tapstate;       // see enum JtagStateEncode
    int                 irq;            //JTAG IRQ number
    u32                 flag;
    u32                 prevAstJtagCtrl;
    u32                 prevAstJtagSw;
    wait_queue_head_t   jtag_wq;
    bool                is_open;
    struct cdev         cdev;
};

// this structure represents a TMS cycle, as expressed in a set of bits and a count of bits (note: there are no start->end state transitions that require more than 1 byte of TMS cycles)
typedef struct {
    unsigned char tmsbits;
    unsigned char count;
} TmsCycle;

// these are the string representations of the TAP states corresponding to the enums literals in JtagStateEncode
const char* const c_statestr[] = {"TLR", "RTI", "SelDR", "CapDR", "ShfDR", "Ex1DR", "PauDR", "Ex2DR", "UpdDR", "SelIR", "CapIR", "ShfIR", "Ex1IR", "PauIR", "Ex2IR", "UpdIR"};

struct ast_jtag_info *ast_jtag;

// this is the complete set TMS cycles for going from any TAP state to any other TAP state, following a “shortest path” rule
const TmsCycle _tmsCycleLookup[][16] = {
/*   start*/ /*TLR      RTI      SelDR    CapDR    SDR      Ex1DR    PDR      Ex2DR    UpdDR    SelIR    CapIR    SIR      Ex1IR    PIR      Ex2IR    UpdIR    destination*/
/*     TLR*/{ {0x00,0},{0x00,1},{0x02,2},{0x02,3},{0x02,4},{0x0a,4},{0x0a,5},{0x2a,6},{0x1a,5},{0x06,3},{0x06,4},{0x06,5},{0x16,5},{0x16,6},{0x56,7},{0x36,6} },
/*     RTI*/{ {0x07,3},{0x00,0},{0x01,1},{0x01,2},{0x01,3},{0x05,3},{0x05,4},{0x15,5},{0x0d,4},{0x03,2},{0x03,3},{0x03,4},{0x0b,4},{0x0b,5},{0x2b,6},{0x1b,5} },
/*   SelDR*/{ {0x03,2},{0x03,3},{0x00,0},{0x00,1},{0x00,2},{0x02,2},{0x02,3},{0x0a,4},{0x06,3},{0x01,1},{0x01,2},{0x01,3},{0x05,3},{0x05,4},{0x15,5},{0x0d,4} },
/*   CapDR*/{ {0x1f,5},{0x03,3},{0x07,3},{0x00,0},{0x00,1},{0x01,1},{0x01,2},{0x05,3},{0x03,2},{0x0f,4},{0x0f,5},{0x0f,6},{0x2f,6},{0x2f,7},{0xaf,8},{0x6f,7} },
/*     SDR*/{ {0x1f,5},{0x03,3},{0x07,3},{0x07,4},{0x00,0},{0x01,1},{0x01,2},{0x05,3},{0x03,2},{0x0f,4},{0x0f,5},{0x0f,6},{0x2f,6},{0x2f,7},{0xaf,8},{0x6f,7} },
/*   Ex1DR*/{ {0x0f,4},{0x01,2},{0x03,2},{0x03,3},{0x02,3},{0x00,0},{0x00,1},{0x02,2},{0x01,1},{0x07,3},{0x07,4},{0x07,5},{0x17,5},{0x17,6},{0x57,7},{0x37,6} },
/*     PDR*/{ {0x1f,5},{0x03,3},{0x07,3},{0x07,4},{0x01,2},{0x05,3},{0x00,0},{0x01,1},{0x03,2},{0x0f,4},{0x0f,5},{0x0f,6},{0x2f,6},{0x2f,7},{0xaf,8},{0x6f,7} },
/*   Ex2DR*/{ {0x0f,4},{0x01,2},{0x03,2},{0x03,3},{0x00,1},{0x02,2},{0x02,3},{0x00,0},{0x01,1},{0x07,3},{0x07,4},{0x07,5},{0x17,5},{0x17,6},{0x57,7},{0x37,6} },
/*   UpdDR*/{ {0x07,3},{0x00,1},{0x01,1},{0x01,2},{0x01,3},{0x05,3},{0x05,4},{0x15,5},{0x00,0},{0x03,2},{0x03,3},{0x03,4},{0x0b,4},{0x0b,5},{0x2b,6},{0x1b,5} },
/*   SelIR*/{ {0x01,1},{0x01,2},{0x05,3},{0x05,4},{0x05,5},{0x15,5},{0x15,6},{0x55,7},{0x35,6},{0x00,0},{0x00,1},{0x00,2},{0x02,2},{0x02,3},{0x0a,4},{0x06,3} },
/*   CapIR*/{ {0x1f,5},{0x03,3},{0x07,3},{0x07,4},{0x07,5},{0x17,5},{0x17,6},{0x57,7},{0x37,6},{0x0f,4},{0x00,0},{0x00,1},{0x01,1},{0x01,2},{0x05,3},{0x03,2} },
/*     SIR*/{ {0x1f,5},{0x03,3},{0x07,3},{0x07,4},{0x07,5},{0x17,5},{0x17,6},{0x57,7},{0x37,6},{0x0f,4},{0x0f,5},{0x00,0},{0x01,1},{0x01,2},{0x05,3},{0x03,2} },
/*   Ex1IR*/{ {0x0f,4},{0x01,2},{0x03,2},{0x03,3},{0x03,4},{0x0b,4},{0x0b,5},{0x2b,6},{0x1b,5},{0x07,3},{0x07,4},{0x02,3},{0x00,0},{0x00,1},{0x02,2},{0x01,1} },
/*     PIR*/{ {0x1f,5},{0x03,3},{0x07,3},{0x07,4},{0x07,5},{0x17,5},{0x17,6},{0x57,7},{0x37,6},{0x0f,4},{0x0f,5},{0x01,2},{0x05,3},{0x00,0},{0x01,1},{0x03,2} },
/*   Ex2IR*/{ {0x0f,4},{0x01,2},{0x03,2},{0x03,3},{0x03,4},{0x0b,4},{0x0b,5},{0x2b,6},{0x1b,5},{0x07,3},{0x07,4},{0x00,1},{0x02,2},{0x02,3},{0x00,0},{0x01,1} },
/*   UpdIR*/{ {0x07,3},{0x00,1},{0x01,1},{0x01,2},{0x01,3},{0x05,3},{0x05,4},{0x15,5},{0x0d,4},{0x03,2},{0x03,3},{0x03,4},{0x0b,4},{0x0b,5},{0x2b,6},{0x00,0} },
};

static inline u32 ast_jtag_read(struct ast_jtag_info *ast_jtag, u32 reg) {
#ifdef INTEL_JTAG_ADDITIONS
    return ioread32((void * __iomem)ast_jtag->reg_base + reg);
#else
    return readl(ast_jtag->reg_base + reg);
#endif
}

static inline void ast_jtag_write(struct ast_jtag_info *ast_jtag, u32 val, u32 reg) {
#ifdef INTEL_JTAG_ADDITIONS
	iowrite32(val, (void * __iomem)ast_jtag->reg_base + reg);
#else
    writel(val, ast_jtag->reg_base + reg);
#endif
}

#ifndef INTEL_JTAG_ADDITIONS
void ast_jtag_set_freq(struct ast_jtag_info *ast_jtag, unsigned int freq) {
    u16 i;
    for(i = 0; i < 0x7ff; i++) {
        if((ast_get_pclk()/(i + 1) ) <= freq)
            break;
    }
    ast_jtag_write(ast_jtag, ((ast_jtag_read(ast_jtag, AST_JTAG_TCK) & ~JTAG_TCK_DIVISOR_MASK) | i),  AST_JTAG_TCK);
}

unsigned int ast_jtag_get_freq(struct ast_jtag_info *ast_jtag) {

    return ast_get_pclk() / (JTAG_GET_TCK_DIVISOR(ast_jtag_read(ast_jtag, AST_JTAG_TCK)) + 1);
}
#endif

static u8 TCK_Cycle(struct ast_jtag_info *ast_jtag, u8 TMS, u8 TDI) {
    u32 regwriteval = JTAG_SW_MODE_EN | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO);

    // TCK = 0
    ast_jtag_write(ast_jtag, regwriteval, AST_JTAG_SW);

    ast_jtag_read(ast_jtag, AST_JTAG_SW);

    // TCK = 1
    ast_jtag_write(ast_jtag, JTAG_SW_MODE_TCK | regwriteval, AST_JTAG_SW);

    return (ast_jtag_read(ast_jtag, AST_JTAG_SW) & JTAG_SW_MODE_TDIO) ? 1 : 0;
}

#ifndef INTEL_JTAG_ADDITIONS
void ast_jtag_wait_instruction_complete(struct ast_jtag_info *ast_jtag) {
    wait_event_interruptible(ast_jtag->jtag_wq, (ast_jtag->flag == JTAG_INST_COMPLETE));
    ast_jtag->flag = 0;
}

void ast_jtag_wait_data_pause_complete(struct ast_jtag_info *ast_jtag) {
    wait_event_interruptible(ast_jtag->jtag_wq, (ast_jtag->flag == JTAG_DATA_PAUSE));
    ast_jtag->flag = 0;
}

void ast_jtag_wait_data_complete(struct ast_jtag_info *ast_jtag) {
    wait_event_interruptible(ast_jtag->jtag_wq, (ast_jtag->flag == JTAG_DATA_COMPLETE));
    ast_jtag->flag = 0;
}
#endif

/*************************************************************************************/
void ast_jtag_bitbang(struct ast_jtag_info *ast_jtag, struct tck_bitbang *bitbang) {
    bitbang->tdo = TCK_Cycle(ast_jtag, bitbang->tms, bitbang->tdi);
}

/******************************************************************************/
static int ast_jtag_set_tapstate(struct ast_jtag_info *ast_jtag, unsigned int tapstate) {
    unsigned char i;
    unsigned char tmsbits;
    unsigned char count;

    // ensure that the requested and current tap states are within 0 to 15.
    if ((ast_jtag->tapstate >= sizeof(_tmsCycleLookup[0])/sizeof(_tmsCycleLookup[0][0])) ||  // Column
        (tapstate >= sizeof(_tmsCycleLookup)/sizeof _tmsCycleLookup[0])) {  // row
        return -1;
    }

    if (tapstate == JtagTLR) {
        // clear tap state and go back to TLR
        for(i = 0; i < 9; i++) {
            TCK_Cycle(ast_jtag, 1, 0);
        }
        ast_jtag->tapstate = JtagTLR;
        return 0;
    }

    tmsbits = _tmsCycleLookup[ast_jtag->tapstate][tapstate].tmsbits;
    count   = _tmsCycleLookup[ast_jtag->tapstate][tapstate].count;

    if (count == 0) return 0;

    for (i=0; i<count; i++) {
        TCK_Cycle(ast_jtag, (tmsbits & 1), 0);
        tmsbits >>= 1;
    }
    ast_jtag->tapstate = tapstate;
    return 0;
}

/******************************************************************************/
void ast_jtag_readwrite_scan(struct ast_jtag_info *ast_jtag, struct scan_xfer *scan_xfer) {
    unsigned int  bit_index = 0;
    unsigned char* tdi_p = scan_xfer->tdi;
    unsigned char* tdo_p = scan_xfer->tdo;

    if ((ast_jtag->tapstate != JtagShfDR) && (ast_jtag->tapstate != JtagShfIR)) {
        if ((ast_jtag->tapstate >= 0) && (ast_jtag->tapstate < sizeof(c_statestr)/sizeof(c_statestr[0]))) {
            printk(KERN_ERR "readwrite_scan bad current tapstate = %s\n", c_statestr[ast_jtag->tapstate]);
        }
        return;
    }
    if (scan_xfer->length == 0) {
        printk(KERN_ERR "readwrite_scan bad length 0\n");
        return;
    }

    if (scan_xfer->tdi == NULL && scan_xfer->tdi_bytes != 0) {
        printk(KERN_ERR "readwrite_scan null tdi with nonzero length %u!\n", scan_xfer->tdi_bytes);
        return;
    }

    if (scan_xfer->tdo == NULL && scan_xfer->tdo_bytes != 0) {
        printk(KERN_ERR "readwrite_scan null tdo with nonzero length %u!\n", scan_xfer->tdo_bytes);
        return;
    }
    
    while (bit_index < scan_xfer->length) {
        int bit_offset = (bit_index % 8);
        int this_input_bit = 0;
        int tms_high_or_low;
        int this_output_bit;
        if (bit_index / 8 < scan_xfer->tdi_bytes){
            // If we are on a byte boundary, increment the byte pointers
            // Don't increment on 0, pointer is already on the first byte
            if (bit_index % 8 == 0 && bit_index != 0) {
                tdi_p++;
            }
            this_input_bit = (*tdi_p >> bit_offset) & 1;
        }
        // If this is the last bit, leave TMS high
        tms_high_or_low = (bit_index == scan_xfer->length - 1) && (scan_xfer->end_tap_state != JtagShfDR) && (scan_xfer->end_tap_state != JtagShfIR);

        this_output_bit = TCK_Cycle(ast_jtag, tms_high_or_low, this_input_bit);
        // If it was the last bit in the scan and the end_tap_state is something other than shiftDR or shiftIR then go to Exit1.
        // IMPORTANT Note: if the end_tap_state is ShiftIR/DR and the next call to this function is a shiftDR/IR then the driver will not change state!
        if (tms_high_or_low){
            ast_jtag->tapstate = (ast_jtag->tapstate == JtagShfDR) ? JtagEx1DR : JtagEx1IR;
        }
        if (bit_index / 8 < scan_xfer->tdo_bytes){
            if (bit_index % 8 == 0) {
                if (bit_index != 0) {
                    tdo_p++;
                }
                // Zero the output buffer before we write data to it
                *tdo_p = 0;
            }
            *tdo_p |= this_output_bit << bit_offset;
        }
        bit_index++;
    }

    // Go to the requested tap state (may be a NOP, as we may be already in the JtagShf state)
    ast_jtag_set_tapstate(ast_jtag, scan_xfer->end_tap_state);
}

/*************************************************************************************/
#ifndef INTEL_JTAG_ADDITIONS
static irqreturn_t ast_jtag_interrupt (int this_irq, void *dev_id) {
    u32 status;
    struct ast_jtag_info *ast_jtag = dev_id;

    status = ast_jtag_read(ast_jtag, AST_JTAG_ISR);

    if (status & JTAG_INST_COMPLETE) {
        ast_jtag_write(ast_jtag, JTAG_INST_COMPLETE | (status & 0xf), AST_JTAG_ISR);
        ast_jtag->flag = JTAG_INST_COMPLETE;
    }

    if (status & JTAG_DATA_PAUSE) {
        ast_jtag_write(ast_jtag, JTAG_DATA_PAUSE | (status & 0xf), AST_JTAG_ISR);
        ast_jtag->flag = JTAG_DATA_PAUSE;
    }

    if (status & JTAG_DATA_COMPLETE) {
        ast_jtag_write(ast_jtag, JTAG_DATA_COMPLETE | (status & 0xf),AST_JTAG_ISR);
        ast_jtag->flag = JTAG_DATA_COMPLETE;
    }

    if (ast_jtag->flag) {
        wake_up_interruptible(&ast_jtag->jtag_wq);
        return IRQ_HANDLED;
    }
    else {
        printk ("TODO Check JTAG's interrupt %x\n",status);
        return IRQ_NONE;
    }

}
#endif

/*************************************************************************************/
#ifndef INTEL_JTAG_ADDITIONS
static inline void ast_jtag_slave(void) {
    u32 currReg = readl((void *)(ast_scu_base + AST_SCU_RESET));
    // unlock scu
    writel(SCU_PROTECT_UNLOCK, (void *)ast_scu_base);
    writel(currReg | SCU_RESET_JTAG, (void *)(ast_scu_base + AST_SCU_RESET));
    // lock scu
    writel(0xaa,(void *)ast_scu_base);
}

static inline void ast_jtag_master(void) {
    ast_scu_init_jtag();
    ast_jtag_write(ast_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN ,AST_JTAG_CTRL); //Eanble Clock
    ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, AST_JTAG_SW);
    ast_jtag_write(ast_jtag, JTAG_INST_PAUSE | JTAG_INST_COMPLETE |
        JTAG_DATA_PAUSE | JTAG_DATA_COMPLETE |
        JTAG_INST_PAUSE_EN | JTAG_INST_COMPLETE_EN |
        JTAG_DATA_PAUSE_EN | JTAG_DATA_COMPLETE_EN,
        AST_JTAG_ISR);        // Enable Interrupt

    // When leaving Slave mode, we do not know what state the TAP is in. This
    // loop will reset the state to TLR, regardless of the previous unknown
    // state.
    ast_jtag_set_tapstate(ast_jtag, JtagTLR);
}
#endif

#ifndef INTEL_JTAG_ADDITIONS
static long jtag_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
#else
long intel_jtag_ioctl(unsigned int cmd, unsigned long arg) {
#endif
    int ret = 0;
#ifndef INTEL_JTAG_ADDITIONS
    struct ast_jtag_info *ast_jtag = file->private_data;
#endif
    void __user *argp = (void __user *)arg;
    struct tck_bitbang bitbang;
    struct scan_xfer scan_xfer;

    switch (cmd) {
#ifndef INTEL_JTAG_ADDITIONS
        case AST_JTAG_SIOCFREQ:
            if((unsigned int)arg > ast_get_pclk())
                ret = -EFAULT;
            else
                ast_jtag_set_freq(ast_jtag, (unsigned int)arg);
            break;
        case AST_JTAG_GIOCFREQ:
             ret = put_user(ast_jtag_get_freq(ast_jtag), (unsigned int __user *)arg);
            break;
#endif
        case AST_JTAG_BITBANG:
            if (copy_from_user(&bitbang, argp, sizeof(struct tck_bitbang)))
                ret = -EFAULT;
            else
                ast_jtag_bitbang(ast_jtag, &bitbang);
            if (copy_to_user(argp, &bitbang, sizeof(struct tck_bitbang)))
                ret = -EFAULT;
            break;
        case AST_JTAG_SET_TAPSTATE:
            ast_jtag_set_tapstate(ast_jtag, (unsigned int)arg);
            break;
        case AST_JTAG_READWRITESCAN:
            if (copy_from_user(&scan_xfer, argp, sizeof(struct scan_xfer)))
                ret = -EFAULT;
            else
                ast_jtag_readwrite_scan(ast_jtag, &scan_xfer);
            if (copy_to_user(argp, &scan_xfer, sizeof(struct scan_xfer)))
                ret = -EFAULT;
            break;
        case AST_JTAG_SLAVECONTLR:
#ifndef INTEL_JTAG_ADDITIONS
			if (arg)
                ast_jtag_slave();
            else
                ast_jtag_master();
#endif
            break;
        case AST_JTAG_ASD_INIT:
            ret = intel_jtag_init();
            break;
        case AST_JTAG_ASD_DEINIT:
            intel_jtag_exit();
            break;
        default:
            return -ENOTTY;
    }

    return ret;
}

#ifdef INTEL_JTAG_ADDITIONS
int intel_jtag_init(void)
{
	JTAG_DBUG("Intel jtag initializing...\n");

	if (!(ast_jtag = kzalloc(sizeof(struct ast_jtag_info), GFP_KERNEL))) {
		printk(KERN_WARNING "Intel jtag memory allocation init failed (ret=%d)!\n", -ENOMEM);
		return -ENOMEM;
	}

	if (!ast_jtag_v_add) {
		printk(KERN_WARNING "No JTAG base address set, driver init failed (ret=%d)!\n", -EIO);
		return -EIO;
	}

	ast_jtag->reg_base = (void __iomem*)ast_jtag_v_add;
    ast_jtag->prevAstJtagCtrl = ast_jtag_read(ast_jtag, AST_JTAG_CTRL);
    ast_jtag->prevAstJtagSw = ast_jtag_read(ast_jtag, AST_JTAG_SW);
    ast_jtag_write(ast_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN, AST_JTAG_CTRL); //Eanble Clock
	ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, AST_JTAG_SW);

	ast_jtag->flag = 0;

	TCK_Cycle(ast_jtag, 1, 0);
	TCK_Cycle(ast_jtag, 1, 0);
	TCK_Cycle(ast_jtag, 1, 0);
	TCK_Cycle(ast_jtag, 1, 0);
	TCK_Cycle(ast_jtag, 1, 0);
	TCK_Cycle(ast_jtag, 1, 0);
	TCK_Cycle(ast_jtag, 1, 0);
	TCK_Cycle(ast_jtag, 1, 0);
	TCK_Cycle(ast_jtag, 1, 0);

	ast_jtag->tapstate = JtagTLR;

	printk("Intel JTAG initialized successfully!\n");

	return 0;
}

void intel_jtag_exit(void)
{
    if (!ast_jtag) {
        ast_jtag_write(ast_jtag, ast_jtag->prevAstJtagCtrl, AST_JTAG_CTRL);
        ast_jtag_write(ast_jtag, ast_jtag->prevAstJtagSw, AST_JTAG_SW);
        kfree(ast_jtag);
    }
}
#else 
static int jtag_open(struct inode *inode, struct file *file) {
    spin_lock(&jtag_state_lock);
    if (ast_jtag->is_open) {
        spin_unlock(&jtag_state_lock);
        return -EBUSY;
    }

    ast_jtag->is_open = true;
    file->private_data = ast_jtag;

    spin_unlock(&jtag_state_lock);

    return 0;
}

static int jtag_release(struct inode *inode, struct file *file) {
    struct ast_jtag_info *drvdata = file->private_data;

    spin_lock(&jtag_state_lock);

    drvdata->is_open = false;

    spin_unlock(&jtag_state_lock);

    return 0;
}

static ssize_t show_tdo(struct device *dev,
    struct device_attribute *attr, char *buf) {
    struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", ast_jtag_read(ast_jtag, AST_JTAG_SW) & JTAG_SW_MODE_TDIO? "1":"0");
}

static DEVICE_ATTR(tdo, S_IRUGO, show_tdo, NULL);

static ssize_t store_tdi(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count) {
    u32 tdi;
    struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

    tdi = simple_strtoul(buf, NULL, 1);

    ast_jtag_write(ast_jtag, ast_jtag_read(ast_jtag, AST_JTAG_SW) | JTAG_SW_MODE_EN | (tdi * JTAG_SW_MODE_TDIO), AST_JTAG_SW);

    return count;
}

static DEVICE_ATTR(tdi, S_IWUSR, NULL, store_tdi);

static ssize_t store_tms(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count) {
    u32 tms;
    struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

    tms = simple_strtoul(buf, NULL, 1);

    ast_jtag_write(ast_jtag, ast_jtag_read(ast_jtag, AST_JTAG_SW) | JTAG_SW_MODE_EN | (tms * JTAG_SW_MODE_TMS), AST_JTAG_SW);

    return count;
}

static DEVICE_ATTR(tms, S_IWUSR, NULL, store_tms);

static ssize_t store_tck(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count) {
    u32 tck;
    struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

    tck = simple_strtoul(buf, NULL, 1);

    ast_jtag_write(ast_jtag, ast_jtag_read(ast_jtag, AST_JTAG_SW) | JTAG_SW_MODE_EN | (tck * JTAG_SW_MODE_TDIO), AST_JTAG_SW);

    return count;
}

static DEVICE_ATTR(tck, S_IWUSR, NULL, store_tck);

static ssize_t show_sts(struct device *dev,
    struct device_attribute *attr, char *buf) {
    struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);
    if ((ast_jtag->tapstate >= 0) && (ast_jtag->tapstate < sizeof(c_statestr)/sizeof(c_statestr[0]))) {
        return sprintf(buf, "%s\n", c_statestr[ast_jtag->tapstate]);
    } else {
        return sprintf(buf, "ERROR\n");
    }
}

static DEVICE_ATTR(sts, S_IRUGO, show_sts, NULL);

static ssize_t show_frequency(struct device *dev,
    struct device_attribute *attr, char *buf) {
    struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);
    return sprintf(buf, "Frequency : %d\n", ast_get_pclk() / (JTAG_GET_TCK_DIVISOR(ast_jtag_read(ast_jtag, AST_JTAG_TCK)) + 1));
}

static ssize_t store_frequency(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count) {
    u32 val;
    struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

    val = simple_strtoul(buf, NULL, 10);
    ast_jtag_set_freq(ast_jtag, val);

    return count;
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, show_frequency, store_frequency);

static struct attribute *jtag_sysfs_entries[] = {
    &dev_attr_freq.attr,
    &dev_attr_sts.attr,
    &dev_attr_tck.attr,
    &dev_attr_tms.attr,
    &dev_attr_tdi.attr,
    &dev_attr_tdo.attr,
    NULL
};

static struct attribute_group jtag_attribute_group = {
    .attrs = jtag_sysfs_entries,
};

static const struct file_operations ast_jtag_fops = {
    .owner             = THIS_MODULE,
    .unlocked_ioctl    = jtag_ioctl,
    .open              = jtag_open,
    .release           = jtag_release,
};

static int ast_jtag_probe(struct platform_device *pdev) {
    struct resource *res;
    int ret=0;

    JTAG_DBUG("ast_jtag_probe load started\n");

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (NULL == res) {
        dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
        ret = -ENOENT;
        goto out;
    }

    if (!request_mem_region(res->start, resource_size(res), res->name)) {
        dev_err(&pdev->dev, "cannot reserved region\n");
        ret = -ENXIO;
        goto out;
    }

    if (!(ast_jtag = kzalloc(sizeof(struct ast_jtag_info), GFP_KERNEL))) {
        return -ENOMEM;
    }

    ast_jtag->reg_base = ioremap(res->start, resource_size(res));
    if (!ast_jtag->reg_base) {
        ret = -EIO;
        goto out_region;
    }

    ast_jtag->irq = platform_get_irq(pdev, 0);
    if (ast_jtag->irq < 0) {
        dev_err(&pdev->dev, "no irq specified\n");
        ret = -ENOENT;
        goto out_region;
    }

    ret = request_irq(ast_jtag->irq, ast_jtag_interrupt, IRQF_DISABLED, "ast-jtag", ast_jtag);
    if (ret) {
        printk("JTAG Unable to get IRQ");
        goto out_region;
    }

    ast_jtag->flag = 0;
    init_waitqueue_head(&ast_jtag->jtag_wq);

    cdev_init(&ast_jtag->cdev, &ast_jtag_fops);
    ast_jtag->cdev.owner = THIS_MODULE;
    ret = cdev_add(&ast_jtag->cdev, MKDEV(JTAG_DEV_MAJOR, 0), 1);
    if (ret){
        printk(KERN_ERR "JTAG : Can't register dev\n");
        goto out_irq;
    }

    platform_set_drvdata(pdev, ast_jtag);

    ret = sysfs_create_group(&pdev->dev.kobj, &jtag_attribute_group);
    if (ret) {
        printk(KERN_ERR "ast_jtag: failed to create sysfs device attributes.\n");
        return -1;
    }

    ast_jtag_slave();

    printk("JTAG driver loaded successfully!\n");

    return 0;

out_irq:
    free_irq(ast_jtag->irq, NULL);
out_region:
    release_mem_region(res->start, res->end - res->start + 1);
out:
    printk(KERN_WARNING "ast_jtag: driver init failed (ret=%d)!\n", ret);
    return ret;
}

static int ast_jtag_remove(struct platform_device *pdev) {
    struct resource *res;
    struct ast_jtag_info *ast_jtag = platform_get_drvdata(pdev);

    if (ast_jtag == NULL)
        return 0;

    JTAG_DBUG("ast_jtag_remove\n");

    sysfs_remove_group(&pdev->dev.kobj, &jtag_attribute_group);
    cdev_del(&ast_jtag->cdev);

    free_irq(ast_jtag->irq, ast_jtag);

    iounmap(ast_jtag->reg_base);
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res != NULL)
        release_mem_region(res->start, res->end - res->start + 1);

    platform_set_drvdata(pdev, NULL);

    if (res != NULL)
        release_mem_region(res->start, res->end - res->start + 1);

    kfree(ast_jtag);

    ast_jtag_slave();

    printk("JTAG driver removed successfully!\n");

    return 0;
}

#ifdef CONFIG_PM
static int
ast_jtag_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int
ast_jtag_resume(struct platform_device *pdev)
{
    return 0;
}

#else
#define ast_jtag_suspend        NULL
#define ast_jtag_resume         NULL
#endif

static struct platform_driver ast_jtag_driver = {
    .remove         = ast_jtag_remove,
    .suspend        = ast_jtag_suspend,
    .resume         = ast_jtag_resume,
    .driver         = {
        .name   = "ast-jtag",
        .owner  = THIS_MODULE,
    },
};

module_platform_driver_probe(ast_jtag_driver, ast_jtag_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST JTAG LIB Driver");
MODULE_LICENSE("GPL");
#endif
