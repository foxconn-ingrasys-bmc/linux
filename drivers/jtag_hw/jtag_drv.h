/********************************************************************************
*
* File Name:    jtag_drv_h
* Description:  JTAG driver additional header
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
#ifndef __JTAG_DRV_H__
#define __JTAG_DRV_H__

#ifndef INTEL_JTAG_ADDITIONS
struct tck_bitbang {
    unsigned char     tms;
    unsigned char     tdi;        // TDI bit value to write
    unsigned char     tdo;        // TDO bit value to read
};

struct scan_xfer {
    unsigned int     length;      // number of bits to clock
    unsigned char    *tdi;        // data to write to tap (optional)
    unsigned int     tdi_bytes;
    unsigned char    *tdo;        // data to read from tap (optional)
    unsigned int     tdo_bytes;
    unsigned int     end_tap_state;
};
#endif

typedef enum {
    JtagTLR,
    JtagRTI,
    JtagSelDR,
    JtagCapDR,
    JtagShfDR,
    JtagEx1DR,
    JtagPauDR,
    JtagEx2DR,
    JtagUpdDR,
    JtagSelIR,
    JtagCapIR,
    JtagShfIR,
    JtagEx1IR,
    JtagPauIR,
    JtagEx2IR,
    JtagUpdIR
} JtagStates;

#ifndef INTEL_JTAG_ADDITIONS
#define JTAGIOC_BASE    'T'

#define AST_JTAG_SIOCFREQ           _IOW( JTAGIOC_BASE, 3, unsigned int)
#define AST_JTAG_GIOCFREQ           _IOR( JTAGIOC_BASE, 4, unsigned int)
#define AST_JTAG_BITBANG            _IOWR(JTAGIOC_BASE, 5, struct tck_bitbang)
#define AST_JTAG_SET_TAPSTATE       _IOW( JTAGIOC_BASE, 6, unsigned int)
#define AST_JTAG_READWRITESCAN      _IOWR(JTAGIOC_BASE, 7, struct scan_xfer)
#define AST_JTAG_SLAVECONTLR        _IOW( JTAGIOC_BASE, 8, unsigned int)
#define AST_JTAG_ASD_INIT           _IOW( JTAGIOC_BASE, 9, unsigned int)
#define AST_JTAG_ASD_DEINIT         _IOW( JTAGIOC_BASE, 10, unsigned int)
#endif

#endif
