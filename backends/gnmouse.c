/*
 * QEMU Genius GM-6000 serial mouse emulation
 *
 * Adapted from msmouse
 *
 * Copyright (c) 2012 Naour Romain (romain.naour@openwide.fr)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../qemu-common.h"
#include "../qemu-char.h"
#include "../console.h"
#include "gnmouse.h"
#include "qemu-timer.h"

//#define DEBUG_GENIUS_MOUSE

#ifdef DEBUG_GENIUS_MOUSE
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "gnmouse: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) \
do {} while (0)
#endif

/* 
 * This define is used for slow down the speed of the mouse.
 * dx and dy values are dropped two out of three time when GENIUS_MOUSE_DROP = 3.
 */
#define GENIUS_MOUSE_DROP 3

/*
 * struct gnmouse_save:
 * This structure is used to save private info for Genius mouse.
 * 
 * dx: deltas on x-axis saved since last frame send to emulated system.
 * dy: deltas on y-axis saved since last frame send to emulated system.
 * transmit_timer: Qemu's timer
 * transmit_time: reload value for transmit_timer
 * data: frame to be sent
 * index: used to save current state of the state machine. see type states below
 */
typedef struct gnmouse_save{
  int dx;
  int dy;
  int button;
  struct QEMUTimer *transmit_timer;	/* Qemu timer */
  uint64_t transmit_time;		/* time to transmit a char in ticks*/
  unsigned char data[5];
  int index;
} gnmouse_save;


/* states  */
typedef enum
{
  START,  /* 0 */
  CHAR_1, /* 1 : BP */
  CHAR_2, /* 2 : Dx */
  CHAR_3, /* 3 : Dy */
  CHAR_4, /* 4 : Dx */
  CHAR_5, /* 5 : Dy */
  STOP    /* 6 */
}
states;

/**
 * gnmouse_chr_write: this function is used when Qemu 
 * try to write something to mouse port.
 * Nothing is send to the emulated mouse.
 * 
 * Return: lengh of the buffer
 * 
 * @s: address of the CharDriverState used by the mouse
 * @buf: buffer to write
 * @len: lengh of the buffer to write
 */
static int gnmouse_chr_write (struct CharDriverState *s, const uint8_t *buf, int len)
{
    /* Ignore writes to mouse port */
    return len;
}

/**
 * gnmouse_chr_close: this function close the mouse port.
 * It stop and free the Qemu's timer and free gnmouse_save struct.
 * 
 * Return: void
 * 
 * @chr: address of the CharDriverState used by the mouse
 */
static void gnmouse_chr_close (struct CharDriverState *chr)
{
    /* stop and free the Qemu's timer */
    qemu_del_timer( ((gnmouse_save *)chr->opaque)->transmit_timer);
    qemu_free_timer(((gnmouse_save *)chr->opaque)->transmit_timer);
    /* free gnmouse_save struct */
    g_free(chr->opaque);
    g_free (chr);
}

/**
 * gnmouse_handler: send a byte on serial port to the guest system
 * This handler is called on each timer timeout or directly by gnmouse_event()
 * when no transmission is underway.
 * It use a state machine in order to know with byte of the frame must be send.
 * 
 * Returns void
 * 
 * @opaque: address of the CharDriverState used by the mouse
 */
static void gnmouse_handler (void *opaque)
{
    CharDriverState *chr = (CharDriverState *)opaque;
    gnmouse_save *save = (gnmouse_save *)chr->opaque;
    unsigned char *data = save->data;
    int dx_tmp, dy_tmp;
/* 
 * Byte 0:  1,  0,  0,  0,  0,  L,  M,  R
 * Byte 1: X7, X6, X5, X4, X3, X2, X1, X0
 * Byte 2: Y7, Y6, Y5, Y4, Y3, Y2, Y1, Y0
 * Byte 3: X7, X6, X5, X4, X3, X2, X1, X0
 * Byte 4: Y7, Y6, Y5, Y4, Y3, Y2, Y1, Y0
 */
    switch (save->index)
    {
	case CHAR_4:
	  if( (save->dx && save->dy) ){
	    if(save->dx >= 128){
	      DPRINTF("overflow dx= %d\n", save->dx);
	      save->dx -=128;
	      dx_tmp = 128;
	    }else if(save->dx <= -127){
	      DPRINTF("overflow dx= %d\n", save->dx);
	      save->dx +=127;
	      dx_tmp = -127;
	    }else{
	      dx_tmp = save->dx;
	      save->dx = 0;
	    }

	    if(save->dy >= 128){
	      DPRINTF("overflow dy= %d\n", save->dy);
	      save->dy -=128;
	      dy_tmp = 128;
	    }else if(save->dy <= -127){
	      DPRINTF("overflow dy= %d\n", save->dy);
	      save->dy +=127;
	      dy_tmp = -127;
	    }else{
	      dy_tmp = save->dy;
	      save->dy = 0;
	    }

	    DPRINTF("dx= %d\n", save->dx);
	    DPRINTF("dy= %d\n", save->dy);

	    data[3] = dx_tmp;
	    data[4] = -(dy_tmp);

	  }
	  break;

	case STOP:
	  if( !(save->dx && save->dy) ){
	    /* no more data */
	    DPRINTF("no more data\n");
	    return;
	  }else{
	    /* data saved */
	    DPRINTF("data saved\n");
	    save->index = START;
	  }
	  /* No break, pass-through START */
	case START:
	  /* New serial frame */
	  /* Buttons */
	  data[0] = save->button;

	  save->index = CHAR_1;

	  /* No break, pass-through CHAR_1 */
	case CHAR_1:
	  /* avoid overflow on dx or dy */
	  if(save->dx >= 128){
	    DPRINTF("overflow dx= %d\n", save->dx);
	    save->dx -=128;
	    dx_tmp = 128;
	  }else if(save->dx <= -127){
	    DPRINTF("overflow dx= %d\n", save->dx);
	    save->dx +=127;
	    dx_tmp = -127;
	  }else{
	    dx_tmp = save->dx;
	    save->dx = 0;
	  }

	  if(save->dy >= 128){
	    DPRINTF("overflow dy= %d\n", save->dy);
	    save->dy -=128;
	    dy_tmp = 128;
	  }else if(save->dy <= -127){
	    DPRINTF("overflow dy= %d\n", save->dy);
	    save->dy +=127;
	    dy_tmp = -127;
	  }else{
	    dy_tmp = save->dy;
	    save->dy = 0;
	  }

	  DPRINTF("dx= %d\n", save->dx);
	  DPRINTF("dy= %d\n", save->dy);

	  /* Movement deltas */
	  data[1] = dx_tmp;
	  data[2] = -(dy_tmp);
	  data[3] = 0;
	  data[4] = 0;

	case CHAR_2:
	case CHAR_3:
	case CHAR_5:
	  break;
	default:
	  return;
    }

    /* reload timer */
    qemu_mod_timer(save->transmit_timer, qemu_get_clock_ns(vm_clock) + save->transmit_time);
    DPRINTF("mod_timer: %d\n", save->index);
    /* write date on serial port */
    qemu_chr_be_write(chr, &(data[save->index - 1]) , 1);
    DPRINTF("write :%x\n", data[save->index - 1] );
    /* next state */
    save->index += 1;
}

/**
 * gnmouse_event: event handler called by SDL functions 
 * on each mouse movement or button press.
 * 
 * Return void
 * 
 * @opaque: address of the CharDriverState used by the mouse 
 * @dx: deltas on the x-axis since last event
 * @dy: deltas on the y-axis since last event
 * @dz: deltas on the z-axis since last event (not used)
 * @button_state: status of mouse button
 */
static void gnmouse_event(void *opaque,
                          int dx, int dy, int dz, int buttons_state)
{
    CharDriverState *chr = (CharDriverState *)opaque;
    gnmouse_save *save = (gnmouse_save *)chr->opaque;
    char BP = 0x80;
    static int drop = 1;
    
    /* slow down the speed of the mouse */
    if(drop % GENIUS_MOUSE_DROP){
      drop += 1;
      /* drop deltas */
      save->dx +=  0;
      save->dy +=  0;
    }else{
      drop = 1;
      /* save deltas */
      save->dx +=  dx;
      save->dy +=  dy;
    }

    DPRINTF("dx= %d; dy= %d; buttons=%x\n", dx, dy, buttons_state);

    /* Buttons */
    BP |= (buttons_state & 0x01 ? 0x00 : 0x04); // BP1 = L
    BP |= (buttons_state & 0x02 ? 0x00 : 0x01); // BP2 = R
    BP |= (buttons_state & 0x04 ? 0x00 : 0x02); // BP4 = M

    save->button = BP;
    if(save->index == STOP){
      /* no transmission is underway, start a new transmission */
      save->index = START;
      gnmouse_handler( (void*) chr);
    }
}

/**
 * qemu_chr_open_gnmouse: Init function for Genius mouse
 * allocate a gnmouse_save structure to save data used by gnmouse emulation.
 * allocate a new CharDriverState.
 * create a new Qemu's timer with gnmouse_handler() as timeout handler.
 * calculate the transmit_time for 1200 bauds transmission.
 * 
 * Return address of the initialized CharDriverState
 * 
 * @opts: argument not used
 */
CharDriverState *qemu_chr_open_gnmouse (QemuOpts *opts)
{
    CharDriverState *chr;
    gnmouse_save * save;

    DPRINTF("qemu_chr_open_gnmouse\n");

    /* allocate CharDriverState and gnmouse_save */
    chr = g_malloc0(sizeof(CharDriverState));
    save = g_malloc0(sizeof(gnmouse_save));

    chr->chr_write = gnmouse_chr_write;
    chr->chr_close = gnmouse_chr_close;

    /* create a new Qemu's timer with gnmouse_handler() as timeout handler. */
    save->transmit_timer = qemu_new_timer_ns(vm_clock, (QEMUTimerCB *) gnmouse_handler, chr);
    /* calculate the transmit_time for 1200 bauds transmission */
    save->transmit_time = (get_ticks_per_sec() / 1200) * 10; /* 1200 bauds */
    
    DPRINTF("transmit_time = %lld\n", save->transmit_time);
    
    /* init state machine */
    save->index = STOP;
    
    /* keep address of gnmouse_save */
    chr->opaque = save;

    qemu_add_mouse_event_handler(gnmouse_event, chr, 0, "QEMU Genius GM-6000 Mouse");
    
    return chr;
}
