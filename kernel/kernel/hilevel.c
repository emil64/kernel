/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

/* We assume there will be two user processes, stemming from execution of the
 * two user programs P1 and P2, and can therefore
 *
 * - allocate a fixed-size process table (of PCBs), and then maintain an index
 *   into it to keep track of the currently executing process, and
 * - employ a fixed-case of round-robin scheduling: no more processes can be
 *   created, and neither can be terminated, so assume both are always ready
 *   to execute.
 */

pcb_t procTab[ MAX_PROCS ]; pcb_t* executing = NULL;

void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid = '?', next_pid = '?';

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

    PL011_putc( UART0, '[',      true );
    PL011_putc( UART0, prev_pid, true );
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    PL011_putc( UART0, next_pid, true );
    PL011_putc( UART0, ']',      true );

    executing = next;                           // update   executing process to P_{next}

  return;
}

void schedule( ctx_t* ctx ) {
 /* if     ( executing->pid == procTab[ 0 ].pid ) {
    dispatch( ctx, &procTab[ 0 ], &procTab[ 1 ] );  // context switch P_1 -> P_2

    procTab[ 0 ].status = STATUS_READY;             // update   execution status  of P_1
    procTab[ 1 ].status = STATUS_EXECUTING;         // update   execution status  of P_2
  }
  else if( executing->pid == procTab[ 1 ].pid ) {
    dispatch( ctx, &procTab[ 1 ], &procTab[ 2 ] );  // context switch P_2 -> P_1

    procTab[ 1 ].status = STATUS_READY;             // update   execution status  of P_2
    procTab[ 2 ].status = STATUS_EXECUTING;         // update   execution status  of P_1
  }
  else if( executing->pid == procTab[ 2 ].pid ) {
    dispatch( ctx, &procTab[ 2 ], &procTab[ 0 ] );  // context switch P_2 -> P_1

    procTab[ 2 ].status = STATUS_READY;             // update   execution status  of P_2
    procTab[ 0 ].status = STATUS_EXECUTING;         // update   execution status  of P_1
  }
*/

  int min_priority = 150;
  int next = 0;
  for( int i = 0; i < MAX_PROCS; i++ ) {
    int pr = procTab[ i ].priority + procTab[ i ].niceness + procTab[ i ].age;
    if ( pr <= min_priority && (procTab[ i ].status == STATUS_READY || procTab[ i ].status == STATUS_EXECUTING)){
      next = i;
      min_priority = pr;
    }
  }

  for( int i = 0; i < MAX_PROCS; i++ ) {
    procTab[ i ].age -= 2;
  }
  procTab[ next ].age = 0;

  pcb_t *prev_p = executing;
  dispatch( ctx, prev_p, &procTab[ next ] );  // context switc0h
  procTab[ prev_p->pid ].status = STATUS_READY;
  procTab[ next ].status = STATUS_EXECUTING;
  procTab[ next ].cpu_time += 1;

  return;
}

extern void     main_P1();
extern uint32_t tos_P1;
extern void     main_P2();
extern uint32_t tos_P2;
extern void     main_P3();
extern uint32_t tos_P3;

void hilevel_handler_rst( ctx_t* ctx              ) {

 TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
 TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
 TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
 TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
 TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

 GICC0->PMR          = 0x000000F0; // unmask all            interrupts
 GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
 GICC0->CTLR         = 0x00000001; // enable GIC interface
 GICD0->CTLR         = 0x00000001; // enable GIC distributor

 int_enable_irq();

  /* Invalidate all entries in the process table, so it's clear they are not
   * representing valid (i.e., active) processes.
   */

  for( int i = 0; i < MAX_PROCS; i++ ) {
    procTab[ i ].status = STATUS_INVALID;
  }

  /* Automatically execute the user programs P1 and P2 by setting the fields
   * in two associated PCBs.  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR mode,
   *   with IRQ interrupts enabled, and
   * - the PC and SP values match the entry point and top of stack.
   */

  memset( &procTab[ 0 ], 0, sizeof( pcb_t ) ); // initialise 0-th PCB = P_1
  procTab[ 0 ].pid      = 1;
  procTab[ 0 ].age      = 0;
  procTab[ 0 ].priority = 80;
  procTab[ 0 ].niceness = 0;
  procTab[ 0 ].cpu_time = 0.0f;
  procTab[ 0 ].status   = STATUS_READY;
  procTab[ 0 ].tos      = ( uint32_t )( &tos_P1  );
  procTab[ 0 ].ctx.cpsr = 0x50;
  procTab[ 0 ].ctx.pc   = ( uint32_t )( &main_P1 );
  procTab[ 0 ].ctx.sp   = procTab[ 0 ].tos;

  memset( &procTab[ 1 ], 0, sizeof( pcb_t ) ); // initialise 1-st PCB = P_2
  procTab[ 1 ].pid      = 2;
  procTab[ 1 ].age      = 0;
  procTab[ 1 ].priority = 80;
  procTab[ 1 ].niceness = -10;
  procTab[ 1 ].cpu_time = 0.0f;
  procTab[ 1 ].status   = STATUS_READY;
  procTab[ 1 ].tos      = ( uint32_t )( &tos_P2  );
  procTab[ 1 ].ctx.cpsr = 0x50;
  procTab[ 1 ].ctx.pc   = ( uint32_t )( &main_P2 );
  procTab[ 1 ].ctx.sp   = procTab[ 1 ].tos;

  memset( &procTab[ 2 ], 0, sizeof( pcb_t ) ); // initialise 1-st PCB = P_2
  procTab[ 2 ].pid      = 3;
  procTab[ 2 ].age      = 0;
  procTab[ 2 ].priority = 80;
  procTab[ 2 ].niceness = 0;
  procTab[ 3 ].cpu_time = 0.0f;
  procTab[ 2 ].status   = STATUS_READY;
  procTab[ 2 ].tos      = ( uint32_t )( &tos_P3  );
  procTab[ 2 ].ctx.cpsr = 0x50;
  procTab[ 2 ].ctx.pc   = ( uint32_t )( &main_P3 );
  procTab[ 2 ].ctx.sp   = procTab[ 2 ].tos;

  /* Once the PCBs are initialised, we arbitrarily select the 0-th PCB to be
   * executed: there is no need to preserve the execution context, since it
   * is invalid on reset (i.e., no process was previously executing).
   */

  dispatch( ctx, NULL, &procTab[ 0 ] );

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {

  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    schedule( ctx );
    //PL011_putc( UART0, 'T', true );
    TIMER0->Timer1IntClr = 0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identifier (i.e., the immediate operand) extracted from the
   * svc instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call, then
   * - write any return value back to preserved usr mode registers.
   */

  switch( id ) {
    case 0x00 : { // 0x00 => yield()
      schedule( ctx );

      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

      ctx->gpr[ 0 ] = n;

      break;
    }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}
