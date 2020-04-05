#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-narrowing-conversions"

#include "hilevel.h"


pcb_t procTab[MAX_PROCS];
pcb_t *executing = NULL;
int n_pcb, n_pid;
uint32_t stack_offset = 0x2000;
fildes_t fdTab[128];

void try_read(int pcb);
void try_write(int pcb);

void dispatch(ctx_t *ctx, pcb_t *prev, pcb_t *next) {
    char prev_pid = '?', next_pid = '?';

    if (NULL != prev) {
        memcpy(&prev->ctx, ctx, sizeof(ctx_t)); // preserve execution context of P_{prev}
        prev_pid = '0' + prev->pid;
    }
    if (NULL != next) {
        memcpy(ctx, &next->ctx, sizeof(ctx_t)); // restore  execution context of P_{next}
        next_pid = '0' + next->pid;
    }

    PL011_putc(UART0, '[', true);
    PL011_putc(UART0, prev_pid, true);
    PL011_putc(UART0, '-', true);
    PL011_putc(UART0, '>', true);
    PL011_putc(UART0, next_pid, true);
    PL011_putc(UART0, ']', true);

    executing = next;
}

void schedule(ctx_t *ctx) {

    for (int i = 0; i < n_pcb; i++){
        if(procTab[i].status == STATUS_WAITING_READ)
            try_read(i);
        if(procTab[i].status == STATUS_WAITING_WRITE)
            try_write(i);
    }

    int min_priority = 150;
    int next = 0;
    for (int i = 0; i < n_pcb; i++) {
        int pr = procTab[i].priority + procTab[i].niceness + procTab[i].age;
        if (pr <= min_priority &&
            (procTab[i].status == STATUS_READY || procTab[i].status == STATUS_EXECUTING)) {

            //PL011_putc( UART0, '0'+i,      true );
            next = i;
            min_priority = pr;
        }
    }

    for (int i = 0; i < n_pcb; i++) {
        procTab[i].age -= 2;
    }
    procTab[next].age = 0;

    pcb_t *prev_p = executing;
    dispatch(ctx, prev_p, &procTab[next]);  // context switch
    if (prev_p->status == STATUS_EXECUTING)
        prev_p->status = STATUS_READY;
    procTab[next].status = STATUS_EXECUTING;
    procTab[next].cpu_time += 1;
}

extern void main_console();

extern uint32_t tos_console;
extern uint32_t tos_general;

void hilevel_handler_rst(ctx_t *ctx) {

    TIMER0->Timer1Load = 0x00100000; // select period = 2^20 ticks ~= 1 sec
    TIMER0->Timer1Ctrl = 0x00000002; // select 32-bit   timer
    TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
    TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
    TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

    GICC0->PMR = 0x000000F0; // unmask all            interrupts
    GICD0->ISENABLER1 |= 0x00000010; // enable timer          interrupt
    GICC0->CTLR = 0x00000001; // enable GIC interface
    GICD0->CTLR = 0x00000001; // enable GIC distributor

    int_enable_irq();

    /* Invalidate all entries in the process table, so it's clear they are not
     * representing valid (i.e., active) processes.
     */

    for (int i = 0; i < MAX_PROCS; i++) {
        procTab[i].status = STATUS_CREATED;
    }

    //initialise file descriptor table

    for (int i = 0; i < 128; i++) {
        fdTab[i].access = FREE;
    }

    fdTab[0].access = READ;   //STDIN_FILENO
    fdTab[1].access = WRITE;  //STDOUT_FILENO
    fdTab[2].access = WRITE;  //STDERR_FILENO

    memset(&procTab[0], 0, sizeof(pcb_t)); // initialise 0-th PCB = console
    procTab[0].pid = 1;
    procTab[0].age = 0;
    procTab[0].priority = 80;
    procTab[0].niceness = 0;
    procTab[0].cpu_time = 0.0f;
    procTab[0].status = STATUS_READY;
    procTab[0].tos = (uint32_t) (&tos_console);
    procTab[0].ctx.cpsr = 0x50;
    procTab[0].ctx.pc = (uint32_t) (&main_console);
    procTab[0].ctx.sp = procTab[0].tos;

    n_pcb = 1;
    n_pid = 1;

    dispatch(ctx, NULL, &procTab[0]);

}

void hilevel_handler_irq(ctx_t *ctx) {

    // Step 2: read  the interrupt identifier so we know the source.

    uint32_t id = GICC0->IAR;

    // Step 4: handle the interrupt, then clear (or reset) the source.

    if (id == GIC_SOURCE_TIMER0) {
        schedule(ctx);
        //PL011_putc( UART0, 'T', true );
        TIMER0->Timer1IntClr = 0x01;
    }

    // Step 5: write the interrupt identifier to signal we're done.

    GICC0->EOIR = id;

}

void sys_write(ctx_t *ctx){
    int fd = (int) (ctx->gpr[0]);
    char *x = (char *) (ctx->gpr[1]);
    int n = (int) (ctx->gpr[2]);

    if (fd == 1) { //STDOUT_FILENO
        for (int i = 0; i < n; i++) {
            PL011_putc(UART0, *x++, true);
        }
        ctx->gpr[0] = n;

    } else if (fdTab[fd].access == WRITE) {

        pipe_t *p = fdTab[fd].pipe;

        if(p->size != 0){
            executing->status = STATUS_WAITING_WRITE;
            schedule(ctx);
            return;
        }

        //start writing to pipe
        for (int i = 0; i < n; i++, x++) {
            p->buffer[(p->size)] = *x;
            p->size++;
        }
        ctx->gpr[0] = n;
    } else {
        ctx->gpr[0] = -1;  //file descriptor not meant for writing
    }
}

void sys_read(ctx_t *ctx){
    int fd = (int) (ctx->gpr[0]);
    char *x = (char *) (ctx->gpr[1]);
    int n = (int) (ctx->gpr[2]);

    if(fdTab[fd].access == READ){
        pipe_t *p = fdTab[fd].pipe;

        if(p->size == 0){
            executing->status = STATUS_WAITING_READ;
            schedule(ctx);
            return;
        }

        //start reading from the pipe
        for(int i =0; i< p->size; i++){
            *(x+i) = p->buffer[i];
        }

        ctx->gpr[0] = p->size;
        p->size = 0;
    }
}

void sys_fork(ctx_t *ctx){
    n_pid++;
    PL011_putc(UART0, '\n', true);
    PL011_putc(UART0, 'F', true);
    PL011_putc(UART0, 'O', true);
    PL011_putc(UART0, 'R', true);
    PL011_putc(UART0, 'K', true);
    PL011_putc(UART0, ' ', true);
    PL011_putc(UART0, '0' + n_pid, true);
    PL011_putc(UART0, '\n', true);
    int gap = -1;
    for (int i = 0; i < n_pcb; i++)
        if (procTab[i].status == STATUS_TERMINATED) {
            gap = i;
            break;
        }


    if (gap == -1) {
        gap = n_pcb;
        n_pcb++;
        memset(&procTab[gap], 0, sizeof(pcb_t));

        //allocate stack for the new process
        procTab[gap].tos = (uint32_t) &tos_general - (n_pcb - 1) * stack_offset;
    }

    // initialise process
    procTab[gap].pid = n_pid;
    procTab[gap].age = 0;
    procTab[gap].priority = 80;
    procTab[gap].niceness = 0;
    procTab[gap].cpu_time = 0.0f;
    procTab[gap].status = STATUS_READY;

    //replicate state
    memcpy(&procTab[gap].ctx, ctx, sizeof(ctx_t));

    //copy stack from the parent
    uint32_t size = executing->tos - (uint32_t) ctx->sp;
    procTab[gap].ctx.sp = procTab[gap].tos - size;
    memcpy((uint32_t *) (procTab[gap].ctx.sp), (uint32_t *) ctx->sp, size);

    //set return values
    ctx->gpr[0] = n_pid;           //parent
    procTab[gap].ctx.gpr[0] = 0;   //child

}

void sys_exit(ctx_t *ctx){
    int signal = ctx->gpr[0];
    executing->status = STATUS_TERMINATED;
    schedule(ctx);
}

void sys_exec(ctx_t *ctx){
    ctx->pc = ctx->gpr[0];
    ctx->sp = executing->tos;
}

void sys_kill(ctx_t *ctx){
    int pid = ctx->gpr[0];

    PL011_putc(UART0, '\n', true);
    PL011_putc(UART0, 'K', true);
    PL011_putc(UART0, 'I', true);
    PL011_putc(UART0, 'L', true);
    PL011_putc(UART0, 'L', true);
    PL011_putc(UART0, ' ', true);
    PL011_putc(UART0, '0' + pid, true);
    PL011_putc(UART0, '\n', true);
    //pcb_t *process = NULL;

    int i;
    for (i = 0; i < n_pcb; i++) {
        if (pid == procTab[i].pid && procTab[i].status != STATUS_TERMINATED) {
            procTab[i].status = STATUS_TERMINATED;
            break;
        }
    }
    ctx->gpr[0] = 0; //success
    //procTab[ 1 ].status = STATUS_TERMINATED;
    schedule(ctx);
}

void sys_nice(ctx_t *ctx){
    int pid = ctx->gpr[0];
    int nice = ctx->gpr[1];
    for (int i = 0; i < n_pcb; i++) {
        if (pid == procTab[i].pid && procTab[i].status != STATUS_TERMINATED) {
            procTab[i].niceness = nice;
            break;
        }
    }
}

void sys_pipe(ctx_t *ctx){
    int* fildes;
    fildes = (int* ) (ctx->gpr[0]);
//            fildes = (int *) 0x7007a6b8;

    pipe_t *pipe = (pipe_t *) malloc(sizeof(pipe_t));

    if (pipe == NULL) {
        ctx->gpr[0] = -1;
        return;
    }

    pipe->size = 0;
    pipe->readers = 1;
    pipe->writers = 1;

    int read_fd = -1;
    int write_fd = -1;

    for (int i = 0; i < 128; i++) {
        if (fdTab[i].access == FREE) {
            if (read_fd == -1) {
                read_fd = i;
            } else {
                write_fd = i;
                break;
            }
        }
    }

    //no fd available
    if (read_fd == -1 || write_fd == -1) {
        free(pipe);
        ctx->gpr[0] = -1;
    } else {

        fdTab[read_fd].access = READ;
        fdTab[read_fd].pipe = pipe;

        fdTab[write_fd].access = WRITE;
        fdTab[write_fd].pipe = pipe;

        *(fildes) = read_fd;
        *(fildes+1) = write_fd;

        ctx->gpr[0] = 0;

        PL011_putc(UART0, 'K', true);
    }
}

void sys_closepipe(ctx_t *ctx){
    int fd = ctx->gpr[0];
    pipe_t *p = fdTab[fd].pipe;
    if(fdTab[fd].access == READ)
        p->readers--;
    if(fdTab[fd].access == WRITE)
        p->writers--;

    if(p->writers == 0 && p->readers == 0)
        free(p);

    fdTab[fd].access = FREE;

    ctx->gpr[0] = 0;
}

void sys_read_nb(ctx_t *ctx){
    int fd = (int) (ctx->gpr[0]);
    char *x = (char *) (ctx->gpr[1]);
    int n = (int) (ctx->gpr[2]);

    if(fdTab[fd].access == READ){
        pipe_t *p = fdTab[fd].pipe;

        if(p->size != 0){
            //start reading from the pipe
            for(int i =0; i< p->size; i++){
                *(x+i) = p->buffer[i];
            }
        }
        ctx->gpr[0] = p->size;
        p->size = 0;
    }
}

void hilevel_handler_svc(ctx_t *ctx, uint32_t id) {
    /* Based on the identifier (i.e., the immediate operand) extracted from the
     * svc instruction,
     *
     * - read  the arguments from preserved usr mode registers,
     * - perform whatever is appropriate for this system call, then
     * - write any return value back to preserved usr mode registers.
     */

    switch (id) {
        case 0x00 : { // 0x00 => yield()

            schedule(ctx);
            break;
        }

        case 0x01 : { // 0x01 => write( fd, x, n )

            sys_write(ctx);
            break;
        }

        case 0x02 : { //SYS_READ

            sys_read(ctx);
            break;
        }

        case 0x03 : { //SYS_FORK

            sys_fork(ctx);
            break;
        }

        case 0x04 : { //SYS_EXIT (signal)

            sys_exit(ctx);
            break;
        }

        case 0x05 : { //SYS_EXEC (x)

            sys_exec(ctx);
            break;
        }

        case 0x06 : { //SYS_KILL (pid, signal)

            sys_kill(ctx);
            break;
        }

        case 0x07 : { //SYS_NICE (pid, x)

            sys_nice(ctx);
            break;
        }

        case 0x08 : { //SYS_PIPE

            sys_pipe(ctx);
            break;

        }
        case 0x09 : { //SYS_CLOSEPIPE

            sys_closepipe(ctx);
            break;
        }

        case 0x0a : { //SYS_READ_NON_BLOCKING
            sys_read_nb(ctx);
        }

        default   : { // 0x?? => unknown/unsupported
            break;
        }
    }

}

//invoked at the beging of the scheduler
void try_write(int pcb) {
    int fd = (int) (procTab[pcb].ctx.gpr[0]);
    char *x = (char *) (procTab[pcb].ctx.gpr[1]);
    int n = (int) (procTab[pcb].ctx.gpr[2]);

    pipe_t *p = fdTab[fd].pipe;

    if(p->size == 0) {
        for (int i = 0; i < n; i++, x++) {
            p->buffer[p->size] = *x;
            p->size++;
        }
        procTab[pcb].ctx.gpr[0] = n;
        procTab[pcb].status = STATUS_READY;
    }
}

void try_read(int pcb) {
    int fd = (int) (procTab[pcb].ctx.gpr[0]);
    char *x = (char *) (procTab[pcb].ctx.gpr[1]);
    int n = (int) (procTab[pcb].ctx.gpr[2]);

    pipe_t *p = fdTab[fd].pipe;

    if(p->size != 0) {
        for(int i =0; i< p->size; i++){
            *(x+i) = p->buffer[i];
        }
        procTab[pcb].ctx.gpr[0] = p->size;
        p->size = 0;
        procTab[pcb].status = STATUS_READY;
    }

}