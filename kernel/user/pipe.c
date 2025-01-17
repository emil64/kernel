#include "pipe.h"

void main_pipe() {
    int fds[ 2 ];
    int status = pipe( fds );

    if(status == -1)
        write( STDOUT_FILENO, "Could not create any fds\n", 12 );


    switch ( fork() ) {
        case -1:
        {/* Handle error */
            break;
        }
        case 0:
        { /* Child - reads from pipe */
            char buf[ 100 ];
            int  nbytes;

            write( STDOUT_FILENO, "Child reads\n", 12 );

            nbytes = read(fds[0], buf, 14);

            write( STDOUT_FILENO, "Child: received ", 16 );
            write( STDOUT_FILENO, buf, nbytes );

            nbytes = read(fds[0], buf, 18);

            write( STDOUT_FILENO, "Child: received ", 16 );
            write( STDOUT_FILENO, buf, nbytes );

            close(fds[0] );
            write( STDOUT_FILENO, "Child out.\n", 11 );
            exit( EXIT_SUCCESS );
        }

        default:
        { /* Parent - writes to pipe */
            const char msg1[] = "\"Hello world from another world\"\n";
            const char msg2[] = "\"Another Message\"\n";

            write( STDOUT_FILENO, "Parent1: sending \"Hello world\"\n", 32 );
            write(fds[1], msg1, 30);


            //sleep
            sleep(3000);

            write( STDOUT_FILENO, "Parent2: sending \"Another Message\"\n", 34 );
            write(fds[1], msg2, 18 );

            close(fds[1]);
            write( STDOUT_FILENO, "Parent out\n", 11 );
            exit( EXIT_SUCCESS );
        }
    }
}
