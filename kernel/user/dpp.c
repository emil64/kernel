//
// Created by emil on 04/04/2020.
//

#include "dpp.h"

void print_id(int id){
    char cid[10];
    itoa(cid, id);
    write(STDOUT_FILENO, cid, strlen(cid));
}

void print_status(int id, char *status){
    write(STDOUT_FILENO, "Philosopher ", 12);
    print_id(id);
    write(STDOUT_FILENO, status, strlen(status));
}

double_channel_t pipes[n_phil];

void take_rightchopstick(int id){
    bool grabbed = false;
    char msg[100];
    while(!grabbed){
        write(pipes[id].p_w[1], "request the right chopstick", 27);
        read(pipes[id].w_p[0], msg, 100);
        if(strcmp(msg, "ok") == 0)
            grabbed = true;
        else
            sleep(10);
    }
}

void take_leftchopstick(int id){
    bool grabbed = false;
    char msg[100];
    while(!grabbed){
        write(pipes[id].p_w[1], "request the left chopstick", 26);
        read(pipes[id].w_p[0], msg, 100);
        if(strcmp(msg, "ok") == 0)
            grabbed = true;
        else
            sleep(10);
    }
}

void putdown_chopsticks(int id){
    write(pipes[id].p_w[1], "here are the chopsticks", 23);
}

void take_chopsticks(int id){
    take_leftchopstick(id);
    take_rightchopstick(id);
}

void eat(int id){
    print_status(id, " is eating\n");
    sleep(3000); //always think more than eat :)
}

void think(int id){
    print_status(id, " is thinking\n");
    sleep(5000);
}

void hungry(int id){
    print_status(id, " is hungry\n");
}

void philosopher(int id){

    while(1) {
        hungry(id);
        take_chopsticks(id);
        eat(id);
        putdown_chopsticks(id);
        think(id);
    }
}

int left(int i){
    return (i)   % n_phil;
}

int right(int i){
    return (i+1) % n_phil;
}

void main_dpp(){

    chopstick_t chopsticks[n_phil];

    for(int i=0; i<n_phil; i++){
        pipe(pipes[i].p_w); //philosopher to waiter
        pipe(pipes[i].w_p); //waiter to philosopher
        chopsticks[i] = FREE;
    }

    for(int i=0; i<n_phil; i++){
        switch (fork()){
            case 0:
                philosopher(i);
                exit(EXIT_SUCCESS);
                break;
            default:
                break;
        }
    }

    while (1){
        
        for(int i=0; i<n_phil; i++){

            char msg[100]="";
            int len = read_nb(pipes[i].p_w[0], msg, 100);
//            print_status(i, msg);
//            write(STDOUT_FILENO, "\n")
            if(len != 0){
                if(strcmp(msg, "request the left chopstick") == 0){
                    if(chopsticks[left(i)] == FREE && chopsticks[right(i)] == FREE){
                        chopsticks[left(i)] = TAKEN;
                        chopsticks[right(i)] = RESERVED;
                        write(pipes[i].w_p[1], "ok", 2);
                    }
                    else
                        write(pipes[i].w_p[1], "nope", 4);
                }
                if(strcmp(msg, "request the right chopstick") == 0){
                    if(chopsticks[right(i)] == FREE || chopsticks[right(i)] == RESERVED){
                        chopsticks[right(i)] = TAKEN;
                        write(pipes[i].w_p[1], "ok", 2);
                    }
                    else
                        write(pipes[i].w_p[1], "nope", 4);
                }
                if(strcmp(msg, "here are the chopsticks") == 0){
                    chopsticks[left(i)] = FREE;
                    chopsticks[right(i)] = FREE;
                }
            }
        }
        sleep(250);
    }



    exit(EXIT_SUCCESS);
}