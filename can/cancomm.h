#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include "raw.h"
#include "terminal.h"
#include "lib.h"
#include <iostream>

struct canfd_frame canrecieve(int argc, char **argv);

bool cansend(int can_id, int num_of_bytes, int message);

int idx2dindex(int ifidx, int socket);

void sigterm(int signo);

void sprint_canframe(char *buf , struct canfd_frame *cf, int sep, int maxdlen);
