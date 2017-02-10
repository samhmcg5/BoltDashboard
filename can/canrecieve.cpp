#include "cancomm.h"

#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define MAXCOL 6      /* number of different colors for colorized output */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

#define SILENT_INI 42 /* detect user setting on commandline */
#define SILENT_OFF 0  /* no silent mode */
#define SILENT_ANI 1  /* silent mode with animation */
#define SILENT_ON  2  /* silent mode (completely silent) */

#define BOLD    ATTBOLD
#define RED     ATTBOLD FGRED
#define GREEN   ATTBOLD FGGREEN
#define YELLOW  ATTBOLD FGYELLOW
#define BLUE    ATTBOLD FGBLUE
#define MAGENTA ATTBOLD FGMAGENTA
#define CYAN    ATTBOLD FGCYAN

const char col_on [MAXCOL][19] = {BLUE, RED, GREEN, BOLD, MAGENTA, CYAN};
const char col_off [] = ATTRESET;

static char *cmdlinename[MAXSOCK];
static __u32 dropcnt[MAXSOCK];
static __u32 last_dropcnt[MAXSOCK];
static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 
const int canfd_on = 1;

#define MAXANI 4
const char anichar[MAXANI] = {'|', '/', '-', '\\'};
const char extra_m_info[4][4] = {"- -", "B -", "- E", "B E"};

extern int optind, opterr, optopt;

static volatile int running = 1;

void sigterm(int signo)
{
	running = 0;
}

int idx2dindex(int ifidx, int socket) {

	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

	/* create new interface index cache entry */

	/* remove index cache zombies first */
	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i]) {
			ifr.ifr_ifindex = dindex[i];
			if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
				dindex[i] = 0;
		}
	}

	for (i=0; i < MAXIFNAMES; i++)
		if (!dindex[i]) /* free entry */
			break;

	if (i == MAXIFNAMES) {
		fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
		       MAXIFNAMES);
		exit(1);
	}

	dindex[i] = ifidx;

	ifr.ifr_ifindex = ifidx;
	if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
		perror("SIOCGIFNAME");

	if (max_devname_len < strlen(ifr.ifr_name))
		max_devname_len = strlen(ifr.ifr_name);

	strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
	printf("new index %d (%s)\n", i, devname[i]);
#endif

	return i;
}

struct canfd_frame canrecieve(int argc, char **argv)
{
	fd_set rdfs;
	int s[MAXSOCK];
	int bridge = 0;
	useconds_t bridge_delay = 0;
	unsigned char timestamp = 0;
	unsigned char dropmonitor = 0;
	unsigned char extra_msg_info = 0;
	unsigned char silent = SILENT_INI;
	unsigned char silentani = 0;
	unsigned char color = 0;
	unsigned char view = 0;
	unsigned char log = 0;
	unsigned char logfrmt = 0;
	int count = 0;
	int rcvbuf_size = 0;
	int opt, ret;
	int currmax, numfilter;
	int join_filter;
	char *ptr, *nptr;
	struct sockaddr_can addr;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	struct iovec iov;
	struct msghdr msg;
	struct cmsghdr *cmsg;
	struct can_filter *rfilter;
	can_err_mask_t err_mask;
	struct canfd_frame frame;
    struct canfd_frame emptyframe;
	int nbytes, i, maxdlen;
	struct ifreq ifr;
	struct timeval tv, last_tv;
	struct timeval timeout, timeout_config = { 0, 0 }, *timeout_current = NULL;
    FILE *logfile = NULL;

	signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);

	last_tv.tv_sec  = 0;
	last_tv.tv_usec = 0;

	while ((opt = getopt(argc, argv, "t:ciaSs:b:B:u:ldxLn:r:heT:?")) != -1) {
		switch (opt) {
		case 't':
			timestamp = optarg[0];
			if ((timestamp != 'a') && (timestamp != 'A') &&
			    (timestamp != 'd') && (timestamp != 'z')) {			
				timestamp = 0;
			}
			break;

		case 'c':
			color++;
			break;

		case 'i':
			view |= CANLIB_VIEW_BINARY;
			break;

		case 'a':
			view |= CANLIB_VIEW_ASCII;
			break;

		case 'S':
			view |= CANLIB_VIEW_SWAP;
			break;

		case 'e':
			view |= CANLIB_VIEW_ERROR;
			break;

		case 's':
			silent = atoi(optarg);
			if (silent > SILENT_ON) {
                return emptyframe;
			}
			break;

		case 'b':
		case 'B':
			if (strlen(optarg) >= IFNAMSIZ) {
                return emptyframe;
			} else {
				bridge = socket(PF_CAN, SOCK_RAW, CAN_RAW);
				if (bridge < 0) {
                    //perror("bridge socket");
                    return emptyframe;
				}
				addr.can_family = AF_CAN;
				strcpy(ifr.ifr_name, optarg);
				if (ioctl(bridge, SIOCGIFINDEX, &ifr) < 0)
                    //perror("SIOCGIFINDEX");
				addr.can_ifindex = ifr.ifr_ifindex;
		
				if (!addr.can_ifindex) {
                    perror("invalid bridge interface");
                    return emptyframe;
				}

				/* disable default receive filter on this write-only RAW socket */
				setsockopt(bridge, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

				if (opt == 'B') {
					const int loopback = 0;

					setsockopt(bridge, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
						   &loopback, sizeof(loopback));
				}

				if (bind(bridge, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                    perror("bridge bind");
                    return emptyframe;
				}
			}
			break;
	    
		case 'u':
			bridge_delay = (useconds_t)strtoul(optarg, (char **)NULL, 10);
			break;

		case 'l':
			log = 1;
			break;

		case 'd':
			dropmonitor = 1;
			break;

		case 'x':
			extra_msg_info = 1;
			break;

		case 'L':
			logfrmt = 1;
			break;

		case 'n':
			count = atoi(optarg);
			if (count < 1) {
                //print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		case 'r':
			rcvbuf_size = atoi(optarg);
			if (rcvbuf_size < 1) {
                //print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		case 'T':
			errno = 0;
			timeout_config.tv_usec = strtol(optarg, NULL, 0);
			if (errno != 0) {
                //print_usage(basename(argv[0]));
				exit(1);
			}
			timeout_config.tv_sec = timeout_config.tv_usec / 1000;
			timeout_config.tv_usec = (timeout_config.tv_usec % 1000) * 1000;
			timeout_current = &timeout;
			break;
		default:
            //print_usage(basename(argv[0]));
			exit(1);
			break;
		}
	}

	if (optind == argc) {
        //print_usage(basename(argv[0]));
		exit(0);
	}
	
	if (logfrmt && view) {
        fprintf(stderr, "Log file format selected: Please disable ASCII/BINARY/SWAP options!\n");
		exit(0);
	}

	if (silent == SILENT_INI) {
		if (log) {
            fprintf(stderr, "Disabled standard output while logging.\n");
			silent = SILENT_ON; /* disable output on stdout */
		} else
			silent = SILENT_OFF; /* default output */
	}

	currmax = argc - optind; /* find real number of CAN devices */

	if (currmax > MAXSOCK) {
        fprintf(stderr, "More than %d CAN devices given on commandline!\n", MAXSOCK);
        return emptyframe;
	}

	for (i=0; i < currmax; i++) {

		ptr = argv[optind+i];
		nptr = strchr(ptr, ',');

#ifdef DEBUG
		printf("open %d '%s'.\n", i, ptr);
#endif

		s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s[i] < 0) {
            perror("socket");
            return emptyframe;
		}

		cmdlinename[i] = ptr; /* save pointer to cmdline name of this socket */

		if (nptr)
			nbytes = nptr - ptr;  /* interface name is up the first ',' */
		else
			nbytes = strlen(ptr); /* no ',' found => no filter definitions */

		if (nbytes >= IFNAMSIZ) {
            fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
            return emptyframe;
		}

		if (nbytes > max_devname_len)
			max_devname_len = nbytes; /* for nice printing */

		addr.can_family = AF_CAN;

		memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
		strncpy(ifr.ifr_name, ptr, nbytes);

#ifdef DEBUG
        printf("using interface name '%s'.\n", ifr.ifr_name);
#endif

		if (strcmp(ANYDEV, ifr.ifr_name)) {
			if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0) {
				perror("SIOCGIFINDEX");
				exit(1);
			}
			addr.can_ifindex = ifr.ifr_ifindex;
		} else
			addr.can_ifindex = 0; /* any can interface */

		if (nptr) {

			/* found a ',' after the interface name => check for filters */

			/* determine number of filters to alloc the filter space */
			numfilter = 0;
			ptr = nptr;
			while (ptr) {
				numfilter++;
				ptr++; /* hop behind the ',' */
				ptr = strchr(ptr, ','); /* exit condition */
			}

            rfilter = (struct can_filter*) malloc(sizeof(struct can_filter) * numfilter);
			if (!rfilter) {
                fprintf(stderr, "Failed to create filter space!\n");
                return emptyframe;
			}

			numfilter = 0;
			err_mask = 0;
			join_filter = 0;

			while (nptr) {

				ptr = nptr+1; /* hop behind the ',' */
				nptr = strchr(ptr, ','); /* update exit condition */

				if (sscanf(ptr, "%x:%x",
					   &rfilter[numfilter].can_id, 
					   &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					numfilter++;
				} else if (sscanf(ptr, "%x~%x",
						  &rfilter[numfilter].can_id, 
						  &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_id |= CAN_INV_FILTER;
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					numfilter++;
				} else if (*ptr == 'j' || *ptr == 'J') {
					join_filter = 1;
				} else if (sscanf(ptr, "#%x", &err_mask) != 1) { 
                    fprintf(stderr, "Error in filter option parsing: '%s'\n", ptr);
                    return emptyframe;
				}
			}

			if (err_mask)
				setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
					   &err_mask, sizeof(err_mask));

			if (join_filter && setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS,
						      &join_filter, sizeof(join_filter)) < 0) {
                perror("setsockopt CAN_RAW_JOIN_FILTERS not supported by your Linux Kernel");
                return emptyframe;
			}

			if (numfilter)
				setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FILTER,
					   rfilter, numfilter * sizeof(struct can_filter));

			free(rfilter);

		} /* if (nptr) */

		/* try to switch the socket into CAN FD mode */
		setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

		if (rcvbuf_size) {

			int curr_rcvbuf_size;
			socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

			/* try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
			if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUFFORCE,
				       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
#ifdef DEBUG
                printf("SO_RCVBUFFORCE failed so try SO_RCVBUF ...\n");
#endif
				if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUF,
					       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
                    perror("setsockopt SO_RCVBUF");
                    return emptyframe;
				}

				if (getsockopt(s[i], SOL_SOCKET, SO_RCVBUF,
					       &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0) {
                    perror("getsockopt SO_RCVBUF");
                    return emptyframe;
				}

				/* Only print a warning the first time we detect the adjustment */
				/* n.b.: The wanted size is doubled in Linux in net/sore/sock.c */
                //if (!i && curr_rcvbuf_size < rcvbuf_size*2)
                    //fprintf(stderr, "The socket receive buffer size was "
                        //"adjusted due to /proc/sys/net/core/rmem_max.\n");
			}
		}

		if (timestamp || log || logfrmt) {

			const int timestamp_on = 1;

			if (setsockopt(s[i], SOL_SOCKET, SO_TIMESTAMP,
				       &timestamp_on, sizeof(timestamp_on)) < 0) {
                perror("setsockopt SO_TIMESTAMP");
                return emptyframe;
			}
		}

		if (dropmonitor) {

			const int dropmonitor_on = 1;

			if (setsockopt(s[i], SOL_SOCKET, SO_RXQ_OVFL,
				       &dropmonitor_on, sizeof(dropmonitor_on)) < 0) {
                perror("setsockopt SO_RXQ_OVFL not supported by your Linux Kernel");
                return emptyframe;
			}
		}

		if (bind(s[i], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("bind");
            return emptyframe;
		}
	}

	if (log) {
		time_t currtime;
		struct tm now;
		char fname[sizeof("candump-2006-11-20_202026.log")+1];

		if (time(&currtime) == (time_t)-1) {
            perror("time");
            return emptyframe;
		}

		localtime_r(&currtime, &now);

        sprintf(fname, "candump-%04d-%02d-%02d_%02d%02d%02d.log",
			now.tm_year + 1900,
			now.tm_mon + 1,
			now.tm_mday,
			now.tm_hour,
			now.tm_min,
            now.tm_sec);

		if (silent != SILENT_ON)
            printf("\nWarning: console output active while logging!");

        fprintf(stderr, "\nEnabling Logfile '%s'\n\n", fname);

        logfile = fopen(fname, "w");
        if (!logfile) {
            perror("logfile");
            return emptyframe;
        }
	}

	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;
    int k = 0;
    while (k < 1) {
        k++;
		FD_ZERO(&rdfs);
		for (i=0; i<currmax; i++)
			FD_SET(s[i], &rdfs);

		if (timeout_current)
			*timeout_current = timeout_config;

        if ((ret = select(s[currmax-1]+1, &rdfs, NULL, NULL, timeout_current)) <= 0) {
            perror("select");
			running = 0;
			continue;
        }

		for (i=0; i<currmax; i++) {  /* check all CAN RAW sockets */

			if (FD_ISSET(s[i], &rdfs)) {

				int idx;

				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);  
				msg.msg_flags = 0;

				nbytes = recvmsg(s[i], &msg, 0);
				if (nbytes < 0) {
					perror("read");
                    return emptyframe;
				}

				if ((size_t)nbytes == CAN_MTU)
					maxdlen = CAN_MAX_DLEN;
				else if ((size_t)nbytes == CANFD_MTU)
					maxdlen = CANFD_MAX_DLEN;
				else {
                    fprintf(stderr, "read: incomplete CAN frame\n");
                    return emptyframe;
				}

				if (count && (--count == 0))
					running = 0;

				if (bridge) {
					if (bridge_delay)
						usleep(bridge_delay);

					nbytes = write(bridge, &frame, nbytes);
					if (nbytes < 0) {
                        perror("bridge write");
                        return emptyframe;
					} else if ((size_t)nbytes != CAN_MTU && (size_t)nbytes != CANFD_MTU) {
                        fprintf(stderr,"bridge write: incomplete CAN frame\n");
                        return emptyframe;
					}
				}
		    
				for (cmsg = CMSG_FIRSTHDR(&msg);
				     cmsg && (cmsg->cmsg_level == SOL_SOCKET);
				     cmsg = CMSG_NXTHDR(&msg,cmsg)) {
					if (cmsg->cmsg_type == SO_TIMESTAMP)
						tv = *(struct timeval *)CMSG_DATA(cmsg);
					else if (cmsg->cmsg_type == SO_RXQ_OVFL)
						dropcnt[i] = *(__u32 *)CMSG_DATA(cmsg);
				}

				/* check for (unlikely) dropped frames on this specific socket */
				if (dropcnt[i] != last_dropcnt[i]) {

					__u32 frames = dropcnt[i] - last_dropcnt[i];

                    if (silent != SILENT_ON)
						printf("DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops %d)\n",
						       frames, (frames > 1)?"s":"", cmdlinename[i], dropcnt[i]);

					if (log)
						fprintf(logfile, "DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops %d)\n",
							frames, (frames > 1)?"s":"", cmdlinename[i], dropcnt[i]);

					last_dropcnt[i] = dropcnt[i];
				}

				idx = idx2dindex(addr.can_ifindex, s[i]);

				/* once we detected a EFF frame indent SFF frames accordingly */
				if (frame.can_id & CAN_EFF_FLAG)
					view |= CANLIB_VIEW_INDENT_SFF;

                if (log) {
                    char buf[CL_CFSZ]; /* max length */

                    /* log CAN frame with absolute timestamp & device */
                    //sprint_canframe(buf, &frame, 0, maxdlen);
                    fprintf(logfile, "(%010ld.%06ld) %*s %s\n",
                        tv.tv_sec, tv.tv_usec,
                        max_devname_len, devname[idx], buf);
                }

				if (logfrmt) {
					char buf[CL_CFSZ]; /* max length */

                    //print CAN frame in log file style to stdout
                    //sprint_canframe(buf, &frame, 0, maxdlen);
                    printf("(%010ld.%06ld) %*s %s\n",
                           tv.tv_sec, tv.tv_usec,
                           max_devname_len, devname[idx], buf);
                    goto out_fflush; //no other output to stdout
				}

				if (silent != SILENT_OFF){
					if (silent == SILENT_ANI) {
                        printf("%c\b", anichar[silentani%=MAXANI]);
						silentani++;
					}
					goto out_fflush; /* no other output to stdout */
				}
		      
                //printf(" %s", (color>2)?col_on[idx%MAXCOL]:"");

				switch (timestamp) {

				case 'a': /* absolute with timestamp */
                    //printf("(%010ld.%06ld) ", tv.tv_sec, tv.tv_usec);
					break;

				case 'A': /* absolute with date */
				{
					struct tm tm;
					char timestring[25];

					tm = *localtime(&tv.tv_sec);
					strftime(timestring, 24, "%Y-%m-%d %H:%M:%S", &tm);
                    printf("(%s.%06ld) ", timestring, tv.tv_usec);
				}
				break;

				case 'd': /* delta */
				case 'z': /* starting with zero */
				{
					struct timeval diff;

					if (last_tv.tv_sec == 0)   /* first init */
						last_tv = tv;
					diff.tv_sec  = tv.tv_sec  - last_tv.tv_sec;
					diff.tv_usec = tv.tv_usec - last_tv.tv_usec;
					if (diff.tv_usec < 0)
						diff.tv_sec--, diff.tv_usec += 1000000;
					if (diff.tv_sec < 0)
						diff.tv_sec = diff.tv_usec = 0;
                    printf("(%03ld.%06ld) ", diff.tv_sec, diff.tv_usec);
				
					if (timestamp == 'd')
						last_tv = tv; /* update for delta calculation */
				}
				break;

				default: /* no timestamp output */
					break;
				}

                //printf(" %s", (color && (color<3))?col_on[idx%MAXCOL]:"");
                //printf("%*s", max_devname_len, devname[idx]);

				if (extra_msg_info) {

                    if (msg.msg_flags & MSG_DONTROUTE)
                        printf ("  TX %s", extra_m_info[frame.flags & 3]);

                        printf ("  RX %s", extra_m_info[frame.flags & 3]);
				}

			}

		out_fflush:
			fflush(stdout);
		}
	}

	for (i=0; i<currmax; i++)
		close(s[i]);

	if (bridge)
		close(bridge);

    if (log)
        fclose(logfile);

    return frame;
}
