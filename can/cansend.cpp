#include "cancomm.h"

bool cansend(int can_id, int num_of_bytes, int message)
{
    int nbytes;
    int s;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    char *ifname = "can0"; //can0

    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return false;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return false;
    }

    frame.can_id  = can_id;
    frame.can_dlc = num_of_bytes;
    for (int i = 0; i < num_of_bytes; i++)
    {
        frame.data[i] = (message >> 4*i) & 0xF;
    }

    nbytes = write(s, &frame, sizeof(struct can_frame));

    return true;
}
