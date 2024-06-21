#include "rmd_can.h"

static rmd_can obj;

extern rmd_motor _DEV_MC[12];

pthread_mutex_t rmd_can::mutex_reference[12];
ST_CAN  rmd_can::reference_msg[12];

rmd_can::rmd_can()
{
    //  spi 1.0
    //  channel a
    reference_msg[0].header = 0x89;     reference_msg[0].id = 0x141;
    reference_msg[1].header = 0x89;     reference_msg[1].id = 0x142;
    reference_msg[2].header = 0x89;     reference_msg[2].id = 0x143;
    //  channel b
    reference_msg[3].header = 0x77;     reference_msg[3].id = 0x144;
    reference_msg[4].header = 0x77;     reference_msg[4].id = 0x145;
    reference_msg[5].header = 0x77;     reference_msg[5].id = 0x146;
    //  spi 1.1
    //  channel c
    reference_msg[6].header = 0x89;     reference_msg[6].id = 0x147;
    reference_msg[7].header = 0x89;     reference_msg[7].id = 0x148;
    reference_msg[8].header = 0x89;     reference_msg[8].id = 0x149;
    //  channel d
    reference_msg[9].header = 0x77;     reference_msg[9].id = 0x14a;
    reference_msg[10].header = 0x77;     reference_msg[10].id = 0x14b;
    reference_msg[11].header = 0x77;     reference_msg[11].id = 0x14c;

    for(int i=0; i<12; i++)
    {
        pthread_mutex_init(&mutex_reference[i], NULL);
        reference_msg[i].dlc = 8;
    }
}