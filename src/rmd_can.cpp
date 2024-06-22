#include "rmd_can.h"

static rmd_can obj;

extern rmd_motor _DEV_MC[24];

pthread_mutex_t rmd_can::mutex_reference[24];
ST_CAN  rmd_can::reference_msg[24];

rmd_can::rmd_can()
{
    //  spi 1.0
    //  channel a
    reference_msg[0].header = 0x89;     reference_msg[0].id = 0x141;
    reference_msg[1].header = 0x89;     reference_msg[1].id = 0x142;
    reference_msg[2].header = 0x89;     reference_msg[2].id = 0x143;
    reference_msg[3].header = 0x89;     reference_msg[3].id = 0x144;
    reference_msg[4].header = 0x89;     reference_msg[4].id = 0x145;
    reference_msg[5].header = 0x89;     reference_msg[5].id = 0x146;
    //  channel b
    reference_msg[6].header = 0x77;     reference_msg[6].id = 0x147;
    reference_msg[7].header = 0x77;     reference_msg[7].id = 0x148;
    reference_msg[8].header = 0x77;     reference_msg[8].id = 0x149;
    reference_msg[9].header = 0x77;     reference_msg[9].id = 0x14a;
    reference_msg[10].header = 0x77;     reference_msg[10].id = 0x14b;
    reference_msg[11].header = 0x77;     reference_msg[11].id = 0x14c;
    
    //  spi 1.1
    //  channel c
    reference_msg[12].header = 0x89;     reference_msg[12].id = 0x14d;
    reference_msg[13].header = 0x89;     reference_msg[13].id = 0x14e;
    reference_msg[14].header = 0x89;     reference_msg[14].id = 0x14f;
    reference_msg[15].header = 0x89;     reference_msg[15].id = 0x150;
    reference_msg[16].header = 0x89;     reference_msg[16].id = 0x151;
    reference_msg[17].header = 0x89;     reference_msg[17].id = 0x152;
    //  channel d
    reference_msg[18].header = 0x77;     reference_msg[18].id = 0x153;
    reference_msg[19].header = 0x77;     reference_msg[19].id = 0x154;
    reference_msg[20].header = 0x77;     reference_msg[20].id = 0x155;
    reference_msg[21].header = 0x77;     reference_msg[21].id = 0x156;
    reference_msg[22].header = 0x77;     reference_msg[22].id = 0x157;
    reference_msg[23].header = 0x77;     reference_msg[23].id = 0x158;


    for(int i=0; i<24; i++)
    {
        pthread_mutex_init(&mutex_reference[i], NULL);
        reference_msg[i].dlc = 8;
    }

}