MISC
====

LIBRARIES AND FUNTIONS CREATED BY OTHERS AND USED - and MODIFIED


note: AURA MEGA needs a larger Serial buffer I.e #define SERIAL_BUFFER_SIZE 128

#if (RAMEND < 1000)
  #define SERIAL_BUFFER_SIZE 16
#else
  #define SERIAL_BUFFER_SIZE 64
#endif

//#if defined (HABJOE_MEGAMAIN)
// #define SERIAL_BUFFER_SIZE 128
//#endif
