#ifndef _MODLIB_EEPROM_CONFIG_
#define _MODLIB_EEPROM_CONFIG_
#define MEM_EEPROM_FLAG             0x80
#define MEM_SLAVE_FLAG              0x81
#define MEM_SLAVE_ADDRESS           0x82
#define MEM_HARDWARE_VC             0x83
#define MEM_DEVICE_ID               0x84
#define MEM_SCHEMA_LEN_HIGH         0x9B
#define MEM_SCHEMA_LEN_LOW          0x9C
#define MEM_SCHEMA                  0x9D
#define MEM_BOARD_CONTEXT_ADDRESS (MEM_SCHEMA + 600) //eeprom memory address for saving board context
#endif