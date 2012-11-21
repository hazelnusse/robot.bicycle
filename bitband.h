#ifndef BITBAND_H
#define BITBAND_H

#define BITBAND(addr,bitnum) ((addr & 0xF0000000) + 0x02000000 + ((addr & 0x000FFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile uint32_t *) (addr))

#endif
