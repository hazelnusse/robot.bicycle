#include "VectorTable.h"
#include "hal.h"

extern vectors_t _vectors;

vectors_t VectorTable::vtable_ __attribute__ ((section("ram_vectors")));

VectorTable::VectorTable()
{

}

void VectorTable::Relocate()
{
  vtable_ = _vectors; //  copy existing vector table to memory;
}
