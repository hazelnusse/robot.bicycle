#include "VectorTable.h"
#include "hal.h"

extern vectors_t _vectors;

vectors_t VectorTable::vtable_;

VectorTable::VectorTable()
{

}

void VectorTable::Relocate()
{
  vtable_ = _vectors; //  copy existing vector table to memory;
  SCB_VTOR = reinterpret_cast<uint32_t>(&vtable_); // 
}
