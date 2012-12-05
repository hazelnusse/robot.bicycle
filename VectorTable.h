#ifndef VECTORTABLE_H
#define VECTORTABLE_H

// #include "Singleton.h"
#include <cstdint>

typedef void (*irq_vector_t)(void);
struct vectors_t {
  uint32_t      *init_stack;
  irq_vector_t  reset_vector;
  irq_vector_t  nmi_vector;
  irq_vector_t  hardfault_vector;
  irq_vector_t  memmanage_vector;
  irq_vector_t  busfault_vector;
  irq_vector_t  usagefault_vector;
  irq_vector_t  vector1c;
  irq_vector_t  vector20;
  irq_vector_t  vector24;
  irq_vector_t  vector28;
  irq_vector_t  svcall_vector;
  irq_vector_t  debugmonitor_vector;
  irq_vector_t  vector34;
  irq_vector_t  pendsv_vector;
  irq_vector_t  systick_vector;
  irq_vector_t  vectors[82];
};

class VectorTable {
 public:
  VectorTable();
  void Relocate();

 private:
  VectorTable(const VectorTable&) = delete;
  VectorTable & operator=(const VectorTable&) = delete;

  static vectors_t vtable_; __attribute__ ((section("ram_vectors")));
};

#endif
