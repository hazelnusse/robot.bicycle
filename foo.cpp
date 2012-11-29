#include "foo.h"

foo::foo()
  : i(0)
{
  ++i;
}

foo::~foo()
{
  cleanup();
  i = -123;
}

void foo::cleanup()
{
  i = i * 2003;
}
