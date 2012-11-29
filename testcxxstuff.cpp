#include <algorithm>
#include <vector>
#include <new>

#include <cstdint>
#include "foo.h"

void testcxxstuff()
{
  using namespace std;
  {
    char *a = new char;
    *a = 'c';
    delete a;
  }
  
  {
    char *a = new (std::nothrow) char;
    *a = 'c';
    delete a;
  }

  {
    uint32_t *a = new uint32_t[10];
    for (int i = 0; i < 10; ++i) {
      a[i] = i;
    }
    delete [] a;
  }
  
  {
    uint32_t *a = new (std::nothrow) uint32_t[10];
    for (int i = 0; i < 10; ++i) {
      a[i] = i;
    }
    delete [] a;
  }

  vector<int> myvector;
  int * p;
  unsigned int i;

  // allocate an array with space for 5 elements using vector's allocator:
  p = myvector.get_allocator().allocate(5);

  // construct values in-place on the array:
  for (i=0; i<5; i++) myvector.get_allocator().construct(&p[i],i);


  // destroy and deallocate:
  for (i=0; i<5; i++) myvector.get_allocator().destroy(&p[i]);
  myvector.get_allocator().deallocate(p,5);

  {
    vector<foo> myfoos(10);
    foo f, g;
    f.cleanup();
    myfoos.push_back(f);
    myfoos.push_back(g);
    myfoos.push_back(g);
    sort(myfoos.begin(), myfoos.end());
  }

  static foo g;
}
