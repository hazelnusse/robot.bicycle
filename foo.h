#ifndef FOO_H
#define FOO_H

class foo
{
 public:
  foo();
  ~foo();
  void cleanup();
  int i;
};

inline bool operator<(const foo & a, const foo & b)
{
  return a.i < b.i;
}

#endif
