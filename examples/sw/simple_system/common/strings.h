
#ifndef STRINGS
#define STRINGS 1

#include <stddef.h>
#include <stdint.h>


void * memcpy (void *dest, const void *src, size_t len);
int memcmp(const void *p1, const void *p2, size_t n);
void * memmove(void *s1, const void *s2, size_t n);

#endif

