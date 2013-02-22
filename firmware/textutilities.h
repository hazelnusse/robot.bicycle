#ifndef TEXTUTILITIES_H
#define TEXTUTILITIES_H

/*
 * Convert a length 6 char array of the form 
 *
 * {-+}xx.xx
 *
 * to a 32 bit float.
 *
 * s : array of chars
 */
float tofloat(const char * s);

#endif
