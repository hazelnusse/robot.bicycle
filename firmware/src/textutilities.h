#ifndef TEXTUTILITIES_H
#define TEXTUTILITIES_H

/*
 * Convert a char array input to a 32 bit float.
 *
 * The char array has the form 
 *
 * "[-|+][x...][.][y...]"
 *
 * where x... and y... represent an arbitrary number of digits.
 *
 * s : array of chars
 */
float tofloat(const char * s);

#endif
