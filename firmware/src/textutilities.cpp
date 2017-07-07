#include "textutilities.h"

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
float tofloat(const char * s)
{
  // Define and initialize variables for checking the sign of the number
  int index = 0;           // index of a character in the array
  char c = s[index];       // a character from the array
  bool is_positive = true; // true if the number is positive, false otherwise
  
  // Check whether the number is negative
  if (c == '-')          // character array starts with '-'
    is_positive = false; // the number is negative
  
  // Define and initialize variables for converting the character array to a positive number
  int integer = 0;        // the integer part of the number in the character array, e.g. 12 in "-12.34"
  int numerator = 0;      // the numerator for the fractional part of the number, e.g. 34 in "-12.34"
  int denominator = 1;    // the denominator for the fractional part of the number, e.g. 100 in "-12.34": -(12 + 34/100)
  bool is_integer = true; // becomes false once a dot has been detected, meaning the number has a fractional part
  
  // Loop over the character array to convert it to a positive number 
  while (c != '\0')           // character array ends with null terminator '\0'
  {
    // Parse the current character
    if ((c>='0') && (c<='9')) // the current character is a number
    {
      // Update the value of the number
      if (is_integer)         // no dot has been found yet, so we are dealing with the integer part
      {
        integer *= 10;        // multiply the integer part by 10 | e.g. 156 = 10*15
        integer += c - '0';   // add the character value         |                  + 6
      }
      else                    // a dot has been found, so we are dealing with the fractional part
      {
        numerator *= 10;      // multiply the numerator by 10   | e.g. 0.345 = (10*34
        numerator += c - '0'; // add the character value        |                     + 5)
        denominator *= 10;    // multiply the denominator by 10 |                          / (10*100)
      }
    }
    else if (c == '.')        // current character is a dot
      is_integer = false;     // a dot has been found, so the number has a fractional part
    
    // Update the index and retrieve the next character
    c = s[++index];
  }
  
  // Return the number with the correct sign
  float number = integer + static_cast<float>(numerator) / denominator;
  return (is_positive ? number : -number);
}
