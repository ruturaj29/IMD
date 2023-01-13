#ifndef _CONVERT_H_
#define _CONVERT_H_

void ftoa(float n, char *res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char *str, int len); 
int ascii_integer(char *string);
float stof(char* s);
void ftoa_signed(float a, char *data, int place); // converts signed float also


#endif
