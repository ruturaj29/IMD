
/*************************** include Header files ***************************************/
#include "convert.h"
#include <math.h>

/*****************************************************************************************
*                     Convert Float to ASCII      																			 *
*****************************************************************************************/
void ftoa(float n, char *res, int afterpoint)
{

	// Extract integer part
	int ipart = (int)n;
 	
	// Extract floating part
	float fpart = n - (float)ipart;
	 
	// convert integer part to string
	int i = intToStr(ipart, res, 2);
	 
	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.';  // add dot
		
		// Get the value of fraction part upto given no.
	    // of points after dot. The third parameter is needed
	    // to handle cases like 233.007
	   // fpart = fpart * pow(10, afterpoint);
		fpart = fpart * pow(10, afterpoint)+0.5;
	 	intToStr((int)fpart, res + i + 1, afterpoint);
	 }
}

/*****************************************************************************************
*                     Convert Integer to String      																		 *
*****************************************************************************************/		
int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x)
	{
		str[i++] = (x%10) + '0';
	    x = x/10;
	} 
	
	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
	str[i++] = '0';
	
	reverse(str, i);
	str[i] = '\0';
	return i;
}

/*****************************************************************************************
*                     reverses a string 'str' of length 'len'      											 *
*****************************************************************************************/
void reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
	    str[i] = str[j];
	    str[j] = temp;
	    i++; j--;
	 }
}

/*****************************************************************************************
*                     Convert the ASCII to Integer                											 *
*****************************************************************************************/
int ascii_integer(char *string){
   int result = 0;
  while (*string)
  {
     if (*string >='0' && *string <='9')
     {
        result *= 10;
        result += *string - '0';
     }
     string++;
		 if(*string == '.')
		 {
		   *string='\0';
		 }
  }
  return result;
}

/*****************************************************************************************
*                     Convert the String to Float                 											 *
*****************************************************************************************/
float stof(char* s){
	int point_seen,d;
  float rez = 0, fact = 1;
  if (*s == '-'){
    s++;
    fact = -1;
  }
  for (point_seen = 0; *s; s++){
    if (*s == '.'){
      point_seen = 1; 
      continue;
    }
    d = *s - '0';
    if (d >= 0 && d <= 9){
      if (point_seen) fact /= 10.0f;
      rez = rez * 10.0f + (float)d;
    }
  }
  return rez * fact;
}

/*****************************************************************************************
*                     Convert the Float to ASCII Signed            											 *
*****************************************************************************************/
void ftoa_signed(float a, char *data, int place) //definition
{
     int temp=a;
     float x=0.0;
     int digits=0;
     int i=0,mu=1;
     int j=0;
     if(a<0)
     {
            a=a*-1;
            data[i]='-';
            i++;
      }
	  if((a==0) || (a<1) )
     {
            
            data[i]='0';
            i++;
      }

     //exponent component
     while(temp!=0)
     {
         temp=temp/10;
         digits++;          
     }
     while(digits!=0)
     {
         if(digits==1)mu=1;
         else  for(j=2;j<=digits;j++)mu=mu*10;
         
         x=a/mu;
         a=a-((int)x*mu);
         data[i]=0x30+((int)x);
         i++;
         digits--;
         mu=1;
     }
     //mantissa component
     data[i]='.';
     i++;
     digits=0;
     for(j=1;j<=place;j++)mu=mu*10;
     x=(a-(int)a)*mu; //shift places
     a=x;
     temp=a;
     x=0.0;
     mu=1;
     digits=place;
     while(digits!=0)
     {
         if(digits==1)mu=1;
         else  for(j=2;j<=digits;j++)mu=mu*10;
         
         x=a/mu;
         a=a-((int)x*mu);
         data[i]=0x30+((int)x);
         i++;
         digits--;
         mu=1;
     }   
     
    data[i]='\0';
}

