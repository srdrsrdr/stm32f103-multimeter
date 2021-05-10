/*
 * util.h
 *
 *  Created on: 30 Nis 2021
 *      Author: Sr_Dr
 */

#ifndef UTIL_H_
#define UTIL_H_

typedef char* String;

void* constrain(x,low,high)
{
	if(x < low) return low;
	if(x > high) return high;
}

#endif /* UTIL_H_ */
