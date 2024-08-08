/*
 * reprintf.h
 *
 *  Created on: Mar 31, 2024
 *      Author: ECA1WX
 */

#ifndef INC_REPRINTF_H_
#define INC_REPRINTF_H_

#ifdef __GNUC__

int _write(int file, char *ptr, int len);

#else

int fputc(int ch, FILE *f);

#endif

#endif /* INC_REPRINTF_H_ */
