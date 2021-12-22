/*
 * controllfc.h
 *
 *  Created on: 22.12.2021
 *      Author: sebas
 */

#ifndef INC_CONTROLLFC_H_
#define INC_CONTROLLFC_H_

#include "main.h"


#include <stdlib.h>   /* For EXIT_SUCCESS */
#include <stdbool.h>  /* For true  */
#include <ctype.h> /* For toupper */

#define MAX_VOLUME 8
#define MIN_VOLUME 0

#define MAX_DURATION 200
#define MIN_DURATION 10

void pianoController(unsigned char c, int8_t *octave);
int8_t keyToNote(unsigned char c);


#endif /* INC_CONTROLLFC_H_ */
