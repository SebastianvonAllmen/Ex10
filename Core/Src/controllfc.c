#include "controllfc.h"

//This function maps every key entry to a note writen in int format
int8_t keyToNote(unsigned char c) {
	switch (c) {
	case 'a':
		return 0;
	case 'd':
		return 4;
	case 'e':
		return 3;
	case 'f':
		return 5;
	case 'g':
		return 7;
	case 'h':
		return 9;
	case 'j':
		return 11;
	case 'k':
		return 12;
	case 'l':
		return 14;
	case 'o':
		return 13;
	case 'p':
		return 15;
	case 's':
		return 2;
	case 't':
		return 6;
	case 'u':
		return 10;
	case 'w':
		return 1;
	case 'z':
		return 8;
	default:
		return -1;
	}
}

//This function handles all the other Buttons that manipulate how the piano sounds
void pianoController(unsigned char c, int8_t *octave) {
	static int8_t volume = 6; //Set inital volume to somewhere in the middle
	static int8_t toneduration = 50; //Set initial tone duration to roughly 0.2s
	switch (c) {
	case 'y':
		if ((*octave) > 0) //If octave is not between 0-16 we have run in to a problem
			(*octave)--;
		break;
	case 'x':
		if ((*octave) < 16) //If octave is not between 0-16 we have run in to a problem
			(*octave)++;
		break;
	case 'c':
		if (volume > MIN_VOLUME )
			--volume;
		break;
	case 'v':
		if (volume < MAX_VOLUME)
			++volume;
		break;
	case '1':
		if (toneduration < MAX_DURATION)
			toneduration = toneduration + 10;
		break;
	case '2':
		if (toneduration > MIN_DURATION)
			toneduration = toneduration - 10;
		break;
	default:
		break;
	}

	//After changing the local variables we write them to the registers
	WRITE_REG(TIM2->CCR3, 1 << (volume - 1));
	WRITE_REG(TIM7->ARR, toneduration);
}

