/**
 * @file chromatic_scale.h
 * @author wlm2
 * @date 22.11.2021
 * @brief This header file defines an array of prescaler values to generate notes
 * with a timer.
 */

#ifndef INC_CHROMATIC_SCALE_H_
#define INC_CHROMATIC_SCALE_H_

#define NUMBER_OF_NOTES (128U)
#define CONCERT_PITCH_INDEX (69)

extern uint16_t const NOTES_PSC_16MHz[NUMBER_OF_NOTES];

uint16_t const NOTES_PSC_16MHz[NUMBER_OF_NOTES] = { 3819, 3605, 3402, 3211,
		3031, 2861, 2700, 2549, 2406, 2271, 2143, 2023, 1909, 1802, 1701, 1605,
		1515, 1430, 1350, 1274, 1202, 1135, 1071, 1011, 954, 900, 850, 802, 757,
		714, 674, 636, 601, 567, 535, 505, 477, 450, 424, 401, 378, 357, 337,
		318, 300, 283, 267, 252, 238, 224, 212, 200, 189, 178, 168, 158, 149,
		141, 133, 125, 118, 112, 105, 99, 94, 88, 83, 79, 74, 70, 66, 62, 59,
		55, 52, 49, 46, 44, 41, 39, 37, 34, 33, 31, 29, 27, 26, 24, 23, 21, 20,
		19, 18, 17, 16, 15, 14, 13, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 6, 6,
		5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1 };

#endif /* INC_CHROMATIC_SCALE_H_ */
