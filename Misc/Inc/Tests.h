/*
 * Tests.h
 *
 *  Created on: 2 oct. 2017
 *      Author: joel
 */

#ifndef TESTS_H_
#define TESTS_H_

#include "Motors.h"
#include "MPU6050.h"
#include "BTCom.h"

namespace Tests {

	void setObjectsRefs(Motor *motR,Motor *motL,Encoder *motREnc,Encoder *motLEnc,Motors *motors,MPU6050 *mpu,
			SerialBuffer *serialUSB,BTCom *serialHC06);
	void procTestMotorLinearity();

}  // namespace Tests

#endif /* TESTS_H_ */
