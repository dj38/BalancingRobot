/*
 * Tests.cpp
 *
 *  Created on: 12 oct. 2017
 *      Author: joel
 */

#include "Tests.h"
#include "Timer.h"

namespace Tests {
	Motor *m_motR;
	Motor *m_motL;
	Encoder *m_motREnc;
	Encoder *m_motLEnc;
	Motors *m_motors;
	MPU6050 *m_mpu;
	IOSerialStream *m_serialUSB;
	BTCom *m_serialHC06;

	void procTestMotorUnit(float pwmFreq,float force,TimeOut * encTimeOut);

}  // namespace Tests


void Tests::setObjectsRefs(Motor *motR,Motor *motL,Encoder *motREnc,Encoder *motLEnc,Motors *motors,MPU6050 *mpu,
		IOSerialStream *serialUSB,BTCom *serialHC06) {
	m_motR=motR;
	m_motL=motL;
	m_motREnc=motREnc;
	m_motLEnc=motLEnc;
	m_motors=motors;
	m_mpu=mpu;
	m_serialUSB=serialUSB;
	m_serialHC06=serialHC06;
}


void Tests::procTestMotorUnit(float pwmFreq,float force,TimeOut * encTimeOut) {
	m_motors->setPWM(force);
	for (int temp10ms = 0; temp10ms < 200; ++temp10ms) {
		while(!encTimeOut->repeatedTimeoutCheck());
		m_motors->computeSpeed();
	}
	*m_serialUSB << pwmFreq << ";" << force << ";" << m_motLEnc->getSpeed() << ";" << m_motREnc->getSpeed() << "\n\r";
}

void Tests::procTestMotorLinearity() {
	*m_serialUSB << "pwmFreq;power;leftSpeed;rightSpeed \n\r";
	TimeOut encTimeOut;
	encTimeOut.setTimeOut(10000);
	long int freqs[4];
	freqs[0]=17000;
	freqs[1]=17500;
	freqs[2]=18000;
	freqs[3]=18500;
	for (int powFreq = 0; powFreq <=3; ++powFreq) {
		//		long int pwmFreq=200*pow(2,powFreq);
		long int pwmFreq=freqs[powFreq];
		m_motL->pwmFreq(pwmFreq);
		m_motR->pwmFreq(pwmFreq);
		for (int power = -10; power <= -1; ++power) {
			float fpower=(float)power/10.0;
			procTestMotorUnit(pwmFreq,fpower,&encTimeOut);
		}
		for (int power = -9; power <= 0; ++power) {
			float fpower=(float)power/100.0;
			procTestMotorUnit(pwmFreq,fpower,&encTimeOut);
		}
		for (int power = 1; power <= 9; ++power) {
			float fpower=(float)power/100.0;
			procTestMotorUnit(pwmFreq,fpower,&encTimeOut);
		}
		for (int power = 1; power <= 10; ++power) {
			float fpower=(float)power/10.0;
			procTestMotorUnit(pwmFreq,fpower,&encTimeOut);
		}
	}
	m_motors->setPWM(0);
	while (1) {};
}
