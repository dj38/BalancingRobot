/*
 * EEvar.cpp
 *
 *  Created on: 23 avr. 2018
 *      Author: joel
 */

#include <EEvar.h>

EEvarBase::EEvarBase(uint16_t varIndex,uint16_t sizeoff) : m_virtualAdress(EEVARMAXSIZE*varIndex) {
	m_size16bits=sizeoff/2;
	m_size16bits=(m_size16bits<1?1:m_size16bits);
	if(!EE_bookVirtAdd(m_virtualAdress,m_size16bits))
		while(1); // TODO loop forever if address is not free
}

EEvarBase::~EEvarBase() {
}
