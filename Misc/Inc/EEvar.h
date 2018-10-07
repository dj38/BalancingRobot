/*
 * EEvar.h
 *
 *  Created on: 23 avr. 2018
 *      Author: joel
 */

#ifndef INC_EEVAR_H_
#define INC_EEVAR_H_

#include "eeprom.h"
#define EEVARMAXSIZE 4 // size in byte

class EEvarBase
{
public:
	EEvarBase(uint16_t varIndex,uint16_t sizeoff);
	virtual ~EEvarBase();
	virtual bool write(bool force=false)=0;
protected:
	uint8_t m_size16bits; // size in multiples of 16 bits
	uint16_t m_virtualAdress;
};

template <typename T> class EEvar : public EEvarBase
{
public:
	EEvar(T defaultValue, uint16_t varIndex);
	virtual ~EEvar() {};
	virtual bool write(bool force=false);
	inline void setRamVal(T val) {m_ramVal=val;}
	inline T getVal() {return(m_ramVal);}
private:
	bool readEE();
	bool read();
	T m_ramVal, m_EEVal;
};



template <typename T> EEvar<T>::EEvar(T defaultValue, uint16_t varIndex) : EEvarBase(varIndex,sizeof(T))
{
	if(!read()) { //when no data found in EEprom matching virtualAddress
		m_ramVal=defaultValue;
		write(true);
	}
	// todo m_EEval not defined
}

#define FLASH_COMPLETE HAL_OK

template <typename T> bool  EEvar<T>::write(bool force) {
	if(!force&&(m_ramVal==m_EEVal)) return(true); // no need to write if ram and EE values match and force mode disabled
	//*(reinterpret_cast<int*>(&f));
	uint32_t tmp=*(reinterpret_cast<uint32_t*>(&m_ramVal));
	uint16_t write2ByteData;
	for(uint8_t index=0; index < m_size16bits ; ++index) {
		write2ByteData=tmp>>(16*index);
		if (EE_WriteVariable(m_virtualAdress+index,write2ByteData)!=FLASH_COMPLETE)
			return(false);
		// TODO handle other error status (PAGE_FULL, NO_VALID_PAGE...)
	}
	m_EEVal=m_ramVal;
	return(true);
}

template <typename T> bool  EEvar<T>::read() {
	uint32_t tmp=0;
	uint16_t read2ByteData;
	for(uint8_t index=0; index < m_size16bits ; ++index) {
		if (EE_ReadVariable(m_virtualAdress+index,&read2ByteData)!=FLASH_COMPLETE)
			return(false);
		// TODO handle other error status (PAGE_FULL, NO_VALID_PAGE...)
		tmp|=(read2ByteData<<(16*index));
	}
	m_ramVal=m_EEVal=*(reinterpret_cast<T*>(&tmp));
	return(true);
}

#endif /* INC_EEVAR_H_ */
