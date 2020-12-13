#ifndef _DAC_CONSTANTS_H_
#define _DAC_CONSTANTS_H_

//----------------------------------------------------------------------------
// DAC

#define DAC_VALUE_LIMIT 0xFF

#define MAX_DAC_VOLTAGE 1.32f
#define MAX_DAC_VOLTAGE_F16 (132 * 65536 / 100)

inline unsigned char voltageToDACValue(float voltage)
{
    if(voltage < 0.0f) {
        return 0x0;
    }
    uint32_t value = (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
    if(value >= DAC_VALUE_LIMIT) {
        return DAC_VALUE_LIMIT;
    }
    return value;
}

inline unsigned char voltageToDACValueNoBounds(float voltage)
{
    return (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
}

inline int voltageToDACValueFixed16NoBounds(int voltage)
{
    return (uint32_t)(voltage * 65535 / MAX_DAC_VOLTAGE_F16) * 256;
}

#endif /* _DAC_CONSTANTS_H_ */
