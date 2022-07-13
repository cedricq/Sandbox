#include "measurements.hpp"
#include "main_cpp.hpp"
#include "main.h"

#include "crc.hpp"
#include "DataAccessor.hpp"

#include "stm32f3xx_hal.h"

#include <string.h>

#define ADC_IN1_PA0_POUT     0
#define ADC_IN11_PB0_PROX    1
#define ADC_IN6_PC0_I_MOT    2
#define ADC_IN7_PC1_S_MOT    3

#define SFM3219_ADDRESS     0x2E
#define SFM3219_START_AIR   0x3608

typedef enum{
    I2C_INIT = 0,
    I2C_READ = 1
}i2c_states;

const int32_t offsetPout = -366;
const int32_t offsetPprox = -366;
const int32_t offsetQout = -57;

#define ADC_BUF_LEN 4
uint32_t adc_buf[ADC_BUF_LEN];

#define MEASURES_BUF_LEN 5
int32_t Measures[MEASURES_BUF_LEN];
#define MEAS_QOUT     0
#define MEAS_POUT     1
#define MEAS_PPROX    2
#define MEAS_I_MOT    3
#define MEAS_S_MOT    4

char const* data_names[] =
{
        "time",
        "qout",
        "pout",
        "pprox",
        "mot_speed",
        "mot_current"
};



const int32_t MAX_CURRENT   = 300;
const int32_t MAX_SPEED     = 60000;
const int32_t MAX_RAW_ADC   = 4095;

int32_t VoltageTo01mbar(int32_t volt, int32_t offset)
{
    return ( ( ( ( volt - 33 ) * 13790 ) / 264 ) - 6895) - offset;
}

int32_t RawADCToVolt(int32_t raw)
{
    return raw * 330 / MAX_RAW_ADC;
}

int32_t RawSFM3019ToLmin(int32_t raw, int32_t offset)
{
    return ( ( 100 * ( raw + 24576 ) ) / 170 ) - offset;
}

int32_t RawToCal(int32_t raw, int32_t max_cal, int32_t max_raw)
{
    return (raw * max_cal) / max_raw;
}


uint8_t crc = 0;
uint8_t i2c_state = 0;
uint8_t i2c_retry = 0;

int32_t rawQout = 0;

uint8_t cmd[3] = {0x36, 0x08, 0x00};
void InitQoutSensor()
{
    crc = SF04_CalcCrc (cmd, 2);
    cmd[2] = crc;

    HAL_GPIO_WritePin(CmdQout_GPIO_Port, CmdQout_Pin, GPIO_PIN_SET);
    i2c_state = I2C_INIT;
}

#define I2C_INIT_ERROR_MSG "INIT_ERROR\n\0"
#define I2C_CRC_ERROR_MSG  "CHECKSUM_ERROR\n\0"
#define I2C_READ_ERROR_MSG "READING_ERROR\n\0"

void ReadQoutSensor()
{
    static int tick_i2c = 0;
    rawQout = -24576;

    if (i2c_state == I2C_INIT)
    {
        if (tick_i2c <= 300)
        {
            HAL_GPIO_WritePin(CmdQout_GPIO_Port, CmdQout_Pin, GPIO_PIN_SET);
        }
        else if (tick_i2c <= 400)
        {
            HAL_GPIO_WritePin(CmdQout_GPIO_Port, CmdQout_Pin, GPIO_PIN_RESET);
        }
        else
        {
            uint8_t status = HAL_I2C_Master_Transmit(p_hi2c1, SFM3219_ADDRESS<<1, cmd, 3, 1000);
            if (status == HAL_OK)
            {
                i2c_state = I2C_READ;
                i2c_retry = 0;
            }
            else
            {
                char buffer[] = I2C_INIT_ERROR_MSG;
                HAL_UART_Transmit(p_huart3, (uint8_t*)buffer, (uint8_t)(strlen(buffer)), 100U);
            }
            tick_i2c = 0;
        }
        tick_i2c++;
    }
    else if (i2c_state == I2C_READ)
    {
        uint8_t i2c_rcv_buff[3] = {0x00, 0x00, 0x00};
        uint8_t status = HAL_I2C_Master_Receive(p_hi2c1, SFM3219_ADDRESS<<1, i2c_rcv_buff, 3, 1000);
        if (status == HAL_OK)
        {
            if (SF04_CheckCrc (i2c_rcv_buff, 2, i2c_rcv_buff[2]) != CHECKSUM_ERROR)
            {
                rawQout = (int16_t)(((uint16_t)i2c_rcv_buff[0])<<8 | i2c_rcv_buff[0]);
            }
            else
            {
                char buffer[] = I2C_CRC_ERROR_MSG;
                HAL_UART_Transmit(p_huart3, (uint8_t*)buffer, (uint8_t)(strlen(buffer)), 100U);
                i2c_retry++;
            }
        }
        else
        {
            char buffer[] = I2C_READ_ERROR_MSG;
            HAL_UART_Transmit(p_huart3, (uint8_t*)buffer, (uint8_t)(strlen(buffer)), 100U);
            i2c_retry++;
        }
        if (i2c_retry >= 10)
        {
            tick_i2c = 0;
            i2c_state = I2C_INIT;
        }
    }
}

void UpdateMeasurements()
{
    static DataItem qout(QOUT_ID, true);
    static DataItem pout(POUT_ID, true);
    static DataItem pprox(PPROX_ID, true);
    static DataItem motor_current(MOTOR_CURRENT_ID, true);
    static DataItem motor_speed(MOTOR_SPEED_ID, true);

    Measures[MEAS_QOUT]   = RawSFM3019ToLmin(rawQout, offsetQout);
    Measures[MEAS_POUT]   = VoltageTo01mbar( RawADCToVolt( adc_buf[ADC_IN1_PA0_POUT] ), offsetPout );
    Measures[MEAS_PPROX]  = VoltageTo01mbar( RawADCToVolt( adc_buf[ADC_IN11_PB0_PROX] ), offsetPprox );
    Measures[MEAS_I_MOT]  = RawToCal(adc_buf[ADC_IN6_PC0_I_MOT], MAX_CURRENT, MAX_RAW_ADC);
    Measures[MEAS_S_MOT]  = RawToCal(adc_buf[ADC_IN7_PC1_S_MOT], MAX_SPEED, MAX_RAW_ADC);

    qout.set(RawSFM3019ToLmin(rawQout, offsetQout));
    pout.set(VoltageTo01mbar( RawADCToVolt( adc_buf[ADC_IN1_PA0_POUT] ), offsetPout ));
    pprox.set(VoltageTo01mbar( RawADCToVolt( adc_buf[ADC_IN11_PB0_PROX] ), offsetPprox ));
    motor_current.set(RawToCal(adc_buf[ADC_IN6_PC0_I_MOT], MAX_CURRENT, MAX_RAW_ADC));
    motor_speed.set(RawToCal(adc_buf[ADC_IN7_PC1_S_MOT], MAX_SPEED, MAX_RAW_ADC));

}

void PrintMeasurements()
{
    float tick = HAL_GetTick();
    tick /= 1000;

    float measures[6];
    measures[0] = tick;
    measures[1] = ((float)Measures[MEAS_QOUT])/100;
    measures[2] = ((float)Measures[MEAS_POUT])/100;
    measures[3] = ((float)Measures[MEAS_PPROX])/100;
    measures[4] = ((float)Measures[MEAS_S_MOT]);
    measures[5] = ((float)Measures[MEAS_I_MOT])/100;


    //printFloats(measures, 6);
    printFloatsTelePlot(measures, data_names, 6);
}


void initADC()
{
    HAL_ADC_Start_DMA(p_hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
}


void init_measurements()
{
    initADC();
    InitQoutSensor();
}

void tick_measurements()
{
    static int time = 0;

    ReadQoutSensor();
    UpdateMeasurements();

    if (time%10 == 0)
    {
        PrintMeasurements();
    }

    time += 1;
}

