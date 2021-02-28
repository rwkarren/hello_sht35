/*
    Seeed_SHT35.cpp
    Driver for SHT35

    Copyright (c) 2018 Seeed Technology Co., Ltd.
    Website    : www.seeed.cc
    Author     : downey
    Create Time: May 2018
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "Seeed_SHT35.h"


SHT35::SHT35(i2c_inst_t* i2c_bus, u8 scl_pin, u8 sda_pin, u8 IIC_ADDR) {
    set_i2c_bus(i2c_bus);
    set_iic_addr(IIC_ADDR);
    set_scl_pin(scl_pin);
    set_sda_pin(sda_pin);
    CLK_STRCH_STAT = CLK_STRETCH_DISABLE;
}

err_t SHT35::init() {
    err_t ret = NO_ERROR;
    IIC_begin();
    ret = soft_reset();
    return ret;
}



err_t SHT35::soft_reset() {
    err_t ret = NO_ERROR;
    ret = send_command(CMD_SOFT_RST);
    return ret;
}


err_t SHT35::read_meas_data_single_shot(u16 cfg_cmd, float* temp, float* hum) {
    err_t ret = NO_ERROR;
    u8 data[6] = {0};
    u16 temp_hex = 0, hum_hex = 0;
    if (cfg_cmd == HIGH_REP_WITH_STRCH) {
        CLK_STRCH_STAT = CLK_STRETCH_ENABLE;
    }
    CHECK_RESULT(ret, send_command(cfg_cmd));
    CHECK_RESULT(ret, read_bytes(data, sizeof(data), CLK_STRCH_STAT));

    temp_hex = (data[0] << 8) | data[1];
    hum_hex = (data[3] << 8) | data[4];

    *temp = get_temp(temp_hex);
    *hum = get_hum(hum_hex);

    return ret;
}


float SHT35::get_temp(u16 temp) {
    return (temp / 65535.00) * 175 - 45;
}

float SHT35::get_hum(u16 hum) {
    return (hum / 65535.0) * 100.0;
}



u16 SHT35::temp_to_hex(float temp) {
    return (u16)((temp + 45) * 65535.0 / 175);
}

u16 SHT35::hum_to_hex(float hum) {
    return (u16)(hum / 100.0 * 65535);
}


/******************************************************STATUS REG**************************************************/
/******************************************************STATUS REG**************************************************/



err_t SHT35::read_reg_status(u16* value) {
    err_t ret = NO_ERROR;
    *value = 0;
    u8 stat[3] = {0};
    CHECK_RESULT(ret, send_command(CMD_READ_SREG));
    CHECK_RESULT(ret, request_bytes(stat, sizeof(stat)));
    *value |= (u16)stat[0] << 8;
    *value |= stat[1];
    return ret;
}



err_t SHT35::heaterStatus(u16 status, bool stat) {
    stat = ((status >> 13) & 0x01);
    return NO_ERROR;
}

err_t SHT35::heaterStatus(bool stat) {
    err_t ret = NO_ERROR;
    u16 status = 0;
    CHECK_RESULT(ret, read_reg_status(&status));
    stat = ((status >> 13) & 0x01);
    return ret;
}
/****************************************************/



err_t SHT35::reset_check(u16 status, bool stat) {
    stat = ((stat >> 4) & 0x01);
    return NO_ERROR;
}

err_t SHT35::reset_check(bool stat) {
    err_t ret = NO_ERROR;
    u16 status = 0;
    CHECK_RESULT(ret, read_reg_status(&status));
    stat = ((stat >> 4) & 0x01);
    return ret;
}
/****************************************************/

err_t SHT35::cmd_excu_stat(u16 status, bool stat) {
    stat = ((stat >> 1) & 0x01);
    return NO_ERROR;
}

err_t SHT35::cmd_excu_stat(bool stat) {
    err_t ret = NO_ERROR;
    u16 status = 0;
    CHECK_RESULT(ret, read_reg_status(&status));
    stat = ((stat >> 1) & 0x01);
    return ret;
}
/****************************************************/
err_t SHT35::last_write_checksum(u16 status, bool stat) {
    stat = ((status >> 0) & 0x01);
    return NO_ERROR;
}
err_t SHT35::last_write_checksum(bool stat) {
    err_t ret = NO_ERROR;
    u16 status = 0;
    CHECK_RESULT(ret, read_reg_status(&status));
    stat = ((stat >> 0) & 0x01);
    return ret;
}

/***********************************************************************************************/
/**************************************EXEC COMMAND*********************************************/

err_t SHT35::change_heater_status(bool stat) {
    err_t ret = NO_ERROR;

    if (stat) {
        ret = send_command(CMD_HEATER_ON);
    } else {
        ret = send_command(CMD_HEATER_OFF);
    }

    return ret;
}

/***********************************************************************************************/
/*****************************************IIC OPRT**********************************************/
u8 SHT_IIC_OPRTS::crc8(const u8* data, int len) {

    const u8 POLYNOMIAL = 0x31;
    u8 crc = 0xFF;

    for (int j = len; j; --j) {
        crc ^= *data++;

        for (int i = 8; i; --i) {
            crc = (crc & 0x80)
                  ? (crc << 1) ^ POLYNOMIAL
                  : (crc << 1);
        }
    }
    return crc;
}

err_t SHT_IIC_OPRTS::send_command(u16 cmd) {
    s32 ret = 0;

    int frame_len = 2;
    uint8_t frame[frame_len]= {0};
    frame[0] = (cmd >> 8) & 0xFF;
    frame[1] = cmd & 0xFF;
    ret = i2c_write_blocking(_i2c_bus, _IIC_ADDR, frame, frame_len, false);
    
    if (!(ret == PICO_ERROR_GENERIC)) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}


err_t SHT_IIC_OPRTS::I2C_write_bytes(u16 cmd, u8* data, u32 len) {
    u8 crc = 0;
    s32 ret = 0;
    crc = crc8(data, len);

    int frame_len = len + 3; // total frame length is cmd + len + crc
    uint8_t frame[frame_len] = {0};
    frame[0] = (cmd >> 8) & 0xFF;
    frame[1] = cmd & 0xFF;
    for (int i = 0; i < len; i++) {
        frame[i + 2] = data[i];
    }
    frame[frame_len - 1] = crc;
    ret = i2c_write_blocking(_i2c_bus, _IIC_ADDR, frame, frame_len, false);

    if (!ret) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}

err_t SHT_IIC_OPRTS::request_bytes(u8* data, u16 data_len) {
    err_t ret = NO_ERROR;
    u32 time_out_count = 0;

    while (data_len != i2c_get_read_available(_i2c_bus)) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        sleep_ms(1);
    }
    i2c_read_blocking(_i2c_bus, _IIC_ADDR, data, data_len, false);

    return NO_ERROR;
}

/*SHT3X device is different from other general IIC device.*/
err_t SHT_IIC_OPRTS::read_bytes(u8* data, u32 data_len, clk_skch_t clk_strch_stat) {
    err_t ret = NO_ERROR;
    u32 time_out_count = 0;

    if (clk_strch_stat == CLK_STRETCH_ENABLE) {
        while (gpio_get(SCK_PIN) == 0) {
            // unsure of pico yield() equivalent? probably need code in here to avoid getting optimized out
            sleep_us(1);
        }
    } else {
        uint8_t dummy = 0;
        while (i2c_read_blocking(_i2c_bus, _IIC_ADDR, &dummy, 1, false) == PICO_ERROR_GENERIC) {
            // no need to begin transmission here as the Pico library doesn't behave the same way as the Arduino one
            sleep_us(1);
        }
    }
    int timeout_ms = 100;
    int result = 0;
    result = i2c_read_timeout_us(_i2c_bus, _IIC_ADDR, data, data_len, false, timeout_ms*1000);

    if (result == PICO_ERROR_TIMEOUT) {
        return ERROR_COMM;
    } 

    return NO_ERROR;
}


void SHT_IIC_OPRTS::set_scl_pin(u8 scl) {
    SCK_PIN = scl;
}

void SHT_IIC_OPRTS::set_sda_pin(u8 sda) {
    SDT_PIN = sda;
}

/** @brief change the I2C address from default.
    @param IIC_ADDR: I2C address to be set
 * */
void SHT_IIC_OPRTS::set_iic_addr(u8 IIC_ADDR) {
    _IIC_ADDR = IIC_ADDR;
}

void SHT_IIC_OPRTS::set_i2c_bus(i2c_inst_t* i2c_bus) {
    _i2c_bus = i2c_bus;
}