/*
 * slave_side.c - COMPLETE FIXED VERSION
 *
 *  Created on: Feb 4, 2026
 *      Author: Amal
 */

#include "main.h"
#include "slave_side.h"
#include "lora_config.h"
#include "protocol.h"
#include "Motor.h"
#include "gps.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi3;

#define EXPECTED_LEN 		8
#define MOTOR_PULSE_TIME 	400

uint8_t motor_continuous = 0;
uint32_t motor_pulse_start = 0;
uint8_t motor_pulse_active = 0;



uint8_t lora_get_status(void);
void process_command(uint8_t cmd, uint16_t param);


/* ================= LOW LEVEL FUNCTIONS ================= */

static void cs_low(void)
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

static void lora_wait_busy(void)
{

    while(HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET);
}

static void lora_cmd(uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t buffer[16];

    buffer[0] = cmd;
    for(uint8_t i = 0; i < len; i++)
        buffer[i+1] = data[i];

    lora_wait_busy();
    cs_low();
    HAL_SPI_Transmit(&hspi3, buffer, len + 1, 100);
    cs_high();
    lora_wait_busy();
}

/* ================= STATUS AND IRQ FUNCTIONS ================= */

uint8_t lora_get_status(void)
{
    uint8_t tx[2] = {0xC0, 0x00};
    uint8_t rx[2];

    cs_low();
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 2, 100);
    cs_high();

    return rx[1];
}

uint16_t lora_get_irq_status(void)
{
    uint8_t cmd = 0x12;   // GetIrqStatus
    uint8_t dummy = 0x00;
    uint8_t status;
    uint8_t irqBuf[2];

    lora_wait_busy();

    cs_low();

    // Send command
    HAL_SPI_Transmit(&hspi3, &cmd, 1, 100);

    // Send dummy to receive status byte
    HAL_SPI_TransmitReceive(&hspi3, &dummy, &status, 1, 100);

    // Now receive IRQ MSB and LSB
    HAL_SPI_Receive(&hspi3, irqBuf, 2, 100);

    cs_high();

    lora_wait_busy();

    return ((uint16_t)irqBuf[0] << 8) | irqBuf[1];
}
void lora_clear_irq(uint16_t irq)
{
    uint8_t cmd = 0x02;  // ClearIrqStatus
    uint8_t buf[2];

    buf[0] = (irq >> 8) & 0xFF;
    buf[1] = irq & 0xFF;

    lora_wait_busy();
    cs_low();
    HAL_SPI_Transmit(&hspi3, &cmd, 1, 100);
    HAL_SPI_Transmit(&hspi3, buf, 2, 100);
    cs_high();
    lora_wait_busy();
}

uint8_t lora_get_op_mode(void)
{
    uint8_t status = lora_get_status();
    return (status >> 4) & 0x07;  // Bits 4-6: 3=RX, 2=TX, 1=STDBY
}

/* ================= RX BUFFER FUNCTIONS ================= */

void lora_get_rx_buffer_status(uint8_t *len, uint8_t *offset)
{
    uint8_t cmd = 0x13;
    uint8_t buf[3];

    lora_wait_busy();
    cs_low();

    HAL_SPI_Transmit(&hspi3, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi3, buf, 3, 100);

    cs_high();
    lora_wait_busy();

    *len = buf[1];
    *offset = buf[2];
}

void lora_read_buffer(uint8_t offset, uint8_t *data, uint8_t len)
{
    uint8_t cmd = 0x1E; // ReadBuffer
    uint8_t dummy = 0;

    lora_wait_busy();
    cs_low();
    HAL_SPI_Transmit(&hspi3, &cmd, 1, 100);
    HAL_SPI_Transmit(&hspi3, &offset, 1, 100);
    HAL_SPI_Transmit(&hspi3, &dummy, 1, 100); // Status byte
    HAL_SPI_Receive(&hspi3, data, len, 100);
    cs_high();
    lora_wait_busy();
}

/* ================= WRITE REGISTER ================= */

void lora_write_reg(uint16_t reg, uint8_t value)
{
    uint8_t buf[3];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = value;

    lora_wait_busy();
    cs_low();
    uint8_t cmd = 0x0D; // WriteRegister
    HAL_SPI_Transmit(&hspi3, &cmd, 1, 100);
    HAL_SPI_Transmit(&hspi3, buf, 3, 100);
    cs_high();
    lora_wait_busy();
}

/* ================= LORA INITIALIZATION ================= */

static void LORA_Init(void)
{
	 // Reset
	    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
	    HAL_Delay(20);
	    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
	    HAL_Delay(50);

	    lora_wait_busy();

	    // Standby RC
	    uint8_t standby = 0x00;
	    lora_cmd(0x80, &standby, 1);


	    uint8_t regMode = 0x01;
	    lora_cmd(0x96, &regMode, 1);

	    // Packet type LoRa
	    uint8_t pktType = 0x01;
	    lora_cmd(0x8A, &pktType, 1);

	    uint8_t enable = 0x01;
	    lora_cmd(0x9D, &enable, 1);

	    lora_write_reg(0x0740, (LORA_SYNC_WORD >> 8));
	    lora_write_reg(0x0741, (LORA_SYNC_WORD & 0xFF));

	    // RF frequency 433 MHz
	    uint32_t frf = 0x1B2C0000;
	    uint8_t f[4] = { frf >> 24, frf >> 16, frf >> 8, frf };
	    lora_cmd(0x86, f, 4);

	    // Calibrate image
	    uint8_t img[2] = {0x6B, 0x6F};
	    lora_cmd(0x98, img, 2);

	    // Modulation params
	    uint8_t mod[4] = { LORA_SF, LORA_BW, LORA_CR, 0x00 };	//sf-7 -0x07,bw-0x04,cr-0x01
	    lora_cmd(0x8B, mod, 4);

	    // Packet params
	    uint8_t pkt[6] = {
	        0x00, 0x08,     // preamble 8
	        0x00,           // explicit header
	        8,              // payload length
	        0x01,           // CRC ON
	        0x00            // normal IQ
	    };
	    lora_cmd(0x8C, pkt, 6);

	    // Buffer base
	    uint8_t bufBase[2] = {0x00, 0x00};
	    lora_cmd(0x8F, bufBase, 2);

	    // Enable IRQ: RX_DONE | CRC_ERR | TIMEOUT
	    uint8_t irqParams[8] = {
	        0x00, 0x42,     // IRQ mask (0x0002 RX_DONE | 0x0040 CRC_ERR)
	        0x00, 0x00,     // DIO1 not used (polling)
	        0x00, 0x00,
	        0x00, 0x00
	    };
	    lora_cmd(0x08, irqParams, 8);

	    // Clear IRQ
	    uint8_t clr[2] = {0xFF, 0xFF};
	    lora_cmd(0x02, clr, 2);

	    // Enter continuous RX
	    uint8_t rxTimeout[3] = {0xFF, 0xFF, 0xFF};
	    lora_cmd(0x82, rxTimeout, 3);
}


void Slave_App_Init(void)
{
    LORA_Init();

}

void Slave_App_Loop(void)
{
	   uint16_t irq = lora_get_irq_status();
	    uint8_t status = lora_get_status();


    // Debug mode every second
    static uint32_t last_debug = 0;
    if(HAL_GetTick() - last_debug > 1000)
    {
        last_debug = HAL_GetTick();



    // Process any IRQ events
    if(irq != 0)
    {
        // Check for CRC error
        if(irq & 0x0004)  // CRC error
        {
            lora_clear_irq(irq);
            // Fast blink on error LED
            HAL_GPIO_TogglePin(DBG_PORT, DBG_OFF_PIN);
        }

        // Check for RX done
        if(irq & 0x0002)  // RX_DONE
        {
            uint8_t payload_len;
            uint8_t offset;
            uint8_t rx_buf[32];

            // Get buffer status
            lora_get_rx_buffer_status(&payload_len, &offset);

            if(payload_len == sizeof(RobotFrame_t))
            {
                // Read the packet
                lora_read_buffer(offset, rx_buf, payload_len);

                // Parse frame
                RobotFrame_t *frame = (RobotFrame_t*)rx_buf;

                // Validate frame
                if(frame->preamble == FRAME_PREAMBLE &&
                   frame->version == FRAME_VERSION)
                {
                    uint8_t crc = protocol_crc(rx_buf, sizeof(RobotFrame_t)-1);
                    if(crc == frame->crc)
                    {
                        // Check if for this robot or broadcast
                        if(frame->uid == ROBOT_UID || frame->uid == UID_BROADCAST)
                        {
                            // Process the command
                           // Slave_LoRa_Process_motor_control(frame->payload);
                        }
                    }
                }
            }

            // Clear the IRQ after processing
            lora_clear_irq(irq);
        }
        else if(!(irq & 0x0004))  // If not CRC error and not RX_DONE, clear other IRQs
        {
            lora_clear_irq(irq);
        }
    }

    // Ensure we stay in RX mode
    static uint32_t last_rx_check = 0;
    if(HAL_GetTick() - last_rx_check > 100)  // Check every 100ms
    {
        last_rx_check = HAL_GetTick();

        uint8_t mode = lora_get_op_mode();
        if(mode != 0x03)  // If not in RX mode (0x03 = RX)
        {
            // Re-enter RX mode
            uint8_t rx_timeout[3] = {0x00, 0x00, 0x00};
            lora_cmd(0x82, rx_timeout, 3);

            // Toggle error LED to show re-entry
            HAL_GPIO_TogglePin(DBG_PORT, DBG_OFF_PIN);
        }
    }

    // Small delay to prevent SPI flooding
    HAL_Delay(1);
}
}

void LORA_RX_Poll(void)
{
    uint16_t irq = lora_get_irq_status();

    // RX_DONE = 0x0002
    if (irq & 0x0002)
    {
        uint8_t len;
        uint8_t offset;

        // Get RX buffer status
        lora_get_rx_buffer_status(&len, &offset);

        uint8_t data[255];
        lora_read_buffer(offset, data, len);

        // Clear IRQ
        uint8_t clr[2] = {0xFF, 0xFF};
        lora_cmd(0x02, clr, 2);

        // --- VALIDATION STARTS HERE ---

        if (len != EXPECTED_LEN)
        {
            // Wrong length → ignore
            goto reenter_rx;
        }

        // Optional: verify software CRC (your XOR)
        uint8_t crc = 0;
        for (int i = 0; i < 7; i++)
            crc ^= data[i];

        if (crc != data[7])
        {
            // Corrupt packet → ignore
            goto reenter_rx;
        }

        // Extract fields
        uint16_t sync  = (data[0] << 8) | data[1];
        uint16_t uid   = (data[2] << 8) | data[3];
        uint8_t  cmd   = data[4];
        uint16_t param = (data[5] << 8) | data[6];

        // Check UID
        if (uid != ROBOT_UID)
        {
            // Not for me → ignore
            goto reenter_rx;
        }

        // ✅ VALID PACKET FOR THIS DEVICE
        process_command(cmd, param);

    reenter_rx:
        // Re-enter continuous RX
        uint8_t rxTimeout[3] = {0xFF, 0xFF, 0xFF};
        lora_cmd(0x82, rxTimeout, 3);
    }
}
void process_command(uint8_t cmd, uint16_t param)
{
    switch(cmd)
    {
        case CMD_MOTOR_ON:
            motor_continuous = 1;
            break;

        case CMD_MOTOR_OFF:
            motor_continuous = 0;
            break;

        case CMD_MOTOR_PULSE:
            motor_pulse_active = 1;
            motor_pulse_start = HAL_GetTick();
            break;
    }
}

void MOTOR_On(void)
{
	HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_INH_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_L_PIN, GPIO_PIN_RESET);
}

void MOTOR_Off(void)
{
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_INH_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_R_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_L_PIN, GPIO_PIN_RESET);
}

void Motor_Task(void)
{
    if(motor_continuous)
    {
        MOTOR_On();
        return;
    }

    if(motor_pulse_active)
    {
        if(HAL_GetTick() - motor_pulse_start < MOTOR_PULSE_TIME)
        {
            MOTOR_On();
        }
        else
        {
            MOTOR_Off();
            motor_pulse_active = 0;
        }
        return;
    }

    MOTOR_Off();
}


