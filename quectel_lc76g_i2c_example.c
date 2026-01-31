#define QUECTEL_I2C_SLAVE_CR_CMD 0xaa51
#define QUECTEL_I2C_SLAVE_CW_CMD 0xaa53
#define QUECTEL_I2C_SLAVE_CMD_LEN 8
#define QUECTEL_I2C_SLAVE_TX_LEN_REG_OFFSET 0x08
#define QUECTEL_I2C_SLAVE_TX_BUF_REG_OFFSET 0x2000
#define QUECTEL_I2C_SLAVE_RX_LEN_REG_OFFSET 0x04
#define QUECTEL_I2C_SLAVE_RX_BUF_REG_OFFSET 0x1000
#define QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW 0x50
#define QUECTEL_I2C_SLAVE_ADDRESS_R 0x54
#define QUECTEL_I2C_SLAVE_ADDRESS_W 0x58
#define MAX_ERROR_NUMBER 20
#define MAX_I2C_BUFFER 1024
typedef enum
{
I2C_ACK = 0,
I2C_NACK = 1
}I2c_Resp_FlagStatus;
typedef enum
{
DEV_REP_SUCCESS = 0,
DEV_REP_ERROR = 1
}Dev_Resp_FlagStatus;
I2c_Resp_FlagStatus I2c_Master_Receive(uint8_t addr, uint8_t *Data, uint16_t Length)

{
 mcu_i2c_start();
 mcu_i2c_send_byte(addr|0x01);
 if(mcu_i2c_wait_ack() != I2C_ACK)
{
mcu_i2c_stop();
return I2C_NACK;
}
 for(int i = 0; i < Length; i++)
 {
 *(Data + i) = mcu_i2c_receive_byte();
 if(i != (Length - 1))
 {
 mcu_i2c_ack();
 }
 }
 mcu_i2c_no_ack();
 mcu_i2c_stop();
return I2C_ACK;
}
I2c_Resp_FlagStatus I2c_Master_Transmit(uint8_t addr, uint8_t *Data, uint8_t Length)
{
 uint8_t i = 0;
uint8_t flag=0;
 mcu_i2c_start();
 mcu_i2c_send_byte(addr);
 if(mcu_i2c_wait_ack() == I2C_NACK)
{
mcu_i2c_stop();
return I2C_NACK;
}
 for(i = 0; i < Length; i++)
 {
 mcu_i2c_send_byte(*(Data+i));
 if(mcu_i2c_wait_ack() == I2C_NACK)
{
mcu_i2c_stop();
return I2C_NACK;
}
 }
 mcu_i2c_stop();
return I2C_ACK;
}

Dev_Resp_FlagStatus Quectel_Dev_Receive(uint8_t* pData, uint16_t maxLength, uint16_t*
pRecLength)
{
uint32_t request_cmd[2];
uint16_t* pRxLength = pRecLength;
uint8_t* pBuff = pData;
uint8_t i2c_master_receive_error_counter = 0;
I2c_Resp_FlagStatus status;
//step 1_a
request_cmd[0] = (uint32_t)((uint32_t)(QUECTEL_I2C_SLAVE_CR_CMD << 16) |
QUECTEL_I2C_SLAVE_TX_LEN_REG_OFFSET);
request_cmd[1] = 4;
i2c_master_receive_error_counter = 0;
while(1)
{
delay_ms(10);
status = I2c_Master_Transmit(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW << 1, (uint8_t
*)request_cmd, QUECTEL_I2C_SLAVE_CMD_LEN);
if(status == I2C_ACK)
{
break;
}
i2c_master_receive_error_counter++;
if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)
{
return DEV_REP_ERROR;
}
}
//step 1_b
i2c_master_receive_error_counter = 0;
while(1)
{
delay_ms(10);
status = I2c_Master_Receive(QUECTEL_I2C_SLAVE_ADDRESS_R << 1,
(uint8_t*)pRxLength, 4);
if(status == I2C_ACK)
{
break;
}

i2c_master_receive_error_counter++;
if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)
{
return DEV_REP_ERROR;
}
}
if(*pRxLength == 0)
{
return DEV_REP_ERROR;
}
if(*pRxLength > MAX_I2C_BUFFER)
{
*pRxLength = MAX_I2C_BUFFER;
}
//step 2_a
request_cmd[0] = (uint32_t)(QUECTEL_I2C_SLAVE_CR_CMD << 16) |
QUECTEL_I2C_SLAVE_TX_BUF_REG_OFFSET;
request_cmd[1] = *pRxLength;
i2c_master_receive_error_counter = 0;
while(1)
{
delay_ms(10);
status = I2c_Master_Transmit(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW << 1, (uint8_t
*)request_cmd, QUECTEL_I2C_SLAVE_CMD_LEN);
if(status == I2C_ACK)
{
break;
}
i2c_master_receive_error_counter++;
if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)
{
*pRxLength = 0;
return DEV_REP_ERROR;
}
}
//step 2_b
i2c_master_receive_error_counter = 0;
while(1)
{

delay_ms(10);
status = I2c_Master_Receive(QUECTEL_I2C_SLAVE_ADDRESS_R << 1, pBuff,
*pRxLength);
if(status == I2C_ACK)
{
return DEV_REP_SUCCESS;
}
i2c_master_receive_error_counter++;
if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)
{
*pRxLength = 0;
return DEV_REP_ERROR;
}
}
return DEV_REP_SUCCESS;
}
Dev_Resp_FlagStatus Quectel_Dev_Transmit(uint8_t *pData, uint16_t dataLength)
{
 uint32_t request_cmd[2];
uint16_t rxBuffLength = 0;
uint8_t i2c_master_receive_error_counter = 0;
I2c_Resp_FlagStatus status;
//step 1_a
request_cmd[0] = (uint32_t)((QUECTEL_I2C_SLAVE_CR_CMD << 16) |
QUECTEL_I2C_SLAVE_RX_LEN_REG_OFFSET);
request_cmd[1] = 4;
i2c_master_receive_error_counter = 0;
while(1)
{
delay_ms(10);
status = I2c_Master_Transmit(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW << 1, (uint8_t
*)request_cmd, QUECTEL_I2C_SLAVE_CMD_LEN);
if(status == I2C_ACK)
{
break;
}

i2c_master_receive_error_counter++;
if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)
{
return DEV_REP_ERROR;
}
}
//step 1_b
i2c_master_receive_error_counter = 0;
while(1)
{
delay_ms(10);
status = I2c_Master_Receive(QUECTEL_I2C_SLAVE_ADDRESS_R << 1,
(uint8_t*)&rxBuffLength, 4);
if(status == I2C_ACK)
{
break;
}
i2c_master_receive_error_counter++;
if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)
{
return DEV_REP_ERROR;
}
}
if(dataLength > rxBuffLength)
{
return DEV_REP_ERROR;
}
//step 2_a
request_cmd[0] = (uint32_t)(QUECTEL_I2C_SLAVE_CW_CMD << 16) |
QUECTEL_I2C_SLAVE_RX_BUF_REG_OFFSET;
request_cmd[1] = dataLength;
i2c_master_receive_error_counter = 0;
while(1)
{
delay_ms(10);
status = I2c_Master_Transmit(QUECTEL_I2C_SLAVE_ADDRESS_CR_OR_CW << 1, (uint8_t
*)request_cmd, QUECTEL_I2C_SLAVE_CMD_LEN);
if(status == I2C_ACK)
{
break;

}
i2c_master_receive_error_counter++;
if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)
{
return DEV_REP_ERROR;
}
}
//step 2_b
i2c_master_receive_error_counter = 0;
while(1)
{
delay_ms(10);
status = I2c_Master_Transmit(QUECTEL_I2C_SLAVE_ADDRESS_W << 1, pData,
dataLength);
if(status == I2C_ACK)
{
return DEV_REP_SUCCESS;
}
i2c_master_receive_error_counter++;
if(i2c_master_receive_error_counter > MAX_ERROR_NUMBER)
{
return DEV_REP_ERROR;
}
}
return DEV_REP_SUCCESS;
}

