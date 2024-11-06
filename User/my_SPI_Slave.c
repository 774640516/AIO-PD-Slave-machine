/*
 * my_SPI_Slave.c
 *
 *  Created on: Apr 7, 2024
 *      Author: GW
 */
#include "debug.h"
#include "my_SPI_Slave.h"
#include "my_PD_Device.h"
#include "PD_Process.h"
#include "my_I2C_Device.h"
#define CONTROL_DATA_LEN 8
// #define DEVICE_INFO   0
// #define
#define CONTROL_DEVICE_SIZE 6

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SPI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

uint8_t test_number[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

#define CONTROL_DEVICE_WRITE_SIZE 4
const uint8_t control_write_addr[CONTROL_DEVICE_WRITE_SIZE] = {
    2, 3, 4, 6};

volatile MY_SPI_DEVICE my_spi;

extern PD_CONTROL PD_Ctl; /* PD Control Related Structures */

uint8_t send_spi_slave_data = 0xff;
uint8_t receive_data[10];
uint8_t receive_addr = 0;

uint8_t receive_buff[10];
uint8_t receive_buff_size = 0;
uint8_t receive_flag = 0;

uint8_t open_flag;
uint8_t close_flag;
uint8_t check_flag;
uint8_t set_addr4_data_falg = 0;
uint8_t set_addr4_data = 0;
uint8_t set_src_flag = 0;
uint8_t set_src_data1 = 0;
uint8_t set_src_data2 = 0;

uint8_t gpio_flag = 0;
uint8_t info_flag = 0;
uint8_t rssi_flag = 0;
uint8_t gpio_data = 0;
uint8_t info_data = 0;
uint8_t rssi_data[6];

static uint8_t aio_id_flag = 0;
static uint8_t aio_id_buff[4];

uint8_t printf_size[64];
uint8_t printf_addr = 0;

uint8_t test_delay_show = 0;

void my_SPI_Data_Init()
{
    my_spi.Start = 0;
    my_spi.Tx_TI_Init = 0;
    my_spi.Write_Len = 0;
    my_spi.Read_Len = 0;
    my_spi.Write_Status = 0;
    my_spi.Read_Status = 0;
    my_spi.Control_Addr = 0;
    for (uint8_t i = 0; i < CONTROL_DATA_LEN; i++)
    {
        my_spi.Control_Data[i] = 0;
    }

    my_spi.DIO_Status = 0;
    my_spi.DIO_Time = 0;

    open_flag = 0;
    close_flag = 0;
    check_flag = 0;
    set_addr4_data = 0;
    set_addr4_data_falg = 0;
    set_src_data1 = 0;
    set_src_data2 = 0;
    set_src_flag = 0;
}

void my_SPI_Device_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef SPI_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
    //        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
    SPI_I2S_SendData(SPI1, 0xff);

    SPI_Cmd(SPI1, ENABLE);

    //    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void my_SPI_Selve_Init()
{
    my_SPI_Data_Init();
    my_SPI_Device_Init();
}

void my_SPI_ControlData_on(uint8_t addr, uint8_t bit)
{
    my_spi.Control_Data[addr] |= (1 << bit);
    printf("set on %d %d\r\n", addr, my_spi.Control_Data[addr]);
}
void my_SPI_ControlData_off(uint8_t addr, uint8_t bit)
{
    my_spi.Control_Data[addr] &= ~(1 << bit);
    // printf("set 0ff %d %d\r\n",addr,my_spi.Control_Data[addr]);
}
void SPI1_IRQHandler(void)
{
    if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != RESET)
    {
        switch (SPI_I2S_ReceiveData(SPI1))
        {
        case 1:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[0]);
            break;
        case 2:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[1]);
            break;
        case 3:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[2]);
            break;
        case 4:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[3]);
            break;
        case 5:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[4]);
            send_spi_slave_data = my_spi.Control_Data[5];
            break;
        case 6:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[6]);
            send_spi_slave_data = my_spi.Control_Data[7];
            break;
        case 7:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[8]);
            send_spi_slave_data = my_spi.Control_Data[3];
            break;
        case 0x0a:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[0x0a]);
            break;
        default:
            send_spi_slave_data = 0xff;
            break;
        }

        receive_data[receive_addr++] = SPI_I2S_ReceiveData(SPI1);
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
        SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
    }
    if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET)
    {
        SPI_I2S_SendData(SPI1, send_spi_slave_data);
        SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_TXE);
    }
    SPI_Cmd(SPI1, ENABLE);
}

void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line4); /* Clear Flag */
        send_spi_slave_data = 0xff;
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
        switch (receive_data[0])
        {
        case 0x82:
            if (receive_data[1] == 0x01)
            {
                open_flag = 1;
            }
            else
            {
                close_flag = 1;
            }
            break;
        case 0x83:
            if (receive_data[1] == 0x01)
            {
                check_flag = 1;
            }
            break;
        case 0x84:
            set_addr4_data = receive_data[1];
            set_addr4_data_falg = 1;
            break;
        case 0x86:
            set_src_data1 = receive_data[1];
            set_src_data2 = receive_data[2];
            set_src_flag = 1;
            break;
        case 0x88:
            gpio_flag = 1;
            gpio_data = receive_data[1];
            // my_pd_GPIO(receive_data[1]);
            break;
        case 0x89:
            info_flag = 1;
            info_data = receive_data[1];
            // my_pd_Factory_Info(receive_data[1]);
            break;
        case 0x8b:
            rssi_flag = 1;
            rssi_data[0] = receive_data[1];
            rssi_data[1] = receive_data[2];
            rssi_data[2] = receive_data[3];
            rssi_data[3] = receive_data[4];
            rssi_data[4] = receive_data[5];
            rssi_data[5] = receive_data[6];
            // my_pd_Factory_Rssi(receive_data[1],receive_data[2],receive_data[3],receive_data[4]);
            break;
        case 0x8c:
            aio_id_flag = 1;
            aio_id_buff[0] = receive_data[1];
            aio_id_buff[1] = receive_data[2];
            aio_id_buff[2] = receive_data[3];
            aio_id_buff[3] = receive_data[4];
            break;
        default:
            for (receive_buff_size = 0; receive_buff_size < receive_addr; receive_buff_size++)
            {
                receive_buff[receive_buff_size] = receive_data[receive_buff_size];
            }
            break;
        }
        printf_size[printf_addr++] = receive_addr;

        receive_data[0] = 0xff;
        receive_flag = 1;
        receive_addr = 0;
    }
}

void my_spi_SNK(uint8_t PDO_Len, uint16_t Current) //
{
    if (Current >= 300)
        Current = 300;
    my_spi.Control_Data[0] = 0x10 + 1;
    my_spi.Control_Data[4] = (PDO_Len << 1) + (Current >> 8);
    my_spi.Control_Data[5] = (uint8_t)Current & 0xff;

    my_spi.DIO_Status = 1;
    my_spi.DIO_Time = 50;

    printf("my_spi.Control_Data[0] = %d\r\n", my_spi.Control_Data[0]);
}

void my_spi_SRC(uint8_t PDO_Len, uint16_t Current, uint8_t status, uint8_t test_mode) //
{
    if (status == 0)
    {
        my_spi.Control_Data[0] = 0x30 + 2;
    }
    else
    {
        if (PDO_Len < 3)
            my_SPI_ControlData_on(3, 3);
        my_spi.Control_Data[0] = 0x20 + 4;
    }
    if (test_mode)
        my_spi.Control_Data[0] |= 0x80;
    printf("my_spi.Control_Data[0] = %d\r\n", my_spi.Control_Data[0]);
    my_spi.Control_Data[4] = (PDO_Len << 1) + (Current >> 8);
    my_spi.Control_Data[5] = (uint8_t)Current & 0xff;

    my_spi.Control_Data[8] = 0xfc;

    my_spi.DIO_Status = 1;
    my_spi.DIO_Time = 50;
}

void my_spi_set_hall(uint8_t hall)
{
    my_spi.Control_Data[8] = 0xfc | hall;
}

void my_spi_disconnect()
{
    my_spi.Control_Data[0] = 0;
    my_spi.Control_Data[1] = 0;
    my_spi.Control_Data[2] = 0;
    my_spi.Control_Data[3] = 0;
    my_spi.Control_Data[8] = 0;
    my_spi.DIO_Status = 0;
    my_spi.DIO_Time = 5;
}

void my_spi_set_valve()
{
    if (my_spi.Control_Data[2])
    {
        my_usb_send(1, 2);
    }
    else
    {
        my_usb_send(my_spi.Control_Data[1], 0);
    }
}

uint8_t my_spi_get()
{
    if (my_spi.Control_Data[2])
    {
        return 2;
    }
    else
    {
        return my_spi.Control_Data[1];
    }
}

uint8_t my_time_tick(uint16_t *data)
{
    if (*data > Tmr_Ms_Dlt)
        *data -= Tmr_Ms_Dlt;
    else
        *data = 0;
    if (*data == 0)
    {
        return 1;
    }

    return 0;
}

void my_spi_handle()
{
    if (my_spi.DIO_Time)
    {
        if (my_spi.DIO_Time >= Tmr_Ms_Dlt)
        {
            my_spi.DIO_Time -= Tmr_Ms_Dlt;
            if (my_spi.DIO_Time == 0)
            {
                printf("set DIO_Status = %d\r\n", my_spi.DIO_Status);
                GPIO_WriteBit(GPIOB, GPIO_Pin_0, my_spi.DIO_Status);
            }
        }
        else
        {
            my_spi.DIO_Time = 0;
            printf("set DIO_Status = %d\r\n", my_spi.DIO_Status);
            GPIO_WriteBit(GPIOB, GPIO_Pin_0, my_spi.DIO_Status);
        }
    }
    if (test_delay_show >= Tmr_Ms_Dlt)
    {
        test_delay_show -= Tmr_Ms_Dlt;
    }
    else
    {
        test_delay_show = 0;
    }

    if (receive_flag == 1 && test_delay_show == 0)
    {
        if (open_flag)
        {
            open_flag = 0;
            printf("spi set valve open\r\n");
            my_SPI_ControlData_off(3, 1);
            my_spi.Control_Data[1] = 1;
            my_spi.Control_Data[2] = 0;
            my_pd_open_valve();
        }
        if (close_flag)
        {
            close_flag = 0;
            printf("spi set valve close\r\n");
            my_SPI_ControlData_off(3, 0);
            my_spi.Control_Data[1] = 0;
            my_spi.Control_Data[2] = 0;
            my_pd_close_valve();
        }
        if (check_flag)
        {
            check_flag = 0;
            my_spi.Control_Data[2] = 1;
            my_SPI_ControlData_off(3, 2);
            printf("spi set valve check\r\n");
            my_pd_check_valve();
        }
        if (set_addr4_data_falg)
        {
            set_addr4_data_falg = 0;
            my_spi.Control_Data[3] = set_addr4_data;
        }
        if (set_src_flag)
        {
            set_src_flag = 0;
            printf("spi set_PD_STATUS_SRC\r\n");
            my_set_Vbus_Current(set_src_data1, set_src_data2);
            // if (PD_Ctl.Flag.Bit.Connected == 0)
            // {
            set_PD_STATUS_SRC();
            set_sc8726_select();
            // }
            // else
            // {
            //     my_switch_state_pd(1);
            // }
        }
        if (gpio_flag)
        {
            // gpio_flag -= 1;
            printf("spi get GPIO %d  %02x\r\n", gpio_flag, gpio_data);
        }
        if (info_flag)
        {
            // info_flag -= 1;
            printf("spi get INFO %d  %02x\r\n", info_flag, info_data);
        }
        if (rssi_flag)
        {
            // rssi_flag -= 1;
            printf("spi get RSSI %d  %02x %02x %02x %02x\r\n", rssi_flag, rssi_data[0], rssi_data[1], rssi_data[2], rssi_data[3]);
        }
        if (gpio_flag || info_flag || rssi_flag)
        {
            uint8_t status = 0;
            if (gpio_flag)
            {
                status |= 0x04;
                gpio_flag = 0;
                // printf("gpio %02x    ", gpio_data);
            }
            if (info_flag)
            {
                status |= 0x02;
                info_flag = 0;
                // printf("info %02x    ", info_data);
            }
            if (rssi_flag)
            {
                status |= 0x01;
                rssi_flag = 0;
                // printf("rssi %02x %02x %02x %02x", rssi_data[0], rssi_data[1], rssi_data[2], rssi_data[3]);
            }
            // printf("\r\n");
            my_pd_Test_Send(status, gpio_data, info_data, rssi_data);
        }
        if(aio_id_flag){
            aio_id_flag = 0;
            my_PD_AIO_ID_Out(aio_id_buff);
            printf("AIO ID %02x  %02x  %02x  %02x\r\n",aio_id_buff[0],aio_id_buff[1],aio_id_buff[2],aio_id_buff[3]);
        }
        receive_flag = 0;

        if (receive_buff_size)
        {
            for (uint8_t i = 0; i < receive_buff_size; i++)
            {
                printf("%02x  ", receive_buff[i]);
            }
            printf("\r\n");
        }
        if (printf_addr)
        {
            printf("printf addr %d    ", printf_addr);
            for (uint8_t i = 0; i < printf_addr; i++)
            {
                printf("%d  ", printf_size[i]);
            }
            printf("\r\n");
            printf_addr = 0;
        }

        //        printf("size = %d\r\n", printf_size);
        receive_buff_size = 0;
        test_delay_show = 10;
    }
}
