/*********************************************************************************************************************
* STC32G Opensourec Library ����STC32G ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ���STC ��Դ���һ����
*
* STC32G ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          MDK FOR C251
* ����ƽ̨          STC32G
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����           ��ע
* 2024-08-01        ��W            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   RX                  �鿴 zf_device_wireless_uart.h �� WIRELESS_UART_RX_PIN  �궨��
*                   TX                  �鿴 zf_device_wireless_uart.h �� WIRELESS_UART_TX_PIN  �궨��
*                   RTS                 �鿴 zf_device_wireless_uart.h �� WIRELESS_UART_RTS_PIN �궨��
*                   VCC                 3.3V��Դ
*                   GND                 ��Դ��
*                   ������������
*                   ------------------------------------
*********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_device_wireless_uart.h"
#include "zf_device_type.h"

#pragma warning disable = 183
#pragma warning disable = 177


static  fifo_struct                                     wireless_uart_fifo;
static  uint8                                           wireless_uart_buffer[WIRELESS_UART_BUFFER_SIZE];

//static          uint8                                   wireless_uart_data          = 0;
#if (1 == WIRELESS_UART_AUTO_BAUD_RATE)
static volatile wireless_uart_auto_baudrate_state_enum  wireless_auto_baud_flag     = WIRELESS_UART_AUTO_BAUD_RATE_INIT;
static volatile uint8                                   wireless_auto_baud_data[3]  = {0x00, 0x01, 0x03};
#endif

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת����ģ�� ��������
// ����˵��     data            8bit ����
// ���ز���     uint32          ʣ�෢�ͳ��� 0-������� 1-δ�������
// ʹ��ʾ��     wireless_uart_send_byte(data);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint32 wireless_uart_send_byte (const uint8 dat)
{
	uint16 time_cnt = WIRELESS_UART_TIMEOUT_COUNT;
	
	while (time_cnt && gpio_get_level(WIRELESS_UART_RTS_PIN)) 
	{
		system_delay_ms(1);
		time_cnt--;
	}
	
	if (time_cnt) 
	{
		uart_write_byte(WIRELESS_UART_INDEX, dat);
	}
	
	return time_cnt == 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת����ģ�� �������ݿ�
// ����˵��     *buff           ���ͻ�����
// ����˵��     len             �������ݳ���
// ���ز���     uint32          ʣ�෢�ͳ���
// ʹ��ʾ��     wireless_uart_send_buffer(buff, 64);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint32 wireless_uart_send_buffer(const uint8 *buff, uint32 len)
{
    #define WIRELESS_PACKET_SIZE 30
    #define DELAY_MS 1
    
    uint16 time_cnt = 0;
	uint16 send_len = 0;
    zf_assert(NULL != buff);  // ȷ���������ǿ�

    while (len && time_cnt < WIRELESS_UART_TIMEOUT_COUNT)
    {
        if (!gpio_get_level(WIRELESS_UART_RTS_PIN))  // RTS�͵�ƽ���ɷ���
        {
            send_len = len < WIRELESS_PACKET_SIZE ? len : WIRELESS_PACKET_SIZE;
            uart_write_buffer(WIRELESS_UART_INDEX, buff, send_len);
            buff += send_len;
            len -= send_len;
            time_cnt = 0;  // ���ó�ʱ����
        }
        else  // RTS�ߵ�ƽ��ģ��æ
        {
            system_delay_ms(DELAY_MS);
            time_cnt++;
        }
    }

    return len;  // ����δ���͵��ֽ���
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת����ģ�� �����ַ���
// ����˵��     *str            Ҫ���͵��ַ�����ַ
// ���ز���     uint32          ʣ�෢�ͳ���
// ʹ��ʾ��     wireless_uart_send_string("Believe in yourself.");
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint32 wireless_uart_send_string (const char *str)
{
    uint32 len = strlen(str);
    zf_assert(NULL != str);
	
    return  wireless_uart_send_buffer(str, len);
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת����ģ�� ��ȡ����
// ����˵��     *buff           ���ջ�����
// ����˵��     len             ��ȡ���ݳ���
// ���ز���     uint32          ʵ�ʶ�ȡ���ݳ���
// ʹ��ʾ��     wireless_uart_read_buffer(buff, 32);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint32 wireless_uart_read_buffer (uint8 *buff, uint32 len)
{
    uint32 data_len = len;
    zf_assert(NULL != buff);
    fifo_read_buffer(&wireless_uart_fifo, buff, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת����ģ�� �����жϻص�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     wireless_uart_callback();
// ��ע��Ϣ     �ú����� ISR �ļ� �����жϳ��򱻵���
//              �ɴ����жϷ��������� wireless_module_uart_handler() ����
//              ���� wireless_module_uart_handler() �������ñ�����
//-------------------------------------------------------------------------------------------------------------------
void wireless_uart_callback (uint8 uart_dat)
{
//    uart_query_byte(WIRELESS_UART_INDEX, &uart_dat);
    fifo_write_buffer(&wireless_uart_fifo, &uart_dat, 1);
#if WIRELESS_UART_AUTO_BAUD_RATE                                                // �����Զ�������
    
    if(WIRELESS_UART_AUTO_BAUD_RATE_START == wireless_auto_baud_flag && 3 == fifo_used(&wireless_uart_fifo))
    {
        uint32 wireless_auto_baud_count = 3;
        wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_GET_ACK;
        fifo_read_buffer(&wireless_uart_fifo, (uint8 *)wireless_auto_baud_data, (uint32 *)&wireless_auto_baud_count, FIFO_READ_AND_CLEAN);
    }
    
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת����ģ�� ��ʼ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     wireless_uart_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 wireless_uart_init (void)
{
    uint8 return_state = 0;
    set_wireless_type(WIRELESS_UART, WIRELESS_UART_INDEX, wireless_uart_callback);
    
    fifo_init(&wireless_uart_fifo, FIFO_DATA_8BIT, wireless_uart_buffer, WIRELESS_UART_BUFFER_SIZE);
    gpio_init(WIRELESS_UART_RTS_PIN, GPIO, GPIO_HIGH, GPIO_NO_PULL);
#if(0 == WIRELESS_UART_AUTO_BAUD_RATE)                                          // �ر��Զ�������
    // ������ʹ�õĲ�����Ϊ115200 Ϊ����ת����ģ���Ĭ�ϲ����� ������������������������ģ�鲢�޸Ĵ��ڵĲ�����
    uart_init (WIRELESS_UART_INDEX, WIRELESS_UART_BUAD_RATE, WIRELESS_UART_RX_PIN, WIRELESS_UART_TX_PIN);   // ��ʼ������
    uart_rx_interrupt(WIRELESS_UART_INDEX, 1);
#elif(1 == WIRELESS_UART_AUTO_BAUD_RATE)                                        // �����Զ�������
    uint8 rts_init_status = 0;
    uint16 time_count = 0;
    
    wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_INIT;
    wireless_auto_baud_data[0] = 0;
    wireless_auto_baud_data[1] = 1;
    wireless_auto_baud_data[2] = 3;
    
    rts_init_status = gpio_get_level(WIRELESS_UART_RTS_PIN);
    gpio_init(WIRELESS_UART_RTS_PIN, GPO, rts_init_status, GPO_PUSH_PULL);      // ��ʼ����������
    
    uart_init (WIRELESS_UART_INDEX, WIRELESS_UART_BUAD_RATE, WIRELESS_UART_RX_PIN, WIRELESS_UART_TX_PIN);   // ��ʼ������
    uart_rx_interrupt(WIRELESS_UART_INDEX, 1);
    
    system_delay_ms(5);                                                         // ģ���ϵ�֮����Ҫ��ʱ�ȴ�
    gpio_set_level(WIRELESS_UART_RTS_PIN, !rts_init_status);                    // RTS�������ߣ������Զ�������ģʽ
    system_delay_ms(100);                                                       // RTS����֮�������ʱ20ms
    gpio_toggle_level(WIRELESS_UART_RTS_PIN);                                   // RTS����ȡ��
    
    do
    {
        wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_START;
        uart_write_byte(WIRELESS_UART_INDEX, wireless_auto_baud_data[0]);       // �����ض����� ����ģ���Զ��жϲ�����
        uart_write_byte(WIRELESS_UART_INDEX, wireless_auto_baud_data[1]);       // �����ض����� ����ģ���Զ��жϲ�����
        uart_write_byte(WIRELESS_UART_INDEX, wireless_auto_baud_data[2]);       // �����ض����� ����ģ���Զ��жϲ�����
        system_delay_ms(20);
        
        if(WIRELESS_UART_AUTO_BAUD_RATE_GET_ACK != wireless_auto_baud_flag)     // �����Զ��������Ƿ����
        {
            return_state = 1;                                                   // ���������뵽������� ˵���Զ�������ʧ����
            break;
        }
        
        time_count = 0;
        
        if( 0xa5 != wireless_auto_baud_data[0] &&                               // �����Զ��������Ƿ���ȷ
                0xff != wireless_auto_baud_data[1] &&                               // �����Զ��������Ƿ���ȷ
                0xff != wireless_auto_baud_data[2] )                                // �����Զ��������Ƿ���ȷ
        {
            return_state = 1;                                                   // ���������뵽������� ˵���Զ�������ʧ����
            break;
        }
        
        wireless_auto_baud_flag = WIRELESS_UART_AUTO_BAUD_RATE_SUCCESS;
        
        gpio_init(WIRELESS_UART_RTS_PIN, GPI, 0, GPI_PULL_UP);                  // ��ʼ����������
        system_delay_ms(10);                                                    // ��ʱ�ȴ� ģ��׼������
    }
    while(0);
    
#endif
    return return_state;
}
