/**
 ******************************************************************************
 * @file    main.c
 * @author  BekaBen 
 * @brief   Main program for SeedNursery Wiznet Surf5 Challenge June 2024.
 ******************************************************************************
 * @ref 		W7500x-Surf5 The project is started from MDN Surf5 Template project from : 
 * 					https://github.com/Wiznet/W7500x-Surf5/
 *
 * June 24th 2024 :o{)
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "wizchip_conf.h"
#include "dhcp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DATA_BUF_SIZE 2048

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
uint8_t test_buf[DATA_BUF_SIZE];
uint16_t SoilMoisture;
uint8_t Pumpstatus;
wiz_NetInfo gWIZNETINFO;

/* Private function prototypes -----------------------------------------------*/
static void UART_Config(void);
static void GPIO_Config(void);
static void DUALTIMER_Config(void);
static void Network_Config(void);
void dhcp_assign(void);
void dhcp_update(void);
void dhcp_conflict(void);
uint32_t WebServer(uint8_t sn, uint8_t* buf, uint16_t port, uint16_t SoilMoisture, uint8_t pumpstatus);
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t milliseconds);
int Read_ADC(int ADC_channel);
void Pump_ctrl(uint8_t cmd);


/**
 * @brief  Main program for Seed Nursury monitoring.
 * @param  None
 * @retval None
 */
int main(void)
{
		uint32_t ret;
    uint8_t dhcp_retry = 0;
	
		//int i = 0;
		SystemInit();
		
		/* SysTick_Config */
    SysTick_Config((GetSystemClock() / 1000));

    /* Set WZ_100US Register */
    setTIC100US((GetSystemClock() / 10000));
		
		UART_Config();
		DUALTIMER_Config();
		GPIO_Config();
		
		printf("SURF5 Wiznet Contest with W750x Standard Peripheral Library version : %d.%d.%d\r\n", __W7500X_STDPERIPH_VERSION_MAIN, __W7500X_STDPERIPH_VERSION_SUB1, __W7500X_STDPERIPH_VERSION_SUB2);
	
    printf("SourceClock : %d\r\n", (int) GetSourceClock());
    printf("SystemClock : %d\r\n", (int) GetSystemClock());
		printf("Link : %s\r\n", PHY_GetLinkStatus() == PHY_LINK_ON ? "Cable connected" : "Network Off");
		
		printf("Network details befor DHCP handshake\r\n");
		Network_Config();
		/* DHCP Process */
    DHCP_init(0, test_buf);
    reg_dhcp_cbfunc(dhcp_assign, dhcp_update, dhcp_conflict);
    if (gWIZNETINFO.dhcp == NETINFO_DHCP) {       // DHCP
        printf("Start DHCP\r\n");
        while (1) {
            ret = DHCP_run();

            if (ret == DHCP_IP_LEASED) {
                printf("DHCP Success\r\n");
                break;
            }
            else if (ret == DHCP_FAILED) {
                dhcp_retry++;
            }

            if (dhcp_retry > 3) {
                printf("DHCP Fail\r\n");
                break;
            }
        }
    }

    
    printf("Network details after DHCP handshake\r\n");
		Network_Config();
		
    printf("Starting reading from sensors\r\n");  
		
	
  while (1) {
		
		SoilMoisture=Read_ADC(7);
		if(SoilMoisture>500){
			Pumpstatus = 0;
			Pump_ctrl(Pumpstatus);
			printf("Shutting down irrigation\r\n");
		}else{
			Pumpstatus = 1;
			Pump_ctrl(Pumpstatus);
			printf("Starting irrigation\r\n");
		};
		WebServer(1, test_buf, 80, SoilMoisture, Pumpstatus);
		
		//Delay(500);	
    }

    return 0;
}



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Configures the UART Peripheral.
 * @note
 * @param  None
 * @retval None
 */
static void UART_Config(void)
{
    UART_InitTypeDef UART_InitStructure;

    UART_StructInit(&UART_InitStructure);

    S_UART_Init(115200);
    S_UART_Cmd(ENABLE);

}

/**
 * @brief  Configures the GPIO Peripheral.
 * @note
 * @param  None
 * @retval None
 */
static void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Direction = GPIO_Direction_IN;
    GPIO_InitStructure.GPIO_Pad = GPIO_Pad_Default;
    GPIO_InitStructure.GPIO_AF = PAD_AF0;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Direction = GPIO_Direction_OUT;
    GPIO_InitStructure.GPIO_Pad = GPIO_OpenDrainDisable | GPIO_HighDrivingStrength | GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_AF = PAD_AF1;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/**
 * @brief  Configures the DUALTIMER Peripheral.
 * @note
 * @param  None
 * @retval None
 */
static void DUALTIMER_Config(void)
{
    DUALTIMER_InitTypDef DUALTIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    DUALTIMER_InitStructure.Timer_Load = GetSystemClock() / 1; //1s
    DUALTIMER_InitStructure.Timer_Prescaler = DUALTIMER_Prescaler_1;
    DUALTIMER_InitStructure.Timer_Wrapping = DUALTIMER_Periodic;
    DUALTIMER_InitStructure.Timer_Repetition = DUALTIMER_Wrapping;
    DUALTIMER_InitStructure.Timer_Size = DUALTIMER_Size_32;
    DUALTIMER_Init(DUALTIMER0_0, &DUALTIMER_InitStructure);

    DUALTIMER_ITConfig(DUALTIMER0_0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DUALTIMER0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DUALTIMER_Cmd(DUALTIMER0_0, ENABLE);
}

/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length.
 * @retval None
 */
void Delay(__IO uint32_t milliseconds)
{
    TimingDelay = milliseconds;
		
		while (TimingDelay != 0) {
        ;
    };
		
}

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    }
}

/**
 * @brief  Reading from ADC Channel Ch.
 * @param  CH : The ADC Channel
 * @retval The ADC returned value
 */
int Read_ADC(int ADC_channel)
{
	int ADC_Value;
	ADC_Cmd(ENABLE);
  ADC_ChannelConfig(ADC_channel);
  ADC_StartOfConversion();
	ADC_Value = ADC_GetConversionValue();
  printf("ADC_GetConversionValue : [%d] : %d,\r\n", ADC_channel, ADC_Value);
  ADC_Cmd(DISABLE);
	return ADC_Value;
}

/**
 * @brief  Reading from ADC Channel Ch.
 * @param  cmd : controlling irrigation 0 shutting down greater than 0 starting irrigation
 * @retval None
 */
void Pump_ctrl(uint8_t cmd)
{
	if(cmd>0){
		GPIO_SetBits(GPIOC, GPIO_Pin_15);
		GPIO_SetBits(GPIOC, GPIO_Pin_14);
	}else{
		GPIO_ResetBits(GPIOC, GPIO_Pin_15);
		GPIO_ResetBits(GPIOC, GPIO_Pin_14);
	}
}

/**
 * @brief  Configures the Network Information.
 * @note
 * @param  None
 * @retval None
 */
static void Network_Config(void)
{
    uint8_t mac_addr[6] = { 0x00, 0x01, 0xDB, 0xE, 0x0C, 0x0A };

    memcpy(gWIZNETINFO.mac, mac_addr, 6);
    gWIZNETINFO.dhcp = NETINFO_DHCP;

    ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);

    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
    printf("IP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
    printf("GW: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
    printf("SN: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
    printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
}

/**
 * @brief  The call back function of ip assign.
 * @note
 * @param  None
 * @retval None
 */
void dhcp_assign(void)
{
    getIPfromDHCP(gWIZNETINFO.ip);
    getGWfromDHCP(gWIZNETINFO.gw);
    getSNfromDHCP(gWIZNETINFO.sn);
    getDNSfromDHCP(gWIZNETINFO.dns);

    ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
}

/**
 * @brief  The call back function of ip update.
 * @note
 * @param  None
 * @retval None
 */
void dhcp_update(void)
{
    ;
}

/**
 * @brief  The call back function of ip conflict.
 * @note
 * @param  None
 * @retval None
 */
void dhcp_conflict(void)
{
    ;
}

/**
 * @brief  WebServer example function.
 * @note
 * @param  sn: Socket number to use.
 * @param  buf: The buffer the socket will use.
 * @param  port: Socket port number to use.
 * @retval Success or Fail of configuration functions
 */
uint32_t WebServer(uint8_t sn, uint8_t* buf, uint16_t port, uint16_t SoilMoisture, uint8_t pumpstatus)
{
    uint8_t i;
    int32_t ret;
    uint16_t size = 0;
    uint8_t destip[4];
    uint16_t destport;
    uint8_t data_buf[128] = { '\0', };

    switch (getSn_SR(sn))
    {
        case SOCK_ESTABLISHED:

            if (getSn_IR(sn) & Sn_IR_CON) {

                getSn_DIPR(sn, destip);
                destport = getSn_DPORT(sn);
                printf("%d:Connected - %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);

                setSn_IR(sn, Sn_IR_CON);
            }

            if ((size = getSn_RX_RSR(sn)) > 0) {
                if (size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
                ret = recv(sn, buf, size);
                if (ret <= 0) return ret;
                printf("%s", buf);

                ret = send(sn, "HTTP/1.1 200 OK\r\n"
                        "Content-Type: text/html\r\n"
                        "Connection: close\r\n"
                        "Refresh: 5\r\n"
                        "\r\n"
                        "<!DOCTYPE HTML>\r\n"
                        "<html>\r\n <Title> Seed Nursery V1.0</Title>\r\n"
												"<body>\r\n", sizeof("HTTP/1.1 200 OK\r\n"
                        "Content-Type: text/html\r\n"
                        "Connection: close\r\n"
                        "Refresh: 5\r\n"
                        "\r\n"
                        "<!DOCTYPE HTML>\r\n"
                        "<html>\r\n <Title> Seed Nursery V1.0</Title>\r\n"
												"<body>\r\n") - 1);
                if (ret < 0) {
                    close(sn);
                    return ret;
                }

         
                sprintf(data_buf, "Soil Mosture is %d<br /> Pump status is %d</br>\r\n", SoilMoisture, pumpstatus);
                ret = send(sn, data_buf, strlen(data_buf));
                if (ret < 0) {
                    close(sn);
                    return ret;
                }
                    
                memset(data_buf, '\0', 128);
                

                ret = send(sn, "</body>\r\n</html>\r\n", sizeof("</body>\r\n</html>\r\n") - 1);
                if (ret < 0) {
                    close(sn);
                    return ret;
                }

                disconnect(sn);
            }

            break;
        case SOCK_CLOSE_WAIT:

            if ((ret = disconnect(sn)) != SOCK_OK) return ret;

            printf("%d:Socket Closed\r\n", sn);

            break;
        case SOCK_INIT:

            printf("%d:Listen, Web server, port [%d]\r\n", sn, port);

            if ((ret = listen(sn)) != SOCK_OK) return ret;

            break;
        case SOCK_CLOSED:

            if ((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;

            break;
        default:
            break;
    }
		return 0;
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
