/* ------------------------ System includes ------------------------------- */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* ------------------------ FreeRTOS includes ----------------------------- */
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------ uIP includes --------------------------------- */
#include "uip.h"

/* ------------------------ FreeModbus includes --------------------------- */
#include "mb.h"
#include "modbus.h"
/* ------------------------ Project includes ------------------------------ */

/* ------------------------ Defines --------------------------------------- */
#define MB_COM_PORT			0		//com0
#define MB_SLAVE_MODE		MB_RTU
#define MB_SLAVE_ADDRESS	0x11
#define MB_COM_BAUD_RATE	115200
#define MB_PARITY_MODE		MB_PAR_NONE

#define PROG                "FreeModbus"

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
/*static*/ volatile USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
/*static*/ volatile USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
static xSemaphoreHandle xSemaphore_MB;

/* ------------------------ Static functions ------------------------------ */

/* ------------------------ Implementation -------------------------------- */

/*---------------------------------------------------------------------------*/

void
vMBTCPSlaveTask( void *pvParameters )
{
    eMBErrorCode    xStatus;
	u8 i;
	sMBControl sMBCTcp;

//	vTaskDelay( configTICK_RATE_HZ*3 );
	
    for( ;; )
    {
		if ( xSemaphore_MB == NULL ){
			vSemaphoreCreateBinary( xSemaphore_MB );
		    if( xSemaphore_MB == NULL ){
				vTaskDelay( configTICK_RATE_HZ*3 );
				continue;
			}
		}

        if( eMBTCPInit( &sMBCTcp, MB_TCP_PORT_USE_DEFAULT ) != MB_ENOERR ) {
            //fprintf( stderr, "%s: can't initialize modbus stack!\r\n", PROG );
        } else if( eMBEnable( &sMBCTcp ) != MB_ENOERR ) {
            //fprintf( stderr, "%s: can't enable modbus stack!\r\n", PROG );
        } else {
            do
            {
                xStatus = eMBPoll( &sMBCTcp );
            }
            while( xStatus == MB_ENOERR );
        }
        /* An error occured. Maybe we can restart. */
        ( void )eMBDisable( &sMBCTcp );
        ( void )eMBClose( &sMBCTcp );
		vTaskDelay( configTICK_RATE_HZ*3 );
    }
}

/*---------------------------------------------------------------------------*/

void
vMBRTUSlaveTask( void *pvParameters )
{
    const unsigned char ucSlaveIDAdditonal[] = { 0xAA, 0xBB, 0xCC };
    eMBErrorCode    xStatus;
	u8 i;
	sMBControl sMBCRtu;

    for( ;; )
    {
		if ( xSemaphore_MB == NULL ){
			vSemaphoreCreateBinary( xSemaphore_MB );
		    if( xSemaphore_MB == NULL ) {
				vTaskDelay( configTICK_RATE_HZ*3 );
				continue;
			}
		}

		if ( eMBInit( &sMBCRtu,MB_SLAVE_MODE, MB_SLAVE_ADDRESS, MB_COM_PORT, MB_COM_BAUD_RATE, MB_PARITY_MODE ) != MB_ENOERR ) {
		} else if( eMBSetSlaveID( 44, TRUE, ucSlaveIDAdditonal, 3 ) != MB_ENOERR ) {
        } else if( eMBEnable( &sMBCRtu ) != MB_ENOERR ) {
            //fprintf( stderr, "%s: can't enable modbus stack!\r\n", PROG );
        } else {
            do
            {
                xStatus = eMBPoll( &sMBCRtu );
		        //usRegInputBuf[0]++;
            }
            while( xStatus == MB_ENOERR );
        }
        /* An error occured. Maybe we can restart. */
        ( void )eMBDisable( &sMBCRtu );
        ( void )eMBClose( &sMBCRtu );
		vTaskDelay( configTICK_RATE_HZ*3 );
   }
}

/*---------------------------------------------------------------------------*/
USHORT
eMBRegHolding_Read( USHORT usAddress )
{
    int iRegIndex;
	USHORT RegVal;

	while ( xSemaphore_MB == NULL )
		vTaskDelay( configTICK_RATE_HZ );

	usAddress ++;
    if( ( usAddress < REG_HOLDING_START ) || \
        ( usAddress > REG_HOLDING_START + REG_HOLDING_NREGS ) )
    	return 0;

	iRegIndex = ( int )( usAddress - usRegHoldingStart );
	xSemaphoreTake( xSemaphore_MB, portMAX_DELAY );
	
	RegVal = usRegHoldingBuf[iRegIndex];
	
	xSemaphoreGive( xSemaphore_MB );
    return RegVal;
}
/*---------------------------------------------------------------------------*/
void
eMBRegHolding_Write( USHORT usAddress, USHORT usRegVal )
{
    int iRegIndex;
	USHORT RegVal;

	while ( xSemaphore_MB == NULL )
		vTaskDelay( configTICK_RATE_HZ );
	
	usAddress ++;
    if( ( usAddress < REG_HOLDING_START ) || \
        ( usAddress > REG_HOLDING_START + REG_HOLDING_NREGS ) )
    	return;

	iRegIndex = ( int )( usAddress - usRegHoldingStart );
	xSemaphoreTake( xSemaphore_MB, portMAX_DELAY );
	
    usRegHoldingBuf[iRegIndex]  = usRegVal;
	
	xSemaphoreGive( xSemaphore_MB );
}
/*---------------------------------------------------------------------------*/
USHORT
eMBRegInput_Read( USHORT usAddress )
{
    int iRegIndex;
	USHORT RegVal;

	while ( xSemaphore_MB == NULL )
		vTaskDelay( configTICK_RATE_HZ );

	usAddress ++;
    if( ( usAddress < REG_INPUT_START ) || \
        ( usAddress > REG_INPUT_START + REG_INPUT_NREGS ) )
    	return 0;

	iRegIndex = ( int )( usAddress - usRegInputStart );
	xSemaphoreTake( xSemaphore_MB, portMAX_DELAY );
	
	RegVal = usRegInputBuf[iRegIndex];
	
	xSemaphoreGive( xSemaphore_MB );
    return RegVal;
}
/*---------------------------------------------------------------------------*/
void
eMBRegInput_Write( USHORT usAddress, USHORT usRegVal )
{
    int iRegIndex;
	USHORT RegVal;

	while ( xSemaphore_MB == NULL )
		vTaskDelay( configTICK_RATE_HZ );

	usAddress ++;
    if( ( usAddress < REG_INPUT_START ) || \
        ( usAddress > REG_INPUT_START + REG_INPUT_NREGS ) )
    	return ;

	iRegIndex = ( int )( usAddress - usRegInputStart );
	xSemaphoreTake( xSemaphore_MB, portMAX_DELAY );
	
    usRegInputBuf[iRegIndex]  = usRegVal;
	
	xSemaphoreGive( xSemaphore_MB );
    return ;
}
/*---------------------------------------------------------------------------*/

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
			vPortEnterCritical();
			
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
			
			vPortExitCritical();
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/*---------------------------------------------------------------------------*/

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
	char str[4];
	
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
			while( usNRegs > 0 )
            {
				vPortEnterCritical();
				
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
				
				vPortExitCritical();
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
				vPortEnterCritical();
				
                usRegHoldingBuf[iRegIndex]  = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
				
				switch ( iRegIndex ){
/*				case MB_DAC0:		
				case MB_DAC1:		
				case MB_DAC2:		
				case MB_DAC3:		
					PWM_DAC_SetmV(iRegIndex - MB_DAC0,usRegHoldingBuf[iRegIndex]);
					break;
				case MB_MF_VMETER_CTL:
					//hv_pwr_ctl(2,(usRegHoldingBuf[iRegIndex]&0x01)?POWER_ON:POWER_OFF);
					break;
				case MB_MF_RELAY_CTL:
					hv_pwr_ctl(0,(usRegHoldingBuf[iRegIndex]&0x01)?POWER_ON:POWER_OFF);
					hv_pwr_ctl(1,(usRegHoldingBuf[iRegIndex]&0x02)?POWER_ON:POWER_OFF);
					//hv_pwr_ctl(3,(usRegHoldingBuf[iRegIndex]&0x04)?POWER_ON:POWER_OFF);
					break;
				default :
					break;*/
					case MB_VOL_MAX :
					case MB_VOL_SCALE:
					case MB_VOL_ERR_RATE:
					case MB_VOL_STEP:
					case MB_VOL_STEP_INTERVAL:
					case MB_VOL_STEP_TIMEOUT:
					case MB_VOL_LEVEL1:
						
					case MB_CURRRENT_MAX :
					case MB_CUR_SCALE:
					case MB_CUR_ERR_RATE:
					case MB_CUR_STEP:
					case MB_CUR_STEP_INTERVAL:
					case MB_CUR_STEP_TIMEOUT:
					case MB_CUR_CTL_START:

					case VMETER_START_DELAY:
					case VMETER_STOP_DELAY:
						
					case MB_VOL_SET_L:
						//eMBRegInput_Write(MB_HV_ST_L,(eMBRegInput_Read(MB_HV_ST_L) & ~HV_SET_OK));break;
					case MB_VOL_SET_R:
						//eMBRegInput_Write(MB_HV_ST_R,(eMBRegInput_Read(MB_HV_ST_R) & ~HV_SET_OK));break;
						
					case MB_MPUMP_PWR_OFF_FREQ:
						break;
					case MB_CUR_SET_L:
						usRegInputBuf[MB_HV_ST_L] &= 0xFF;
						break;
					case MB_CUR_SET_R:
						usRegInputBuf[MB_HV_ST_R] &= 0xFF;
						break;
					case MB_POWERPUMP_CTL:
						powerpump_ctl(usRegHoldingBuf[iRegIndex]);
						break;
					case MB_MPUMP_CTL:
						mpump_ctl(usRegHoldingBuf[iRegIndex]);
						break;
					case MB_VMETER_CTL:
						vmeter_ctl( usRegHoldingBuf[iRegIndex] );
						break;
					case MB_VMETER_ERR_RATE:
					case MB_VMETER_SET0:
					case MB_VMETER_SET1:
					case MB_VMETER_SET2:
					case MB_VMETER_SET3:
						break;
					case MB_SAMPLE_ANGLE_SET:
						//sample_moto_ctl( usRegHoldingBuf[iRegIndex] );
						break;
					case MB_SAMPLE_CTL	:
						//sample_monitor_ctl(usRegHoldingBuf[iRegIndex]);
						break;
					case MB_SAMPLE_HOLE0:
						break;
					case MB_SAMPLE_START:
						//sample_monitor_start(usRegHoldingBuf[iRegIndex]);
						break;
					case MB_SAMPLE_INTERVAL:
						//sample_monitor_interval(usRegHoldingBuf[iRegIndex]);
						break;
					case MB_SAMPLE_LED :
						sample_led_ctl( usRegHoldingBuf[iRegIndex] );
						break;
					case MB_BAFFLE:
						baffle_ctl( usRegHoldingBuf[iRegIndex] );
						break;
					case MB_BLEED_VALVE:
						bleed_valve_ctl(usRegHoldingBuf[iRegIndex]);
						break;
					case MB_BAFFLE_INTERVAL:
						//sample_baffle_interval();
						break;
					case MB_SYS_AUTOCTL:
						break;
					case MB_MOTOR_CTRL:
						//motor_ctrl(usRegHoldingBuf[iRegIndex]);
						break;
					//-----------------------------------------------------------				
					/*case MB_RELAY_SET:
						mb_bit_fun ( usRegHoldingBuf[iRegIndex], ON, relay_set_ch );
						reg_count ++;
						break;
					case MB_RELAY_CLR:
						mb_bit_fun ( usRegHoldingBuf[iRegIndex], OFF, relay_set_ch );
						reg_count ++;
						break;
					case MB_DO_SET:
						//mb_bit_fun ( usRegHoldingBuf[iRegIndex], ON, io_set_ch );
						reg_count ++;
						break;
					case MB_DO_CLR:
						//mb_bit_fun ( usRegHoldingBuf[iRegIndex], OFF, io_set_ch );
						reg_count ++;
						break;*/
		/*			case MB_FD110A:
						FD110A_ctl(usRegHoldingBuf[iRegIndex]);
						//reg_count ++;
						break;
					case MB_BEEP_ON:
						reg_count ++;
						beep_set(usRegHoldingBuf[iRegIndex], (reg_buf[reg_count*2]<<8) | reg_buf[reg_count*2 + 1] );
						//reg_count ++;
						break;
					case MB_LED_ON:
					case MB_LED_OFF:
						led(usRegHoldingBuf[iRegIndex]);
						//reg_count ++;
						break;
					case MB_L298_CTL:
						l298n(usRegHoldingBuf[iRegIndex]>>8, usRegHoldingBuf[iRegIndex]&0x0F);
						//reg_count ++;
						break;
				*/	case MB_GSM_CTL:
						//uartswSendStr(usRegHoldingBuf[iRegIndex]);
					default :
						break;				}		
				
 				vPortExitCritical();
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/*---------------------------------------------------------------------------*/

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

/*---------------------------------------------------------------------------*/

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

/*---------------------------------------------------------------------------*/

