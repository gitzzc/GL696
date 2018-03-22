#ifndef _MB_REG_MAP_H
#define _MB_REG_MAP_H

#define REG_INPUT_START         1
#define REG_INPUT_NREGS         64
#define REG_HOLDING_START       1
#define REG_HOLDING_NREGS       256

//--------------------------------------
#define MB_RELAY_EN		1
#define MB_RELAY_DIS	2
	#define GL_LED 			(1<<12)
	#define GL_VACUOMETER 	(1<<9)
	#define GL_PUMP 		(1<<8)
	#define GL_BLEED_VALUE 	(1<<4)
	#define GL_MOTOR 		(1<<3)
	#define GL_BAFFLE 		(1<<2)
	#define GL_RHV 			(1<<1)
	#define GL_LHV 			(1<<0)
//--------------------------------------

#define POWER_ON 			(1<<0)
#define POWER_OFF 			(1<<1)


//3-31


//REGISTER MAP
//REGISTER  00001-09999 Coil
//1-32 DO
	//1 		DO0		左右高压枪电源
	//2 		DO1		挡板电源
	//3 		DO2		样品电机电源
	//4 		DO3		放气泵电源
	//5 		DO4		样品通孔检测LED 电源
	//38-65 not use
//33-65 RELAY
	//33		relay0	机械泵、真空计
	//34		relay1	分子泵
	//35	 	not use

//REGISTER  30001-39999 Input Register  (R)
//30001	-	左枪工作状态
#define MB_HV_ST_L 			0	
	#define HV_PWR 			(1<<0)
	#define HV_SET_OK		(1<<1)
	#define HV_LEVEL1		(1<<2)
	#define HV_CUR_SET_OK	(1<<3)
	#define HV_ENABLE		(1<<7)
	
	#define HV_SET_TO 		(1<<8)
	#define HV_CUR_OV		(1<<9)
	#define HV_INCTRL		(1<<10)
//30002	-	左枪电压实际采样值，16位整型数，单位V
#define MB_VOL_FB_L 	1	
//30003	-	左枪电流实际采样值，16位整型数，单位V
#define MB_CUR_FB_L		2	
//30004	-	左枪高压控制端当前设定（仅控制器使用），16位整型数，单位V
#define MB_VOL_CTL_L	3
//30005	-	左枪电流控制端当前设定（仅控制器使用），16位整型数，单位uA
#define MB_CUR_CTL_L	4	

//30006	-	右枪工作状态
#define MB_HV_ST_R 		5	
//30007	-	右枪电压实际采样值，16位整型数，单位V
#define MB_VOL_FB_R 	6
//30008	-	右枪电流实际采样值，16位整型数，单位mA
#define MB_CUR_FB_R		7	
//30009	-	右枪高压控制端当前设定（仅控制器使用），16位整型数，单位V
#define MB_VOL_CTL_R	8
//30010	-	左枪电流控制端当前设定（仅控制器使用），16位整型数，单位uA
#define MB_CUR_CTL_R	9	
//30011	-	左枪电压当前设定，16位整型数，单位V
#define MB_VOL_SET_L_ST	10
//30012	-	左枪电流当前设定，16位整型数，单位uA
#define MB_CUR_SET_L_ST	11
//30013	-	右枪电压当前设定，16位整型数，单位V
#define MB_VOL_SET_R_ST	12
//30014	-	右枪电流当前设定，16位整型数，单位uA
#define MB_CUR_SET_R_ST	13

//机械泵
#define MB_POWERPUMP_ST	16
	#define POWERPUMP_PWR_ON 	POWER_ON
	#define POWERPUMP_PWR_OFF 	POWER_OFF
	
//30017	-	分子泵状态寄存器
#define MB_MPUMP_ST				17
//	#define MPUMP_NORMAL		(1<<2)	//设备正常运行
	#define MPUMP_HIGH_SP		(1<<3)	//设备运行到高速状态
	#define MPUMP_RUN		 	(1<<4)	//设备工作中
	#define MPUMP_OVERCUR		(1<<5)	//设备过流
	#define MPUMP_OVERHOT		(1<<6)	//设备过压
	#define MPUMP_TIMEOUT		(1<<7)	//设备超时

	#define MPUMP_PWR_ON		(1<<8)	//设备已上电
	#define MPUMP_PWR_OFF		(1<<9)	//设备已下电
	#define MPUMP_LOW_SP		(1<<10)	//
	#define MPUMP_STOP			(1<<11)	//
	#define MPUMP_INCTRL		(1<<12)	//设备不可控
	#define MPUMP_PWR_OFFING 	(1<<13)
//30018	-	分子泵当前运行频率，16位整型数，单位Hz
#define MB_MPUMP_FREQ		18	
//30019	-	分子泵当前电流，16位整型数，单位mA
#define MB_MPUMP_CUR		19	
//30020	-	分子泵当前电压，16位整型数，单位V
#define MB_MPUMP_VOL		20

//30025	-	真空计状态
#define MB_VMETER_ST		24
	#define VMETER_PWR_ON 	POWER_ON	//电源开启状态，1表示电源开启
	#define VMETER_PWR_OFF	POWER_OFF	//电源关闭状态，1表示电源关闭
	#define VMETER_WAIT_ON	(1<<2)
	#define VMETER_WAIT_OFF	(1<<3)
//30026	-	真空值,与MB_VMTER1一起组成32位浮点数，(float)((int)(MB_VMETER0<<16) | MB_VMETER1)
#define MB_VMETER0			25	//与MB_VMETER1组成32bit浮点数，bit31-16
//30027	-	
#define MB_VMETER1			26	//与MB_VMETER0组成32bit浮点数，bit15-0

#define MB_SAMPLE_ST		28	//样品信息及状态
	#define SI_TRANSPARENCY	(1<<0)	//样品为透明的，
	#define SI_MONITOR_EN	(1<<2)	//样品监测使用,
	#define SI_PWR_OFF_EN	(1<<3)	//自动关机使能,	
	#define SI_HOLE0		(1<<4)	//样品孔径已经到达hole0
	#define SI_HOLE1		(1<<5)	//样品孔径已经到达hole1
	#define SI_HOLE2		(1<<6)	//样品孔径已经到达hole2
	#define SI_HOLE3		(1<<7)	//样品孔径已经到达hole3
	#define SI_MOTO_PWR_ON 	(1<<8)	//样品电机上电
	#define SI_MOTO_PWR_OFF (1<<9)	//样品电机关闭	
#define MB_SAMPLE_ANGLE		29	//样品放置角度，16位整型数，单位0.1度
#define MB_SAMPLE_HOLE		30	//样品孔径电压值，16位整型数，单位mV

#define MB_SAMPLE_LED_ST	31		//样品LED状态
	#define SLED_PWR_ON		(1<<0)		//设备已上电
	#define SLED_PWR_OFF	(1<<1)		//设备已断电
	#define SLED1_PWR_ON	(1<<2)		//设备已上电
	#define SLED1_PWR_OFF	(1<<3)		//设备已断电

#define MB_BAFFLE_ST		32		//挡板状态
	#define BAFFLE_PWR_ON	POWER_ON	//设备已上电
	#define BAFFLE_PWR_OFF	POWER_OFF	//设备已断电

#define MB_BLEED_VALVE_ST 	33		//放气阀控制
//	#define BLEED_VALVE_PWR_ON		POWER_ON	//设备已上电
//	#define BLEED_VALVE_PWR_OFF		POWER_OFF	//设备已断电
//	#define BLEED_VALVE1_PWR_ON		(1<<2)	//设备已上电
//	#define BLEED_VALVE1_PWR_OFF	(1<<3)	//设备已断电
//	#define BLEED_VALVE2_PWR_ON		(1<<4)	//设备已上电
//	#define BLEED_VALVE2_PWR_OFF	(1<<5)	//设备已断电
	
	
#define MB_SYS_AUTOCTL_ST	34
	#define SYS_AUTO_ON		(1<<0)
	#define SYS_AUTO_OFF	(1<<1)
	#define SYS_AUTO_EN		(1<<8)
	#define SYS_AUTO_ON_ST 	4
	#define SYS_AUTO_OFF_ST 12
	
#define MB_MOTOR_ST			35
	#define MOTOR_FORWARD  (1<<0)
	#define MOTOR_BACKWARD (1<<1)
	#define MOTOR_ENABLE   (1<<7)	
	
#define MB_TEMP00			36			//温度点1温度
#define MB_TEMP01			37			//温度点2温度
#define MB_TEMP10			38			//温度点1温度
#define MB_TEMP11			39			//温度点2温度
	
//10129-10136
//ADC16位采样值
#define MB_ADC0		56	
#define MB_ADC1		57
#define MB_ADC2		58
#define MB_ADC3		59
#define MB_ADC4		60
#define MB_ADC5		61
#define MB_ADC6		62
#define MB_ADC7		63


//----------------------------------------------------------------------------------------------------------------------------------
//REGISTER  40001-49999 Holding Register (R/W)
#define MB_SYS_AUTOCTL			0
//	#define SYS_AUTO_ON			(1<<0)
//	#define SYS_AUTO_OFF		(1<<1)
//	#define SYS_AUTO_EN			(1<<8)
	
//左右高压枪共用设置参数
#define MB_VOL_MAX				1	//最大电压值,16位整型数，单位V
#define MB_VOL_SCALE			2	//高压枪电压采样比例，单位mV/mV
#define MB_VOL_ERR_RATE			3	//电压输出误差比率,16位整型数，单位1%
#define MB_VOL_STEP				4	//电压步进值,16位整型数，单位V
#define MB_VOL_STEP_INTERVAL	5	//电压步进间隔,16位整型数，单位mS
#define MB_VOL_STEP_TIMEOUT		6 	//电压步进输出超时,16位整型数，单位mS
#define MB_VOL_LEVEL1			7	//高压上升到此值后，开始加电流，仅用于自动控制模式，16位整型数，单位V

#define MB_CURRRENT_MAX			9	//最大电流值,16位整型数，单位uA
#define MB_CUR_SCALE			10	//高压枪电流采样比例，单位uA/mV
#define MB_CUR_ERR_RATE			11	//电流输出误差比率,16位整型数，单位1%
#define MB_CUR_STEP				12
#define MB_CUR_STEP_INTERVAL	13
#define MB_CUR_STEP_TIMEOUT		14	
#define MB_CUR_CTL_START		15

#define MB_VOL_SET_L			17	//主机电压设定值,16位整型数，单位V
#define MB_CUR_SET_L			18	//主机电流设定值,16位整型数，单位uA
#define MB_VOL_SET_R			19	//主机电压设定值,16位整型数，单位V
#define MB_CUR_SET_R			20	//主机电流设定值,16位整型数，单位uA

//机械泵
#define MB_POWERPUMP_CTL		24
//	#define PWR_ON		(1<<0)
//	#define PWR_OFF 	(1<<1)
	
//分子泵
#define MB_MPUMP_CTL			25	//分子泵控制及状态寄存器
#define MB_MPUMP_FREQ_SET		26 	//分子泵设定运行频率，16位整型数，单位Hz

//真空计
#define MB_VMETER_CTL			27
#define MB_VMETER_ERR_RATE		28	//真空计误差比率,16位整型数，单位1%
#define MB_VMETER_SET0			29
#define MB_VMETER_SET1			30
#define MB_VMETER_SET2			31
#define MB_VMETER_SET3			32


#define MB_SAMPLE_CTL			33	//设定样品信息及状态
//	#define SI_TRANSPARENCY		(1<<0)	//样品为透明的，
//	#define SI_MONITOR_EN		(1<<2)	//样品监测使能,
//	#define SI_PWR_OFF_EN		(1<<3)	//自动关机使能,
//	#define SI_HOLE0			(1<<4)	//样品孔径hole0
//	#define SI_HOLE1			(1<<5)	//样品孔径hole1
//	#define SI_HOLE2			(1<<6)	//样品孔径hole2
//	#define SI_HOLE3			(1<<7)	//样品孔径hole3
//	#define SI_MOTO_PWR_ON 		(1<<8)	//样品电机上电
//	#define SI_MOTO_PWR_OFF 	(1<<9)	//样品电机关闭	
#define MB_SAMPLE_ANGLE_SET		34		//样品放置角度，16位整型数，单位0.1度
#define MB_SAMPLE_HOLE0			35		//当样品孔径达到hole0时的电压值
#define MB_SAMPLE_START			36		//当样品监测开始时间,xx分钟之后开始
#define MB_SAMPLE_INTERVAL		37		//当样品监测间隔时间,分钟

#define MB_SAMPLE_LED			40		//样品LED状态
//	#define SLED_PWR_ON			POWER_ON	
//	#define SLED_PWR_OFF		POWER_OFF	
	
#define MB_BAFFLE				41		//挡板状态
	#define BAFFLE_PWR_ON		POWER_ON	
	#define BAFFLE_PWR_OFF		POWER_OFF	

#define MB_BLEED_VALVE 			42		//放气阀控制
	#define BLEED_VALVE_PWR_ON		POWER_ON	
	#define BLEED_VALVE_PWR_OFF		POWER_OFF	
	#define BLEED_VALVE1_PWR_ON		(POWER_ON<<2)	
	#define BLEED_VALVE1_PWR_OFF	(POWER_OFF<<2)
	#define BLEED_VALVE2_PWR_ON		(POWER_ON<<4)	
	#define BLEED_VALVE2_PWR_OFF	(POWER_OFF<<4)
	#define BLEED_VALVE3_PWR_ON		(POWER_ON<<6)	
	#define BLEED_VALVE3_PWR_OFF	(POWER_OFF<<6)

#define VMETER_START_DELAY 		43	//真空计开关定时器 － 机械泵开启，延迟数秒后，再开启真空计
#define VMETER_STOP_DELAY	 	44	//真空计关闭开关定时器 － 关闭真空计，延迟数秒后，再关闭机械泵

#define MB_MPUMP_PWR_OFF_FREQ	45	//分子泵低于些频率时可以关闭机械泵
#define MB_BAFFLE_INTERVAL		46	//档板开启间隔,分钟

#define MB_MOTOR_CTRL			47
	#define MOTOR_FORWARD  		(1<<0)
	#define MOTOR_BACKWARD 		(1<<1)
	#define MOTOR_ENABLE   		(1<<7)


#define MB_MF_RELAY_CTL			0x30
#define MB_MF_VMETER_CTL		0x31

#define MB_GSM_CTL 				50

#define MB_TEMP_SET00			0x38
#define MB_TEMP_SET01			0x39
#define MB_TEMP_SET10			0x3A
#define MB_TEMP_SET11			0x3B




#define MB_SMS_CMD      0x40
  #define MB_SMS_SEND   (1<<0)
  
#define MB_SMS_SERVER	0x46
#define MB_SMS_SERVER1	0x47
#define MB_SMS_SERVER2	0x48
#define MB_SMS_SERVER3	0x49
#define MB_SMS_SERVER4	0x4A
#define MB_SMS_SERVER5	0x4B
#define MB_SMS_SERVER6	0x4C
#define MB_SMS_SERVER7	0x4D
#define MB_SMS_SERVER8	0x4E
#define MB_SMS_SERVER9	0x4F

#define MB_SMS_PHONE	0x50
#define MB_SMS_PHONE1	0x51
#define MB_SMS_PHONE2	0x52
#define MB_SMS_PHONE3	0x53
#define MB_SMS_PHONE4	0x54
#define MB_SMS_PHONE5	0x55
#define MB_SMS_PHONE6	0x56
#define MB_SMS_PHONE7	0x57
#define MB_SMS_PHONE8	0x58
#define MB_SMS_PHONE9	0x59

#define MB_SMS_TEXT		0x60	
#define MB_SMS_TEXT1 	0x61	
#define MB_SMS_TEXT2 	0x62	
#define MB_SMS_TEXT3 	0x63	
#define MB_SMS_TEXT4 	0x64	
#define MB_SMS_TEXT5 	0x65	
#define MB_SMS_TEXT6 	0x66	
#define MB_SMS_TEXT7 	0x67	
#define MB_SMS_TEXT8 	0x68	
#define MB_SMS_TEXT9 	0x69	
#define MB_SMS_TEXT10 	0x6A	
#define MB_SMS_TEXT11 	0x6B	
#define MB_SMS_TEXT12 	0x6C	
#define MB_SMS_TEXT13 	0x6D	
#define MB_SMS_TEXT14 	0x6E	
#define MB_SMS_TEXT15 	0x6F	
#define MB_SMS_TEXT16 	0x70	
#define MB_SMS_TEXT17 	0x71	
#define MB_SMS_TEXT18 	0x72	
#define MB_SMS_TEXT19 	0x73	
#define MB_SMS_TEXT20 	0x74	
#define MB_SMS_TEXT21 	0x75	
#define MB_SMS_TEXT22 	0x76	
#define MB_SMS_TEXT23 	0x77	
#define MB_SMS_TEXT24 	0x78	
#define MB_SMS_TEXT25 	0x79	
#define MB_SMS_TEXT26 	0x7A	
#define MB_SMS_TEXT27 	0x7B	
#define MB_SMS_TEXT28 	0x7C
#define MB_SMS_TEXT29 	0x7D	
#define MB_SMS_TEXT30 	0x7E	
#define MB_SMS_TEXT31 	0x7F	
#define MB_SMS_TEXT32 	0x80
#define MB_SMS_TEXT33 	0x81	
#define MB_SMS_TEXT34 	0x82	
#define MB_SMS_TEXT35 	0x83	
#define MB_SMS_TEXT36 	0x84	
#define MB_SMS_TEXT37 	0x85	
#define MB_SMS_TEXT38 	0x86	
#define MB_SMS_TEXT39 	0x87	
#define MB_SMS_TEXT40 	0x88	
#define MB_SMS_TEXT41 	0x89	
#define MB_SMS_TEXT42 	0x8A	
#define MB_SMS_TEXT43 	0x8B	
#define MB_SMS_TEXT44 	0x8C	
#define MB_SMS_TEXT45 	0x8D	
#define MB_SMS_TEXT46 	0x8E	
#define MB_SMS_TEXT47 	0x8F	
#define MB_SMS_TEXT48 	0x90	
#define MB_SMS_TEXT49 	0x91	
#define MB_SMS_TEXT50 	0x92	
#define MB_SMS_TEXT51 	0x93	
#define MB_SMS_TEXT52 	0x94	
#define MB_SMS_TEXT53 	0x95	
#define MB_SMS_TEXT54 	0x96	
#define MB_SMS_TEXT55 	0x97	
#define MB_SMS_TEXT56 	0x98	
#define MB_SMS_TEXT57 	0x99	
#define MB_SMS_TEXT58 	0x9A	
#define MB_SMS_TEXT59 	0x9B	
#define MB_SMS_TEXT60 	0x9C	
#define MB_SMS_TEXT61 	0x9D	
#define MB_SMS_TEXT62 	0x9E	
#define MB_SMS_TEXT63 	0x9F	

//----------------------------------------------------------------------

#define MB_FD110A		0xA0
#define MB_BEEP_ON		0xA1
#define MB_BEEP_OFF		0xA2
#define MB_LED_ON		0xA3
#define MB_LED_OFF		0xA4
#define MB_L298_CTL		0xA5

//DAC输出电压设定，16位整型数，单位mV
#define MB_DAC0			0xB0
#define MB_DAC1			0xB1
#define MB_DAC2			0xB2
#define MB_DAC3			0xB3
#define MB_DAC4			0xB4
#define MB_DAC5			0xB5
#define MB_DAC6			0xB6
#define MB_DAC7			0xB7


#endif
