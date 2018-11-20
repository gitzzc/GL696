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
	//1 		DO0		���Ҹ�ѹǹ��Դ
	//2 		DO1		�����Դ
	//3 		DO2		��Ʒ�����Դ
	//4 		DO3		�����õ�Դ
	//5 		DO4		��Ʒͨ�׼��LED ��Դ
	//38-65 not use
//33-65 RELAY
	//33		relay0	��е�á���ռ�
	//34		relay1	���ӱ�
	//35	 	not use

//REGISTER  30001-39999 Input Register  (R)
//30001	-	��ǹ����״̬
#define MB_HV_ST_L 			0	
	#define HV_PWR 			(1<<0)
	#define HV_SET_OK		(1<<1)
	#define HV_LEVEL1		(1<<2)
	#define HV_CUR_SET_OK	(1<<3)
	#define HV_ENABLE		(1<<7)
	
	#define HV_SET_TO 		(1<<8)
	#define HV_CUR_OV		(1<<9)
	#define HV_INCTRL		(1<<10)
//30002	-	��ǹ��ѹʵ�ʲ���ֵ��16λ����������λV
#define MB_VOL_FB_L 	1	
//30003	-	��ǹ����ʵ�ʲ���ֵ��16λ����������λV
#define MB_CUR_FB_L		2	
//30004	-	��ǹ��ѹ���ƶ˵�ǰ�趨����������ʹ�ã���16λ����������λV
#define MB_VOL_CTL_L	3
//30005	-	��ǹ�������ƶ˵�ǰ�趨����������ʹ�ã���16λ����������λuA
#define MB_CUR_CTL_L	4	

//30006	-	��ǹ����״̬
#define MB_HV_ST_R 		5	
//30007	-	��ǹ��ѹʵ�ʲ���ֵ��16λ����������λV
#define MB_VOL_FB_R 	6
//30008	-	��ǹ����ʵ�ʲ���ֵ��16λ����������λmA
#define MB_CUR_FB_R		7	
//30009	-	��ǹ��ѹ���ƶ˵�ǰ�趨����������ʹ�ã���16λ����������λV
#define MB_VOL_CTL_R	8
//30010	-	��ǹ�������ƶ˵�ǰ�趨����������ʹ�ã���16λ����������λuA
#define MB_CUR_CTL_R	9	
//30011	-	��ǹ��ѹ��ǰ�趨��16λ����������λV
#define MB_VOL_SET_L_ST	10
//30012	-	��ǹ������ǰ�趨��16λ����������λuA
#define MB_CUR_SET_L_ST	11
//30013	-	��ǹ��ѹ��ǰ�趨��16λ����������λV
#define MB_VOL_SET_R_ST	12
//30014	-	��ǹ������ǰ�趨��16λ����������λuA
#define MB_CUR_SET_R_ST	13

//��е��
#define MB_POWERPUMP_ST	16
	#define POWERPUMP_PWR_ON 	POWER_ON
	#define POWERPUMP_PWR_OFF 	POWER_OFF
	
//30017	-	���ӱ�״̬�Ĵ���
#define MB_MPUMP_ST				17
//	#define MPUMP_NORMAL		(1<<2)	//�豸��������
	#define MPUMP_HIGH_SP		(1<<3)	//�豸���е�����״̬
	#define MPUMP_RUN		 	(1<<4)	//�豸������
	#define MPUMP_OVERCUR		(1<<5)	//�豸����
	#define MPUMP_OVERHOT		(1<<6)	//�豸��ѹ
	#define MPUMP_TIMEOUT		(1<<7)	//�豸��ʱ

	#define MPUMP_PWR_ON		(1<<8)	//�豸���ϵ�
	#define MPUMP_PWR_OFF		(1<<9)	//�豸���µ�
	#define MPUMP_LOW_SP		(1<<10)	//
	#define MPUMP_STOP			(1<<11)	//
	#define MPUMP_INCTRL		(1<<12)	//�豸���ɿ�
	#define MPUMP_PWR_OFFING 	(1<<13)
//30018	-	���ӱõ�ǰ����Ƶ�ʣ�16λ����������λHz
#define MB_MPUMP_FREQ		18	
//30019	-	���ӱõ�ǰ������16λ����������λmA
#define MB_MPUMP_CUR		19	
//30020	-	���ӱõ�ǰ��ѹ��16λ����������λV
#define MB_MPUMP_VOL		20

//30025	-	��ռ�״̬
#define MB_VMETER_ST		24
	#define VMETER_PWR_ON 	POWER_ON	//��Դ����״̬��1��ʾ��Դ����
	#define VMETER_PWR_OFF	POWER_OFF	//��Դ�ر�״̬��1��ʾ��Դ�ر�
	#define VMETER_WAIT_ON	(1<<2)
	#define VMETER_WAIT_OFF	(1<<3)
//30026	-	���ֵ,��MB_VMTER1һ�����32λ��������(float)((int)(MB_VMETER0<<16) | MB_VMETER1)
#define MB_VMETER0			25	//��MB_VMETER1���32bit��������bit31-16
//30027	-	
#define MB_VMETER1			26	//��MB_VMETER0���32bit��������bit15-0

#define MB_SAMPLE_ST		28	//��Ʒ��Ϣ��״̬
	#define SI_TRANSPARENCY	(1<<0)	//��ƷΪ͸���ģ�
	#define SI_MONITOR_EN	(1<<2)	//��Ʒ���ʹ��,
	#define SI_PWR_OFF_EN	(1<<3)	//�Զ��ػ�ʹ��,	
	#define SI_HOLE0		(1<<4)	//��Ʒ�׾��Ѿ�����hole0
	#define SI_HOLE1		(1<<5)	//��Ʒ�׾��Ѿ�����hole1
	#define SI_HOLE2		(1<<6)	//��Ʒ�׾��Ѿ�����hole2
	#define SI_HOLE3		(1<<7)	//��Ʒ�׾��Ѿ�����hole3
	#define SI_MOTO_PWR_ON 	(1<<8)	//��Ʒ����ϵ�
	#define SI_MOTO_PWR_OFF (1<<9)	//��Ʒ����ر�	
#define MB_SAMPLE_ANGLE		29	//��Ʒ���ýǶȣ�16λ����������λ0.1��
#define MB_SAMPLE_HOLE		30	//��Ʒ�׾���ѹֵ��16λ����������λmV

#define MB_SAMPLE_LED_ST	31		//��ƷLED״̬
	#define SLED_PWR_ON		(1<<0)		//�豸���ϵ�
	#define SLED_PWR_OFF	(1<<1)		//�豸�Ѷϵ�
	#define SLED1_PWR_ON	(1<<2)		//�豸���ϵ�
	#define SLED1_PWR_OFF	(1<<3)		//�豸�Ѷϵ�

#define MB_BAFFLE_ST		32		//����״̬
	#define BAFFLE_PWR_ON	POWER_ON	//�豸���ϵ�
	#define BAFFLE_PWR_OFF	POWER_OFF	//�豸�Ѷϵ�

#define MB_BLEED_VALVE_ST 	33		//����������
//	#define BLEED_VALVE_PWR_ON		POWER_ON	//�豸���ϵ�
//	#define BLEED_VALVE_PWR_OFF		POWER_OFF	//�豸�Ѷϵ�
//	#define BLEED_VALVE1_PWR_ON		(1<<2)	//�豸���ϵ�
//	#define BLEED_VALVE1_PWR_OFF	(1<<3)	//�豸�Ѷϵ�
//	#define BLEED_VALVE2_PWR_ON		(1<<4)	//�豸���ϵ�
//	#define BLEED_VALVE2_PWR_OFF	(1<<5)	//�豸�Ѷϵ�
	
	
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
	
#define MB_TEMP00			36			//�¶ȵ�1�¶�
#define MB_TEMP01			37			//�¶ȵ�2�¶�
#define MB_TEMP10			38			//�¶ȵ�1�¶�
#define MB_TEMP11			39			//�¶ȵ�2�¶�
	
//10129-10136
//ADC16λ����ֵ
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
	
//���Ҹ�ѹǹ�������ò���
#define MB_VOL_MAX				1	//����ѹֵ,16λ����������λV
#define MB_VOL_SCALE			2	//��ѹǹ��ѹ������������λmV/mV
#define MB_VOL_ERR_RATE			3	//��ѹ���������,16λ����������λ1%
#define MB_VOL_STEP				4	//��ѹ����ֵ,16λ����������λV
#define MB_VOL_STEP_INTERVAL	5	//��ѹ�������,16λ����������λmS
#define MB_VOL_STEP_TIMEOUT		6 	//��ѹ���������ʱ,16λ����������λmS
#define MB_VOL_LEVEL1			7	//��ѹ��������ֵ�󣬿�ʼ�ӵ������������Զ�����ģʽ��16λ����������λV

#define MB_CURRRENT_MAX			9	//������ֵ,16λ����������λuA
#define MB_CUR_SCALE			10	//��ѹǹ����������������λuA/mV
#define MB_CUR_ERR_RATE			11	//�������������,16λ����������λ1%
#define MB_CUR_STEP				12
#define MB_CUR_STEP_INTERVAL	13
#define MB_CUR_STEP_TIMEOUT		14	
#define MB_CUR_CTL_START		15

#define MB_VOL_SET_L			17	//������ѹ�趨ֵ,16λ����������λV
#define MB_CUR_SET_L			18	//���������趨ֵ,16λ����������λuA
#define MB_VOL_SET_R			19	//������ѹ�趨ֵ,16λ����������λV
#define MB_CUR_SET_R			20	//���������趨ֵ,16λ����������λuA

//��е��
#define MB_POWERPUMP_CTL		24
//	#define PWR_ON		(1<<0)
//	#define PWR_OFF 	(1<<1)
	
//���ӱ�
#define MB_MPUMP_CTL			25	//���ӱÿ��Ƽ�״̬�Ĵ���
#define MB_MPUMP_FREQ_SET		26 	//���ӱ��趨����Ƶ�ʣ�16λ����������λHz

//��ռ�
#define MB_VMETER_CTL			27
#define MB_VMETER_ERR_RATE		28	//��ռ�������,16λ����������λ1%
#define MB_VMETER_SET0			29
#define MB_VMETER_SET1			30
#define MB_VMETER_SET2			31
#define MB_VMETER_SET3			32


#define MB_SAMPLE_CTL			33	//�趨��Ʒ��Ϣ��״̬
//	#define SI_TRANSPARENCY		(1<<0)	//��ƷΪ͸���ģ�
//	#define SI_MONITOR_EN		(1<<2)	//��Ʒ���ʹ��,
//	#define SI_PWR_OFF_EN		(1<<3)	//�Զ��ػ�ʹ��,
//	#define SI_HOLE0			(1<<4)	//��Ʒ�׾�hole0
//	#define SI_HOLE1			(1<<5)	//��Ʒ�׾�hole1
//	#define SI_HOLE2			(1<<6)	//��Ʒ�׾�hole2
//	#define SI_HOLE3			(1<<7)	//��Ʒ�׾�hole3
//	#define SI_MOTO_PWR_ON 		(1<<8)	//��Ʒ����ϵ�
//	#define SI_MOTO_PWR_OFF 	(1<<9)	//��Ʒ����ر�	
#define MB_SAMPLE_ANGLE_SET		34		//��Ʒ���ýǶȣ�16λ����������λ0.1��
#define MB_SAMPLE_HOLE0			35		//����Ʒ�׾��ﵽhole0ʱ�ĵ�ѹֵ
#define MB_SAMPLE_START			36		//����Ʒ��⿪ʼʱ��,xx����֮��ʼ
#define MB_SAMPLE_INTERVAL		37		//����Ʒ�����ʱ��,����

#define MB_SAMPLE_LED			40		//��ƷLED״̬
//	#define SLED_PWR_ON			POWER_ON	
//	#define SLED_PWR_OFF		POWER_OFF	
	
#define MB_BAFFLE				41		//����״̬
	#define BAFFLE_PWR_ON		POWER_ON	
	#define BAFFLE_PWR_OFF		POWER_OFF	

#define MB_BLEED_VALVE 			42		//����������
	#define BLEED_VALVE_PWR_ON		POWER_ON	
	#define BLEED_VALVE_PWR_OFF		POWER_OFF	
	#define BLEED_VALVE1_PWR_ON		(POWER_ON<<2)	
	#define BLEED_VALVE1_PWR_OFF	(POWER_OFF<<2)
	#define BLEED_VALVE2_PWR_ON		(POWER_ON<<4)	
	#define BLEED_VALVE2_PWR_OFF	(POWER_OFF<<4)
	#define BLEED_VALVE3_PWR_ON		(POWER_ON<<6)	
	#define BLEED_VALVE3_PWR_OFF	(POWER_OFF<<6)

#define VMETER_START_DELAY 		43	//��ռƿ��ض�ʱ�� �� ��е�ÿ������ӳ�������ٿ�����ռ�
#define VMETER_STOP_DELAY	 	44	//��ռƹرտ��ض�ʱ�� �� �ر���ռƣ��ӳ�������ٹرջ�е��

#define MB_MPUMP_PWR_OFF_FREQ	45	//���ӱõ���ЩƵ��ʱ���Թرջ�е��
#define MB_BAFFLE_INTERVAL		46	//���忪�����,����

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


//----------------------------------------------------------------------
#define MB_BASE		0X100
#define MB_POWER_PUMP		(MB_BASE+0x01)
#define MB_MOLECULAR_PUMP	(MB_BASE+0x02)
#define MB_VMETER			(MB_BASE+0x03)

//----------------------------------------------------------------------

#define MB_FD110A		0xA0
#define MB_BEEP_ON		0xA1
#define MB_BEEP_OFF		0xA2
#define MB_LED_ON		0xA3
#define MB_LED_OFF		0xA4
#define MB_L298_CTL		0xA5

//DAC�����ѹ�趨��16λ����������λmV
#define MB_DAC0			0xB0
#define MB_DAC1			0xB1
#define MB_DAC2			0xB2
#define MB_DAC3			0xB3
#define MB_DAC4			0xB4
#define MB_DAC5			0xB5
#define MB_DAC6			0xB6
#define MB_DAC7			0xB7


#endif
