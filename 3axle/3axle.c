/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;  // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

#include <stdio.h>
#include <math.h>
#include "init_rx62n.h"
#include "iodefine.h"

#define ON							1
#define OFF							0

#define D//ebug1		//�X�^�[�g�{�^�������O�̏������Ȃ�
#define D//ebug2		//�R���g���[���[�Ȃ��ŃX�^�[�g

#define ROUTE_SQUARE		1	//�����`���s
#define CIRCLE						2	//�~���s

#define AIR							PORTD.DR.BIT.B7

//�V���A���ʐM
#define MODE_SCIDATA_BOX 	6

#define CALIBRATION			3	//�L�����u��
#define MANUAL_CONTROL	4	//�蓮���c
#define AUTO_CONTROL		5	//�������c

/*���[�h�ؑ�*/
#define OUTPUT_MODE			MANUAL_CONTROL
#define ROUTE_MODE			SQUARE
#define SERIAL_MODE			ON

#define INTERRUPT_START	CMT.CMSTR0.BIT.STR0 = 1;//�J�E���g�J�n

#define PWM_PERIOD			((48000000/1)/100000)

/*LED*/
#define LED_P8						PORT8.DR.BYTE
#define LED_P80					PORT8.DR.BIT.B0
#define LED_P81					PORT8.DR.BIT.B1

/*���荞�ݎ���*/
#define INTERRUPT_TIME 		0.005

/*PD����*/
#if OUTPUT_MODE == CALIBRATION //�L�����u��
	#define ROCK_P_GAIN			20.0
	#define ROCK_D_GAIN			45.0
#else
	#define ROCK_P_GAIN			1.5//5.0
	#define ROCK_D_GAIN			10.0//20.0
#endif

#define STRAIGHT_P_GAIN	0.2
#define STRAIGHT_D_GAIN	0.0

/*�ԑ̑���*/
#define MAX_VELOCITY			400.0
#define MAX_DUTY				g_max_duty
#define PWM_PER					90.0
#define OPERATE_DEGREE		180.0//120.0	//1000ms Max�ŃX�e�b�N��|�����Ƃ��ǂꂾ����]���邩�i�x�j

/*���s��������*/
#define MILEAGE					6000.0

/*�~�̔��a*/
#define RADIUS						1000.0

/*�����*/
#define LEFT_TIRE_CW			PORT7.DR.BIT.B5
#define LEFT_TIRE_CCW		PORT7.DR.BIT.B7
#define LEFT_TIRE_DUTY		MTU4.TGRD
#define RIGHT_TIRE_CW		PORT7.DR.BIT.B4
#define RIGHT_TIRE_CCW		PORT7.DR.BIT.B6
#define RIGHT_TIRE_DUTY	MTU4.TGRB
#define BACK_TIRE_CW			PORT7.DR.BIT.B0
#define BACK_TIRE_CCW		PORT7.DR.BIT.B2
#define BACK_TIRE_DUTY		MTU6.TGRB

#define M_DUTY					MTU6.TGRD
#define	M_CW						PORT7.DR.BIT.B3
#define	M_CCW						PORT7.DR.BIT.B1

/*�ʓx�@�A�x���@�@�ϊ�*/
#define M_PI							3.141592653
#define R_TO_D(x) 				( x * ( 180 / M_PI ) )
#define D_TO_R(x)				( x * ( M_PI / 180 ) )

/*����肪�����Ȃ��X�e�B�b�N�̒l�͈̔�*/
#define STICK_NO_MOVE_RANGE 			0.2		//�}20%

/*���[�^�[�o��*/
#define	FREE							99999

/*�G���R�[�_�J�E���g*/
#define ENCF()						mtu1_count()
#define ENCL()						mtu8_count()
#define ENCR()						mtu2_count()

/*���S����G���R�[�_�܂ł̋���*/
#define CENTER_TO_ENC		237.0

#define PULSE						2000.0

/*�G���R�[�_�^�C�����a*/
#define ENC_DIAMETER_F 		51.0//( 51.00901999 )
#define ENC_DIAMETER_L		51.0//( 51.43542 )
#define ENC_DIAMETER_R		51.0//( 51.13076 )

#define BUZZER						PORT2.DR.BIT.B2

/*�������W*/
#define POSITION_X				0.00
#define POSITION_Y				0.00

#define END 							'#'			//�ʐM�f�[�^�̏I�[����
#define RECEIVE_STR_COLUMN 32		//1�f�[�^������̍ő啶����		��: a123# (6����)

#define LEFT_STICK_WIDE			g_atoz_value[(int)('a' - 'a')]
#define LEFT_STICK_HIGH			g_AtoZ_value[(int)('A' - 'A')]
#define RIGHT_STICK_WIDE		g_atoz_value[(int)('b' - 'a')]
#define RIGHT_STICK_HIGH		g_AtoZ_value[(int)('B' - 'A')]
#define UP_SW							g_atoz_value[(int)('c' - 'a')]
#define RIGHT_SW					g_atoz_value[(int)('d' - 'a')]
#define DOWN_SW					g_atoz_value[(int)('e' - 'a')]
#define LEFT_SW						g_atoz_value[(int)('f' - 'a')]
#define TRIANGLE_SW				g_atoz_value[(int)('g' - 'a')]
#define CIRCLE_SW					g_atoz_value[(int)('h' - 'a')]
#define CROSS_SW					g_atoz_value[(int)('i' - 'a')]
#define SQUARE_SW					g_atoz_value[(int)('j' - 'a')]
#define START_SELECT_SW		g_atoz_value[(int)('k' - 'a')]
#define PS_SW							g_atoz_value[(int)('l' - 'a')] 
#define ACC_X							g_atoz_value[(int)('m' - 'a')]
#define ACC_Y							g_atoz_value[(int)('n' - 'a')]
#define ACC_Z							g_atoz_value[(int)('o' - 'a')]


//�O���[�o���ϐ��Ɋi�[����ꍇ	���΂��ȗ�
float	g_atoz_value[26]	=	{	0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
											0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
											0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
								
float	g_AtoZ_value[26]	=	{	0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
											0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
											0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

/*1ms�J�E���g*/
float	g_count_time			= 0.00,
		g_left_duty_time	= 0.00,
		g_right_duty_time	= 0.00,
		g_back_duty_time	= 0.00,
		g_time					= 0.00,
		g_duty_time			= 0.00,
		g_stop_time			= 0.00;

/*overfloaw underfloaw �J�E���g*/
int		g_mtu1_over		= 0, 
		g_mtu1_under	= 0,
		g_mtu2_over		= 0,
		g_mtu2_under	= 0,
		g_mtu8_over		= 0,
		g_mtu8_under	= 0;
		
int g_start_switch = 0;
		
/*R1350N*/
int		g_Rate		= 0,
		g_Angle		= 0,
		g_X_acc 	= 0,
		g_Y_acc 	= 0,
		g_Z_acc 	= 0;

float	g_Rate_f	= 0.00,
		g_Angle_f	= 0.00;
		
/*���Ȉʒu*/
float	g_vertex_b_x	= 0.0,	g_vertex_b_y	= 0.0,
		g_vertex_l_x	= 0.0,	g_vertex_l_y	= 0.0,
		g_vertex_r_x	= 0.0,	g_vertex_r_y	= 0.0,
		g_x_c			= 0.0,	g_y_c			= 0.0;
		
float	g_now_x		= 0.00,
		g_now_y		= 0.00,
		g_degree		= 0.00,
		g_velocity		= 0.00,
		g_angular_velocity = 0.00;
		
/*�ő呬�x*/
float	g_max_duty = 95;	
float  g_max_velocity = MAX_VELOCITY;
float	g_motor_output_l = 0.00,
		g_motor_output_r = 0.00,
		g_motor_output_b = 0.00;

typedef struct Target
{
	float x_c;
	float y_c;
}Target;

typedef struct{
	float sci_data1;
	float sci_data2;
	float sci_data3;
	float sci_data4;
	float sci_data5;
	float sci_data6;
	float sci_data7;
	float sci_data8;
}Sci_data;

/*ps2�R���f�[�^*/
union psdate1{
		unsigned long dword;
		struct{
			unsigned char byte1;
			unsigned char model_number;
			unsigned char byte3;
			unsigned char select_sw:1;
			unsigned char l3_sw:1;
			unsigned char r3_sw:1;
			unsigned char start_sw:1;
			unsigned char up_sw:1;
			unsigned char right_sw:1;
			unsigned char down_sw:1;
			unsigned char left_sw:1;
		}byte;
	};
	union psdate2{
		unsigned long dword;
		struct{
			unsigned char l2_sw:1;
			unsigned char r2_sw:1;
			unsigned char l1_sw:1;
			unsigned char r1_sw:1;
			unsigned char triangle_sw:1;
			unsigned char circle_sw:1;
			unsigned char cross_sw:1;
			unsigned char square_sw:1;
			unsigned char right_stick_wide;
			unsigned char right_stick_high;
			unsigned char left_stick_wide;
		}byte;
	};
	union psdate3{
		unsigned long dword;
		struct{
			unsigned char left_stick_high;
		}byte;
	};
	
volatile unsigned long	g_controller_receive_1st		= 0;	//�R���g���[������A���Ă���f�[�^�̊i�[�t�H���_1st�͕s��ł悢�B
volatile unsigned long	g_controller_receive_2nd	= 0;
volatile unsigned long	g_controller_receive_3rd		= 0;

int 	getdate1 = 0,
		getdate2 = 0,
		getdate3 = 0;

/*�v���g�^�C�v�錾*/
void timer( void );
void over_1( void );
void under_1( void );
void over_2( void );
void under_2( void );
void over_8( void );
void under_8( void );
float mtu1_count( void );
float mtu2_count( void );
float mtu8_count( void );
unsigned long Rspi_send_1(unsigned long moji);
unsigned long Rspi_send_short_1(unsigned short int moji);
void Rspi_recive_send_line_dualshock(void);
void transmit( char str[ ] , int ch);
char Receive_uart_c( void );
float Limit_ul(float upper,float lower,float figure);
void input_R1350N(void);
int checksum_r1350n(char *str);
float revision_degree( float degree );
float pd_rock( float present , float target );
float pd_straight( float present , float target );
void move_left_tire( float left_duty );
void move_right_tire( float right_duty );
void move_back_tire( float back_duty );
float get_motor_output_l(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_r(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_b(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_x( float straight , float target_degree );
float get_motor_output_y( float straight , float target_degree );
void Move( float left_duty , float right_duty , float back_duty );
float pick_out_atoz_value(char character);
void throw_away_atoz_value(char character, float write);
float change_float(char *str);
void receive_order_depot(int target_box, char *storage_str, int minus_flag, int after_point_count, int large_size_flag);
void receive_order_c(char character);
void receive_order_s(char *word);
int stop_duty( void );
void buzzer_cycle( float time );
float straight_output_x( void );
float straight_output_y( void );
float turn_output( void );
void initialization( void );
float get_present_target_dis( float , float );
float get_target_degree( float deviation_x , float deviation_y );
float get_horizontal_distance( float future_degree , float now_degree , float distance );
float get_vertical_distance( float future_degree , float now_degree , float distance );
float get_target_velocity( float distance_rest , float vertical_distance , float a_up ,  float a_down );
void free_output( void );
void position_rock(float target_x, float target_y, float now_x, float now_y, float now_degree );
void get_robot_inf(void);
void sci_transformer(Sci_data	*send);
void move_m( float duty );

void initial_config( void )
{
	init_clock();
	init_CMT0();
	init_pwm();
	init_SCI0();
	init_SCI1();
	init_SCI2();
	init_encorder();
	//init_Rspi_dualshock();
	
	INTERRUPT_START
}

void io_config( void )
{
	//LED
	PORT8.DDR.BYTE = 3;
	/*�u�U�[*/
	PORT2.DDR.BIT.B2 = 1;
	/*�d����*/
	PORTD.DDR.BIT.B7  = 1;
}

void main(void)
{
	#if SERIAL_MODE == ON
		char	str[30] = { 0 };
	#endif
	
	float	Motor_output_x = 0.00,
			Motor_output_y = 0.00;
	
	float	target_x	= 0.00,
			target_y	= 0.00;
			
	float	radius	= RADIUS,		//���a
			theta		= 0.00;			//��
			
	float	motor_output			= 0.00,
			motor_output_l		= 0.00,
			motor_output_r		= 0.00,
			motor_output_b		= 0.00,
			m_out					= 0.00,
			output_l					= 0.00,
			output_r				= 0.00,
			output_b				= 0.00,
			motor_output_turn	= 0.00;
			
	float present_target_distance	= 0.00,
			vertical_distance			= 0.00,
			horizontal_distance			= 0.00;
			
	float old_x	= 0.00;
	float old_y	= 0.00;
			
	float	future_degree		= 0.00,
			rock_degree			= 0.00,
			old_target_degree	= 0.00,
			next_degree			= 0.00,
			target_degree		= 0.00;
			
	float target_velocity	= 0.00;
	
	float straight	=	0.00;
	
	int		task				= 1,
			task_box		= 0,
			mileage_flag	= 0,
			flag				= 0,
			stop_flag		= 0;

	int		ps_switch		= 0,
			up_switch		= 0,
			circle_switch	= 0,
			down_switch	= 0;
	
	Sci_data send = {0.0};
	
	Target pattern[ 6 ] =
		{	{ POSITION_X , POSITION_Y },
			{ 1000 , 0 },
			{ 1000 , 1000 },
			{ 0 , 1000 },		
			{ 0 , 0 },
			{ 1000 , 0 },
		};
	
	//�����ݒ�
	initial_config();
	//io�ݒ�
	io_config();
	
	while( 1 ){
		if( g_count_time >= INTERRUPT_TIME ){
			g_count_time = 0;
			
			get_robot_inf(); //���Ȉʒu�v�Z
			
			#ifndef Debug2
				stop_flag = stop_duty();
			#endif
			
			if( (int)START_SELECT_SW == 1 ){
				LED_P81 = 1;
				g_start_switch = 1;
			}
			
			if( PS_SW == 1 ){
				ps_switch = 1;
			}
			
			if( UP_SW >= 2 ){
				g_max_duty = 95;
			}
			
			if(DOWN_SW >= 2){
				g_max_duty = 50;
			}
			
			#ifndef Debug1
				if( g_start_switch == 0 ){
					initialization();
				}
			#endif
			
			#ifdef Debug2
				if( g_stop_time >= 6.00 ){
					g_start_switch = 1;
					BUZZER = 0;
				}else{
					buzzer_cycle( 1.00 );
				}
			#endif
		
			if( g_start_switch == 1 ){
/**************�蓮���[�h***************************/
				if( CIRCLE_SW >= 1 && TRIANGLE_SW == 0 ){
					m_out = 80;
					
				//}//else{
				//	move_m( 0 );
				//}
				
				}else if( TRIANGLE_SW >= 1 && CIRCLE_SW == 0 ){
					m_out = - 80;
				}else{
					m_out = 0;
				}
				
				if( SQUARE_SW >= 1 ){
					AIR = 1;
				}else{
					AIR = 0;
				}
			//}else{
				//	circle_switch = 0;
				//}
				//}else{
				//	m_out = 0;
			//	}
				
				if(flag == 0){
					flag = 1;
				}else if( flag == 1 ){
					flag = 0;
				}			
				
				if( flag == 1 ){
					//AIR = ON;
				}else{
					//AIR = OFF;
				}

				#if OUTPUT_MODE == MANUAL_CONTROL
					//x����
					Motor_output_x = straight_output_x();
					//y����
					Motor_output_y = straight_output_y();
					//�ԑ̊p�x����
					target_degree += turn_output();
					target_degree = revision_degree( target_degree );
					
					//�E�X�e�B�b�N�A���X�e�B�b�N����Ȃ����@�~�{�^�������ꂽ�Ƃ�
					if( (Motor_output_x == 0 && Motor_output_y == 0 && old_target_degree == target_degree ) || CROSS_SW >= 2){
						position_rock( old_x , old_y , g_now_x , g_now_y , g_degree);
						motor_output_l	= g_motor_output_l;
						motor_output_r	= g_motor_output_r;
						motor_output_b	= g_motor_output_b;
						
					}else{
						motor_output_l	= get_motor_output_l( Motor_output_x, Motor_output_y, 0 );
						motor_output_r	= get_motor_output_r( Motor_output_x, Motor_output_y, 0 );
						motor_output_b	= get_motor_output_b( Motor_output_x, Motor_output_y, 0 );
						old_x = g_now_x;
						old_y = g_now_y;
					}
					
					//�ԑ̊p�x�����ς���
					if(Motor_output_x == 0 && Motor_output_y == 0 && old_target_degree != target_degree){
						motor_output_l	= 0;
						motor_output_r	= 0;
						motor_output_b	= 0;	
					}
					
					motor_output_turn = pd_rock( g_degree , target_degree );

					old_target_degree = target_degree;
/***************�������[�h************************************/					
				#elif OUTPUT_MODE == AUTO_CONTROL
				/*********�����`********/
					#if ROUTE_MODE == SQUARE
						//���ݒn����ڕW���W�̊p�x
						future_degree	= get_target_degree( pattern[ task ].x_c - g_now_x , pattern[ task ].y_c - g_now_y );
						//���̖ڕW���玟�̖ڕW�̊p�x
						next_degree		= get_target_degree( pattern[ task ].x_c - pattern[ task-1 ].x_c , pattern[ task ].y_c - pattern[ task - 1].y_c );
						//���ݍ��W����ڕW���W�܂ł̋���
						present_target_distance = get_present_target_dis( pattern[ task ].x_c - g_now_x , pattern[ task ].y_c - g_now_y );
						//��������
						vertical_distance = get_vertical_distance( future_degree , g_degree , present_target_distance );
						//��������
						horizontal_distance = get_horizontal_distance( future_degree , g_degree , present_target_distance );
						
						//�ڕW���x�v�Z
						switch( task ){
							case 1:
								target_velocity 	= get_target_velocity( present_target_distance , vertical_distance , 50 , 500 );
							break;
							
							case 2:
								target_velocity 	= get_target_velocity( present_target_distance , horizontal_distance , 50 , 500 );
							break;
							
							case 3:
								target_velocity 	= get_target_velocity( present_target_distance , ( 1)*vertical_distance , 50 , 500 );
							break;
							
							case 4:
								target_velocity 	= get_target_velocity( present_target_distance , (-1)*horizontal_distance , 50 , 500 );
							break;
						}
						
						straight				= pd_straight( g_velocity , target_velocity );
						Motor_output_x	= get_motor_output_x( straight , next_degree );
						Motor_output_y	= get_motor_output_y( straight , next_degree );
						
						switch( task ){
							case 1:
								motor_output_turn	= pd_rock( g_degree , 0 );
								if( vertical_distance <= 50 ){
									task	= 2;
								}break;
							
							case 2:
								motor_output_turn	= pd_rock( g_degree , 0 );
								if( horizontal_distance <= 50 ){
									task	= 3;
								}break;
								
							case 3:
								motor_output_turn	= pd_rock( g_degree , 0 );
								if( (-1)*vertical_distance <= 50 ){
									task	= 4;
								}break;
							
							case 4:
								motor_output_turn	= pd_rock( g_degree , 0 );
								if( (-1)*horizontal_distance <= 50 ){
									task	= 5;
								}break;
							
							case 5:
								free_output();
							break;

							default:
								free_output();
							break;
						}
			/************�~**************/
					#elif ROUTE_MODE == CIRCLE
						//�~�̖ڕW���W
						target_x = radius * ( cos( R_TO_D( revision_degree( theta ) ) ) + 1 );
						target_y = radius * sin( R_TO_D( revision_degree( theta ) ));

						//���ݒn����ڕW���W�̊p�x
						future_degree	= get_target_degrere( target_x - g_now_x , target_y - g_now_y );
						//���ݍ��W����ڕW���W�܂ł̋���
						present_target_distance = get_present_target_dis( target_x - g_now_x , target_y - g_now_y );
						
						if( vertical_distance <= 10 ){
							theta += 5;
						}
					#endif //�������[�h�����`���~
					
				#elif OUTPUT_MODE == CALIBRATION
					motor_output_turn = pd_rock( g_degree , 0 );
					
				#endif//�蓮���������L�����u����
				
			}//start_switch
		
			if( g_start_switch == 0  || stop_flag >= 100 || ps_switch == 1){
				motor_output_l	= 0.00;
				motor_output_r	= 0.00;
				motor_output_b	= 0.00;
				motor_output_turn = 0.00;
				free_output();
			}
			
			if( stop_flag >= 100 ){
				g_start_switch	= 0;
				LED_P8				= 0;
				buzzer_cycle( 0.5 );
			}
			
			if( g_start_switch == 1 ){
				move_m( m_out );
				#if OUTPUT_MODE == CALIBRATION
					Move( motor_output_turn , motor_output_turn , motor_output_turn );
				#else
					Move( motor_output_l + motor_output_turn , motor_output_r + motor_output_turn , motor_output_b + motor_output_turn );
				#endif
			}else{
				Move( 0 , 0 , 0 );
			}
			
			#if SERIAL_MODE == ON
				if( g_time >= 0.1 ){
					g_time = 0;
					
					/*if( mileage_flag	== 0 ){
						sprintf(str,",%f ,\n\r", g_velocity );
					}else{
						sprintf(str,", mileage,%f ,\n\r", old_mileage );
					}*/
					
					//sprintf(str,"%d \n\r", task );
					//sprintf(str,"%d, %.4f, %.4f, %.4f,\n\r", task, g_now_x, g_now_y, g_degree );
					//sprintf(str,"%d, %.4f, %.4f, %.4f, %.4f, %.4f\n\r", task, g_now_x, g_now_y, g_degree, next_degree, future_degree );
					//sprintf(str,"%d, %f \n\r", g_start_switch , g_stop_time );
					//sprintf(str,"%.4f ,%.4f,%d\n\r", vertical_distance , horizontal_distance , task );
					//sprintf(str,"%d \n\r", stop_flag );
					//sprintf(str," ,%.4f, %.4f, %.4f, %.4f, \n\r", g_now_x , g_now_y , g_x_c , g_y_c );
					//sprintf(str,",%.4f ,%.4f \n\r", g_degree , g_Rate_f );
					//sprintf(str,",%f ,\n\r", g_velocity );
					//sprintf(str,"%d, %d, %d,\n\r", mtu1_count() , mtu2_count() ,mtu8_count() );
					//sprintf(str," %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, \n\r", LEFT_STICK_WIDE , LEFT_STICK_HIGH , RIGHT_STICK_WIDE , RIGHT_STICK_HIGH, UP_SW , RIGHT_SW, DOWN_SW , LEFT_SW , TRIANGLE_SW , CIRCLE_SW , CROSS_SW , SQUARE_SW	, START_SELECT_SW , PS_SW, ACC_X, ACC_Y,ACC_Z);
					//sprintf(str," %.4f , %.4f , %.4f , %.4f , %.4f , %d, %d ,\n\r", LEFT_STICK_WIDE , LEFT_STICK_HIGH , RIGHT_STICK_WIDE , ACC_X , ACC_Y , motor_output_l , stop_flag );
					//sprintf(str," %.4f , %.4f , %.4f %.4f, \n\r", LEFT_STICK_WIDE , LEFT_STICK_HIGH , RIGHT_STICK_WIDE , START_SELECT_SW);
					//sprintf(str,"%.4f %.4f, %.4f,\n\r", ENCF() , ENCL() ,ENCR() );
					//sprintf( str,"%.4f,%f,%d,%d,%d,\n\r",g_Rate_f,g_Angle_f,g_X_acc,g_Y_acc,g_Z_acc );
					//sprintf(str,"%.4f, \n\r",  motor_output_turn);
					//sprintf( str,"%.4f, %.4f, %.4f , %.4f, %.4f, %.4f, %.4f\n\r", motor_output_l, motor_output_r, motor_output_b, g_velocity , target_velocity, Motor_output_x , Motor_output_y );
					//transmit( str , 1);
					send.sci_data1 = m_out;
					//send.sci_data2 = ENCR();
					//send.sci_data3 = ENCF();
					//send.sci_data4 = g_degree;
					//send.sci_data1 = mtu8_count();
					//send.sci_data2 = START_SELECT_SW;
					//send.sci_data3 = mtu1_count();
					//send.sci_data4 = g_Rate_f;
					//send.sci_data5 = g_velocity;
					//sci_transformer(&send);
				}
			#endif
		}//INTERRUPT_TIME
	}//while(1)
}//main

void timer(void)
{
	IR(CMT0,CMI0) 		= 0;
	g_count_time 		+= 0.001;
	g_left_duty_time	+= 0.001;
	g_right_duty_time	+= 0.001;
	g_back_duty_time	+= 0.001;
	g_duty_time			+= 0.001;
	g_time					+= 0.001;
	g_stop_time			+= 0.001;
}

void over_1( void )
{	
	g_mtu1_over ++;
}

void under_1( void )
{
	g_mtu1_under ++;
}

void over_2( void )
{	
	g_mtu2_over ++;
}

void under_2( void )
{
	g_mtu2_under ++;
}

void over_8( void )
{	
	g_mtu8_over ++;
}

void under_8( void )
{
	g_mtu8_under ++;
}

float mtu1_count( void )
{
	return ( MTU1.TCNT + g_mtu1_over * 65536 - ( g_mtu1_under * 65536 ) );
}

float mtu2_count( void )
{
	return ( MTU2.TCNT + g_mtu2_over * 65536 - ( g_mtu2_under * 65536 ) );
}

float mtu8_count( void )
{
	return ( MTU8.TCNT + g_mtu8_over * 65536 - ( g_mtu8_under * 65536 ) );
}

/******************************************************************************
*	�^�C�g�� �F SPI�ʐM�Ŏ�M�������̂�Ԃ�
*	  �֐��� �F Rspi_send_1
*	  �߂�l �Funsigned lond�^
*	    ���� �F unsingned long moji
******************************************************************************/
unsigned long Rspi_send_1(unsigned long moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//��M�o�b�t�@�ɂȂɂ�����܂ő҂�
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	�^�C�g�� �F SPI�ʐM�Ŏ�M�������̂�Ԃ�
*	  �֐��� �F Rspi_send_1
*	  �߂�l �Funsigned lond�^
*	    ���� �F unsingned long moji
******************************************************************************/
unsigned long Rspi_send_short_1(unsigned short int moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//��M�o�b�t�@�ɂȂɂ�����܂ő҂�
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	�^�C�g�� �F �f���A���V���b�N����̑��M�f�[�^���i�[
*	  �֐��� �F Rspi_receive_send_line_dualshock
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
******************************************************************************/
void Rspi_recive_send_line_dualshock(void)	//DualShock�A�i���O�R���g���[��(�A�i���O���[�h��LED)�p���M�v���O����
{
	while( RSPI1.SPSR.BIT.SPRF == 1 ){				//��M�o�b�t�@���t���Ȃ烊�[�h���ăN���A����
		RSPI1.SPDR.LONG;
	}
	g_controller_receive_1st = Rspi_send_1(0x00004201);
	g_controller_receive_2nd = Rspi_send_1(0x00000000);
	g_controller_receive_3rd = Rspi_send_short_1(0x00);
}

/*void transmit( char str[ ] , int ch)
{
	int z = 0;
	
	if( ch == 0 ){
		while( str[ z ] != '\0' ){
			if( SCI0.SSR.BIT.TDRE == 1 ){														//TDRE���W�X�^����TDRE���W�X�^�Ƀf�[�^���]�����ꂽ�Ƃ�
				SCI0.TDR = str[ z ];
				z ++;
				SCI0.SSR.BIT.TDRE = 0;
			}
		}
	}else if( ch == 1 ){
		while( str[ z ] != '\0' ){
			if( SCI1.SSR.BIT.TDRE == 1 ){														//TDRE���W�X�^����TDRE���W�X�^�Ƀf�[�^���]�����ꂽ�Ƃ�
				SCI1.TDR = str[ z ];
				z ++;
				SCI1.SSR.BIT.TDRE = 0;
			}
		}
	}else if( ch == 2 ){
		while( str[ z ] != '\0' ){
			if( SCI2.SSR.BIT.TDRE == 1 ){														//TDRE���W�X�^����TDRE���W�X�^�Ƀf�[�^���]�����ꂽ�Ƃ�
				SCI2.TDR = str[ z ];
				z ++;
				SCI2.SSR.BIT.TDRE = 0;
			}
		}
	}
}
*/

void transmit( char str[ ])
{
	int z = 0;
	
	while( str[ z ] != '\0' ){
		if( SCI1.SSR.BIT.TDRE == 1 ){														//TDRE���W�X�^����TDRE���W�X�^�Ƀf�[�^���]�����ꂽ�Ƃ�
			SCI1.TDR = str[ z ];
			z ++;
			SCI1.SSR.BIT.TDRE = 0;
		}
	}
}

char Receive_uart_c(void)
{
	while (SCI2.SSR.BIT.RDRF == 0);		//RDRF = 0�FSCRDR �ɗL���Ȏ�M�f�[�^���i�[����Ă��Ȃ����Ƃ�\��
	SCI2.SSR.BIT.RDRF = 0;				//RDRF��ҋ@��ԂɕύX	
	return SCI2.RDR;
}

char Receive_uart_c_0(void)
{
	while (SCI0.SSR.BIT.RDRF == 0);		//RDRF = 0�FSCRDR �ɗL���Ȏ�M�f�[�^���i�[����Ă��Ȃ����Ƃ�\��
	SCI0.SSR.BIT.RDRF = 0;				//RDRF��ҋ@��ԂɕύX	
	return SCI0.RDR;
}

/******************************************************************************
*	�^�C�g�� �F �ݒ肵���͈͓��̒l��Ԃ�
*	  �֐��� �F Limit_ul
*	  �߂�l �F float�^ �o�͒l
*	   ����1 �F float�^ upper  ����̐��l
*	   ����2 �F float�^ lower  �����̐��l
*	   ����3 �F float�^ figure  ��r���鐔�l
*	  �쐬�� �F �s�� �q��
*	  �쐬�� �F 2011/08/31
******************************************************************************/
float Limit_ul(float upper,float lower,float figure)
{
	if(upper < figure){
		return(upper);
	}else if(figure < lower){
		return(lower);
	}else{
		return(figure);
	}
}

float revision_degree( float degree )
{
	while( degree > 180 ){
		degree	= degree - 360;
	}
	while( degree < 180 * ( - 1 ) ){
		degree	= degree + 360;
	}
	/*for( ; degree > 180; ){
		degree -= 360;
	}

	for(; degree < -180;){
		degree += 360;
	}*/
	return ( degree );
}

float pd_rock( float present , float target )					//pd ���i���̃��b�N
{
	float	output		= 0.00,
			deviation	= 0.00;											//deviation �΍�
			
	static float old_deviation = 0.00;
	
	deviation = revision_degree( target - present );
	
	output = ( ROCK_P_GAIN * deviation ) + ( ROCK_D_GAIN * ( deviation - old_deviation ) );

	output = Limit_ul( 100 , -100 , output );
	
	old_deviation = deviation;
	
	return output;
}

float pd_straight( float present , float target )											//pd ���i deviation �΍�
{							
	float 	output		= 0.00,
			deviation	= 0.00;																		//deviation �΍�
			
	static float old_deviation = 0.00;
	
	deviation = target - present;
	
	output = ( STRAIGHT_P_GAIN * deviation ) + ( STRAIGHT_D_GAIN * ( deviation - old_deviation ) );
	
	output = Limit_ul( 100 , -100 , output );
	
	old_deviation = deviation;
	
	return output;
}

void move_left_tire( float left_duty )
{		
	static int i = 0;
	static float old_duty = 0.00;
	float left_duty_sub = 0.00;
	
	 if( left_duty == FREE ){
		LEFT_TIRE_CW		= 0;
		LEFT_TIRE_CCW 	= 0;

	}else if( left_duty > 0 ){
		LEFT_TIRE_CW 		= 1;
		LEFT_TIRE_CCW 	= 0; 
	 	if( i == 1 ){
			 g_left_duty_time = 0;
		}
		i = 0;
		
	}else if( left_duty < 0 ){
		LEFT_TIRE_CW 		= 0;
		LEFT_TIRE_CCW 	= 1;
		left_duty *= ( -1 );
	 	if( i == 0 ){
			g_left_duty_time = 0;
		}
		i = 1;
	}
	
	if( g_left_duty_time <= 0.010 ){
		LEFT_TIRE_CW		= 0;
		LEFT_TIRE_CCW 	= 0;
	}
	if( left_duty <= 10 ){
		LEFT_TIRE_CW		= 0;
		LEFT_TIRE_CCW 	= 0;
	}
	
	left_duty_sub = fabs( left_duty - old_duty );
	
	if( left_duty_sub > 5 ){
		left_duty = (old_duty + left_duty) / 2;
	}
	
	old_duty = left_duty;
	
	left_duty = Limit_ul( MAX_DUTY , 0 , left_duty );
	LEFT_TIRE_DUTY = ( ( PWM_PERIOD * left_duty ) /100 );
}

void move_right_tire( float right_duty )
{		
	static int i = 0;
	static float old_duty = 0.00;
	float right_duty_sub = 0.00;
	
	 if( right_duty == FREE ){
		RIGHT_TIRE_CW	= 0;
		RIGHT_TIRE_CCW 	= 0;

	}else if( right_duty > 0 ){
		RIGHT_TIRE_CW 	= 1;
		RIGHT_TIRE_CCW 	= 0; 
	 	if( i == 1 ){
			g_right_duty_time = 0;
		}
		i = 0;	
		
	}else if( right_duty < 0 ){
		RIGHT_TIRE_CW 	= 0;
		RIGHT_TIRE_CCW 	= 1;
		right_duty *= ( -1 );
	 	if( i == 0 ){
			g_right_duty_time = 0;
		}
		i = 1;
	}
	
	if( g_right_duty_time <= 0.010 ){
		RIGHT_TIRE_CW	= 0;
		RIGHT_TIRE_CCW 	= 0;
	}
	if( right_duty <= 10 ){
		RIGHT_TIRE_CW	= 0;
		RIGHT_TIRE_CCW 	= 0;
	}
	
	right_duty_sub = fabs( right_duty - old_duty );
	
	if( right_duty_sub > 5 ){
		right_duty = (old_duty + right_duty ) /2;
	}
	
	old_duty = right_duty;
	
	right_duty = Limit_ul( MAX_DUTY , 0 , right_duty );
	RIGHT_TIRE_DUTY = ( ( PWM_PERIOD * right_duty ) /100 );
}

void move_back_tire( float back_duty )
{		
	static int i = 0;
	static float old_duty = 0.00;
	float back_duty_sub = 0.00;
	
	 if( back_duty == FREE ){
		BACK_TIRE_CW		= 0;
		BACK_TIRE_CCW 	= 0;

	}else if( back_duty > 0 ){
		BACK_TIRE_CW 	= 1;
		BACK_TIRE_CCW 	= 0; 
	 	if( i == 1 ){
			g_back_duty_time = 0;
		}
		i = 0;
		
	}else if( back_duty < 0 ){
		BACK_TIRE_CW 	= 0;
		BACK_TIRE_CCW 	= 1;
		back_duty *= ( -1 );
	 	if( i == 0 ){
			g_back_duty_time = 0;
		}
		i = 1;
	}
	
	if( g_back_duty_time <= 0.010 ){
		BACK_TIRE_CW		= 0;
		BACK_TIRE_CCW 	= 0;
	}
	if( back_duty <= 10 ){
		BACK_TIRE_CW		= 0;
		BACK_TIRE_CCW 	= 0;
	}
	
	back_duty_sub = fabs( back_duty - old_duty );
	
	if( back_duty_sub > 5 ){
		back_duty = ( old_duty + back_duty ) / 2;
	}
	
	old_duty = back_duty;
	
	back_duty = Limit_ul( MAX_DUTY , 0 , back_duty );
	BACK_TIRE_DUTY = ( ( PWM_PERIOD * back_duty ) /100 );
}

/******************************************************************************
*	�^�C�g�� �F ���I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_l
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_l(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_l = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
	}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_l = fabs(motor_output_x) * cos(D_TO_R(revision_degree(degree_now + (150.0 + degree_reverse_x)))) + fabs(motor_output_y) * sin(D_TO_R(revision_degree(degree_now + (150.0 + degree_reverse_y))));
	
	return(motor_output_l);
}

void move_m( float duty )
{		
	static int i = 0;
	static float old_duty = 0.00;
	float duty_sub = 0.00;
	
	 if( duty == FREE ){
		M_CW		= 0;
		M_CCW 	= 0;

	}else if( duty > 0 ){
		M_CW 	= 1;
		M_CCW = 0; 
	 	if( i == 1 ){
			g_duty_time = 0;
		}
		i = 0;
		
	}else if( duty < 0 ){
		M_CW 	= 0;
		M_CCW 	= 1;
		duty *= ( -1 );
	 	if( i == 0 ){
			g_duty_time = 0;
		}
		i = 1;
	}
	
	if( g_duty_time <= 0.010 ){
		M_CW		= 0;
		M_CCW 	= 0;
	}
	if( duty <= 10 ){
		M_CW		= 0;
		M_CCW 	= 0;
	}
	
	duty_sub = fabs( duty - old_duty );
	
	//if( duty_sub > 5 ){
		//duty = ( old_duty + duty ) / 2;
	//}
	
	old_duty = duty;
	
	duty = Limit_ul( MAX_DUTY , 0 , duty );
	M_DUTY = ( ( PWM_PERIOD * duty ) /100 );
}

/******************************************************************************
*	�^�C�g�� �F �E�I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_r
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_r(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_r = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
	}
	
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_r = fabs(motor_output_x) * cos(D_TO_R(revision_degree(degree_now +( 30.0 + degree_reverse_x)))) + fabs(motor_output_y) * sin(D_TO_R(revision_degree(degree_now + (30.0 + degree_reverse_y))));
	
	return(motor_output_r);
}

/******************************************************************************
*	�^�C�g�� �F ���I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_b
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_b(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_b = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
	}
	
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_b = fabs(motor_output_x) * cos(D_TO_R(revision_degree(degree_now  + (degree_reverse_x - 90.0)))) + fabs(motor_output_y) * sin(D_TO_R(revision_degree(degree_now +( degree_reverse_y - 90.0))));
	
	return(motor_output_b);
}

void Move( float left_duty , float right_duty , float back_duty )
{
	move_left_tire(left_duty);
	move_right_tire(right_duty);
	move_back_tire(back_duty);
}

/******************************************************************************
*	�^�C�g�� �FR1350N�p��M�֐�
*	  �֐��� �F input_R1350N
*	  �߂�l �F void�^ 
*	   ����1 �F void
*	  �쐬�� �F �L�{��
*	  �쐬�� �F 2014/01/29
******************************************************************************/
void input_R1350N(void)
{
	static int i = 0;
	static unsigned char receive_pac[15] = {0};
	static int read_start = OFF;
	static float 	start_Rate_f	= 0.00;
	static float old_rate_f		= 0.00;
	unsigned int angle;
	unsigned int rate;
	unsigned int x_acc;
	unsigned int y_acc;
	unsigned int z_acc;
	unsigned char check_sum;
	
	static int flag	= 0;
	char str[20] = {0};
	//unsigned char index;
	//unsigned int reserved;
	
	receive_pac[i] = Receive_uart_c_0();//�󂯎��

	//HEADER�l����
	if(receive_pac[0] == 0xAA){//�w�b�_�[�lAA
		read_start = ON;
	}else{
		read_start = OFF;
		i = 0;
		sprintf(str,"0xAA not found");
		transmit( str );
	}
	
	if(read_start == ON){
		i++;
		//0�`14�܂ł�1�Z�b�g�̕�����
		if(i >= 15){
			i = 0;
			read_start = OFF;

			//�p�P�b�g�̃w�b�_�[�����m�F����
			if(receive_pac[0] != 0xAA){
				sprintf(str, "Heading ERROR");
				transmit( str);
			}
			
			//�f�[�^��g�ݗ��Ă�
			//index = receive_pac[2];
			rate = (receive_pac[3] & 0xFF) | ((receive_pac[4] << 8) & 0xFF00);
			angle = (receive_pac[5] & 0xFF) | ((receive_pac[6] << 8) & 0XFF00);
			x_acc = (receive_pac[7] & 0xFF) | ((receive_pac[8] << 8) & 0xFF00);
			y_acc = (receive_pac[9] & 0xFF) | ((receive_pac[10] << 8) & 0XFF00);
			z_acc = (receive_pac[11] & 0xFF) | ((receive_pac[12] << 8) & 0xFF00);
			//reserved = receive_pac[13];
			
			//�`�F�b�N�T���̊m�F
			check_sum = receive_pac[2] + receive_pac[3] + receive_pac[4] + receive_pac[5]
							+ receive_pac[6] + receive_pac[7] + receive_pac[8] + receive_pac[9]
					     	+ receive_pac[10] + receive_pac[11] + receive_pac[12] + receive_pac[13];
			
			if(check_sum != receive_pac[14]){
				sprintf(str, "Check_Sum ERROR");
				transmit( str);
			}
			
			//�p�x�Ɗp���x�̒P�ʂ�ʏ�l�i���ɖ߂��f�[�^���L������
			g_Rate = rate / 100;
			g_Angle = angle / 100;
			g_Rate_f = rate / 100.0;
			g_Angle_f = angle / 100.0;
			g_X_acc = x_acc;
			g_Y_acc = y_acc;
			g_Z_acc = z_acc;
				
			if(g_Rate > 180){
				g_Rate = g_Rate - 655;
			}
			
			if(g_Angle > 180){
				g_Angle = g_Angle - 655;
			}
			
			if(g_Rate_f > 180){
				g_Rate_f = g_Rate_f - 655.35;
			}
			
			if(g_Angle_f > 180){
				g_Angle_f = g_Angle_f - 655.35;
			}
			
			#ifdef Debug1
				g_start_switch = 1;
			#endif
			
			if( g_start_switch == 1 ){
				switch( flag ){
					case 0:
						if( g_Rate_f != 0.00 ){
							flag = 1;
							start_Rate_f = revision_degree( g_Rate_f );
						}break;
					case 1:
						g_Rate_f = ( -1 )*revision_degree( (revision_degree(  g_Rate_f ) - start_Rate_f ) );		
						break;
				}
				old_rate_f = g_Rate_f;
				if( fabs(g_Rate_f - old_rate_f) >= 20 ){
					g_Rate_f = old_rate_f;
				}
			}
		}
	}
}

/******************************************************************************
*	�^�C�g�� �FR1350N�R�}���h�p�`�F�b�N�T���v�Z�֐�
*	  �֐��� �F checksum_r1350n
*	  �߂�l �F int�^ 
*	   ����1 �F char�^
*	  �쐬�� �F �L�{��
*	  �쐬�� �F 2014/01/29
******************************************************************************/
int checksum_r1350n(char *str)
{
	int i,p;
	
	p = 0;

	if(str[0] == '$'){
		for(i=1; i<35; i++){
			if(str[i] == '*'){
				i = 35;
			}else{
				p = p + str[i];
			}
		}
	}else{
		sprintf(str,"CMD_ERROR");
		transmit( str, 1 );
	}

	return (p&0xFF);
}

//�O���[�o���ϐ�����̐��l�f�[�^�̎�o��	�v�f�ԍ��ł̓ǂݏ������߂�ǂ������l�͎g���΁H
float pick_out_atoz_value(char character)
{    
    int box = 0;
    
    if( (character >= 'a') && (character <= 'z') ){
        box = (int)(character - 'a');
        return ( g_atoz_value[box] );
    }else if( (character >= 'A') && (character <= 'Z') ){
        box = (int)(character - 'A');
        return ( g_AtoZ_value[box] );
    }else{
        return ( 0 );
    }
}

//�O���[�o���ϐ��ւ̐��l�f�[�^�̏�������	�v�f�ԍ��ł̓ǂݏ������߂�ǂ������l�͎g���΁H
void throw_away_atoz_value(char character, float write)
{    
    int box = 0;
    
    if( (character >= 'a') && (character <= 'z') ){
        box = (int)(character - 'a');
        g_atoz_value[box] = write;
    }else if( (character >= 'A') && (character <= 'Z') ){
        box = (int)(character - 'A');
        g_AtoZ_value[box] = write;
    }
}

//�������float�^�ɕϊ�	���̊֐��̓��{�e�B�N�X�̃}�C�R���̎��ƂŔz��ꂽ��ł���
float change_float(char *str){
    float n = 0;
    int i = 0;
    while(str[i]!='\0')
    {
        if(str[i]<'0' || str[i]>'9') break;
        n=n*10+str[i]-'0';
        i++;
    }
	return(n);
}

//��͂������߂ɉ����Đ��l���O���[�o���ϐ��Ɋi�[����֐�	���̊֐��̑����݂�����
void receive_order_depot( int target_box , char *storage_str , int minus_flag , int after_point_count , int large_size_flag )
{
    float value = 0.000;
    
    value = change_float(storage_str);

    if(minus_flag == 1){
        value *= ( -1.000 );
    }
    
    value = value * pow( 0.100, after_point_count );

    if( (target_box >= 0) && (target_box <= 25) ){
        if(large_size_flag == 0){
            g_atoz_value[target_box] = value;
        }else{
            g_AtoZ_value[target_box] = value;
        }
    }
}

//�ꕶ�����Ƃɉ�͂���֐�	���[�̂���̗͍�
void receive_order_c(char character)
{
    static int target_box = 255;										//�i�[���߂̊J�n����(ASCII�R�[�h��a�`z,A�`Z)
    static char storage_str[RECEIVE_STR_COLUMN] = "";	//�����̊i�[�p�̕�����
    static int storage_num = 0;											//�����𕶎���̂ǂ��Ɋi�[���邩
    static int minus_flag = 0;											//�}�C�i�X�l���ۂ�
    static int point_flag = 0;							 					//�����_�ȉ����܂܂�Ă��邩�ۂ�
    static int after_point_count = 0;					 				//�����_�ȉ��ɂǂꂾ���������邩
    static int large_size_flag = 0;										//�啶���Ȃ̂��ۂ�
    int reset = 0;										 						//�����̃��Z�b�g�����邩�ۂ�
	const char end = END;								 				//�i�[���߂̏I������
    
    if(character == end){
            storage_str[storage_num] = '\0';
            receive_order_depot(target_box, storage_str, minus_flag, after_point_count, large_size_flag);
            reset = 1;
            target_box = 255;
            large_size_flag = 0;
    }else{
        if( (character >= '0') && (character <= '9') ){
            storage_str[storage_num] = character;
            storage_num++; 
            if( point_flag == 1 ){
                after_point_count++;
            }
        }else if( (character >= 'a') && (character <= 'z') ){
            reset = 1;
            target_box = (int)(character - 'a');
            large_size_flag = 0;
        }else if( (character >= 'A') && (character <= 'Z') ){
            reset = 1;
            target_box = (int)(character - 'A');
            large_size_flag = 1;
        }else if( character == '-' ){
            minus_flag = 1;
        }else if( character == '.' ){
            point_flag = 1;
        }
    }
    
    if( reset == 1 ){
       strcpy(storage_str,"");
       storage_num = 0;
       minus_flag = 0;
       point_flag = 0;
       after_point_count = 0;
    }   
}

//������P�ʂŉ�͂���֐�	�J��Ԃ�����
void receive_order_s(char *word)
{
    while(*word != '\0'){
        receive_order_c(*word);
        word++;
    }
}

//�ʐM���肩��̎�M���荞��	�����̓}�C�R���ɂ���ĈقȂ�
void receive_att( void )
{
    char c = '\0';
	int stert_switch = 0;
	
	LED_P80 = 1;

	if( ( int )START_SELECT_SW == 8  ){
		stert_switch = 1;
	}
	
	if( stert_switch == 0 ){
		buzzer_cycle( 0.3 );
	}else{
		buzzer_cycle( 1 );
	}
	
	c = Receive_uart_c();//��M�f�[�^
	
    receive_order_c( c );
}

int stop_duty( void )
{
	static float old_acc_x	= 0,
					old_acc_y	= 0,
					old_acc_z 	= 0;

	static int	flag	= 0;
				 
	if( old_acc_x == ACC_X && old_acc_y == ACC_Y && old_acc_z == ACC_Z ){
		flag ++;
	}else{
		flag = 0;
	}

	old_acc_x = ACC_X;
	old_acc_y = ACC_Y;
	old_acc_z = ACC_Z;

	return ( flag );
}

void buzzer_cycle( float time )
{
	static float count_time	= 0.00;
	static float off_time			= 0.00;
	static int ignore 				= 0;

	count_time += INTERRUPT_TIME;
	
	if( count_time <= time ){
		BUZZER		= 1;
	}else{
		ignore		= 1;
	}
	if( ignore == 1 ){
		off_time		+= INTERRUPT_TIME;
		BUZZER 	= 0;
	}
	if( off_time >= time ){
		count_time	= 0.00;
		off_time 		= 0.00;
		ignore			= 0;
	}
}

/******************************************************************************
*	�^�C�g�� �F ps3�R���̒l����o�͌���
*	  �֐��� �F straight_output_x
*	  �߂�l �F float�^ �o�͒l
*	    ���� �F �Ȃ�
*	  �쐬�� �F ���{�@�z��
*	  �쐬�� �F 2014/09/15
******************************************************************************/
float straight_output_x( void )
{
	float straight_cal_x = 0.00;
	
	straight_cal_x = ( 255.0 - (float)LEFT_STICK_HIGH ) / 255.0;	//���ɓ|�����Ƃ�1�A�E�ɓ|�����Ƃ�0
	straight_cal_x = ( straight_cal_x - 0.5 ) * 2.0;							//���ɓ|�����Ƃ�1�A�E�ɓ|���Ƃ�-1
	if( fabs( straight_cal_x ) <= STICK_NO_MOVE_RANGE ){			//�V�ѕ���
		straight_cal_x = 0.0;
	}
	straight_cal_x = PWM_PER * straight_cal_x;

	return( straight_cal_x );
}

/******************************************************************************
*	�^�C�g�� �F ps3�R���̒l����o�͌���
*	  �֐��� �F straight_output_y
*	  �߂�l �F float�^ �o�͒l
*	    ���� �F �Ȃ�
*	  �쐬�� �F ���{�@�z��
*	  �쐬�� �F 2014/09/15
******************************************************************************/
float straight_output_y( void )
{
	float straight_cal_y = 0.00;
	
	straight_cal_y = ( 255.0 - (float)LEFT_STICK_WIDE ) / 255.0;	//��ɓ|�����Ƃ�1�A���ɓ|�����Ƃ�0
	straight_cal_y = ( straight_cal_y - 0.5 ) * 2.0;							//��ɓ|�����Ƃ�1�A���ɓ|�����Ƃ�-1
	if( fabs( straight_cal_y ) <= STICK_NO_MOVE_RANGE ){			//�V�ѕ���
		straight_cal_y = 0.0;
	}
	straight_cal_y = PWM_PER * straight_cal_y;
	
	return( straight_cal_y );
}

/******************************************************************************
*	�^�C�g�� �F ps3�R���̒l����o�͌���
*	  �֐��� �F turn_output
*	  �߂�l �F float�^ �o�͒l
*	    ���� �F �Ȃ�
*	  �쐬�� �F ���{�@�z��
*	  �쐬�� �F 2014/09/15
******************************************************************************/
float turn_output( void )
{
	float turn_cal = 0.00;

	turn_cal = ( 255.0 - (float)RIGHT_STICK_WIDE ) / 255.0;	//���ɓ|�����Ƃ�1�A�E�ɓ|�����Ƃ�0
	turn_cal = ( turn_cal - 0.5 ) * 2.0;										//���ɓ|�����Ƃ�1�A�E�ɓ|���Ƃ�-1
	if( fabs(turn_cal) <= STICK_NO_MOVE_RANGE ){				//�V�ѕ���
		turn_cal = 0.0;
	}
	turn_cal = ( OPERATE_DEGREE / ( 1.000 / INTERRUPT_TIME ) ) * turn_cal;	//�X�e�B�b�Nmax��1000ms�łǂ񂾂��p�x���i�ނ�

	return( turn_cal );
}

void initialization( void )
{
	g_mtu1_over		= 0;
	g_mtu1_under	= 0;
	g_mtu2_over		= 0;
	g_mtu2_under	= 0;
	g_mtu8_over		= 0;
	g_mtu8_under	= 0;
	MTU1.TCNT	= 0;
	MTU2.TCNT	= 0;
	MTU8.TCNT	= 0;
	g_now_x	= 0.00;
	g_now_y	= 0.00;
	g_degree	= 0.00;
	g_Rate		= 0;
	g_Angle		= 0;
	g_X_acc 	= 0;
	g_Y_acc 	= 0;
	g_Z_acc 	= 0;
	g_Rate_f	= 0.00;
	g_Angle_f	= 0.00;
}

/******************************************************************************
*	�^�C�g�� �F ��(x^2)+(y^2)
*	  �֐��� �F present_target_dis
*	   ����1 �F float�^ deviation_x  �ڕW��x���W�ƌ��݂�x���W�̕΍�
*	   ����2 �F float�^ deviation_y  �ڕW��y���W�ƌ��݂�y���W�̕΍�
*	  �쐬�� �F ���{�z��
*	  �쐬�� �F 2014/09/19
******************************************************************************/
float get_present_target_dis( float deviation_x , float deviation_y )
{
	return ( sqrt( ( deviation_x ) * ( deviation_x ) + ( deviation_y) * ( deviation_y ) ) );
}

/******************************************************************************
*	�^�C�g�� �F �ڕW�ƌ��݂̍��W�̕΍��̊p�x
*	  �֐��� �F get_target_degree
*	   ����1 �F float�^ deviation_x  �ڕW��x���W�ƌ��݂�x���W�̕΍�
*	   ����2 �F float�^ deviation_y  �ڕW��y���W�ƌ��݂�y���W�̕΍�
*	  �쐬�� �F ���{�z��
*	  �쐬�� �F 2014/09/19
******************************************************************************/
float get_target_degree( float deviation_x , float deviation_y )
{	
	float target_degree = 0.00;
	static float old_target_degree = 0.00;
	
	if( deviation_x !=0 || deviation_y != 0 ){
		target_degree = atan2f( deviation_y , deviation_x) * ( 180 / M_PI );
	}else{
		target_degree = old_target_degree;
	}
	
	return ( target_degree );
}

float get_horizontal_distance( float future_degree , float degree , float distance )
{
	float	horizontal_distance	= 0.00;
	
	if( degree > 90 ){
     	degree	= 180 - degree;
     	horizontal_distance	= ( -1 ) * distance * sin( degree * ( M_PI / 180) );
  
	}else if( degree < -90 ){
		degree	= degree * ( -1 ) - 180;
   		horizontal_distance	= ( -1 ) * distance * sin( degree * (M_PI / 180 ) );
	
	}else{
		horizontal_distance	= distance * sin( degree * ( M_PI / 180 ) );
	}
	return (horizontal_distance );
}

/******************************************************************************
*	�^�C�g�� �F ���������Z�o
*	  �֐��� �F get_vertical_distance
*	   ����1 �F float�^ future_degree  ���ݍ��W����ڕW���W�܂ł̊p�x
*	   ����2 �F float�^ now_degree  ���݂̊p�x
*	   ����3 �F float�^ distance  ���W���狗��
*	  �쐬�� �F ���{�z��
*	  �쐬�� �F 2014/09/19
******************************************************************************/
float get_vertical_distance( float future_degree , float degree , float distance )
{
	float	vertical_distance	= 0.00;
	
	if( degree > 90 ){
     	degree	= 180 - degree;
     	vertical_distance	= ( -1 ) * distance * cos( degree * ( M_PI / 180) );
  
	}else if( degree < -90 ){
		degree	= degree * ( -1 ) - 180;
   		vertical_distance	= ( -1 ) * distance * cos( degree * (M_PI / 180 ) );
	
	}else{
		vertical_distance	= distance * cos( degree * ( M_PI / 180 ) );
	}
	return ( vertical_distance );
}

/******************************************************************************
*	�^�C�g�� �F ��`����
*	  �֐��� �F get_target_velocity
*	   ����1 �F float�^ distance_rest  2��̃��[�g
*	   ����2 �F float�^ vertical_distance  ��������
*	   ����3 �F float�^ a_up  �����x
*	   ����4 �F float�^ a_down  �����x
*	  �쐬�� �F ���{�z��
*	  �쐬�� �F 2014/09/19
******************************************************************************/
float get_target_velocity( float distance_rest , float vertical_distance , float a_up ,  float a_down )	//rest�]��
{	
	static float target_velocity	= 0.00;
	
	if( distance_rest > 0.5 * g_max_velocity * g_max_velocity / a_down ){	
		if( target_velocity < g_max_velocity ){
			target_velocity += ( a_up * INTERRUPT_TIME );
												
		}else{
			target_velocity = g_max_velocity;
		}
	}else if( vertical_distance < 0 ){
		target_velocity = ( -1 ) * sqrt( fabs( 2 * a_down * distance_rest ) );
	}else{
		target_velocity =  sqrt( fabs( 2 * a_down * distance_rest ) );
	}
	
	return ( target_velocity );
}

void free_output( void )
{
	LEFT_TIRE_CW		= 0;
	LEFT_TIRE_CCW	= 0;
	RIGHT_TIRE_CW	= 0;
	RIGHT_TIRE_CCW	= 0;
	BACK_TIRE_CW		= 0;
	BACK_TIRE_CCW	= 0;
}

/******************************************************************************
*	�^�C�g�� �F x�����o�͌���
*	  �֐��� �F get_motor_output_x
*	   ����1 �F float�^ straight  ���i�����̏o��
*	   ����2 �F float�^ target_degree  ���ݍ��W�ƖڕW���W�̊p�x
*	  �쐬�� �F ���{�z��
*	  �쐬�� �F 2014/09/22
******************************************************************************/
float get_motor_output_x( float straight , float target_degree )
{
	float	degree_reverse	= 0.00,
			Motor_output_x	= 0.00;

	if( straight < 0 ){
		degree_reverse = 180.0;
	}else{
		degree_reverse = 0.00;
	}
	
	Motor_output_x = fabs( straight ) * cos( D_TO_R( revision_degree( target_degree + degree_reverse ) ) );

	return( Motor_output_x );
}

/******************************************************************************
*	�^�C�g�� �F y�����o�͌���
*	  �֐��� �F get_motor_output_y
*	   ����1 �F float�^ straight  ���i�����̏o��
*	   ����2 �F float�^ target_degree  ���ݍ��W�ƖڕW���W�̊p�x
*	  �쐬�� �F ���{�z��
*	  �쐬�� �F 2014/09/22
******************************************************************************/
float get_motor_output_y( float straight , float target_degree )
{
	float	degree_reverse	= 0.00,
			Motor_output_y	= 0.00;

	if( straight < 0 ){
		degree_reverse = 180.0;
	}else{
		degree_reverse = 0.00;
	}
	
	Motor_output_y = fabs( straight ) * sin( D_TO_R( revision_degree( target_degree + degree_reverse ) ) );

	return( Motor_output_y );
}

void position_rock(float target_x, float target_y, float now_x, float now_y, float now_degree )
{
	float	output				= 0.00,
			output_x			= 0.00,
			output_y			= 0.00;
	float	p_gain				= 0.03,
			d_gain				= 0.30;
	float	now_gap			= 0.00;
	float target_degree		= 0.00;
	
	static float old_gap		= 0.00,
					old_output	= 0.00;
		
	now_gap = get_present_target_dis( target_x - now_x, target_y - now_y );
	
	output = ( p_gain * now_gap ) + ( d_gain * ( now_gap - old_gap ) );
	
	old_gap = now_gap;
	
	if( fabs( output - old_output ) > 1.0 ){
		output = ( output + old_output ) / 2;
	}
	
	old_output = output;
	
	target_degree = get_target_degree(target_x - now_x , target_y - now_y );
	
	output_x = output * cos( D_TO_R( revision_degree( target_degree ) ) );
	output_y = output * sin( D_TO_R( revision_degree( target_degree ) ) );
	
	g_motor_output_l	= get_motor_output_l( output_x, output_y, now_degree );
	g_motor_output_r	= get_motor_output_r( output_x, output_y, now_degree );
	g_motor_output_b	= get_motor_output_b( output_x, output_y, now_degree );
}

void get_robot_inf( void )
{
	float	enc_dis_f				= 0.00,
			enc_dis_r				= 0.00,
			enc_dis_l				= 0.00,
			enc_dis_subt_f		= 0.00,
			enc_dis_subt_r		= 0.00,
			enc_dis_subt_l		= 0.00,
			radian_f					= 0.00,
			radian_r					= 0.00,
			radian_l					= 0.00,
			degree_reverse_f	= 0.00,
			degree_reverse_r	= 0.00,
			degree_reverse_l 	= 0.00,
			velocity_x				= 0.00,
			velocity_y				= 0.00;
			
	static float	enc_x_f				= 0.00,
					enc_y_f				= 0.00,
					enc_x_r				= 0.00,
					enc_y_r				= 0.00,
					enc_x_l				= 0.00,
					enc_y_l				= 0.00,
					old_x					= 0.00,
					old_y					= 0.00,
					degree				= 0.00,
					old_degree		= 0.00,
					old_enc_dis_f		= 0.00,
					old_enc_dis_r	= 0.00,
					old_enc_dis_l		= 0.00,
					old_velocity		= 0.00;
					
	Sci_data send = {0.0};
					
	enc_dis_f = ( - 1 ) * ENC_DIAMETER_F * M_PI * ( ENCF() / PULSE );
	enc_dis_r = ( -1)*ENC_DIAMETER_R * M_PI * ( ENCR() / PULSE );
	enc_dis_l = ENC_DIAMETER_L * M_PI * ( ENCL() / PULSE );
	
	send.sci_data1 = enc_dis_f;
	send.sci_data2 = enc_dis_r;
	send.sci_data3 = enc_dis_l;
	
	enc_dis_subt_f = enc_dis_f - old_enc_dis_f;
	enc_dis_subt_r = enc_dis_r - old_enc_dis_r;
	enc_dis_subt_l = enc_dis_l - old_enc_dis_l;
	
	//if( fabs(enc_dis_subt_f) >=  100 || fabs( enc_dis_subt_r ) >=  100 || fabs(enc_dis_subt_l) >=  100 ){
		//g_start_switch = 0;
	//}
	
	radian_f = enc_dis_subt_f / CENTER_TO_ENC;
	radian_r = enc_dis_subt_r / CENTER_TO_ENC;
	radian_l = enc_dis_subt_l / CENTER_TO_ENC;

	degree += R_TO_D( ( radian_f + radian_r + radian_l ) / 3 );
	
	if( fabs( degree ) >= 360 * 10 ){
		g_start_switch = 0;
	}
	
	g_degree = revision_degree( degree );
	
	//send.sci_data1 = degree;
	//send.sci_data2 = enc_dis_f;
	//send.sci_data3 = enc_dis_r;
	//send.sci_data4 = enc_dis_l;
	//sci_transformer(&send);
	
	if( enc_dis_subt_f < 0 ){
		 degree_reverse_f = 180.0;
	}
	if( enc_dis_subt_r < 0 ){
		degree_reverse_r = 180.0;
	}
	if( enc_dis_subt_l < 0 ){
		degree_reverse_l = 180.0;
	}

	/*//�≺����
	//�O�p�`�̒��_�̍��W���Z�o
	g_vertex_b_x += (fabs(enc_dis_subt_r) * cos(D_TO_R( revision_degree( g_degree  + (degree_reverse_r - 30 )))) + fabs(enc_dis_subt_l) * cos(D_TO_R( revision_degree(g_degree + (degree_reverse_l - 150)))));
	g_vertex_b_y += (fabs(enc_dis_subt_r) * sin(D_TO_R( revision_degree(g_degree  + (degree_reverse_r - 30 )))) + fabs(enc_dis_subt_l) * sin(D_TO_R( revision_degree(g_degree  + (degree_reverse_l - 150)))));
	g_vertex_l_x += (fabs(enc_dis_subt_l) * cos(D_TO_R( revision_degree(g_degree - 150.0 + degree_reverse_l ))) + fabs(enc_dis_subt_f) * cos(D_TO_R( revision_degree(g_degree + 90.0 + degree_reverse_f))));
	g_vertex_l_y += (fabs(enc_dis_subt_l) * sin(D_TO_R( revision_degree(g_degree - 150.0 + degree_reverse_l ))) + fabs(enc_dis_subt_f) * sin(D_TO_R( revision_degree(g_degree + 90.0 + degree_reverse_f))));
	g_vertex_r_x += (fabs(enc_dis_subt_f) * cos(D_TO_R( revision_degree(g_degree + 90.0 + degree_reverse_f ))) + fabs(enc_dis_subt_r) * cos(D_TO_R( revision_degree( g_degree + (degree_reverse_r - 30)))));
	g_vertex_r_y += (fabs(enc_dis_subt_f) * sin(D_TO_R( revision_degree(g_degree + 90.0 + degree_reverse_f ))) + fabs(enc_dis_subt_r) * sin(D_TO_R( revision_degree(g_degree + (degree_reverse_r - 30)))));
	
	g_x_c = (g_vertex_b_x + g_vertex_l_x + g_vertex_r_x) / 3;
	g_y_c = (g_vertex_b_y + g_vertex_l_y + g_vertex_r_y) / 3;*/
	
	//����
	enc_x_f += 2 * fabs( enc_dis_subt_f ) * cos( D_TO_R( revision_degree( 90 + g_degree + degree_reverse_f ) ) );
	enc_y_f += 2 * fabs( enc_dis_subt_f ) * sin( D_TO_R( revision_degree( 90 + g_degree +degree_reverse_f ) ) );
	enc_x_r += 2 * fabs( enc_dis_subt_r ) * cos( D_TO_R( revision_degree( -30 + g_degree + degree_reverse_r ) ) );
	enc_y_r += 2 * fabs( enc_dis_subt_r ) * sin( D_TO_R( revision_degree(-30 + g_degree + degree_reverse_r ) ) );
	enc_x_l += 2 * fabs( enc_dis_subt_l ) * cos( D_TO_R( revision_degree( 30 + 180 + g_degree + degree_reverse_l ) ) );
	enc_y_l += 2 * fabs( enc_dis_subt_l ) * sin( D_TO_R( revision_degree( 30 + 180 + g_degree + degree_reverse_l ) ) );

	g_now_x = ( enc_x_f + enc_x_r + enc_x_l ) / 3;
	g_now_y = ( enc_y_f + enc_y_r + enc_y_l ) / 3;
	
	old_enc_dis_f = enc_dis_f;
	old_enc_dis_r = enc_dis_r;
	old_enc_dis_l = enc_dis_l;
	
	velocity_x	=( g_now_x - old_x ) / INTERRUPT_TIME;
	velocity_y	=( g_now_y - old_y ) / INTERRUPT_TIME;
	
	g_velocity	= sqrt( ( velocity_x * velocity_x ) + ( velocity_y * velocity_y ) );
	
	g_angular_velocity = ( revision_degree(g_degree - old_degree) ) / INTERRUPT_TIME;
	
	#ifndef Debug1
		if( g_start_switch == 0 ){
			degree = 0;
			enc_x_f = 0;
			enc_y_f = 0;
			enc_x_r = 0;
			enc_y_r = 0;
			enc_x_l = 0;
			enc_y_l = 0;
		}
	#endif
	
	old_x = g_now_x;
	old_y = g_now_y;
	old_degree = g_degree; 
}

/******************************************************************************
*	�^�C�g�� �F Excel�����̂��߂̃f�[�^���V���A�����M
*	  �֐��� �F sci_transformer
*	  �߂�l �F void�^ 
*	    ���� �F �Ȃ�
*	  �쐬�� �F �����N�G
*	  �쐬�� �F 2013/02/25
******************************************************************************/
void sci_transformer(Sci_data	*send)
{	
	#if MODE_SCIDATA_BOX != OFF
		char 	sc1[50],sc2[50],sc3[50],sc4[50],sc5[50],sc6[50],sc7[50],sc8[50];
	#endif
	
	#if MODE_SCIDATA_BOX >= 1
		sprintf(sc1,"%f",(float)send->sci_data1);
		transmit(",");
		transmit(sc1);
	#endif
	#if MODE_SCIDATA_BOX >= 2
		sprintf(sc2,"%f",(float)send->sci_data2);
		transmit(",");
		transmit(sc2);
	#endif
	#if MODE_SCIDATA_BOX >= 3
		sprintf(sc3,"%f",(float)send->sci_data3);
		transmit(",");
		transmit(sc3);
	#endif
	#if MODE_SCIDATA_BOX >= 4
		sprintf(sc4,"%f",(float)send->sci_data4);
		transmit(",");
		transmit(sc4);
	#endif
	#if MODE_SCIDATA_BOX >= 5
		sprintf(sc5,"%f",(float)send->sci_data5);
		transmit(",");
		transmit(sc5);
	#endif
	#if MODE_SCIDATA_BOX >= 6
		sprintf(sc6,"%5d",(long)send->sci_data6);
		transmit(",");
		transmit(sc6);		
	#endif
	#if MODE_SCIDATA_BOX >= 7
		sprintf(sc7,"%5d",(long)send->sci_data7);
		transmit(",");
		transmit(sc7);
	#endif
	#if MODE_SCIDATA_BOX >= 8
		sprintf(sc8,"%5d",(long)send->sci_data8);
		transmit(",");
		transmit(sc8);
	#endif
	
	transmit("\n\r");
}


#ifdef __cplusplus
void abort(void)
{

}
#endif
