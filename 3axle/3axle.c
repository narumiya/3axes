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

#define P//S2
#define PS3

#define ON							1
#define OFF							0

#define PWM_PERIOD			((48000000/1)/100000)

/*LED*/
#define LED_P8						PORT8.DR.BYTE
#define LED_P80					PORT8.DR.BIT.B0
#define LED_P81					PORT8.DR.BIT.B1

/*割り込み時間*/
#define INTERRUPT_TIME 		0.005

/*足回り*/
#define LEFT_TIRE_CW			PORT7.DR.BIT.B4
#define LEFT_TIRE_CCW		PORT7.DR.BIT.B6
#define LEFT_TIRE_DUTY		MTU4.TGRB
#define RIGHT_TIRE_CW		PORT7.DR.BIT.B0
#define RIGHT_TIRE_CCW		PORT7.DR.BIT.B2
#define RIGHT_TIRE_DUTY	MTU6.TGRB
#define BACK_TIRE_CW			PORT7.DR.BIT.B1
#define BACK_TIRE_CCW		PORT7.DR.BIT.B3
#define BACK_TIRE_DUTY		MTU6.TGRD

/*弧度法、度数法　変換*/
#define M_PI							3.141592653
#define R_TO_D(x) 				( x * ( 180 / M_PI ) )
#define D_TO_R(x)				( x * ( M_PI / 180 ) )

/*足回りが動かないスティックの値の範囲*/
#define STICK_NO_MOVE_RANGE 			0.2		//±20%

/*車体操作*/
#define PWM_PER					60
#define OPERATE_DEGREE		90	//1000ms Maxでステックを倒したときどれだけ回転するか（度）

/*モーター出力*/
#define	FREE							4536478

/*エンコーダカウント*/
#define ENCF()						mtu8_count()
#define ENCL()						mtu2_count()
#define ENCR()						mtu1_count()

/*中心からエンコーダまでの距離*/
#define CENTER_TO_ENC		237

#define PULSE						500*4

/*エンコーダタイヤ直径*/
#define ENC_DIAMETER_F 		51
#define ENC_DIAMETER_L		51
#define ENC_DIAMETER_R		51

/*PD制御*/
#define ROCK_P_GAIN			1//10.0
#define ROCK_D_GAIN			1//45.0

#define BUZZER						PORT2.DR.BIT.B2

#define END 							'#'									//通信データの終端文字
#define RECEIVE_STR_COLUMN 32		//1データあたりの最大文字数		例: a123# (6文字)

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

//グローバル変数に格納する場合	おばかな例
float	g_atoz_value[26]	=	{	0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
											0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
											0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
								
float	g_AtoZ_value[26]	=	{	0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
											0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
											0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

/*1msカウント*/
float	g_count_time			= 0.00,
		g_left_duty_time	= 0.00,
		g_right_duty_time	= 0.00,
		g_back_duty_time	= 0.00,
		g_time					= 0.00;

/*overfloaw underfloaw カウント*/
int		g_mtu1_over		= 0, 
		g_mtu1_under	= 0,
		g_mtu2_over		= 0,
		g_mtu2_under	= 0,
		g_mtu8_over		= 0,
		g_mtu8_under	= 0;
		
/*R1350N*/
int		g_Rate		= 0,
		g_Angle		= 0,
		g_X_acc 	= 0,
		g_Y_acc 	= 0,
		g_Z_acc 	= 0;

float	g_Rate_f			= 0.00,
		g_Angle_f			= 0.00;
		
/*自己位置*/
float	g_vertex_b_x	= 0.0,	g_vertex_b_y	= 0.0,
		g_vertex_l_x	= 0.0,	g_vertex_l_y	= 0.0,
		g_vertex_r_x	= 0.0,	g_vertex_r_y	= 0.0,
		g_x_c			= 0.0,	g_y_c			= 0.0;
		
float	g_now_x	= 0.00,
		g_now_y	= 0.00,
		g_degree	= 0.00,
		g_velocity	= 0.00,
		g_imu_x	= 0.00,
		g_imu_y	= 0.00;

/*ps2コンデータ*/		
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
	
volatile unsigned long	g_controller_receive_1st		= 0;	//コントローラから帰ってくるデータの格納フォルダ1stは不定でよい。
volatile unsigned long	g_controller_receive_2nd	= 0;
volatile unsigned long	g_controller_receive_3rd		= 0;

int 	getdate1 = 0,
		getdate2 = 0,
		getdate3 = 0;

/*プロトタイプ宣言*/
void wait( void );
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
void transmit( char str[ ] );
void transmit_2( char str[ ]);
char Receive_uart_c( void );
float Limit_ul(float upper,float lower,float figure);
void input_R1350N(void);
int checksum_r1350n(char *str);
float gap_degree( float degree );
float pd_rock( float present , float target );
void move_left_tire( float left_duty );
void move_right_tire( float right_duty );
void move_back_tire( float back_duty );
float get_motor_output_l(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_r(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_b(float motor_output_x,float motor_output_y,float degree_now);
void Move( float left_duty , float right_duty , float back_duty );
float pick_out_atoz_value(char character);
void throw_away_atoz_value(char character, float write);
float change_float(char *str);
void receive_order_depot(int target_box, char *storage_str, int minus_flag, int after_point_count, int large_size_flag);
void receive_order_c(char character);
void receive_order_s(char *word);
int stop_duty( void );
void buzzer_cycle( float time );
int buzzer_stop( time );
float straight_output_x( void );
float straight_output_y( void );
float turn_output( void );

void initial_config( void )
{
	init_clock();
	init_CMT0();
	init_pwm();
	init_SCI0();
	init_SCI1();
	init_SCI2();
	init_Rspi_dualshock();
	init_encorder();
}

void io_config( void )
{
	//LED
	PORT8.DDR.BYTE = 3;
	
	/*ブザー*/
	PORT2.DDR.BIT.B2 = 1;
}

void calculate( void )
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
					old_enc_dis_f		= 0.00,
					old_enc_dis_r	= 0.00,
					old_enc_dis_l		= 0.00;
					
	enc_dis_f = ( -1 ) * ENC_DIAMETER_F * M_PI * ( ENCF() / PULSE );
	enc_dis_r = ENC_DIAMETER_R * M_PI * ( ENCR() / PULSE );
	enc_dis_l = ENC_DIAMETER_L * M_PI * ( ENCL() / PULSE );
	
	enc_dis_subt_f = enc_dis_f - old_enc_dis_f;
	enc_dis_subt_r = enc_dis_r - old_enc_dis_r;
	enc_dis_subt_l = enc_dis_l - old_enc_dis_l;
	
	radian_f = enc_dis_subt_f / CENTER_TO_ENC;
	radian_r = enc_dis_subt_r / CENTER_TO_ENC;
	radian_l = enc_dis_subt_l / CENTER_TO_ENC;

	g_degree += R_TO_D( ( radian_f + radian_r + radian_l ) / 3 );
	g_degree = gap_degree( g_degree );
	
	if( enc_dis_subt_f < 0 ){
		 degree_reverse_f = 180.0;
	}
	if( enc_dis_subt_r < 0 ){
		degree_reverse_r = 180.0;
	}
	if( enc_dis_subt_l < 0 ){
		degree_reverse_l = 180.0;
	}

	//坂下さん
	//三角形の頂点の座標を算出
	g_vertex_b_x += (fabs(enc_dis_subt_r) * cos(D_TO_R( gap_degree( g_degree  + (degree_reverse_r - 30 )))) + fabs(enc_dis_subt_l) * cos(D_TO_R( gap_degree(g_degree + (degree_reverse_l - 150)))));
	g_vertex_b_y += (fabs(enc_dis_subt_r) * sin(D_TO_R( gap_degree(g_degree  + (degree_reverse_r - 30 )))) + fabs(enc_dis_subt_l) * sin(D_TO_R( gap_degree(g_degree  + (degree_reverse_l - 150)))));
	g_vertex_l_x += (fabs(enc_dis_subt_l) * cos(D_TO_R( gap_degree(g_degree - 150.0 + degree_reverse_l ))) + fabs(enc_dis_subt_f) * cos(D_TO_R( gap_degree(g_degree + 90.0 + degree_reverse_f))));
	g_vertex_l_y += (fabs(enc_dis_subt_l) * sin(D_TO_R( gap_degree(g_degree - 150.0 + degree_reverse_l ))) + fabs(enc_dis_subt_f) * sin(D_TO_R( gap_degree(g_degree + 90.0 + degree_reverse_f))));
	g_vertex_r_x += (fabs(enc_dis_subt_f) * cos(D_TO_R( gap_degree(g_degree + 90.0 + degree_reverse_f ))) + fabs(enc_dis_subt_r) * cos(D_TO_R( gap_degree( g_degree + (degree_reverse_r - 30)))));
	g_vertex_r_y += (fabs(enc_dis_subt_f) * sin(D_TO_R( gap_degree(g_degree + 90.0 + degree_reverse_f ))) + fabs(enc_dis_subt_r) * sin(D_TO_R( gap_degree(g_degree + (degree_reverse_r - 30)))));
	
	g_x_c = (g_vertex_b_x + g_vertex_l_x + g_vertex_r_x) / 3;
	g_y_c = (g_vertex_b_y + g_vertex_l_y + g_vertex_r_y) / 3;
	
	//自分
	enc_x_f += 2 * fabs( enc_dis_subt_f ) * cos( D_TO_R( gap_degree( 90 + g_degree + degree_reverse_f ) ) );
	enc_y_f += 2 * fabs( enc_dis_subt_f ) * sin( D_TO_R( gap_degree( 90 + g_degree +degree_reverse_f ) ) );
	enc_x_r += 2 * fabs( enc_dis_subt_r ) * cos( D_TO_R( gap_degree( -30 + g_degree + degree_reverse_r ) ) );
	enc_y_r += 2 * fabs( enc_dis_subt_r ) * sin( D_TO_R( gap_degree(-30 + g_degree + degree_reverse_r ) ) );
	enc_x_l += 2 * fabs( enc_dis_subt_l ) * cos( D_TO_R( gap_degree( 30 + 180 + g_degree + degree_reverse_l ) ) );
	enc_y_l += 2 * fabs( enc_dis_subt_l ) * sin( D_TO_R( gap_degree( 30 + 180 + g_degree + degree_reverse_l ) ) );

	g_now_x = ( enc_x_f + enc_x_r + enc_x_l ) / 3;
	g_now_y = ( enc_y_f + enc_y_r + enc_y_l ) / 3;

	/*g_now_x += (    (fabs(enc_dis_subt_l) * cos(D_TO_R(g_degree -150 + degree_reverse_l)))
						  + (fabs(enc_dis_subt_r) * cos(D_TO_R(g_degree -30 + degree_reverse_r)))
						  + (fabs(enc_dis_subt_f) * cos(D_TO_R(g_degree + 90 + degree_reverse_f)))  );
						  
	g_now_y += (    (fabs(enc_dis_subt_l) * sin(D_TO_R(g_degree -150 + degree_reverse_l)))
						  + (fabs(enc_dis_subt_r) * sin(D_TO_R(g_degree -30 + degree_reverse_r)))
						  + (fabs(enc_dis_subt_f) * sin(D_TO_R(g_degree + 90 + degree_reverse_f)))  );	*/
	
	old_enc_dis_f = enc_dis_f;
	old_enc_dis_r = enc_dis_r;
	old_enc_dis_l = enc_dis_l;
	
	velocity_x	=( g_now_x - old_x ) / INTERRUPT_TIME;
	velocity_y	=( g_now_y - old_y ) / INTERRUPT_TIME;
	g_velocity	= sqrt( ( velocity_x * velocity_x ) + ( velocity_y * velocity_y ) );
	
	old_x = g_now_x;
	old_y = g_now_y;
}

void imu_calculate( void )
{
	float	velocity			= 0.00,
			velocity_sub	= 0.00,
			displacement	= 0.00;
	static float	old_velocity	= 0.00;
	
	velocity_sub = velocity - old_velocity;
	velocity = velocity_sub + (float)(g_Y_acc) * INTERRUPT_TIME;
	
	displacement = velocity_sub * INTERRUPT_TIME + 0.5 * (float)(g_Y_acc) * INTERRUPT_TIME * INTERRUPT_TIME;
	
	g_imu_x += displacement * cos( g_Rate_f );
	g_imu_y += displacement * sin( g_Rate_f );
	
	old_velocity = velocity;
}

void main(void)
{
	char	str[30] = { 0 };
	
	float	Motor_output_x = 0.00,		//出力方向
			Motor_output_y = 0.00;
	
	float	motor_output_l		= 0.00,
			motor_output_r		= 0.00,
			motor_output_b		= 0.00,
			motor_output_turn	= 0.00;
			
	float	target_degree	= 0.00;
	
	int		start_switch	= 0,
			stop_flag		= 0;
			
	#ifdef PS2
		float	nutral_x = 127,					//スティック中心の値
				nutral_y = 127;
		float	pwm_percent			= 60,
				pwm_percent_turn = 40;
		union psdate1 getdate1;
		union psdate2 getdate2;
		union psdate3 getdate3;
	#endif
	
	initial_config();
	io_config();
	
	while( 1 ){	
		if( g_count_time >= INTERRUPT_TIME ){
			g_count_time = 0;
			
			calculate(); //自己位置計算
			
			#ifdef PS2
				//ps2コンデータ取得
				Rspi_recive_send_line_dualshock();
				getdate1.dword = g_controller_receive_1st;
				getdate2.dword = g_controller_receive_2nd;
				getdate3.dword = g_controller_receive_3rd;

				if( getdate1.byte.start_sw == 0 ){
					start_switch = 1;
				}
					
				//スティックによるｘ、ｙ方向の出力決め
				//x方向
				if( getdate3.byte.left_stick_high >=  0xBF){
					Motor_output_x = ( -1 ) * fabs( ( (float)getdate3.byte.left_stick_high - ( nutral_x + STICK_NO_MOVE_RANGE ) ) * ( pwm_percent / ( nutral_x - STICK_NO_MOVE_RANGE + 1) ));
				}else if( getdate3.byte.left_stick_high <=  0x3F){
					Motor_output_x = (( nutral_x - STICK_NO_MOVE_RANGE ) - (float)getdate3.byte.left_stick_high ) * ( pwm_percent / ( nutral_x - STICK_NO_MOVE_RANGE ) );	
				}else{
					Motor_output_x = 0.0;
				}
				//y方向
				if( getdate2.byte.left_stick_wide >= 0xBF ){
					Motor_output_y = ( -1 ) * fabs( ( (float)getdate2.byte.left_stick_wide - ( nutral_y + STICK_NO_MOVE_RANGE ) ) * ( pwm_percent / ( nutral_y - STICK_NO_MOVE_RANGE + 1) ) );
				}else if( getdate2.byte.left_stick_wide <= 0x3F ){
					Motor_output_y = ( ( nutral_y - STICK_NO_MOVE_RANGE ) - (float)getdate2.byte.left_stick_wide ) * ( pwm_percent / ( nutral_y - STICK_NO_MOVE_RANGE ) );	
				}else{
					Motor_output_y = 0.0;
				}
				
				//車体角度操作
				if((getdate2.byte.right_stick_wide  > 0x3F) && (getdate2.byte.right_stick_wide < 0xBF)){
					motor_output_turn = pd_rock(  g_degree , target_degree );
					//motor_output_turn = 0;
				}else{
					target_degree = g_degree;
					if(getdate2.byte.right_stick_wide >= 0xBF ){
						motor_output_turn = (-1) * ( (float)getdate2.byte.right_stick_wide - ( nutral_y + STICK_NO_MOVE_RANGE ) ) * ( (pwm_percent_turn) / ( nutral_y - STICK_NO_MOVE_RANGE + 1) );
					}else if(getdate2.byte.right_stick_wide <= 0x3F ){
						motor_output_turn = ( ( nutral_y - STICK_NO_MOVE_RANGE ) - (float)getdate2.byte.right_stick_wide ) * ( (pwm_percent_turn) / ( nutral_y - STICK_NO_MOVE_RANGE ) );	
					}
				}
				
				motor_output_l	= get_motor_output_l( Motor_output_x, Motor_output_y, 0.0 ) + motor_output_turn;
				motor_output_r	= get_motor_output_r( Motor_output_x, Motor_output_y, 0.0 ) + motor_output_turn;
				motor_output_b	= get_motor_output_b( Motor_output_x, Motor_output_y, 0.0 ) + motor_output_turn;
				
				if( getdate2.byte.cross_sw == 0 ){
					motor_output_l	= 0;
					motor_output_r	= 0;
					motor_output_b	= 0;
				}
				
				if( start_switch == 0 || getdate1.byte.model_number == 0x41 || getdate1.byte.model_number == 0xff ){
					LEFT_TIRE_CW		= 0;
					LEFT_TIRE_CCW	= 0;
					RIGHT_TIRE_CW	= 0;
					RIGHT_TIRE_CCW	= 0;
					BACK_TIRE_CW		= 0;
					BACK_TIRE_CCW	= 0;
					motor_output_l		= 0;
					motor_output_r 		= 0;
					motor_output_b 	= 0;
				}
				
				if( start_switch == 1 && getdate1.byte.model_number == 0x73 ){
					Move( motor_output_l, motor_output_r ,motor_output_b );
				}
			#endif
			
			#ifdef PS3
				stop_flag = stop_duty();
				
				if( ( int )START_SELECT_SW == 8 ){
					LED_P81 = 1;
					start_switch = 1;
				}
				
				if( start_switch == 1 ){
					//x方向
					Motor_output_x = straight_output_x();
					//y方向
					Motor_output_y = straight_output_y();
					//車体角度操作
					target_degree += turn_output();

					motor_output_turn = pd_rock( g_degree , gap_degree( target_degree ) );

					motor_output_l	= get_motor_output_l( Motor_output_x, Motor_output_y, g_degree ) + motor_output_turn;
					motor_output_r	= get_motor_output_r( Motor_output_x, Motor_output_y, g_degree ) + motor_output_turn;
					motor_output_b	= get_motor_output_b( Motor_output_x, Motor_output_y, g_degree ) + motor_output_turn;
				
					if( CROSS_SW >= 2 || stop_flag >= 100 ){
						motor_output_l	= 0;
						motor_output_r	= 0;
						motor_output_b	= 0;
					}
					
					if( start_switch == 0  || stop_flag >= 100 || CROSS_SW >= 2 ){
						LEFT_TIRE_CW		= 0;
						LEFT_TIRE_CCW	= 0;
						RIGHT_TIRE_CW	= 0;
						RIGHT_TIRE_CCW	= 0;
						BACK_TIRE_CW		= 0;
						BACK_TIRE_CCW	= 0;
					}
					
					if( stop_flag >= 100 ){
						start_switch	= 0;
						LED_P8			= 0;
						buzzer_cycle( 0.5 );
					}
					
					Move( motor_output_l , motor_output_r , motor_output_b );
				}
			#endif
			
			if( g_time >= 0.1 ){
				g_time = 0;
				//sprintf(str,"%x\n\r", getdate1.byte.model_number);
				//sprintf(str,"%.4f,%.4f,%.4f,%.4f,%.4f\n\r", g_now_x , g_now_y , g_x_c ,g_y_c , g_degree );
				//sprintf(str,"%.4f ,%.4f \n\r", g_degree , g_Rate_f );
				//sprintf(str,"%.4f ,\n\r", target_degree );
				//sprintf(str,"%d %d, %d,\n\r", mtu1_count() , mtu2_count() ,mtu8_count() );
				//sprintf(str," %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, \n\r", LEFT_STICK_WIDE , LEFT_STICK_HIGH , RIGHT_STICK_WIDE , RIGHT_STICK_HIGH, UP_SW , RIGHT_SW, DOWN_SW , LEFT_SW , TRIANGLE_SW , CIRCLE_SW , CROSS_SW , SQUARE_SW	, START_SELECT_SW , PS_SW, ACC_X, ACC_Y,ACC_Z);
				//sprintf(str," %.4f , %.4f , %.4f , %.4f , %.4f , %d, %d ,\n\r", LEFT_STICK_WIDE , LEFT_STICK_HIGH , RIGHT_STICK_WIDE , ACC_X , ACC_Y , motor_output_l , stop_flag );
				//sprintf(str," %.4f , %.4f , %.4f %.4f, \n\r", LEFT_STICK_WIDE , LEFT_STICK_HIGH , RIGHT_STICK_WIDE , START_SELECT_SW);
				//sprintf(str,"%.4f %.4f, %.4f,\n\r", ENCF() , ENCL() ,ENCR() );
				//sprintf( str,"%.4f,%f,%d,%d,%d,\n\r",g_Rate_f,g_Angle_f,g_X_acc,g_Y_acc,g_Z_acc );
				//sprintf(str,"%.4f, \n\r",  motor_output_turn);
				sprintf( str,"%.4f, %.4f , %.4f, %.4f, %.4f\n\r", motor_output_l, motor_output_r, motor_output_b , g_degree , target_degree);
				transmit( str );
			}
		}
	}
}

void timer(void)
{
	IR(CMT0,CMI0) 		= 0;
	g_count_time 		+= 0.001;
	g_left_duty_time	+= 0.001;
	g_right_duty_time	+= 0.001;
	g_back_duty_time	+= 0.001;
	g_time					+= 0.001;
}

void over_1( void )
{	
	//MTU1.TSR.BIT.TCFV = 0;
	g_mtu1_over ++;
}

void under_1( void )
{
	//MTU1.TSR.BIT.TCFU = 0;
	g_mtu1_under ++;
}

void over_2( void )
{	
	//MTU2.TSR.BIT.TCFV = 0;
	g_mtu2_over ++;
}

void under_2( void )
{
	//MTU2.TSR.BIT.TCFU = 0;
	g_mtu2_under ++;
}

void over_8( void )
{	
	//MTU8.TSR.BIT.TCFV = 0;
	g_mtu8_over ++;
}

void under_8( void )
{
	//MTU8.TSR.BIT.TCFU = 0;
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
*	タイトル ： SPI通信で受信したものを返す
*	  関数名 ： Rspi_send_1
*	  戻り値 ：unsigned lond型
*	    引数 ： unsingned long moji
******************************************************************************/
unsigned long Rspi_send_1(unsigned long moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//受信バッファになにか来るまで待つ
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	タイトル ： SPI通信で受信したものを返す
*	  関数名 ： Rspi_send_1
*	  戻り値 ：unsigned lond型
*	    引数 ： unsingned long moji
******************************************************************************/
unsigned long Rspi_send_short_1(unsigned short int moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//受信バッファになにか来るまで待つ
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	タイトル ： デュアルショックからの送信データを格納
*	  関数名 ： Rspi_receive_send_line_dualshock
*	  戻り値 ： void型
*	    引数 ： なし
******************************************************************************/
void Rspi_recive_send_line_dualshock(void)	//DualShockアナログコントローラ(アナログモード緑LED)用送信プログラム
{
	while( RSPI1.SPSR.BIT.SPRF == 1 ){				//受信バッファがフルならリードしてクリアする
		RSPI1.SPDR.LONG;
	}	
	g_controller_receive_1st = Rspi_send_1(0x00004201);
	g_controller_receive_2nd = Rspi_send_1(0x00000000);
	g_controller_receive_3rd = Rspi_send_short_1(0x00);
}

void transmit( char str[ ])
{
	int z = 0;

	while( str[ z ] != '\0' ){
		if( SCI1.SSR.BIT.TDRE == 1 ){														//TDREレジスタからTDREレジスタにデータが転送されたとき
			SCI1.TDR = str[ z ];
			z ++;
			SCI1.SSR.BIT.TDRE = 0;
		}
	}
}

void transmit_2( char str[ ])
{
	int z = 0;

	while( str[ z ] != '\0' ){
		if( SCI2.SSR.BIT.TDRE == 1 ){														//TDREレジスタからTDREレジスタにデータが転送されたとき
			SCI2.TDR = str[ z ];
			z ++;
			SCI2.SSR.BIT.TDRE = 0;
		}
	}
}

char Receive_uart_c(void)
{
	while (SCI2.SSR.BIT.RDRF == 0);		//RDRF = 0：SCRDR に有効な受信データが格納されていないことを表示
	SCI2.SSR.BIT.RDRF = 0;				//RDRFを待機状態に変更	
	return SCI2.RDR;
}

char Receive_uart_c_0(void)
{
	while (SCI0.SSR.BIT.RDRF == 0);		//RDRF = 0：SCRDR に有効な受信データが格納されていないことを表示
	SCI0.SSR.BIT.RDRF = 0;				//RDRFを待機状態に変更	
	return SCI0.RDR;
}

/******************************************************************************
*	タイトル ： 設定した範囲内の値を返す
*	  関数名 ： Limit_ul
*	  戻り値 ： float型 出力値
*	   引数1 ： float型 upper  上限の数値
*	   引数2 ： float型 lower  下限の数値
*	   引数3 ： float型 figure  比較する数値
*	  作成者 ： 市川 智章
*	  作成日 ： 2011/08/31
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

float gap_degree( float degree )
{
	while( degree > 180 ){
		degree	= degree - 360;
	}
	while( degree < 180 * ( - 1 ) ){
		degree	= degree + 360;
	}
	return degree;
}

float pd_rock( float present , float target )					//pd 直進中のロック
{
	float	output		= 0.00,
			deviation	= 0.00;											//deviation 偏差
			
	static float old_deviation = 0.00;
	
	deviation = gap_degree( target - present );
	
	output = ( ROCK_P_GAIN * deviation ) + ( ROCK_D_GAIN * ( deviation - old_deviation ) );

	output = Limit_ul( 100 , -100 , output );
	
	old_deviation = deviation;
	
	return output;
}

void move_left_tire( float left_duty )
{		
	static int i = 0;
	
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
	
	left_duty = Limit_ul( 30 , 0 , left_duty );
	LEFT_TIRE_DUTY = ( ( PWM_PERIOD * left_duty ) /100 );
}

void move_right_tire( float right_duty )
{		
	static int i = 0;
	
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
	right_duty = Limit_ul( 30 , 0 , right_duty );
	RIGHT_TIRE_DUTY = ( ( PWM_PERIOD * right_duty ) /100 );
}

void move_back_tire( float back_duty )
{		
	static int i = 0;
	
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
		BACK_TIRE_CW 		= 0;
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
	
	back_duty = Limit_ul( 30 , 0 , back_duty );
	BACK_TIRE_DUTY = ( ( PWM_PERIOD * back_duty ) /100 );
}

/******************************************************************************
*	タイトル ： 左オムニタイヤの出力決定
*	  関数名 ： get_motor_output_l
*	  戻り値 ： float型
*	    引数1 ：float型 motor_output_x
*	    引数2 ：float型 motor_output_y
*	    引数3 ：float型 degree_now
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/11/21
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
	
	motor_output_l = fabs(motor_output_x) * cos(D_TO_R(degree_now + (150.0 + degree_reverse_x))) + fabs(motor_output_y) * sin(D_TO_R(degree_now + (150.0 + degree_reverse_y)));
	return(motor_output_l);
}

/******************************************************************************
*	タイトル ： 右オムニタイヤの出力決定
*	  関数名 ： get_motor_output_r
*	  戻り値 ： float型
*	    引数1 ：float型 motor_output_x
*	    引数2 ：float型 motor_output_y
*	    引数3 ：float型 degree_now
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/11/21
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
	
	motor_output_r = fabs(motor_output_x) * cos(D_TO_R(degree_now +( 30.0 + degree_reverse_x))) + fabs(motor_output_y) * sin(D_TO_R(degree_now + (30.0 + degree_reverse_y)));
	return(motor_output_r);
}

/******************************************************************************
*	タイトル ： 後ろオムニタイヤの出力決定
*	  関数名 ： get_motor_output_b
*	  戻り値 ： float型
*	    引数1 ：float型 motor_output_x
*	    引数2 ：float型 motor_output_y
*	    引数3 ：float型 degree_now
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/11/21
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
	
	motor_output_b = fabs(motor_output_x) * cos(D_TO_R(degree_now  + (degree_reverse_x - 90.0))) + fabs(motor_output_y) * sin(D_TO_R(degree_now +( degree_reverse_y - 90.0)));
	return(motor_output_b);
}

void Move( float left_duty , float right_duty , float back_duty )
{
	move_left_tire(left_duty);
	move_right_tire(right_duty);
	move_back_tire(back_duty);
}

/******************************************************************************
*	タイトル ：R1350N用受信関数
*	  関数名 ： input_R1350N
*	  戻り値 ： void型 
*	   引数1 ： void
*	  作成者 ： 有本光
*	  作成日 ： 2014/01/29
******************************************************************************/
void input_R1350N(void)
{
	static int i = 0;
	static unsigned char receive_pac[15] = {0};
	static int read_start = OFF;
	static float 	start_Rate_f	= 0.00;
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
	
	receive_pac[i] = Receive_uart_c();//受け取る

	//HEADER値発見
	if(receive_pac[0] == 0xAA){//ヘッダー値AA
		read_start = ON;
	}else{
		read_start = OFF;
		i = 0;
		sprintf(str,"0xAA not found");
		transmit(str);
	}
	
	if(read_start == ON){
		i++;
		//0～14までで1セットの文字列
		if(i >= 15){
			i = 0;
			read_start = OFF;

			//パケットのヘッダー情報を確認する
			if(receive_pac[0] != 0xAA){
				sprintf(str, "Heading ERROR");
				transmit(str);
			}
			
			//データを組み立てる
			//index = receive_pac[2];
			rate = (receive_pac[3] & 0xFF) | ((receive_pac[4] << 8) & 0xFF00);
			angle = (receive_pac[5] & 0xFF) | ((receive_pac[6] << 8) & 0XFF00);
			x_acc = (receive_pac[7] & 0xFF) | ((receive_pac[8] << 8) & 0xFF00);
			y_acc = (receive_pac[9] & 0xFF) | ((receive_pac[10] << 8) & 0XFF00);
			z_acc = (receive_pac[11] & 0xFF) | ((receive_pac[12] << 8) & 0xFF00);
			//reserved = receive_pac[13];
			
			//チェックサムの確認
			check_sum = receive_pac[2] + receive_pac[3] + receive_pac[4] + receive_pac[5]
							+ receive_pac[6] + receive_pac[7] + receive_pac[8] + receive_pac[9]
					     	+ receive_pac[10] + receive_pac[11] + receive_pac[12] + receive_pac[13];
			
			if(check_sum != receive_pac[14]){
				sprintf(str, "Check_Sum ERROR");
				transmit(str);
			}
			
			//角度と角速度の単位を通常値（元に戻しデータを記憶する
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
			
		switch( flag ){
			case 0:
				if( g_Rate_f != 0.00 ){
					flag = 1;
					start_Rate_f = gap_degree( g_Rate_f );
				}break;
			case 1:
				g_Rate_f = ( -1 )*gap_degree( (gap_degree(  g_Rate_f ) - start_Rate_f ) );		
				break;
			}
		}
	}
}

/******************************************************************************
*	タイトル ：R1350Nコマンド用チェックサム計算関数
*	  関数名 ： checksum_r1350n
*	  戻り値 ： int型 
*	   引数1 ： char型
*	  作成者 ： 有本光
*	  作成日 ： 2014/01/29
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
		transmit(str);
	}

	return (p&0xFF);
}

//グローバル変数からの数値データの取出し	要素番号での読み書きがめんどくさい人は使えば？
float pick_out_atoz_value(char character){
    
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

//グローバル変数への数値データの書き込み	要素番号での読み書きがめんどくさい人は使えば？
void throw_away_atoz_value(char character, float write){
    
    int box = 0;
    
    if( (character >= 'a') && (character <= 'z') ){
        box = (int)(character - 'a');
        g_atoz_value[box] = write;
    }else if( (character >= 'A') && (character <= 'Z') ){
        box = (int)(character - 'A');
        g_AtoZ_value[box] = write;
    }
}

//文字列をfloat型に変換	この関数はロボティクスのマイコンの授業で配られたやつですね
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

//解析した命令に応じて数値をグローバル変数に格納する関数	下の関数の続きみたいな
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

//一文字ごとに解析する関数	いーのくんの力作
void receive_order_c(char character)
{
    static int target_box = 255;										//格納命令の開始文字(ASCIIコードのa～z,A～Z)
    static char storage_str[RECEIVE_STR_COLUMN] = "";	//文字の格納用の文字列
    static int storage_num = 0;											//文字を文字列のどこに格納するか
    static int minus_flag = 0;											//マイナス値か否か
    static int point_flag = 0;							 					//小数点以下が含まれているか否か
    static int after_point_count = 0;					 				//小数点以下にどれだけ数があるか
    static int large_size_flag = 0;										//大文字なのか否か
    int reset = 0;										 						//処理のリセットをするか否か
	const char end = END;								 				//格納命令の終了文字
    
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

//文字列単位で解析する関数	繰り返すだけ
void receive_order_s(char *word)
{
    while(*word != '\0'){
        receive_order_c(*word);
        word++;
    }
}

//通信相手からの受信割り込み	ここはマイコンによって異なる
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
	
	c = Receive_uart_c_0();//受信データ
	
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
	
	if( count_time >= time && ignore == 0 ){
		BUZZER		= 1;
		ignore		= 1;
	}
	if( ignore == 1 ){
		off_time	 += INTERRUPT_TIME;
	}
	if( off_time >= time ){
		count_time	= 0.00;
		off_time 		= 0.00;
		ignore			= 0;
		BUZZER 		= 0;
	}
}

int buzzer_stop( time )
{
	static  float count_time = 0.00;
	
	count_time += INTERRUPT_TIME;

	if( count_time >= time ){
		BUZZER = 0;
		return 1;
	}
	return 0;
}

/******************************************************************************
*	タイトル ： ps3コンの値から出力決定
*	  関数名 ： straight_output_x
*	  戻り値 ： float型 出力値
*	    引数 ： なし
*	  作成者 ： 成宮　陽生
*	  作成日 ： 2014/09/15
******************************************************************************/
float straight_output_x( void )
{
	float straight_cal_x = 0.00;
	
	straight_cal_x = ( 255.0 - (float)LEFT_STICK_HIGH ) / 255.0;	//左に倒したとき1、右に倒したとき0
	straight_cal_x = ( straight_cal_x - 0.5 ) * 2.0;				//左に倒したとき1、右に倒しとき-1
	if( fabs( straight_cal_x ) <= STICK_NO_MOVE_RANGE ){			//遊び部分
		straight_cal_x = 0.0;
	}
	straight_cal_x = PWM_PER * straight_cal_x;

	return( straight_cal_x );
}

/******************************************************************************
*	タイトル ： ps3コンの値から出力決定
*	  関数名 ： straight_output_y
*	  戻り値 ： float型 出力値
*	    引数 ： なし
*	  作成者 ： 成宮　陽生
*	  作成日 ： 2014/09/15
******************************************************************************/
float straight_output_y( void )
{
	float straight_cal_y = 0.00;
	
	straight_cal_y = ( 255.0 - (float)LEFT_STICK_WIDE ) / 255.0;	//上に倒したとき1、下に倒したとき0
	straight_cal_y = ( straight_cal_y - 0.5 ) * 2.0;				//上に倒したとき1、下に倒したとき-1
	if( fabs( straight_cal_y ) <= STICK_NO_MOVE_RANGE ){			//遊び部分
		straight_cal_y = 0.0;
	}
	straight_cal_y = PWM_PER * straight_cal_y;
	
	return( straight_cal_y );
}

/******************************************************************************
*	タイトル ： ps3コンの値から出力決定
*	  関数名 ： turn_output
*	  戻り値 ： float型 出力値
*	    引数 ： なし
*	  作成者 ： 成宮　陽生
*	  作成日 ： 2014/09/15
******************************************************************************/
float turn_output( void )
{
	float turn_cal = 0.00;

	turn_cal = ( 255.0 - (float)RIGHT_STICK_WIDE ) / 255.0;	//左に倒したとき1、右に倒したとき0
	turn_cal = ( turn_cal - 0.5 ) * 2.0;					//左に倒したとき1、右に倒しとき-1
	if( fabs(turn_cal) <= STICK_NO_MOVE_RANGE ){			//遊び部分
		turn_cal = 0.0;
	}
	turn_cal = ( OPERATE_DEGREE / ( 1.000 / INTERRUPT_TIME ) ) * turn_cal;	//スティックmax時1000msでどんだけ角度が進むか

	return( turn_cal );
}
	
#ifdef __cplusplus
void abort(void)
{

}
#endif
