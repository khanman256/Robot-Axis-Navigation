#include <avr/io.h>

#define F_CPU 16000000 //must be before delay

// include statements
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include "3pi_32u4_drivers.h"
//#include "robot_3pi_drivers.h"
// define statements
#define LEFT_MOTOR_PWM_LOCATION 6
#define RIGHT_MOTOR_PWM_LOCATION 5
#define LEFT_MOTOR_DIRECTION_LOCATION 2
#define RIGHT_MOTOR_DIRECTION_LOCATION 1
#define A_BUTTON_LOCATION 3
#define C_BUTTON_LOCATION 0
#define B_BUTTON_LOCATION 5
#define PWM_TOP 100
#define INCREMENT 5

// Configures buttons and motors on the robot using bit masking
void configure_A_button(){ //Configures button A
	DDRB &= ~(1<< A_BUTTON_LOCATION);
	PORTB |= (1<< A_BUTTON_LOCATION);
}
void configure_C_button(){ // Configures button C
	DDRB &= ~(1<< C_BUTTON_LOCATION);
	PORTB |= (1<< C_BUTTON_LOCATION);
}
void configure_B_button(){ // Configures button B
	DDRD &= ~(1<< B_BUTTON_LOCATION);
	PORTD |= (1<< B_BUTTON_LOCATION);
}
void configure_L_Motor(){ // Configures Left motor
DDRB &= ~(1<< LEFT_MOTOR_PWM_LOCATION);
PORTB |= (1<< LEFT_MOTOR_PWM_LOCATION);
}
void configure_R_Motor(){ // Configures Right motor
DDRB &= ~(1<< RIGHT_MOTOR_PWM_LOCATION);
PORTB |= (1<< RIGHT_MOTOR_PWM_LOCATION);
}
void configure_L_Motor_direction(){ // Configures Left motor direction
DDRB &= ~(1<< LEFT_MOTOR_DIRECTION_LOCATION);
PORTB |= (1<< LEFT_MOTOR_DIRECTION_LOCATION);
}
void configure_R_Motor_direction(){ // Configures Right motion direction
DDRB &= ~(1<< RIGHT_MOTOR_DIRECTION_LOCATION);
PORTB |= (1<< RIGHT_MOTOR_DIRECTION_LOCATION);
}
void turn_on_left_motor() // Function to start Left motor
{
PORTB |= (1<<LEFT_MOTOR_PWM_LOCATION);
}
void turn_on_right_motor() // Function to start Right motor
{
PORTB |= (1<<RIGHT_MOTOR_PWM_LOCATION); 
}

// Functions to switch off motors
void turn_off_left_motor() 
{
PORTB &= ~(1<<LEFT_MOTOR_PWM_LOCATION);
}
void turn_off_right_motor()
{
PORTB &= ~(1<<RIGHT_MOTOR_PWM_LOCATION);
}

// Function to switch motor direction backwards
void turn_left_motor_backward()
{
PORTB |= (1<<LEFT_MOTOR_DIRECTION_LOCATION);
}
void turn_right_motor_backward()
{
PORTB |= (1<<RIGHT_MOTOR_DIRECTION_LOCATION);
}

// Functions to switch motor direction forwards
void turn_right_motor_forward()
{
PORTB &= ~(1<<RIGHT_MOTOR_DIRECTION_LOCATION);
}
void turn_left_motor_forward()
{
PORTB &= ~(1<<LEFT_MOTOR_DIRECTION_LOCATION);
}
// This function allows the user to choose a distance on the robot using the A and C buttons. C would increase, A would decrease (-25 to 25)
int distchoose(uint8_t times)
{
	// Initialize variables
	int distance = 0;
	unsigned int last_button_A_state = (button_a_is_up());
	unsigned int button_A_pressed = 0;
	unsigned int last_button_B_state = (button_b_is_up());
	unsigned int button_B_pressed = 0;
	unsigned int last_button_C_state = (button_c_is_up());
	unsigned int button_C_pressed = 0;

	while(1)
	{	
		if(button_a_is_up() != last_button_A_state){
			if(button_a_is_up() == 0 ){
				button_A_pressed = 1;
			}
			last_button_A_state = (button_a_is_up());
		}
		else{
			button_A_pressed = 0;
		}

		if(button_c_is_up() != last_button_C_state){
			if(button_c_is_up() == 0 ){
				button_C_pressed = 1;
			}
			last_button_C_state = button_c_is_up();
		}
		else{
			button_C_pressed = 0;
		}
		if(button_b_is_up() != last_button_B_state){
			if(button_b_is_up() == 0 ){
				button_B_pressed = 1;
			}
			last_button_B_state = button_b_is_up();
		}
		else{
			button_B_pressed = 0;
		}
		// Executes if button A is pressed and the distance is greater than -25
		if (button_A_pressed == 1 && distance > -25){
			distance--;
		}
		// Executes if button C is pressed and the distance is less than 25
		if (button_C_pressed == 1 && distance < 25){
			distance++;
		}
		// Breaks out of while loop when button B is pressed
		if (button_B_pressed){
			break;
		}
		// If inputted distance is negative, place negative distance onto lcd screen
		if (distance < 0){
			LCD_set_cursor(times,0);
			LCD_putchar((char) (120+times));
			LCD_set_cursor(times,1);
			LCD_putchar((char) 58);
			LCD_set_cursor(times,2);
			LCD_putchar((char) 45);
			LCD_set_cursor(times,3);
			LCD_putchar((char)(abs((distance/10)%10) +48));
			LCD_set_cursor(times,4);
			LCD_putchar((char)(abs(distance%10) + 48));
		}
		// If inputted distance is positive, place positive distance onto lcd screen
		else{
			LCD_set_cursor(times,0);
			LCD_putchar((char) (120+times));
			LCD_set_cursor(times,1);
			LCD_putchar((char) 58);
			LCD_set_cursor(times,2);
			LCD_putchar((char) 32);
			LCD_set_cursor(times,3);
			LCD_putchar((char)(abs((distance/10)%10) +48));
			LCD_set_cursor(times,4);
			LCD_putchar((char)((abs(distance%10) + 48)));
		}
		turn_off_right_motor();
		turn_off_left_motor();
		_delay_us(10);
	}
	return distance;
}

// Code for the robot performing a 90 degree left turn using quadurture counters
void leftTurn( uint32_t prev_left_quad_counter)
{
	uint8_t pwm_counter =0;
	// Motor configuration for turning left
	turn_left_motor_backward();
	turn_right_motor_forward();
	_delay_us(10);
	while(1)
	{
		pwm_counter = pwm_counter + 1;
		if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}

	if (pwm_counter < 10) {
		turn_on_left_motor();
		turn_on_right_motor();
	}
	else {
		turn_off_left_motor();
		turn_off_right_motor();
	}
	_delay_us(1);
	if (get_left_quadrature_counter() > prev_left_quad_counter + 240)
	{
		break;
	}
	}
	// Stops the robot after turning
	turn_off_right_motor();
	turn_off_left_motor();
}

// Code for the robot performing a 90 degree right turn using quadurture counters
void rightTurn( uint32_t prev_left_quad_counter)
{
	uint8_t pwm_counter =0;
	turn_right_motor_backward();
	turn_left_motor_forward();
	_delay_us(10);
	while(1)//double check numbers
	{
		pwm_counter = pwm_counter + 1;
		if( pwm_counter >= PWM_TOP ){
			pwm_counter = 0;
		}

		if (pwm_counter < 10) {
			turn_on_left_motor();
			turn_on_right_motor();
		}
		else {
			turn_off_left_motor();
			turn_off_right_motor();
		}
		_delay_us(1);
		if (get_left_quadrature_counter() > prev_left_quad_counter + 240)
		{
			break;
		}
	}
	turn_off_right_motor();
	turn_off_left_motor();
}

// Code for moving forward based on distance
void moveForward(unsigned int dist, uint32_t prev_left_quad_counter)
{
	int pwm_counter = 0;
	turn_right_motor_forward();
	turn_left_motor_forward();

	while(1)
	{
		pwm_counter = pwm_counter + 1;
		if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}

	if (pwm_counter < 10) {
		turn_on_left_motor();
		turn_on_right_motor();
	}
	else {
		turn_off_left_motor();
		turn_off_right_motor();
	}
	if (get_left_quadrature_counter() > prev_left_quad_counter + 90*dist)
	{
		break;
	}
	_delay_us(10);
	}
	turn_off_left_motor();
	turn_off_right_motor();
}

//Use get_quadrature_counter to redo distance moved
int main(void)
{
	// Configure robot
	configure_3pi();
	configure_R_Motor_direction();
	configure_L_Motor_direction();
	configure_R_Motor();
	configure_L_Motor();	
	turn_off_left_motor();
	turn_off_right_motor();
	LCD_simple_command(0x0F);
	uint32_t prev_left_quad_counter = 0;
	int xdist = distchoose(0);
	int ydist = distchoose(1);

	_delay_ms(100);
	// Situation where y is positive
	if (ydist > 0)
	{
		moveForward(ydist, prev_left_quad_counter);
		prev_left_quad_counter = get_left_quadrature_counter();

		_delay_ms(100);
		if(xdist > 0) // Situation where x is positive
		{
			rightTurn(prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
			moveForward(xdist, prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);

		}
		else if(xdist < 0) // Situation where x is negative
		{
			leftTurn(prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
			moveForward(abs(xdist), prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
		}
		// No need for where x is zero since it is being executed last, so nothing happens
	}
	else if (ydist < 0) // Sitution where y is negative
	{
		leftTurn(prev_left_quad_counter);
		prev_left_quad_counter = get_left_quadrature_counter();
		_delay_ms(100);
		leftTurn(prev_left_quad_counter);
		prev_left_quad_counter = get_left_quadrature_counter();
		_delay_ms(100);
		moveForward(abs(ydist), prev_left_quad_counter);
		prev_left_quad_counter = get_left_quadrature_counter();
		_delay_ms(100);
		if(xdist > 0)
		{
			leftTurn(prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
			moveForward(xdist, prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);

		}
		else if(xdist < 0)
		{
			rightTurn(prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
			moveForward(abs(xdist), prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
		}
		// No need for where x is zero since it is being executed last, so nothing happens
	}
	else // Situation where y is 0
	{
		if(xdist > 0) // Situation where x is positive
		{
			rightTurn(prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
			moveForward(xdist, prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
		}
		else if(xdist < 0) // Situation where x negative
		{
			leftTurn(prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);
			moveForward(xdist, prev_left_quad_counter);
			prev_left_quad_counter = get_left_quadrature_counter();
			_delay_ms(100);		
		}
		// No need for where x is zero since it is being executed last, so nothing happens

	}


}