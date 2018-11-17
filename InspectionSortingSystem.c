//#######################################################################
//# TITLE: MECH 458 - FinalProject
//# REVISION#: 10
//# GROUP: 8
//# NAME 1: Amber MacNeill, V00827818
//# NAME 2: Rachel Brennan, V00819977
//# DESCRIPTION: UVic Inspection/Sorting System
//# DATE: December 2, 2017
//#######################################################################

#include <stdlib.h> //Standard Library of C
#include <avr/io.h> //I/O port
#include <util/delay_basic.h> 
#include <avr/interrupt.h> //Needed for interrupt functionality
#include "LinkedQueue.h" //Library for Linked Queue

//*************GLOBAL VARIABLES******************************************
enum item_state { //various states of state machine
	no_state = 0,
	falling, 
	mid_exit, //item is inside sensor
	after_exit //item has slipped past sensor
};

//variables to hold the states of 1st and 2nd item of Queue
volatile unsigned int item = no_state; 
volatile unsigned int next_item = no_state; 
volatile char reset_timer = 0;

//variables related to stepper
volatile int step = 0;
volatile int direction = 1;
volatile int stepper_config = 0; 
volatile int initial = 0; 
volatile int current = 0; 
volatile int desired = 0; 


//variables related to stepper acceleration
volatile int baby_steps = 5; //size of array
volatile int full_speed = 6; 
volatile int dir_difference = 0; 
volatile int diff_curr_init;
volatile int diff_curr_des;
const char accel_array[] = { 
    20,16,12,9,7
};

//belt direction
volatile unsigned char belt_dir = 0b0001;

//Related to linked queue
link *head;
link *tail;
link *newLink;
link *rtnLink;
volatile element e_head;
volatile element e_next;

//for ADC conversion
volatile unsigned int lowest_ADC; 
volatile unsigned char ADC_result_flag;
volatile unsigned int value;

//for Pause Button
volatile unsigned char pause_button_flag = 0;
volatile unsigned char pause_debounced = 0;

//for Ramp Down
volatile unsigned int rd_check_1 = 0;

// Item count for pause/ramp down: A, S, W, B, Unsorted 
char item_count_array[] = { 
    0b10000000,0b01000000,0b00100000,0b00010000,0b11110000
};

//Hardcoded ADC calibration - with some tolerance
volatile unsigned int black_low = 936; 
volatile unsigned int black_high = 980;
volatile unsigned int steel_low = 251;
volatile unsigned int steel_high = 750;
volatile unsigned int white_low = 820;
volatile unsigned int white_high = 935;
volatile unsigned int aluminum_low = 0;
volatile unsigned int aluminum_high = 250;

// steps for stepper motor. Two Phase Full Step
const char step_array[] = { 
    0b00110110,0b00101110,0b00101101,0b00110101
};
//*************End of Global Variables***********************************
void timer_with_reset(unsigned int count){
    unsigned int i;
    i=0;

    TCCR1B |=_BV(WGM12); //Compare Math Mode
    OCR1A = 0x03e8; // Output Compare Register 1000 cycles = 1ms
    TCNT1 = 0x0000; // Time Counter
    
    TIFR1|=_BV(OCF1A); // clear interrupt flag
    
    //Timer is started when dropping an item
	//Timer resets if an item with same material is next
	do {
		reset_timer = 0;
		i = 0; 
		//while reset timer flag is not set
	    while(i<count && !reset_timer){ 
	        if((TIFR1&0x02)==0x02){
	            TIFR1|=_BV(OCF1A); //clear interrupt flag
	            i++; 
	        }
	    }
	} while(reset_timer); 
} 

void pwm(void){ 
	//Fast PWM mode, OCRA is updated at TOP
	TCCR0A |= _BV(WGM01);
	TCCR0A |= _BV(WGM00);

	//Timer/counter control register A - Cleared at TOP
	TCCR0A |= _BV(COM0A1);
	
	//Frequency of PWM to 500Hz by changing prescalar (N = 8) 
	TCCR0B |= _BV(CS01);
	
	//Start up Duty Cycle of 0% 
	OCR0A = 0x00; 
}

void system_setup(void){
    pwm();
	
	TCCR1B |=_BV(CS10); //sets timer 1 to run at CPU clock, no prescale
	
	//PORT setup
    DDRC = 0xFF; //output for LEDs
    DDRA = 0xFF; //output for Stepper Motor
    DDRD = 0xF0; //input for bits 0 to 3, output for onboard LEDs
    DDRB = 0x8F; //output for dc motor (conveyor)
	DDRF = 0x00; //input for ADC & Ferro sensor 
	DDRE = 0x00; //input for Ramp Down & Optical @ Ferro 

    cli(); //disables all interrupts
    
	//ADC interrupt
    ADCSRA |= _BV(ADEN); //enable ADC
    ADCSRA |= _BV(ADIE); //enable interrupt of ADC
    ADMUX |= _BV(REFS0)|_BV(MUX0); //right justified, 10 bit config
	lowest_ADC = 0x3FF; //so that lowest ADC isn't initially zero

	//End of Travel Sensor Interrupt
	EICRA |= _BV(ISC20); //any edge interrupt
	EIMSK |= _BV(INT2); //enable INT2 - Exit Sensor
	
	//Optical @ Reflective Sensor Interrupt
	EICRA |=(_BV(ISC30)); // any edge interrupt
	EIMSK |=(_BV(INT3)); //enable INT3 - OR Sensor

	//Pause Button Interrupt
	EICRA |= (_BV(ISC01)); // falling edge interrupt
	EIMSK |= (_BV(INT0)); //enable INT0 - Pause Button

	//Ramp Down Button Interrupt
	EICRB |=(_BV(ISC51)); //falling edge interrupt
	EIMSK |=(_BV(INT5)); //enable INT5 - Ramp Down Button
	
    sei(); //enables all interrupts
}

//Delay Timer
void mTimer(int count){
    int i;
    i=0;

    TCCR1B |=_BV(WGM12); // Compare Math Mode
    OCR1A = 0x03e8; //1000 cycles = 1ms
    TCNT1 = 0x0000; //Time Counter    
    TIFR1|=_BV(OCF1A); //Clear interrupt flag
    
    // Count until timer has reached OCR1A
    while(i<count){
        if((TIFR1&0x02)==0x02){
            TIFR1|=_BV(OCF1A); //Clear interrupt flag
            i++; 
        }
    }
} 

void interruptTimer_config(void){
	TCCR3B |= _BV(WGM32); //set WGM to CTC mode 
	TCCR3B |= (_BV(CS32)|_BV(CS30)); //set to clk/1024 prescale
	TIFR3 |= _BV(OCF3A); //output compare flag register
}

void config_stepper(void){ 
    unsigned int i;
    unsigned int j;
	
    //Synchronize Stepper
    for(i=0;i<50;i++){ 
        step = (step + direction)%4; //update step    
		PORTA = step_array[step]; //move
		mTimer(20); //speed
    }
	
	cli(); //disable all interrupts

	//Hall Effect Sensor Enable
	EICRA |=(_BV(ISC11)); //falling edge interrupt
	EIMSK |=(_BV(INT1)); //enable INT1 - Hall Effect Sensor 
	EIFR  |=(_BV(INT1)); //flag 

	sei(); //enable all interrupts 
    
	//Move Stepper Until HE Interrupt is Fired
    while(stepper_config == 0){ 	
        step = (step + direction)%4; //update step		
		PORTA = step_array[step];// move
        mTimer(20); //speed
    }

	//correct for centering stepper
	for(j=0;j<8;j++){
		//update step
		if(step == 0){
			step = 3; 
		}else{
			step = (step - 1) % 4;
		}
		PORTA = step_array[step]; //move
		mTimer(20); //speed
	}
}

void set_speed(void){
	//calculate differences between current&initial, and current&desired
	diff_curr_init = abs(current - initial);
	diff_curr_des = abs(desired - current);

	//Correct for the direction that the stepper would move
	if(diff_curr_init > 100){ 
		diff_curr_init = 200 - diff_curr_init; 
	}
	if(diff_curr_des > 100){
		diff_curr_des = 200 - diff_curr_des; 
	}
	
	//Set Speed of Stepper Depending on Position on Accel Ramp
	unsigned int min_diff = diff_curr_init < diff_curr_des ? diff_curr_init : diff_curr_des;
	// acceleration/deceleration
	if(min_diff < baby_steps){ 
		mTimer(accel_array[min_diff]);
	//move at full speed
	}else{ 
		mTimer(full_speed); 
	}
}

void update_stepper(){	
	//Move if stepper is not in desired position
	while(current != desired){
	    //if travel is greater than 180 degrees, go the other way
		dir_difference = current - desired; 
		if(abs(dir_difference) <=100){ 
			if(dir_difference > 0){
				direction = -1;
			}else{
				direction = 1;
			}
		//for S to B or B to S 
		}else{ 
			if(desired == 150){
				direction = -1;
			}else if(desired == 0){
				direction = 1;
			}			
		}

		//Update Step of Motor (1 to 4)
		if((step == 0) && (direction == -1)){
			step = 3; 
		}else{
			step = (step + direction) % 4;
		}

		PORTA = step_array[step]; //move stepper

		//Update Current Position (1 to 200)
		if((current == 0) && (direction == -1)){
			current = 199;
		}else{
			current = (current + direction)%200; 
		}	

		set_speed(); 
	} 
}

void pause_routine(void){
	int i = 0;
	item_count_array[4] |= size(&head,&tail); //add unsorted items

	mTimer(20); //debounce to pause
	while((PIND && 0x01) == 0x00){}
	mTimer(20); //debounce	
	pause_debounced = 1; 
	
	//Loop Through Materials and Display Counts Sorted
	//Also Display The Number of Unsorted Items
	while(pause_button_flag){	
		PORTC = item_count_array[i];
		mTimer(1000);
		i = (i+1)%5;
	}
	item_count_array[4] &= 0xF0; //to reset unsorted 	
	
	mTimer(20); //to resume
	while((PIND && 0x01) == 0x00){}
	mTimer(20); //debounce	
	pause_debounced = 0; 

}


//*************INTERRUPT SERVICE ROUTINES*********************************
//If an unexpected interrupt occurs - Display Error Code
ISR(BADISR_vect){
	while(1){	
		PORTC = 0b11001100;
		mTimer(1000);
		PORTC = 0b00110011;
		mTimer(1000);
	}
}

//ADC interrupt - Will convert on first edge of OR sensor
//Will stop on second edge of OR sensor (flag set in OR ISR)
ISR(ADC_vect){
	if(ADC_result_flag){
		value = ADC; 
		//keep lowest value of ADC
		if(lowest_ADC > value){ 
			lowest_ADC = value;
		}
		// start conversion again
		ADCSRA |= _BV(ADSC); 
	}	
}

//Timer Interrupt For Ramp Down - Timer depends on status of items
ISR(TIMER3_COMPA_vect){
	//if there are no items in the queue & this is the fist check
	if((isEmpty(&head)) && (rd_check_1 == 0)){
		OCR3A = 2000;
		rd_check_1 = 1; 
	//if check already and queue is still empty, stop belt, display counts	
	}else if((isEmpty(&head)) && (rd_check_1 == 1)){
		PORTB &= 0xF0; 	
		pause_button_flag = 1;
		pause_routine(); 
	//if items are in queue, check again after some time	
	}else if(!isEmpty(&head)){
		OCR3A = 2000; 	
	}
}

//Pause Button Interrupt - pause button flag set for main
ISR(INT0_vect){
	//If button has actually been pressed
	if((PIND & 0x01) == 0x00){
		//if resumed start belt again
		if(pause_debounced){
			pause_button_flag = 0;	
			PORTB |= belt_dir;
			PORTC = 0; 
		//if paused stop belt, get ready for resume
		}else{
			pause_button_flag = 1;
			PORTB &= 0xF0;
			PORTC = 0; 
		}
	}
}

//Hall Effect Sensor Interrupt
ISR(INT1_vect){
    //Set configuration flag & current position to 0
	if(stepper_config == 0){
		stepper_config = 1; 
	    current = 0; 
	}

	EIMSK &= ~_BV(INT1); //disable interrupt		   
}

//Exit Sensor Interrupt
ISR(INT2_vect){
	//On first edge
	if((PIND&_BV(PIND2))!=(_BV(PIND2))){ 
		//If there is an item already falling
		if(item == falling){
			//If this item is the same - continue
			if(e_next.itemCode == desired){ }
			//If this item is diff. - stop belt & update state: mid exit
			else if(e_next.itemCode != desired){
				PORTB &= 0xF0; 		
				next_item = mid_exit;
			}
		//If there isn't already an item falling
		}else{
			//If stepper not in position, stop belt
			if(current != desired){
				PORTB &= 0xF0;		
			}
		}
	//On second edge	
	}else{ 
		//If previous item's state is falling
		if(item == falling){
			//If this item is the same, reset timer for new item & dequeue			
			if(e_next.itemCode == desired){ 
				reset_timer = 1;
				dequeue(&head,&tail,&rtnLink);
				free(&rtnLink);
			//If this item is diff. update state to after exit	
			}else{
				next_item = after_exit;
			}
		//if last item has no state, set state to falling	
		}else if(item == no_state){
			item = falling;
		}
		
		//Count on dequeue items that have been sorted (pause/rd display)
       	if(desired == 0){
			item_count_array[3] += 1;		
		}
       	if(desired == 50){
			item_count_array[0] += 1;		
		}
       	if(desired == 100){
			item_count_array[2] += 1;		
		}
       	if(desired == 150){
			item_count_array[1] += 1;		
		}
	}
}

//OR Sensor Interrupt
ISR(INT3_vect){
	//At first edge, start ADC conversion
	if((PIND&0x08)==0x08){ 
		ADCSRA |= _BV(ADSC); 
		ADC_result_flag = 1; 
	//At second edge, stop ADC conversion & classify		
	}else{ 
		ADC_result_flag = 0; 
		if((lowest_ADC <= black_high)&&(lowest_ADC >= black_low)){ 
			initLink(&newLink);
			newLink->e.itemCode = 0; 
			enqueue(&head,&tail,&newLink);
		}else if((lowest_ADC <= steel_high)&&(lowest_ADC >= steel_low)){ 
			initLink(&newLink);
			newLink->e.itemCode = 150;
			enqueue(&head,&tail,&newLink);
		}else if((lowest_ADC <= white_high)&&(lowest_ADC >= white_low)){
			initLink(&newLink);
			newLink->e.itemCode = 100; 
			enqueue(&head,&tail,&newLink);
		}else if((lowest_ADC <= aluminum_high)&&(lowest_ADC >= aluminum_low)){ 
			initLink(&newLink); 
			newLink->e.itemCode = 50; 
			enqueue(&head,&tail,&newLink);
		//Error Display if out of range, sort to black (default) 
		}else{
			PORTC = 0b01010101;
			initLink(&newLink);
			newLink->e.itemCode = 0; 
			enqueue(&head,&tail,&newLink);
		}
		lowest_ADC = 0x3FF; //Reset Lowest_ADC for next part
	}
}

//Ramp Down Button Interrupt
ISR(INT5_vect){		   
	EIMSK &= ~_BV(INT5); //disable interrupt
	TCNT3 = 0; 
	OCR3A = 500; 
	TIMSK3 |= _BV(OCIE3A); //enable timer interrupt
}
//*************End of Interrupt Service Routines**************************

int main(int argc, char *argv[]){
	//setup
	rtnLink = NULL;
	newLink = NULL;
	system_setup();
    config_stepper();
	interruptTimer_config();
    setup(&head,&tail); 
    
    //Turn on Belt
	OCR0A = 0x69;
	PORTB |= belt_dir;

	//Main Routine
	while(1){
		//Just wait while queue is empty
		while(isEmpty(&head)){
			//Enter pause routine if pause flag was set
			if(pause_button_flag){			
				pause_routine();
				PORTB |= belt_dir;
			}
		} 
		
		//update desired item
		if(!isEmpty(&head)){
			e_head = firstValue(&head);      
			desired = e_head.itemCode; 
		}	
		
		//grab the next item
		nextValue(&head, &e_next);

		// While we have a state, we must handle it
		while(item != no_state){
			// dequeue if item is falling
			if(item == falling){
				timer_with_reset(80); //optimize this number
				dequeue(&head,&tail,&rtnLink); 
		        free(rtnLink); 
				
				//update desired if there is another item
				if(!isEmpty(&head)){
					e_head = firstValue(&head);      
					desired = e_head.itemCode;
				}
				
				//grab next value, set item to the item after, reset state 
				nextValue(&head, &e_next);
				item = next_item;
				next_item = no_state;
				
			//update stepper if item is mid EX, reset state
			//will be updated by interrupt upon exit
			}else if(item == mid_exit){
				update_stepper();
				item = no_state;
				next_item = no_state;
				PORTB |= belt_dir;
			
			//update stepper if after exit, set item to falling
			//reset next item state
			}else if(item == after_exit){
				update_stepper();
				item = falling;
				next_item = no_state;
				PORTB |= belt_dir;
			}
		}

		update_stepper();	
		
		//Enter pause routine if pause flag was set
		if(pause_button_flag){
			pause_routine();
			PORTB |= belt_dir;					
		}
		
		//reset initial
		initial = current; 
		
		//start belt if off
		if((PORTB & 0xF0)==0){
			PORTB |= belt_dir;
		}
    }
} 



