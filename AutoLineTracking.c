//Robot code for class project
//Author <Geoffrey Nielsen><Nov 6th 2019>

#include "p24HJ128GP502.h"	//Include device library file
#include "timer.h"          //Include Timer library file
#include "adc.h"            //Include ADC library file
#include "outcompare.h"		//Include Output Compare (PWM) library file
#include "libpic30.h"

#define BUFF_SIZE 32
#define SERVO_MAX 3848
#define SERVO_MIN 3036

//Configuration Bits
_FOSCSEL(FNOSC_FRC & IESO_OFF);
_FOSC	(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSDCMD);
_FWDT	(FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR32 & WDTPOST_PS1);
_FPOR	(FPWRT_PWR1 & ALTI2C_ON);
_FICD	(ICS_PGD1 & JTAGEN_OFF);

//Function Headers
void InitIO (void);									
void InitTimer (void);				
void InitUART(void);
void InitPWM(void);
void ADC (void);				
void ProcessData(void);
void SendData(void);
long Map(long x, long in_min, long in_max, long out_min, long out_max);
void ComsLoss(void);
void ManualDrive(void);
void AutoDrive(void);
void ManualArm(void);
void AutoArm(void);

//Global Variables 
unsigned int ONTimeR = 0;                    //Variable used to control the RC Servo Motor's Position			
unsigned int ONTimeL = 0;
unsigned int ONTimeReceived = 0;            //Variable used to store value of ONTime while it is being received and mapped
unsigned int OFFTime = 3000;                //Variable used to control the period of the square wave	
unsigned int TmrState = 0;                  //Variable used to store the state of the interrupt		
unsigned int TmrVal = 0;                    //Variable used to store the value used to setup Timer1 which controls the ON and OFF time by utilizing Timer1's Interrupt
unsigned int ControlState = 0;
unsigned int LineLeft = 0;
unsigned int LineRight = 0;

unsigned char SendDataArray[BUFF_SIZE];     //Array to store the data to be transmitted through UART1 TX pin
unsigned char ReceiveDataArray[BUFF_SIZE];  //Array to store the data received from UART1 RX pin
unsigned char ReceivedChar = 0;             //Variable used to store the last byte received from UART1 RX pin
unsigned char DriveState = 0;               //Variable for if drive in auto or human

unsigned int CommsLossCount = 0;            //Store the number of times there was a communication loss
unsigned short Communicating;               //Store the value ReceiveDataArray[20], if 1 then new data received

//Main code function
int main (void){
    //Call start-up functions
    InitIO();		
	InitPWM();		
    InitTimer();  
    InitUART();
    //main program loop 
    while (1){
        //loop for <auto code>
        while (ControlState == 1){
            ProcessData();	
            SendData(); 
            
            if(CommsLossCount>200){
                ComsLoss();
            }
            else{
                ManualDrive();
                ManualArm();
            }
        }
        //loop for <human operated code>
        while (ControlState == 0){
            ProcessData();	
            SendData(); 
            
            if(CommsLossCount>200){    
                ComsLoss();             
            }
            else{
                AutoDrive();
                AutoArm();
            }
            
        }     
    }
    
}
//Manual Drive code
void ManualDrive (void){
    
}
//Auto line tracking code
void AutoDrive (void){
    //If the robot is left of the line turn it right
    if (LineLeft +100 > LineRight)	
	{									
		ONTimeR--;
        ONTimeL++;
	}	
    //If the robot it right of the line turn it left
	else if (LineLeft < LineRight + 100)				
	{
		ONTimeR++;
        ONTimeL--;
	}
    //If the robot is within the line but the Right motor is faster
    //Ramp up the left motors speed
    else if (ONTimeR > ONTimeL){
        ONTimeL++;
    }
    //If the robot is within the line but the Left motor is faster
    //Ramp up the Right motors speed
    else if (ONTimeL > ONTimeR){
        ONTimeR++;
    }
    //Set motors full speed forward 
    else{
        ONTimeL = 4000;
        ONTimeR = 4000;
    }
}
//manual manipulator movement code
void MaunalArm (void){
    
}
//Auto manipulator movement code
void AutoArm (void){
    
}
//ADC pins to be sampled (AN0)
void ADC (void){ 
    //ADC's to read if in manual drive
    if (ControlState == 1){                               
        OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
                    ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
            ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
            ADC_DMA_BUF_LOC_1,
            ENABLE_AN4_ANA,
            0,		
            0,
            0);
                                        //Select AN4
        SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN4);
        AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
        while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
        Photocell = ReadADC1(0);        //Photocell = converted results
        AD1CON1bits.ADON = 0;           //Turn off ADC hardware module
    }
    //ADC's to read if in line tracking auto mode
    else if (ControlState == 0){
        //ADC for left line sensor
        OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
                    ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
            ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
            ADC_DMA_BUF_LOC_1,
            ENABLE_AN12_ANA,
            0,		
            0,
            0);
                                        //Select AN4
        SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN12);
        AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
        while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
        LineLeft = ReadADC1(0);        //Photocell = converted results
        AD1CON1bits.ADON = 0;           //Turn off ADC hardware module
        
        //ADC for right line sensor 
        OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
                    ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
            ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
            ADC_DMA_BUF_LOC_1,
            ENABLE_AN14_ANA,
            0,		
            0,
            0);
                                        //Select AN4
        SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN14);
        AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
        while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
        LineRight = ReadADC1(0);        //Photocell = converted results
        AD1CON1bits.ADON = 0;           //Turn off ADC hardware module
    }
}
//Data to be received from the controller
void ProcessData(void){	
	//SendDataArray[1] = (Photocell >> 8);                                //Populate the array one byte at a time
	//SendDataArray[2] = Photocell;

	ONTimeReceived = (ReceiveDataArray[1] << 8) + ReceiveDataArray[2];  //Build integer from array of bytes
	ONTimeReceived = Map(ONTimeReceived,0,4095,SERVO_MIN,SERVO_MAX);    //Make sure the RC Servo Motor values are within its limits
    ONTimeR = ONTimeReceived;   //This is to ensure the interrupt handler does not use a value of ONTime that has not yet been fully calculated. Reduces twitching in RC Servo Motor.
    ONTimeReceived = (ReceiveDataArray[3] << 8) + ReceiveDataArray[4];  //Build integer from array of bytes
	ONTimeReceived = Map(ONTimeReceived,0,4095,SERVO_MIN,SERVO_MAX);    //Make sure the RC Servo Motor values are within its limits
    ONTimeL = ONTimeReceived;   //This is to ensure the interrupt handler
    
    DriveState = (ReceiveDataArray[5]); 
    ControlState = (ReceiveDataArray[6]);
    
    SendDataArray[20] = 1;                  //Sending a 1 for controller to check for communication
    Communicating = ReceiveDataArray[20];   //Checking if the controller sent us a 1, which will let us know if we
                                            //have communication    
    
    if(Communicating){                      //If there is communication, reset the communication loss counter
        CommsLossCount = 0;
    }
    else if(!Communicating){                //If there is an interruption (i.e. single loss of communication),
        CommsLossCount++;                   //then increment the communication loss counter
    }
    ReceiveDataArray[20] = 0;               //Reset the communication to 0, If next time we look at it and it's 
                                            //still 0, then no communication. If next time we look at it and
                                            //controller has changed it to 1, then we have communication
}
//data to be sent to the controller
void SendData(void){
	for (i=0;i<BUFF_SIZE;i++)           //Index through the array from the start to the end of the array 
	{                                   //Note: The first byte is an ASCII Character "s" to denote the start
		WriteUART1(SendDataArray[i]);	//Send one byte of the data array to UART TX Pin
		while(BusyUART1());             //Wait while the UART1 is busy
	}
}
//Function used for mapping analog values for motor PWM signals
long Map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
//Shutdown function to be used when needed
void ComsLoss(void){
    LATAbits.LATA4 = 1;     //Turn on communication error LED 
}
//IO pins to be used and how
void InitIO (void){
	TRISBbits.TRISB2 = 1;	//Set RB2 as input for Photocell connected to AN4
	TRISBbits.TRISB6 = 0;	//Set RB6 as output for RC Servo Motor
    TRISAbits.TRISA4 = 0;   //Set RA4 as output for LED to indicate communication loss
    TRISBbits.TRISB2 = 1;	//Set RB2 as input (Left Side Phototransistor of Line Tracking Sensor)
	TRISBbits.TRISB3 = 1;	//Set RB3 as input (Right Side Phototransistor of Line Tracking Sensor)
    TRISBbits.TRISB14 = 0;  //PWM sig for Right Drive Motor
    TRISBbits.TRISB12 = 0;  //PWM sig for Left Drive Motor

	//RP8 TO U1RX           //Set the RP8 pin to UART1 RX pin
	RPINR18bits.U1RXR = 8;	//See TABLE 11-1: SELECTABLE INPUT SOURCES (MAPS INPUT TO FUNCTION),Page 136
                            //and REGISTER 11-8: RPINR18: PERIPHERAL PIN SELECT INPUT REGISTER 18,Page 146

    CNPU2bits.CN22PUE = 1;	//Enable weak pull-up of Receive pin CN22 (Corresponds to RP8)
                            //This is needed for v1.06 Bluetooth boards to pull up the receive line

	//RP9 TO U1TX           //Set the RP9 pin to UART1 TX pin
	RPOR4bits.RP9R = 3;     //See TABLE 11-2: OUTPUT SELECTION FOR REMAPPABLE PIN (RPn), Page 137
                            //and REGISTER 11-19: RPOR4: PERIPHERAL PIN SELECT OUTPUT REGISTERS 4, Page 154
}
//Use Timer 2 for PWM
void InitPWM(void){
	DisableIntT2;		//Disable Timer2 Interrupt
	DisableIntOC1;		//Disable OC1 Interrupt
	DisableIntOC2;		//Disable OC2 Interrupt
                        //Timer2 is the clock source for OC1 and OC2
                        //Configure PWM mode for OC1 and OC2
	OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE, 1, 1);
	OpenOC2(OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE, 1, 1);							
                        //Prescaler = 1:1 and Period = 0xFFFF
	OpenTimer2(T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_1 & T2_32BIT_MODE_OFF & T2_SOURCE_INT, 0xFFFF);
}
//UART stuff for coms
void InitUART(void){
	IEC0bits.U1TXIE = 0;            //Disable UART1 TX interrupt
    IFS0bits.U1RXIF = 0;            //Clear the Receive Interrupt Flag
	U1MODEbits.STSEL = 0;           //1 Stop bit
	U1MODEbits.PDSEL = 0;           //8-bit data, no parity
	U1MODEbits.BRGH = 0;            //16x baud clock, Standard mode
	U1MODEbits.URXINV = 0;          //Idle State 1 for RX pin
	U1MODEbits.ABAUD = 0;           //Auto-Baud Disabled
	U1MODEbits.RTSMD = 1;           //Simplex Mode, no flow control
	U1MODEbits.UARTEN = 1;          //Enable UART1
	U1STAbits.UTXISEL0 = 0;         //Interrupt after one TX character is transmitted
	U1STAbits.UTXISEL1 = 0;         //Interrupt after one TX character is transmitted
	U1STAbits.UTXEN = 1;            //Enable UART1 to control TX pin
	U1BRG = 1;                      //BAUD Rate Setting for 115200
	IEC0bits.U1RXIE = 1;            //Enable UART1 RX interrupt
}
//Timer 1 set up
void InitTimer(void){
        //It's possible to do the equation by hand, or with the PIC timer calculator to obtain proper values
        //Prescaler = 1:8
        //Period = 0xFFFF (hexadecimal) = 65535 (decimal), this is the PR value in pic timer calculator program

        OpenTimer1 (T1_ON & T1_PS_1_8 & T1_SOURCE_INT & T1_GATE_OFF & T1_IDLE_STOP, 0xFFFF);

        //Turn Timer1 interrupt on
        ConfigIntTimer1 (T1_INT_PRIOR_7 & T1_INT_ON);
        
        //Wrong...Changed from a 125ms delay to a 500ms delay <Geoffrey Nielsen><sept 18th, 2019>
        WriteTimer1(7957);
}
//Timer 1 call
void InitUART(void){
	IEC0bits.U1TXIE = 0;            //Disable UART1 TX interrupt
    IFS0bits.U1RXIF = 0;            //Clear the Receive Interrupt Flag
	U1MODEbits.STSEL = 0;           //1 Stop bit
	U1MODEbits.PDSEL = 0;           //8-bit data, no parity
	U1MODEbits.BRGH = 0;            //16x baud clock, Standard mode
	U1MODEbits.URXINV = 0;          //Idle State 1 for RX pin
	U1MODEbits.ABAUD = 0;           //Auto-Baud Disabled
	U1MODEbits.RTSMD = 1;           //Simplex Mode, no flow control
	U1MODEbits.UARTEN = 1;          //Enable UART1
	U1STAbits.UTXISEL0 = 0;         //Interrupt after one TX character is transmitted
	U1STAbits.UTXISEL1 = 0;         //Interrupt after one TX character is transmitted
	U1STAbits.UTXEN = 1;            //Enable UART1 to control TX pin
	U1BRG = 1;                      //BAUD Rate Setting for 115200
	IEC0bits.U1RXIE = 1;            //Enable UART1 RX interrupt
}
//Timer 1 Interrupt
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void){
	DisableIntT1;           //Disable Timer1 Interrupt
    InterruptCount++;       //Increments by 1 in order to count multiple 125ms delays
    WriteTimer1(7957);      
	IFS0bits.T1IF = 0;      //Reset Timer1 interrupt flag
	EnableIntT1;            //Enable Timer1 interrupt
}
// UART1 Receive Interrupt
void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
	DisableIntU1RX;             //Disable the UART1 receive interrupt
	IFS0bits.U1RXIF = 0;        //Reset the UART1 receive interrupt flag
	ReceivedChar = U1RXREG;     //Store the latest received character

//Need to synchronize the data being received by looking for the 's' which denotes the start of the array
    if ((uartcount == 0) && (ReceivedChar == 's'))  //Note: uartcount=0 until we receive a 's'
    {
        ReceiveDataArray[uartcount] = ReceivedChar; //Store 's' into the 0 element of the array
        uartcount++;                                //Increment array index for next byte being received
    }
    else if (uartcount != 0)
//Data has been synchronized; update the array of data being received until buffer size has been reached
    {
        ReceiveDataArray[uartcount] = ReceivedChar; //Update array with the latest data byte being received
        uartcount++;                                //Increment array index for next byte being received
        if (uartcount==BUFF_SIZE) uartcount=0;      //All data in array has been received
    }
     EnableIntU1RX;             //Enable the UART1 receive interrupt
}