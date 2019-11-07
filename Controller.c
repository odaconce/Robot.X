//Code for group 4 controller

#include "p24HJ128GP502.h"  //Include device library file
#include "timer.h"          //Include TIMER library file
#include "uart.h"           //Include UART library file
#include "adc.h"            //Include ADC library file

#define BUFF_SIZE 32

//Configuration Bits
_FOSCSEL(FNOSC_FRC & IESO_OFF);
_FOSC	(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSDCMD);
_FWDT	(FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR32 & WDTPOST_PS1);
_FPOR	(FPWRT_PWR1 & ALTI2C_ON);
_FICD	(ICS_PGD1 & JTAGEN_OFF);

//Function prototypes
void InitIO (void);
void InitTimer (void);					
void InitUART(void);
void ADC (void);			
void ProcessData(void);
void SendData(void);
void Shutdown(void);
void Gripper(void);

//Global variables
unsigned int ONTimeL = 0;       //Signal for right drive motor
unsigned int ONTimeR = 0;       //Signal for left drive motor
unsigned int ArmPosition = 0;       //Signal for arm motor
unsigned int Gripper = 0;   //Open gripper if 1, close if 2, don't move if 0

unsigned char SendDataArray[BUFF_SIZE];     //Array to store the data to be transmitted through UART1 TX pin
unsigned char ReceiveDataArray[BUFF_SIZE];  //Array to store the data received from UART1 RX pin
unsigned char ReceivedChar = 0;             //Variable used to store the last byte received from UART1 RX pin
int i;                                      //Variable used to index through arrays
int uartcount = 0;                          //The current array index of the received data from the UART RX pin

unsigned int CommsLossCount = 0;            //Store the number of times there was a communication loss
unsigned short Communicating;               //Store the value ReceiveDataArray[20], if 1 then new data received

int main (void)
{
	InitIO();               //Call InitIO which configures the input and output pins
	InitTimer();            //Call InitTimer which configures Timer1 for controlling LED pulse rate
	InitUART();             //Call InitUART which configures the UART1 hardware module for communications
                            //with the HC-05 Bluetooth Module
	
	for (i=0; i<BUFF_SIZE; i++) SendDataArray[i] = 0;   //Initialize the array of chars to zero
	SendDataArray[0] = 's';                             //Set first element as 's' for data synchronization
                                                        //and for frame error checking
	while (1)               //Infinite loop
	{
        ProcessData();      //Call ProcessData to update variables for UART1 Communications
		SendData();         //Call SendData to send data through the UART1 TX pin to HC-05 Bluetooth Module
        
        if(CommsLossCount>200){     
            Shutdown();             
        }
        //Human Driving
        else if (PORTBbits.RB5){    //If switch on RB5 is high     
            LATAbits.LATA4 = 0;     //Turn off Communication loss LED
            ADC();                  //ADC sets variables for drive speed and manipulator position
            Gripper();              //Controls movements of gripper motor
        } 
        //Auto Driving
        else {
            LATAbits.LATA4 = 0;
        }
    }
}
void InitIO (void){
	TRISAbits.TRISA0 = 1;	//Set RA0 (AN0) as input
	TRISAbits.TRISA4 = 0;   //Set RA4 as output for LED to indicate communication loss
    TRISBbits.TRISB5 = 1;	//Set RB5 as input for switch for drive mode
    

	//RP8 TO U1RX           //Set the RP8 pin to UART1 RX pin
	RPINR18bits.U1RXR = 8;	//See TABLE 11-1: SELECTABLE INPUT SOURCES (MAPS INPUT TO FUNCTION),Page 136
                            //and REGISTER 11-8: RPINR18: PERIPHERAL PIN SELECT INPUT REGISTER 18,Page 146

    CNPU2bits.CN22PUE = 1;	//Enable weak pull-up of Receive pin CN22 (Corresponds to RP8)
                            //This is needed for v1.06 Bluetooth boards to pull up the receive line

	//RP9 TO U1TX           //Set the RP9 pin to UART1 TX pin
	RPOR4bits.RP9R = 3;     //See TABLE 11-2: OUTPUT SELECTION FOR REMAPPABLE PIN (RPn), Page 137
                            //and REGISTER 11-19: RPOR4: PERIPHERAL PIN SELECT OUTPUT REGISTERS 4, Page 154
}
void InitTimer(void){                                                   //Prescaler = 1:8 and Period = 0xFFFF					
	OpenTimer3 (T3_ON & T3_IDLE_STOP & T3_GATE_OFF & T3_PS_1_8 & T3_SOURCE_INT, 0xFFFF);							
	ConfigIntTimer3 (T3_INT_PRIOR_1 & T3_INT_ON);   //Set interrupt priority and Turn Timer3's interrupt on
}
void InitUART(void){
 	IEC0bits.U1TXIE = 0; 		//Disable UART1 TX Interrupt
    IFS0bits.U1RXIF = 0;		//Clear the Receive Interrupt Flag
	U1MODEbits.STSEL = 0; 		//1 Stop bit
	U1MODEbits.PDSEL = 0;	 	//8-bit data, no parity 
	U1MODEbits.BRGH = 0;		//16x baud clock, Standard mode
	U1MODEbits.URXINV = 0;		//Idle State 1 for RX pin
	U1MODEbits.ABAUD = 0;		//Auto-Baud Disabled
	U1MODEbits.RTSMD = 1;		//Simplex Mode, no flow control
	U1MODEbits.UARTEN = 1; 		//Enable UART1
	U1STAbits.UTXISEL0 = 0; 	//Interrupt after one TX character is transmitted
	U1STAbits.UTXISEL1 = 0; 	//Interrupt after one TX character is transmitted
	U1STAbits.UTXEN = 1; 		//Enable UART1 to control TX pin
	U1BRG = 1;                  //BAUD Rate Setting for 115200
	IEC0bits.U1RXIE = 1;		//Enable UART1 RX interrupt
}
void ADC (void){                                   
    //Read ADC AN0 on pin 2 for left drive siganl
	OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
		ADC_DMA_BUF_LOC_1,
		ENABLE_AN0_ANA,
		0,		
		0,
		0);
                                    //Select AN0
	SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN0);
	AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
	while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
	ONTimeL = ReadADC1(0);           //ONTimeL = converted results
	AD1CON1bits.ADON = 0;           //Turn off ADC hardware module
    
    //Read ADC AN1 on pin 3 for Right Drive Signal
    	OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
		ADC_DMA_BUF_LOC_1,
		ENABLE_AN1_ANA,
		0,		
		0,
		0);
                                    //Select AN1
	SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN1);
	AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
	while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
	ONTimeR = ReadADC1(0);           //ONTimeR = converted results
	AD1CON1bits.ADON = 0;           //Turn off ADC hardware module
    
    //Read ADC AN2 on pin 4 for Arm Position signal
    	OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
		ADC_DMA_BUF_LOC_1,
		ENABLE_AN2_ANA,
		0,		
		0,
		0);
                                    //Select AN2
	SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN2);
	AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
	while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
	ArmPosition = ReadADC1(0);           //ONTimeA = converted results
	AD1CON1bits.ADON = 0;           //Turn off ADC hardware module
}
void Gripper(void){
    //Open gripper if right joystick is pressed
            if (PORTBbits.RB11){Gripper = 1;}
            //Close gripper is left joystick is pressed 
            else if (PORTBbits.RB10){Gripper = 2;}
            //Don't move gripper if nothing is pressed
            else{Gripper = 0;}
}
void ProcessData(void){
    //Send analog pot values
	SendDataArray[1] = (ONTimeL >> 8);       
	SendDataArray[2] = ONTimeL;
    SendDataArray[3] = (ONTimeR >> 8);       
	SendDataArray[4] = ONTimeR;
    SendDataArray[5] = (ArmPosition >> 8);       
	SendDataArray[6] = ArmPosition;
    //Send digital button values as char using gripper variable
    

	//Photocell = (ReceiveDataArray[1] << 8) + ReceiveDataArray[2];   //Build integer from array of bytes

    SendDataArray[20] = 1;                  //Sending a 1 for robot to check for communication (i.e. new data)
    Communicating = ReceiveDataArray[20];   //Checking if the robot sent us a 1, which will indicate communication
    
    if(Communicating){                      //If there is communication, reset the communication loss counter
        CommsLossCount = 0;
    }
    else if(!Communicating){                //If there is an interruption (i.e. single loss of communication),
        CommsLossCount++;                   //then increment the communication loss counter
    }
    ReceiveDataArray[20] = 0;               //Reset the communication to 0, If next time we look at it and it's 
                                            //still 0, then no communication. If next time we look at it and
                                            //robot has changed it to 1, then we have communication
}
void SendData(void){
	for (i=0;i<BUFF_SIZE;i++)           //Index through the array from the start to the end of the array 
	{                                   //Note: The first byte is an ASCII Character "s" to denote the start
		WriteUART1(SendDataArray[i]);	//Send one byte of the data array to UART TX Pin
		while(BusyUART1());             //Wait while the UART1 is busy (sending the last byte)
	}
}
void Shutdown(void){
    //This function is called when there's a communication loss. When communication is lost then the last values
    //received (ReceiveDataArray) will not change, so if any motors are running they will continue to run and may
    //cause problems. Therefore, enter your code to disable/stop anything that could potentially keep running
    
    LATAbits.LATA4 = 1;                 //Turn on communication error LED 
}
void __attribute__((interrupt, auto_psv)) _T3Interrupt(void){
	DisableIntT3;                       //Disable Timer3 Interrupt
	LATBbits.LATB5 = 1 ^ PORTBbits.RB5; //Toggle LED connected to RB5
	WriteTimer3(Photocell*16);          //Pulse rate value from photocell on Robot MCU
	IFS0bits.T3IF = 0;                  //Reset Timer3 interrupt flag
	EnableIntT3;                        //Enable Timer3 interrupt
}
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
     EnableIntU1RX;                                 //Enable the UART1 receive interrupt
}
