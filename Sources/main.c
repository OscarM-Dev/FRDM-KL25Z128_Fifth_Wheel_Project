/*
 * main implementation: use this 'C' sample to create your own application
 *
 */
#include "derivative.h" /* include peripheral declarations */
#include "math.h"
#define SLAVE_ADDR 0x1D     /* 001 1101. */
#define errorn 0
#define error 1
#define ERR_ARB_LOST 0x02
#define ERR_BUS_BUSY 0x03
int readI2C;
int writeI2C;
char datos[6];//lista que almacena la aceleración de los ejes, parte alta y parte baja de xyz
int count;
short salidasejes[3];//lista que almacena ya la salida de los ejes, los 14 bits.
short magnitudesejes[3];
float inclinacion;
unsigned char estabilidad = 1;
unsigned char ninter;
unsigned short altoant = 0;//Variable que captura el tiempo del alto anterior
unsigned short periodo4;//Variable de periodo/4 de la señal de entrada
unsigned long periodo;//Variable de periodo de la señal de entrada
unsigned long periodoant;//variable de periodo anterior
unsigned short frecuencia;//Frecuencia de la señal de entrada
unsigned short frecuencian;
unsigned short nvueltass;//número de vueltas por segundo
unsigned short velocidadm;//Velocidad de motor ms
unsigned short frecuenciar = 0;//frecuencia de referencia (indica la velocidad de referencia)
unsigned short frecuenciarj = 0;//frecuencia de referencia (indica la velocidad de referencia)
unsigned short frecuenciarb = 0;//frecuencia de referencia (indica la velocidad de referencia)
unsigned char paropresionado = 0;
unsigned char paroinductivo = 0;
unsigned char parotemperatura = 0;
unsigned char parobateria = 0;
unsigned char referenciapresionado;
unsigned short altoac;//tiempo actual 
unsigned short rpm;//rpm del motor
unsigned short nvueltasc = 0;//número de vueltas de contador
unsigned short voltajeDAC = 1200;//Voltaje inicial de DAC en mv
unsigned short voltajeDACb;
unsigned char in;

unsigned short valor_senald; //se hace variable global 
unsigned char entradah = 0;
unsigned char entradas[] = {(1<<6) + (1<<3),(1<<6)+ (1<<3) + (1<<0),(1<<6) + (1<<3) + (1<<2),(1<<6) + (1<<3) + (1<<2) + (1<<0)};//PTB0 ea8 (Batería) PTB1 ea9 (Temperatura) PTB2 ea12(Corriente) PTB3 ea13 (joystick)
unsigned short voltajebateria;
unsigned short temp_grados;
unsigned short salidainductivo;
unsigned short voltjoy;//Voltaje de retorno de joystick
unsigned char btm1;
unsigned char btm2;
unsigned char LPTMINT;//número de interrupciones de LPT
//I2C
int I2C0write(unsigned char controlbyte, unsigned char direccion, unsigned char data) 
{

/*Solo para asegurar bus en estado idle	
	int retry = 1000;
    
    while (I2C0_S & (1<<5)) 		{    // wait until bus is available
        if (--retry <= 0)
            return ERR_BUS_BUSY;
        delayUs(100);
    }
*/
	//se envia el start
    I2C0_C1 |= (1<<4);//Se selecciona el modo de transmitir
    I2C0_C1 |= (1<<5);//Se manda señal de start, micro es maestro
    
    //se envia el controlbyte
    I2C0_D = (controlbyte << 1);//Control byte desplazado 1 para tener 0 en el LSB
	while(!(I2C0_S & (1<<1)));//Se espera a que se transfiera el byte
	I2C0_S |= (1<<1);//Se apaga la bandera

    /*
    if (I2C0_S & 0x10) {       // arbitration lost
        I2C0_S |= 0x10;        // clear IF 
        return ERR_ARB_LOST;
    }
    */
    
	if (I2C0_S & 1)//no se recibio ACK del esclavo
		return error;

    //se envia la direccion del registro
    I2C0_D = direccion;
	while(!(I2C0_S & (1<<1)));//Se espera a que se transfiera el byte
	I2C0_S |= (1<<1);//Se apaga la bandera
	if (I2C0_S & 1)//no se recibio ACK del esclavo
		return error;

    // se envia dato
    I2C0_D = data;
	while(!(I2C0_S & (1<<1)));//Se espera a que se transfiera el byte
	I2C0_S |= (1<<1);//Se apaga la bandera
	if (I2C0_S & 1)//no se recibio ACK del esclavo
		return error;

    // se envia stop
    I2C0_C1 &= ~(1<<5);//Se manda señal de stop

    return errorn;
}

/* delay n microseconds
 * The CPU core clock is set to MCGFLLCLK at 41.94 MHz in SystemInit().
 */
void delayUs(int n)
{
    int i; int j;
    for(i = 0 ; i < n; i++) {
        for(j = 0; j < 7; j++) ;
    }
}


int I2C0read(unsigned char controlbyte, unsigned char direccion, int nbytesm, unsigned char* data, int* cnt)
{
unsigned char dummy;

	//se manda start
	I2C0_C1 |= (1<<4);//Se selecciona el modo de transmitir
	I2C0_C1 |= (1<<5);//Se manda señal de start, micro es maestro

    //se manda control byte
    I2C0_D = (controlbyte << 1);	// slaveADDR-0-  0 indica write
	while(!(I2C0_S & (1<<1)));//Se espera a que se transfiera el byte
	I2C0_S |= (1<<1);//Se apaga la bandera
	if (I2C0_S & 1)//no se recibio ACK del esclavo
		return error;

    //se manda direccion
    I2C0_D = direccion;
	while(!(I2C0_S & (1<<1)));//Se espera a que se transfiera el byte
	I2C0_S |= (1<<1);//Se apaga la bandera
	if (I2C0_S & 1)//no se recibio ACK del esclavo
		return error;
    
    /* restart */
	I2C0_C1 |= (1<<2);//Se manda nuevamente señal de startI2C0_C1 |= 0x04;           /* send Restart */

    /* send slave address and read flag */
	I2C0_D = (controlbyte << 1) | 1;//Control byte desplazado 1 para tener 0 en el LSB con un or en 1 para indicar que se va a leer
    //I2C0_D = (slaveAddr << 1) | 1;
	while(!(I2C0_S & (1<<1)));//Se espera a que se transfiera el byte
	I2C0_S |= (1<<1);//Se apaga la bandera
	
	if (I2C0_S & 1)//no se recibio ACK del esclavo
		return error;

    /* change bus direction to read */
	I2C0_C1 &= ~0x18;          /* Tx off, prepare to give ACK */
	//Se configura micro como esclavo y se pone I2C en modo recibir
    if (nbytesm == 1)
        I2C0_C1 |= 0x08;       /* prepare to give NACK */
    	//I2C0_C1 |= (1<<3);//Se prepara a enviar un no ack después del siguiente byte recibido
    dummy = I2C0_D;            /* dummy read to initiate bus read */

    /* read data */
    while (nbytesm  > 0) {
        if (nbytesm  == 1)
        	//Se prepara a enviar un no ack después del siguiente byte recibido
        	I2C0_C1 |= 0x08;       /* prepare to give NACK for last byte */
    	while(!(I2C0_S & (1<<1)));//Se espera a que se transfiera el byte
    	I2C0_S |= (1<<1);//Se apaga la bandera
        if (nbytesm == 1) 
        {
        	//Se manda señal de stop
        	I2C0_C1 &= ~0x20;      /* stop the bus before reading last byte */
        }
        *data++ = I2C0_D;// read received data */
        nbytesm--;
        (*cnt)++;
    }

    return errorn;
}

//interrupciones 
//TPM
void FTM1_IRQHandler()
{	

	
	if ((TPM1_SC & (1<<7))!= 0)//Se se reinicia el contador TOF activada
	{
		TPM1_SC|= (1<<7);//Se apaga la bandera TOF
		nvueltasc ++;//número de vueltas actual + 1
		
		if (nvueltasc >= 10)//La silla esta detenida
		{
			//Se supera el periodo mín de 100ms del periodo más lento del sensor 
			periodo4 = 0;
			periodo = 0;
			frecuencia = 0;//Se calcula la frecuencia del periodo hz
			nvueltass = frecuencia/15;//Se calcula en numero de vueltas por segundo
			velocidadm = nvueltass * .5186;//Se calcula velocidad
			rpm = nvueltass * 60;//Se calculan las rpm
		}
	}
	
	else if ((TPM1_C1SC & (1<<7)) != 0)//Si se interrumpe el canal 1 CHF1 activada
	{
		TPM1_C1SC |=  (1<<7);//Se apaga la bandera CHF
		altoac = TPM1_C1V;
		if(altoac > altoant) periodo4 = altoac - altoant + (nvueltasc * 65536);//Se calcula el periodo/4 en caso de que el alto nuevo pasa después del alto anterior en otra vuelta del contador 
		else if (altoac < altoant) periodo4 = altoac - altoant + ((nvueltasc-1) * 65536);//Se calcula el periodo/4 en caso de que el alto nuevo pasa antes del alto anterior en otra vuelta del contador
		periodo = 4 * periodo4;//Se calcula el periodo real 
		frecuencia = 1000000/periodo;//Se calcula la frecuencia del periodo hz
		nvueltass = frecuencia/15;//Se calcula en numero de vueltas por segundo
		velocidadm = nvueltass * .5186;//Se calcula velocidad en ms
		rpm = nvueltass * 60;//Se calculan las rpm
		altoant = TPM1_C1V;//Se almacena valor en el que se dió el alto anterior
		nvueltasc = 0;
		if ( (frecuencia >= frecuenciar - 2) && (frecuencia <= frecuenciar + 2) ) estabilidad =1;//se llega a la referencia
		else estabilidad = 0;//no se llega a la referencia
		if (btm2 == 1)
		{
			ninter ++;//Se cuentan las interrupciones
			if(ninter >300) ninter = 0;
			if ( (estabilidad == 0) && ( (ninter >= 10) && (ninter <= 60) ) )//impulso 
			{
			    if( (periodo > (periodoant - 100)) )//caso de querer frenar
				{
					frecuenciar = frecuencian;//Frecuencia calculada es la de referencia

				    if(frecuenciar < 15)
				    {
				    	frecuenciar = 0;
				    	frecuencian = 0;
				    }
				}
				if ( (periodo < (periodoant - 100)) )//caso de querer acelerar
				{
					frecuenciar = frecuencian;//Frecuencia calculada es la de referencia
				    if(frecuenciar >= 80)
				    {
				    	frecuenciar = 80;
				    	frecuencian = 80;
				    }
				}
			    ninter = 0;	    
			}
		}
	}
	if(frecuencia < (frecuenciar - 8))
		{
			voltajeDAC = voltajeDAC + 10;// aumenta voltaje en un paso 10mV
			if (voltajeDAC >= 3130) voltajeDAC = 3130;
		}
	if((frecuencia > (frecuenciar + 8)) && (frecuenciar != 0))
		{
			voltajeDAC = voltajeDAC - 10;//se disminuye voltaje en un paso 10mV
			if (voltajeDAC <= 1150) voltajeDAC = 1150;	
		}
	if ( (frecuenciar == 0) && ((btm1 == 1) || (btm2 == 1)) )
	{
		voltajeDAC = voltajeDAC - 10;//se aumenta voltaje en un paso 10mV
		if (voltajeDAC <= 1150) voltajeDAC = 1150;
	}
	
	unsigned short dvalue;//instruccion 12 bits para DAC
	dvalue = (voltajeDAC * 4095) / 3300;//Instrucción de 12 bits a mandar a DAC
	DAC0_DAT0H = (dvalue>>8);//Parte alta delimitada
	DAC0_DAT0L = (dvalue & 0xff);//Parte baja delimitada
	//Se hace conversión de señal a digital
	//Se tiene voltaje deseado en PTE30
	if(btm2 == 1)
	{
		periodoant = periodo;//periodo actual pasa a ser el anterior
		frecuencian = frecuencia;//frecuencia anterior
	}
}



void PORTA_IRQHandler()
{
	if (((PORTA_PCR1 & (1<<24)) != 0))//Interrumpe, boton modo 1
	{
		PORTA_PCR1 |= (1<<24);//Se apaga la bandera
		if(btm1 == 0)
		{
			GPIOC_PCOR = (1<<0);//Se apaga el led verde
			GPIOC_PSOR = (1<<3) + (1<<4);//Se enciende el led rojo y azul
			btm1=1;//Se activa el modo de operación1
			btm2=0;//Se desactiva el modo de operación2	
			ninter = 0;
			PORTA_PCR12 |= (1<<19) + (1<<17);//Se habilita la interrupción en flancos de bajada para el botón de velocidad constante
			voltajeDAC = 1150;
			frecuenciarj = 0;//Se limpia la variable
			frecuenciarb = 0;//Se limpia la variable
			frecuenciar = 0;//Se limpia la variable
			frecuencian = 0;//Se limpia la variable
		}
		else 
		{
			GPIOC_PCOR = (1<<4) + (1<<3) + (1<<0);//Se apaga led rgb
			btm1 = 0;//Se desactiva el modo de operación 1
			PORTA_PCR12 &=~(1<<19) + (1<<17);//Se desactiva la interrupción de botón de velocidad constante
			//Se desactiva entrada del joystick en ADC
			voltajeDAC = 1200;
			frecuenciarj = 0;//Se limpia la variable
			frecuenciarb = 0;//Se limpia la variable
			frecuenciar = 0;//Se limpia la variable
			frecuencian = 0;//Se limpia la variable
		}
	}
	
	else if (((PORTA_PCR2 & (1<<24)) != 0))//interrumpe boton modo 2
	{
		PORTA_PCR2 |= (1<<24);//Se apaga la bandera
		if(btm2 == 0)
		{
			GPIOC_PCOR = (1<<3);//Se apaga el led rojo
			GPIOC_PSOR = (1<<4) + (1<<0);//Se enciende el led azul y verde
			btm2=1;//Se activa el modo de operación 2
			btm1=0;//Se desactiva el modo de operación 1//desactiva la entrada de joystick en ADC
			PORTA_PCR12 &=~(1<<19) + (1<<17);//Se desactiva la interrupción de botón de velocidad constante
			frecuenciarj = 0;//Se limpia la variable
			frecuenciarb = 0;//Se limpia la variable
			frecuenciar = 0;//Se limpia la variable
			frecuencian = 0;//Se limpia la variable
			voltajeDAC = 1150;
		}
		else 
		{
			btm2 = 0;//Se desactiva el modo de operación 2
			ninter = 0;
			GPIOC_PCOR = (1<<4) + (1<<3) + (1<<0);//Se apaga led rgb
			voltajeDAC = 1200;
			frecuenciarj = 0;//Se limpia la variable
			frecuenciarb = 0;//Se limpia la variable
			frecuenciar = 0;//Se limpia la variable
			frecuencian = 0;//Se limpia la variable
		}
	}
	else if (((PORTA_PCR12 & (1<<24)) != 0))//interrumpe botón de velocidad constante
	{
		PORTA_PCR12 |= (1<<24);//Se apaga la bandera
		frecuenciarb = frecuencia;//Se mantiene la frecuencia de joystick
	}
	else if (((PORTA_PCR5 & (1<<24)) != 0))//interrumpe botón de paro
	{
		PORTA_PCR5 |= (1<<24);//Se apaga la bandera
		if (paropresionado == 0)paropresionado = 1;
		else paropresionado = 0;
	}
}
//LPTM
void LPTimer_IRQHandler()//ISR de LPT
{
	LPTMR0_CSR |= (1<<7);//se apaga la bandera de LPT , se deja activada la interrupción de LPT y se deja activado el contador
	//Se reinicia el contador
	ADC0_SC1A =  entradas[entradah] ;//Se habilita la interrupción local de ADC y la entrada analogica i
	//Se inicia la conversión
	
	

	//Se lee la salida del acelerometro cada 100ms

	unsigned char i=0;
	unsigned short msb=0;
	writeI2C = I2C0read(SLAVE_ADDR, 0x01, 6, datos, &count);//Se leen los datos de los registos de la aceleración en los ejes
	 //Se almacenan en la lista de datos tanto la parte alta como la baja
	    
	 //se juntan los 14 bits de los registros para cada salida
	salidasejes[0]=((datos[0]<<8)|(datos[1]))>>2;//salida x
	salidasejes[1]=((datos[2]<<8)|(datos[3]))>>2;//salida y
	salidasejes[2]=((datos[4]<<8)|(datos[5]))>>2;//salida z
	i=0;
	while (i<3) 
	{
		msb=salidasejes[i]>>13;
		if ((msb==1))
		{
			magnitudesejes[i]=-4096+(salidasejes[i] &= ~1<<13);
		}
		else
		{
			magnitudesejes[i]=salidasejes[i];
		}
		i++;	
	}	
	inclinacion=90-(atan2(magnitudesejes[2],magnitudesejes[0])*18000/314);
	
	//Sistema de seguridad
	if((paropresionado == 1) ||(paroinductivo ==1) || (parotemperatura == 1) || (parobateria == 1))
	{
		//en caso de tener el boton presionado
		//en caso de estar en una posición de riesgo
		//en caso de tener una temperatura elevada
		//en caso de tener una carga de batería crítica
		//se pone en paro el sistema
		
		GPIOC_PSOR = (1<<7);//Se enciende led del botón paro
		frecuenciarj = 0;
		frecuenciarb = 0;
		frecuenciar = 0;
		frecuencian = 0;
		voltajeDAC = 1150;
	}
	else if ((paropresionado == 0) && (paroinductivo == 0) && (parotemperatura == 0) && (parobateria == 0))
	{
		GPIOC_PCOR = (1<<7);//Se apaga el led del boton paro
	}
	
	//Sistema de emergencia buzzer

	if (((temp_grados >= 50) || (voltajebateria <= 37000) || (inclinacion>30)) && (LPTMINT == 10))//Se tiene temperatura alta o carga critica o silla esta inclinada en una posición de riesgo
    //Se activa el buzzer cada s //inclinación menor que 150
	//Misma frecuencia para ambos casos
	{
		GPIOE_PTOR = (1<<2);//Se prende/apaga buzzer
		LPTMINT = 0;//Se reinicia contador de interrupciones
	}
	else if((temp_grados < 50) && (voltajebateria > 1300) && (inclinacion<30)) GPIOE_PCOR = (1<<2);// operación segura //Se desactiva buzzer en caso de estar activado
	else if (LPTMINT > 10)LPTMINT = 0;//Se reinicia contador de interrupciones en caso de superar 10
	LPTMINT ++;//Se cuentan las interrupciones

}
//ADC
void ADC0_IRQHandler()//ISR de ADC
{
	if (entradah == 0)//Entrada analogica 8 habilitada (PTB0) // voltaje batería
	{
		voltajebateria = (ADC0_RA * 42000)/224;//voltaje de batería en mV
		if(voltajebateria >= 40000)//Voltaje de batería alto 
		{
			GPIOE_PCOR = (1<<5) + (1<<3);//Apagan led de carga media y baja
			GPIOE_PSOR = (1<<4);//enciende el led verde de carga alta
			parobateria = 0;
		}
		if((voltajebateria >= 38000 ) && (voltajebateria < 40000))//Voltaje de batería medio 
		{
			GPIOE_PCOR = (1<<3);//Apagan led azul
			GPIOE_PSOR = (1<<5) + (1<<4);//enciende el led amarillo de carga media, rojo y verde
			parobateria = 0;
		}
		else if (voltajebateria < 38000)//voltaje de batería bajo
		{
			GPIOE_PCOR = (1<<4) + (1<<3);//Apagan led de carga alta y media, verde y azul
			GPIOE_PSOR = (1<<5);//enciende el led rojo de carga baja
			if (voltajebateria <= 36500) parobateria = 1;//Se enciende el paro del sistema por carga crítica
			else parobateria = 0;
		}
		entradah = 1;//próxima entrada a habilitar
	}
	else if (entradah == 1)//Entrada analogica 9 habilitada (PTB1)//temperatura
	{
		temp_grados = ((((ADC0_RA * 3300) / 255) / 2)) * 0.1;//temperatura en grados
		if (temp_grados >= 50)//temperatura de riesgo para operación
		{
			GPIOB_PSOR = (1<<11);//Se prende led de temperatura elevada
			parotemperatura = 1;
		}
		else
		{
			GPIOB_PCOR = (1<<11);//Se apaga led de temperatura elevada//temperatura segura para operación
			parotemperatura = 0;
		}
		entradah = 2;//próxima entrada a habilitar
	}
	else if (entradah == 2)//Entrada analogica 12 habilitada (PTB2) //sensores inductivos
	{
		salidainductivo = (ADC0_RA * 3300)/255;
		if (salidainductivo >= 2000) paroinductivo = 1;	//sensores inductivos no detectan placa
		//angulo de posición de rueda riesgoso
		else paroinductivo = 0;	//sensores inductivos detectan placa
		//angulo de posición de rueda seguro
		if(btm1 == 1)entradah = 3;//próxima entrada a habilitar//si esta presionado el botón de modo 1 se habilita entrada de joystick
		else entradah = 0;
	}
	else if (entradah == 3)//Entrada analogica 13 habilitada (PTB3)
	{
		voltjoy = (ADC0_RA * 3300)/255;//Voltaje de joystick en mV
		
		if (frecuenciarb == 0)//boton de velocidad constante desactivado
		//posición de reposo joystick es 0hz
		{
			if ((voltjoy >= 1640) && (voltjoy <= 2000)) frecuenciarj = 0;//el motor esta detenido
			else if (voltjoy > 2000)//el motor avanza
			//Se habilitan 90 grados del joystick en el eje positivo
			{
				frecuenciarj = ((((voltjoy - 2000) * (155-0)) / (3300 - 2000)) + 0) * .78;//	Frecuencia de referecnia del joystick conforme al voltaje , regresión lineal
			}
			else frecuenciarj = 0;
		}
		
		else //boton de velocidad constante activado
		//posición de reposo joysitick es la es la frecuencia de referencia de la ultima posición del joystick
		{
			if ((voltjoy >= 1640) && (voltjoy <= 2000)) frecuenciarj = frecuenciarb;//el motor avanza a la frecuencia de referencia del botón
			//la velocidad se mantiene constante
			else if (voltjoy > 2000)//el motor incrementa su velocidad
			//Se habilitan 90 grados del joystick en el eje positivo
			{
				frecuenciarj = ((((voltjoy - 2000) * (155-frecuenciarb)) / (3300 - 2000)) + frecuenciarb)* .78;//	Frecuencia de referecnia del joystick conforme al voltaje , regresión lineal
			}
			else // el motor disminuye la velocidad
			{
				frecuenciarj = ((((voltjoy - 0) * (frecuenciarb-0)) / (1640 - 350)) + 0)*.78;//	Frecuencia de referecnia del joystick conforme al voltaje , regresión lineal
				if (frecuenciarj <= 18) frecuenciarj = 0;
			}
			
		}
		
		if (frecuenciarj >= 155) frecuenciarj = 155;
		else if(frecuenciarj <= 0)frecuenciarj = 0;
		
		if((frecuenciarj == 0))frecuenciar = 0;
		else if (((frecuenciarj !=0) && (paropresionado != 1))) frecuenciar = frecuenciarj;
		entradah = 0;//Próxima entrada a habilitar
	}
	
}

//Configuraciones 
void clk_init (void) //Se configura el reloj del módulo a 4 MHZ 250 ns
{
	// FIRC = 4 MHz. BusClk = 4 MHz// UART0: FIRC. UART1: BusClk. UART2: BusClk. TPM: FIRC. IIC: BusClk
	MCG_C1|=(1<<6) + (1<<1);//MCGOUTCLK : IRCLK. CG: Clock gate, MCGIRCLK enable pag 116
	MCG_C2|=1;//Mux IRCLK : FIRC (4 MHz) pag 116
	MCG_SC=0;//Preescaler FIRC 1:1 pag 116
	SIM_CLKDIV1=0;//OUTDIV4=OUTDIV1= 1:1 pag 116. Busclk 4 MHz
	SIM_SOPT2|=15<<24;//Seleccion MCGIRCLK tanto para UART0 como paraTPM
}
//Configuración TPM1
void conpinTPM1()
{
	//Configuración PTE21 (INPUT CAPTURE)
	SIM_SCGC5|= (1<<13);//Se habilita reloj de puerto e
	PORTE_PCR21 = (1<<9) + (1<<8);//Se configura pte21 como TPM1 canal 1
	
}
void conTPM1()
{
	//Configuración TPM1
	SIM_SCGC6 |= (1<<25);//Se habilita reloj del TPM1
	TPM1_SC |= (1<<6) + (1<<3) + (1<<2);// se habilita interrupción local de TOF, Se configura contador a .25MHZ, se selecciona el reloj del modulo (4MHZ) y un prescaler de 16
	//contador cuenta cada 4us
}
void coninputcTPM1()
{
	//Configuración input capture
	TPM1_C1SC |=  (1<<6) + (1<<2);//Se configura el canal 1 como input capture, capturamos la señal en alto y se habilita la interrupción local
	NVIC_ISER |= (1<<18);//Se habilita la interrupción global 
	
}

void conDAC()
{
	//Se configura DAC
	SIM_SCGC6 |= (1<<31);//Se habilita el reloj del DAC0
	DAC0_C0 = (1<<7) + (1<<5);//Activamos DAC y trigger
}
void conADC()
{
	//Se configura ADC, se utiliza PTB0 que por default es ADC
	SIM_SCGC6 |= (1<<27);//Se activa el reloj del ADC
	NVIC_ISER |= (1<<15);//Se habilitan las interrupciones de ADC dentro del NVIC
}
void conLPT()
{
	//Se configura el LPT
	SIM_SCGC5 |= (1<<0);//Se activa el reloj de LPT
	LPTMR0_PSR = (1<<2) + (1<<0);//Se activa el bypass y se seleciona LPO del MUX
	LPTMR0_CSR |= (1<<6) + (1<<0);//Se activa la interrupción del LPT y y se activa el contador
	NVIC_ISER |= (1<<28);//Se habilita las interrupciones de LPT dentro del NVIC
	LPTMR0_CMR = (99); //Se configura el valor de referencia en ms
	//cada entrada analogica se habilita cada 400ms
}
void conpinGPIO()
{
	//Interrupciones de botones
	SIM_SCGC5|= (1<<9);//Se habilita reloj de puerto A
	
	//Configuración PTA1, boton modo de operación 1
	PORTA_PCR1 = (1<<19) + (1<<16) + (1<<8);//Se configura PTA1 como GPIO, se habilita interrupción local en flancos de subida
	
	//Configuración PTA2, boton modo de operación 2
	PORTA_PCR2 = (1<<19) + (1<<16) + (1<<8);//Se configura PTA2 como GPIO, se habilita interrupción local en flancos de subida
	//por default los pines son entradas
	
	//Configuración PTA12 botón de velocidad constante
	PORTA_PCR12 = (1<<8);//Se configura PTA12 como GPIO
	//por default los pines son entradas
	
	//Se configuta PTA4 botón de paro
	PORTA_PCR5 = (1<<19) + (1<<16) + (1<<8);//Se configura PTA5 como GPIO, se habilita interrupción local en flancos de subida
	//por default los pines son entradas
			
	NVIC_ISER |= (1<<30);//Se habilita la interrupción global del puerto A
	
	SIM_SCGC5 |= (1<<11);//Se habilita el reloj del puerto C
	PORTC_PCR7 = (1<<8);//Se configura pin como GPIO para alimetar led de boton paro
	//botones 
	PORTC_PCR4 = (1<<8);//Se configura azul
	PORTC_PCR0 = (1<<8);//Se configura verde
	PORTC_PCR3 = (1<<8);//Se configura rojo
	GPIOC_PDDR = (1<<7) + (1<<4)+ (1<<3) + (1<<0);//Se configuran los pines como salidas
	
	SIM_SCGC5 |= (1<<13);//Se habilita el reloj del puerto E
	PORTE_PCR5 = (1<<8);//Se configura pin como GPIO para alimentar led de carga baja rojo
	PORTE_PCR4 = (1<<8);//Se configura pin como GPIO para alimentar led de carga alta verde
	PORTE_PCR3 = (1<<8);//Se configura pin como GPIO para alimentar led azul
	PORTE_PCR2 = (1<<8);//Se configura pin como GPIO para activar buzzer
	GPIOE_PDDR = (1<<5) + (1<<4) + (1<<3) + (1<<2);//Se configuran los pines como salidas
	
	SIM_SCGC5 |= (1<<10);//Se habilita el reloj del puerto B
	PORTB_PCR11 = (1<<8);//Se configura pin como GPIO para alimentar led de temperatura alta
	GPIOB_PDDR = (1<<11);//Se configura pin como salida
}
void conpinI2C0()
{
	SIM_SCGC5 |= (1<<13);//Se habilita el reloj del puerto E
	PORTE_PCR24 = (1<<10) + (1<<8);//Se configura el pin como SCL
	PORTE_PCR25 = (1<<10) + (1<<8);//Se configura el pin como SDA
}
void conI2C0()
{
	SIM_SCGC4 |= (1<<6);//Se habilita el reloj del I2C0
	I2C0_F =  (1<<4) +  (1<<3) + (1<<2);//0x1C; //(1<<6) + 21;  //0x1C;             /* set clock to 97.09KHz @13.981MHz bus clock */ //OJO registro muy importante
	I2C0_C1 = (1<<7);//Se habilita el modulo de I2C0
}
int main(void)                                                                  
{
	conDAC();//Configuracion DAC
	conpinGPIO();//Configuracion pines GPIO
	clk_init();//Se llama función de reloj de 4MHZ  
	conpinI2C0();//Se configuran pines para I2C 
	conI2C0();//Se configura I2C0
	readI2C = I2C0write(SLAVE_ADDR,0x2A,1);//Se escribe en el registro de activación del acelerometro
	conpinTPM1();//Configuracion pin TPM1
	conTPM1();//Configuracion TPM1
	coninputcTPM1();//Configuracion Input capture
	conADC();//Configuración ADC
	conLPT();//Configuración LPT
	for(;;) {	   
	}
	
	return 0;
}
