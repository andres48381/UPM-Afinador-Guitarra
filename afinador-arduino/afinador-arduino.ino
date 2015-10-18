/////////////////////////////////////////////////////////////////////////////////////
///////////////// AFINADOR DE GUITARRA. Trabajo de Curso              ///////////////
///////////////// Alumno: ANDRES BARROSO GARCIA                       /////////////// 
///////////////// NºMatricula: 2053                                   ///////////////
///////////////// Asignatura: Microcontroladores y logica programable ///////////////
///////////////// MASTER ELECTROMECNICA. Itinerario MECATRONICA       ///////////////
///////////////// Curso 2012-2013                                    ////////////////
/////////////////////////////////////////////////////////////////////////////////////
 
#define LIN_OUT 1 // Salida modo logarítmica
#define FHT_N 256 // Número de barras que conforman la transformada de Fourier

#include <FHT.h> // include the library

class NOTAS//Cada nota de la escala musical es un objeto de la clase NOTAS
{
  public:
  unsigned long frec;//Frecuencia de la nota
  unsigned long frec_low;//Umbral superior de frecuencia en el que se mantiene la nota
  unsigned long frec_high;//Umbral inferior de frecuencia en el que se mantiene la nota
  char letra;//Nomenclatura de la nota
  
  NOTAS(char n,unsigned long f);
};


NOTAS::NOTAS(char n,unsigned long f){
    letra=n;
    frec=f;
    frec_low=0;//Requiere calcularse a partir de la frecuencia de la nota consecutiva
    frec_high=0;
}
// Sample rate in samples per second.
// This will be rounded so the sample interval
// is a multiple of the ADC clock period.
const uint32_t SAMPLE_RATE = 1024;//Frecuencia de muestreo (Hz) --> 1024Hz/256barras= 4Hz/barra
// Desired sample interval in CPU cycles (will be adjusted to ADC/timer1 period)
const uint32_t SAMPLE_INTERVAL = F_CPU/SAMPLE_RATE;
// Minimum ADC clock cycles per sample interval
const uint16_t MIN_ADC_CYCLES = 15;
const uint8_t ANALOG_PIN = 0;//Pin de INPUT para la lectura de datos
// Reference voltage
// Zero - use External Reference AREF
// (1 << REFS0) - Use Vcc
// (1 << REFS1) | (1 << REFS0) - use Internal 1.1V Reference
uint8_t const ADC_REF_AVCC = (1 << REFS0);

void adcInit(uint32_t ticks, uint8_t pin, uint8_t ref);//Función de inicialización para muestreo. adcInit(FREC. MUESTREO, PIN DE ENTRADA, V referencia)
void adcStart();//Función de activación de la conversión analógico-digital
void adcEnd();//Función de activación de la conversión analógico-digital
uint16_t acquire();//Función de aplicación de FHT 
uint16_t findMax(uint16_t arr[], int n);//Función de busqueda del máximo valor del vector de frecuencias

void wellcome();//Función de bienvendia a la aplicación
void notas(char );//Función de representación de notas en el DISPLAY. P.ej: notas("A")--> El display muestra una "A", la nota "LA" según la nomenclatura
void numero(int);//Función de representación de digitos en el DISPLAY. P.ej: numero(1)--> El display muestra un "1", la primera cuerda de la guitarra
void apagado();//Función de apagado de led´s
void afinacion(char );//Función de afinación. P.ej: afinación('e'). Se realizan las comparaciones en base a la frecuencia del MI
void capture();//Función de promedio y asignación de la frecuencia


unsigned long StartAcq = 0;//Variable control inicio adquisición
unsigned long EndAcq = 0;//Variable control fin adquisición
unsigned long FftCount = 0;//Variable contador de transformadas realizadas
unsigned long SampFreqAvg = 0;//Frecuencia promedio de muestreo
unsigned long AvgFreq = 0;//Frecuencia promedio calculada
unsigned long NumFreq = 0;//Identificador de frecuencia calculada

unsigned long frecuencia=0;//Variable para la representación de la frecuencia promedio calculada

NOTAS *scale[37];//3 escalas (12 notas x 3 escalas=36 notas)más el elemento inicial

int BufPtr = 0;//Buffer

boolean TogState = false; //Variable auxiliar para la contabilización de FTH realizadas
boolean ErrorCond = false;//Variable de estado de ERROR

int VERDE=2,PUSH=3,AMARILLO=4,ROJO=5,a=6,b=7,c=8,d=9,e=10,f=11,g=12,p=13;//Pines de salida para led's
int value,FLANCO=0;//Variables de control del pulsador
int cuerda=1;//Variable indicadora cuerda para afinar
unsigned long tempo,tempo_2,tempo_3,tempo_4;//Variables de control para temporizadores
char ESTADO[10];//Variable de control etapas {INICIO,SELECT,LISTEN}
int marcador=0;//Variables "marcador" auxiliares
int marcador_2=0;
int marcador_3=0;
int marcador_4=0;
int marcador_5=1;
int parpadeo=1;



void setup(){

  Serial.begin(115200); // use the serial port
  Serial.println("*********AFINADOR DE GUITARRA********");
  adcInit(SAMPLE_INTERVAL, ANALOG_PIN, ADC_REF_AVCC);
  
  pinMode(a,OUTPUT);//LED's del DISPLAY 7 Segmentos
  pinMode(b,OUTPUT);
  pinMode(c,OUTPUT);
  pinMode(d,OUTPUT);
  pinMode(e,OUTPUT);
  pinMode(f,OUTPUT);
  pinMode(g,OUTPUT);
  pinMode(p,OUTPUT);
  pinMode(ROJO,OUTPUT);//LED's de colores indicadores de acciones
  pinMode(AMARILLO,OUTPUT);
  pinMode(VERDE,OUTPUT);

  pinMode(PUSH,INPUT);//Pulsador
  
  scale[0]=new NOTAS(0,0);//Objeto neutro inicial
  int i=0,j=0;//Variables auxiliares
  while(j<3)//Rellena el vector con las escalas 2º,3º y 4º. Rango de frecuencia: C2=65Hz-B4=493Hz
  {
    // (Frecuencia nota "i" escala "j" / Frecuencia nota "i" escala "j+1") = 2
    // La frecuencia de una nota en su escala posterior se duplica
    
    scale[i+1]=new NOTAS('c',65*pow(2,j));//Nota DO, DO 2ºescala=65Hz; 3ºescala=130Hz; 4ºescala=260Hz
    scale[i+2]=new NOTAS('C',69*pow(2,j));//DO# (#->sostendio)
    scale[i+3]=new NOTAS('d',73*pow(2,j));//RE
    scale[i+4]=new NOTAS('D',78*pow(2,j));//RE#
    scale[i+5]=new NOTAS('e',82*pow(2,j));//MI
    scale[i+6]=new NOTAS('f',87*pow(2,j));//FA
    scale[i+7]=new NOTAS('F',92*pow(2,j));//FA#
    scale[i+8]=new NOTAS('g',98*pow(2,j));//SOL
    scale[i+9]=new NOTAS('G',104*pow(2,j));//SOL#
    scale[i+10]=new NOTAS('a',110*pow(2,j));//LA
    scale[i+11]=new NOTAS('A',116*pow(2,j));//LA#
    scale[i+12]=new NOTAS('b',123*pow(2,j));//SI
    i+=12;//Incrementa el orden de la escala
    j++;
  }
  
  for(int i=1;i<37;i++){//Cálculo de los limites de frecuencia de cada nota. Mitad del intervalo entre dos notas
   scale[i]->frec_low=scale[i]->frec-(scale[i]->frec-scale[i-1]->frec)/2;
   scale[i-1]->frec_high=scale[i]->frec_low;
  } 
  
  strcpy(ESTADO,"INICIO");//Copia en la variable de etapas el estado INICIO
}


void loop()
{ 
  if(strcmp(ESTADO,"INICIO")==0){//////////////**** MODO INICIO *****////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////  
    wellcome(); //IluminaciÓn de inicio
    strcpy(ESTADO,"SELECT");//Copia en la variable de etapas el estado SELECT
  }
//-------------------FIN MODO INICIO----------------------------------------------
//-------------------------------------------------------------------------------------  
  
  value=digitalRead(PUSH);//Lectura del estado del PULSADOR
  if(value&&marcador_2==0)marcador_2=1;//Flanco subida
  if(marcador_2&&value==0){//Flanco bajada
    FLANCO=1;//Variable de detección para flancos de bajada
    marcador_2=0;//Reseteo de variable auxiliar
  }
  

  if(strcmp(ESTADO,"SELECT")==0){//////////////**** MODO SELECCION *****////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////   
    numero(cuerda);//Representa en el Display el numero de cuerda actual
    
    if(FLANCO){//Si existe un flanco en el puslador...
       cuerda++;//Incrementa el número de cuerda
       FLANCO=0;
      }
    if(cuerda>6)cuerda=1;//Si excede el número de cuerda de 6 reinicia a 1
    
   //CONTROL DE PULSADOR PRESIONADO
    if(value&&marcador==0){//Si el pulsador entrega HIGH...
      tempo=millis();//Fija origen de temporización
      marcador=1; //Activa temporizador
      }
    if(value==0)marcador=0;//Reseteo si pulsador baja a LOW
    if(marcador){//Si el pulsador se encuentra a nivel HIGH
      tempo_2=millis()-tempo;//Contabiliza la duración del nivel HIGH
      if(tempo_2>=1000){//Si supera 1seg...
        marcador=0;
        strcpy(ESTADO,"LISTEN");//Copia en la variable de etapas el estado LISTEN
        }
      }
    } 
    
//-------------------FIN MODO SELECCION----------------------------------------------
//-------------------------------------------------------------------------------------
    
   if(strcmp(ESTADO,"LISTEN")==0){//////////////**** MODO AFINACION *****////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
    
    if(FLANCO){//Evita salirse con el Flanco del pulsador al iniciar modo LISTEN
     FLANCO=0;
     marcador_3++;  
    }      
    
    //Código de ESCUCHAR PARA AFINAR
    capture();//Llamada a las funciones de lectura de datos, transformada de fourier y asignación de FRECUENCIA   
    //El valor de la frecuencia se mueve a la variable global "frecuencia"
    
    switch(cuerda){//Detección de cuerda seleccionada
    case 1: afinacion('e');break;//Llamada a la función de comparación de frecuencias. afinacion("MI").El valor de frecuencia se compara con las frecuencias de MI
    case 2: afinacion('b');break;
    case 3: afinacion('g');break;
    case 4: afinacion('d');break;
    case 5: afinacion('a');break;
    case 6: afinacion('e');break;    
    }
        
    if(frecuencia==0){//Si no hay señal de entrada genera parpadeo de luz
      if(marcador_4==0){
        tempo=millis();//Inicializa temporizador
        marcador_4=1;
      }
      if(marcador_4&&(millis()-tempo>500)){//Si transcurren mas de medio segundo
        marcador_4=0;//Reinicia contador
        marcador_5=marcador_5*(-1);//Alterna el estado de parpadeo
        if(marcador_5==1){//Encendido luz
          digitalWrite(VERDE,HIGH);
        }
        else apagado();//Apagado luces
      }
    } 
    if(marcador_3==2){//Si se detecta un segundo flanco...(el primero sucede al entrar en modo LISTEN tras la presión de 1 seg)
      strcpy(ESTADO,"SELECT");//Copia en la variable de etapas el estado SELECT
      marcador_3=0;//Reinicio contaje de flancos
      apagado();//Apagado luces
      adcEnd();
    }
   }
//-------------------FIN MODO LISTEN----------------------------------------------
//-------------------------------------------------------------------------------------

}

uint16_t acquire()
{
  uint16_t fs, binNum, freq;
  
  adcStart();//Inicio del ADC
  BufPtr = 0;
  
  StartAcq = micros();//Inicialización inicio muestreo
  while(BufPtr < FHT_N)
  {
    millis();
  }
  EndAcq = micros();//Finalización tiempo muestreo
  
  FftCount++;//Incremento de contador de transformadas realizadas
  SampFreqAvg = (SampFreqAvg * (FftCount - 1) + (EndAcq - StartAcq)) / FftCount;//Cálculo de promedio de frecuencia muestreo real
  
  fs = 1.0 / ((double)(EndAcq - StartAcq) / 1000000.0 / FHT_N);//Cálculo ancho de frecuencia real de cada bin (barra)
  
  // process data
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  fht_mag_lin(); // take the output of the fht

  binNum = findMax(fht_lin_out, FHT_N/2);//Busqueda de la barra de frecuencia mayor (frecuencia fundamental)
  freq = binNum * (fs / FHT_N);//Asignación de la frecuencia correspondiente a la barra mayor
  
  return freq;
}


uint16_t findMax(uint16_t arr[], int n)
{
  uint16_t m = 0;
  uint16_t val = 0;
  for (int i = 0; i < n; i++)//Algoritmo busqueda elemento mayor
  {
    if (arr[i] > val)
    {
      m = i;
      val = arr[i];
    }
  }
  return m;
}

void adcInit(uint32_t ticks, uint8_t pin, uint8_t ref) {
  if (ref & ~((1 << REFS0) | (1 << REFS1))) {
    //error("Invalid ADC reference bits");
    ErrorCond = true;
    return;
  }
  // Set ADC reference andlow three bits of analog pin number
  ADMUX = ref | (pin & 7);
#if RECORD_EIGHT_BITS
  // Left adjust ADC result to allow easy 8 bit reading
  ADMUX |= (1 << ADLAR);
#endif  // RECORD_EIGHT_BITS
  
 // trigger on timer/counter 1 compare match B
  ADCSRB = (1 << ADTS2) | (1 << ADTS0);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // the MUX5 bit of ADCSRB selects whether we're reading from channels
  // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
  if (pin < 8) {
    ADCSRB &= ~(1 << MUX5);
    // disable Digital input buffer
    DIDR0 |= 1 << pin;
  } else {
    ADCSRB |= (1 << MUX5);
    // disable Digital input buffer
    DIDR2 |= 1 << (7 & pin);
  }
#else  // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // not a Mega disable Digital input buffer
  if (pin < 6) DIDR0 |= 1 << pin;
#endif  // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#if ADPS0 != 0 || ADPS1 != 1 || ADPS2 != 2
#error unexpected ADC prescaler bits
#endif

  uint8_t adps;  // prescaler bits for ADCSRA
  for (adps = 7; adps > 0; adps--) {
   if (ticks >= (MIN_ADC_CYCLES << adps)) break;
  }
  if (adps < 3)
  {
    Serial.println("Sample Rate Too High");
    ErrorCond = true;
    return;
  }
  
  Serial.print("ADC clock MHz: ");
  Serial.println((F_CPU >> adps)*1.0e-6, 3);

  // set ADC prescaler
  ADCSRA = adps;
  
  // round so interval is multiple of ADC clock
  ticks >>= adps;
  ticks <<= adps;
  
  // Setup timer1
  // no pwm
  TCCR1A = 0;
  
  uint8_t tshift;
  if (ticks < 0X10000) {
  // no prescale, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    tshift = 0;
  } else if (ticks < 0X10000*8) {
    // prescale 8, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    tshift = 3;
  } else if (ticks < 0X10000*64) {
    // prescale 64, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    tshift = 6;
  } else if (ticks < 0X10000*256) {
    // prescale 256, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
    tshift = 8;
  } else if (ticks < 0X10000*1024) {
    // prescale 1024, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10);
    tshift = 10;
  } else {
    Serial.println("Sample Rate Too Slow");
    ErrorCond = true;
    return;
  }
  // divide by prescaler
  ticks >>= tshift;
  // set TOP for timer reset
  ICR1 = ticks - 1;
  // compare for ADC start
  OCR1B = 0;
  
  // multiply by prescaler
  ticks <<= tshift;
  Serial.print("Sample interval usec: ");
  Serial.println(ticks*1000000.0/F_CPU);
  Serial.print("Frecuencia de muestreo (Hz): ");
  Serial.println((float)F_CPU/ticks);
}


void adcStart() {
  // Enable ADC, Auto trigger mode, Enable ADC Interrupt, Start A2D Conversions
  ADCSRA |= (1 << ADATE)  |(1 << ADEN) | (1 << ADIE) | (1 << ADSC) ;
  // enable timer1 interrupts
  TIMSK1 = (1 <<OCIE1B);
  TCNT1 = 0;
}

void adcEnd(){
  // Desactiva la conversion ADC
  ADCSRA |= (0 << ADATE)  |(0 << ADEN) | (0 << ADIE) | (0 << ADSC) ;
  TIMSK1 = (0 <<OCIE1B);
}

// ADC done interrupt
ISR(ADC_vect) {
  // read ADC
#if RECORD_EIGHT_BITS
  uint8_t d = ADCH;
#else  // RECORD_EIGHT_BITS
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  uint16_t d = (high << 8) | low;
#endif  // RECORD_EIGHT_BITS
  
  int k = d - 0x0200; // form into a signed int
  k <<= 6; // form into a 16b signed int
  
  // Only write to the buffer if it's not full.
  if (BufPtr < FHT_N)
  {
    fht_input[BufPtr] = k;
  }
  
  BufPtr++;
}

void capture(){

if (ErrorCond)
  {
    Serial.println("Error");
    return;
  }
  
  uint16_t freq = acquire();//Adquisición del valor de frecuencia calculado
  NumFreq++;//Incrementa el contador de frecuencias tomadas
  AvgFreq = (AvgFreq * (NumFreq - 1) + freq) / NumFreq;//Cálculo de frecuencia promedio
  
  if ((FftCount / 4) % 2 != TogState)//Tras 4 FFT realizadas envia el valor de frecuencia
  {
    frecuencia=AvgFreq;//Copia el valor en la variable final de comparación en la afinación
  /*Serial.print("Numero de FHT's computadas: ");
    Serial.println(FftCount);
    Serial.print("Frecuencia promedio: ");
    Serial.println(AvgFreq);
    Serial.print("Average sampling interval (us): ");
    Serial.println(SampFreqAvg);*/
    TogState = (FftCount / 4) % 2;
    NumFreq = 0;//Reinicio contador frecuencias
  }
}

void wellcome(){
 digitalWrite(a,HIGH);
 delay(300);
 digitalWrite(b,HIGH);
 delay(300);
 digitalWrite(c,HIGH);
 delay(300);
 digitalWrite(d,HIGH);
 delay(300);
 digitalWrite(e,HIGH);
 delay(300);
 digitalWrite(f,HIGH);
 delay(300);
 digitalWrite(g,HIGH);
 digitalWrite(ROJO,HIGH);
 digitalWrite(AMARILLO,HIGH);
 digitalWrite(VERDE,HIGH);  
 delay(600);
 digitalWrite(ROJO,LOW);
 digitalWrite(AMARILLO,LOW); 
 digitalWrite(VERDE,LOW);  
 digitalWrite(a,LOW);digitalWrite(b,LOW);digitalWrite(c,LOW);digitalWrite(d,LOW);
 digitalWrite(e,LOW);digitalWrite(f,LOW);digitalWrite(g,LOW);   
}

void apagado(){
  digitalWrite(a,LOW);
  digitalWrite(b,LOW);
  digitalWrite(c,LOW);
  digitalWrite(d,LOW);
  digitalWrite(e,LOW);
  digitalWrite(f,LOW);
  digitalWrite(g,LOW);
  digitalWrite(p,LOW);
  digitalWrite(VERDE,LOW);
  digitalWrite(ROJO,LOW);
  digitalWrite(AMARILLO,LOW);
}


void afinacion(char letra)
{
  int nota;
  if(letra=='e')nota=5;//Nota "MI", elemento 5º de la escala musical {C,C#,D,D#,E...}
  if(letra=='a')nota=10;
  if(letra=='d')nota=3;
  if(letra=='g')nota=8;
  if(letra=='b')nota=12;
    
   for(int i=0;i<3;i++){//Bucle de tres pasadas, analizando las frecuencias de la nota en las tres escalas
    if(frecuencia>=scale[nota+12*i]->frec_low&&frecuencia<=scale[nota+12*i]->frec_high){//Si la frecuencia esta dentro de rango...
     notas(scale[nota+12*i]->letra);//Representa la nota
     digitalWrite(VERDE,HIGH);//Cuerda afinada. LUZ VERDE
     digitalWrite(ROJO,LOW);
     digitalWrite(AMARILLO,LOW);
     
     if(frecuencia>scale[nota+12*i]->frec+1){//Si supera el margen de 1Hz
       digitalWrite(AMARILLO,LOW);
       digitalWrite(ROJO,HIGH);//Ajuste fino: Dentro del margen de nota pero alejado del centro por exceso
     }
     if(frecuencia<scale[nota+12*i]->frec-1){//Si está por debajo del margen de 1Hz
       digitalWrite(AMARILLO,HIGH);//Ajuste fino: Dentro del margen de nota pero alejado del centro por defecto
       digitalWrite(ROJO,LOW);
     }
    break;
    } 
     if(frecuencia>scale[nota+12*i]->frec_high&&frecuencia<scale[nota+12*i+1]->frec_high){//Si la frecuencia es mayor que el límite alto de la nota y menor que el de la siguiente...
       notas(scale[nota+12*i+1]->letra);//Representa la nota siguiente
       digitalWrite(VERDE,LOW);
       digitalWrite(ROJO,HIGH);//Demasiado apretada. LUZ ROJA
       digitalWrite(AMARILLO,LOW);
       break;
     } 
     if(frecuencia<scale[nota+12*i]->frec_low&&frecuencia>scale[nota+12*i-1]->frec_low){//Si la frecuencia es menor que el límite bajo de la nota y mayor que el de la anterior...
       notas(scale[nota+12*i-1]->letra);//Representa la nota anterior
       digitalWrite(VERDE,LOW);
       digitalWrite(ROJO,LOW);
       digitalWrite(AMARILLO,HIGH);//Cuerda foja.LUZ AMARILLA
       break;
     } 
   }
}
void numero(int num)//Representar números en el DISPLAY
{
  if(num==2||num==3||num==5||num==6)digitalWrite(a,HIGH);//Números {2,3,5,6} --> Segmento "a" del display encendido
  else digitalWrite(a,LOW);//CONTROL SEGMENTO: A    
  if(num==5||num==6)digitalWrite(b,LOW);
  else digitalWrite(b,HIGH);//CONTROL SEGMENTO: B  
  if(num==2)digitalWrite(c,LOW);
  else digitalWrite(c,HIGH);//CONTROL SEGMENTO: C  
  if(num==1||num==4)digitalWrite(d,LOW);
  else digitalWrite(d,HIGH);//CONTROL SEGMENTO: D  
  if(num==2||num==6)digitalWrite(e,HIGH);
  else digitalWrite(e,LOW);//CONTROL SEGMENTO: E  
  if(num==4||num==5||num==6)digitalWrite(f,HIGH);
  else digitalWrite(f,LOW);//CONTROL SEGMENTO: F  
  if(num==1)digitalWrite(g,LOW);
  else digitalWrite(g,HIGH);//CONTROL SEGMENTO: G  
  digitalWrite(p,LOW);//CONTROL PUNTO
 }
 
 void notas(char nota)
 {
   if(nota>96)digitalWrite(p,LOW);//Letras minusculas representan notas sin sostenidos (En código ASCII a=97)
   if(nota<96){//Letras mayusculas representan sostenidos
    digitalWrite(p,HIGH);//Nota sostenida, #
    if(nota=='A')nota='a'; 
    if(nota=='C')nota='c'; 
    if(nota=='D')nota='d'; 
    if(nota=='F')nota='f';
    if(nota=='G')nota='g';     
  }
   
  if(nota=='b'||nota=='d')digitalWrite(a,LOW);
  else digitalWrite(a,HIGH);//CONTROL SEGMENTO: A    
  if(nota=='a'||nota=='d')digitalWrite(b,HIGH);
  else digitalWrite(b,LOW);//CONTROL SEGMENTO: B  
  if(nota=='c'||nota=='e'||nota=='f')digitalWrite(c,LOW);
  else digitalWrite(c,HIGH);//CONTROL SEGMENTO: C  
  if(nota=='a'||nota=='f')digitalWrite(d,LOW);
  else digitalWrite(d,HIGH);//CONTROL SEGMENTO: D  
  digitalWrite(e,HIGH);//CONTROL SEGMENTO: E  
  if(nota=='d')digitalWrite(f,LOW);
  else digitalWrite(f,HIGH);//CONTROL SEGMENTO: F  
  if(nota=='c')digitalWrite(g,LOW);
  else digitalWrite(g,HIGH);//CONTROL SEGMENTO: G  

 }
ISR(TIMER1_COMPB_vect) {}


