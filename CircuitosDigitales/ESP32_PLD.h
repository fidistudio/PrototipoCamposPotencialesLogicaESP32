// UNIVERSIDAD NACIONAL AUTÓNOMA DE MÉXICO
// FACULAD DE INGENIERÍA
// DEPARAMENTO DE MECATRÓNICA

// AUTORES:   M.I. ULISES M. PEÑUELAS RIVAS
//            M.I. YUKIHIRO MINAMI KOYAMA
//
// Versión 2021.2M3

/*********************************************************************
 Todos los derechos reservados. Facultad de Ingeniería de la 
 Universidad Nacional Autónoma de México © 2021. Queda estrictamente 
 prohibidos su uso fuera del ámbito académico, alteración, descarga, 
 difusión o divulgación por cualquier medio, así como su reproducción 
 parcial o total.
********************************************************************/

//Notas de uso de la biblioteca
// El archivo de bibioteca está depurada para funcionar principalmente
// con el microcontrolador ESP32, la tarjeta Arduino AVR ATMega2560 apoya
// a la simulación de la ESP32 sobre el paquete Proteus, en dónde solamente
// se usarán los pines 22-29, 50-55, 57-66 para hacer una interfaz de 
// subcircuito con el gráfico de una tarjeta ESP32 DEV KIT V1.
//
//---Entradas y Salidas---
// Pines de entrada del ESP32
// Todo el lado izquierdo de la tarjeta ESP32 Dev Kit V1 quedará 
// configurado como entrada digital, nombrado como E0 al E12, que 
// corresponden a los pines del 2 al 13.
//
// Pines de salida
// Todo el lado derecho de la tarjeta ESP32 Dev Kit V1 quedará 
// configurado como salida digital, nombrado como S0 al E12, que 
// corresponden a los pines del 30 al 18. No es posible usar la
// salida S2 por el diseño de la tarjeta. La salida S11, corres-
// pondiente al pin 19, tiene conectado el LED que integra la 
// tarjeta.
//
//---Funciones---
// La biblioteca cuenta con las siguientes funciones:
//    pld_ini()
//    Inicialización de la ESP32 como GAL. Esta función deberá ser
//    llamada desde dentro de la función "void setup()"
//
//    pld_555(valor)
//    La función genera señales de pulsos de reloj en la 
//    variable de "pld_clk"; de modo que puede ser usada como señal
//    de reloj interno. Esta función deberá ser llamada desde dentro 
//    de la función "void setup()" después del llamado de la función 
//    "pld_ini". "valor" es un número entero que indica la frecuencia
//    en Hz de la señal de reloj generado.
//
//    Ejemplo: 
//    void setup(){
//      ...
//      pld_ini();    // Inicializa a la ESP32 como PLD
//      pld_555(10);  // Genera una señal cuadrada de 10 Hz
//      ...
//    }
//         
//    void loop(){
//      ...
//      digitalWrite(S11, pld_clk);  // La salida S11 carga del reloj interno
//                      // el LED de la tarjeta parpadea a 10 Hz
//      ...
//    }
//
//****************************************************************

bool pld_clk = false;

//**********************************************
#if defined(__AVR_ATmega2560__)
  int pld_Ent[12] = {22, 23, 24, 25, 26, 27, 28, 29, 53, 52, 51, 50};
  int pld_Sal[12] = {54, 55, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66};

  #define E0 digitalRead(pld_Ent[0])
  #define E1 digitalRead(pld_Ent[1])
  #define E2 digitalRead(pld_Ent[2])
  #define E3 digitalRead(pld_Ent[3])
  #define E4 digitalRead(pld_Ent[4])
  #define E5 digitalRead(pld_Ent[5])
  #define E6 digitalRead(pld_Ent[6])
  #define E7 digitalRead(pld_Ent[7])
  #define E8 digitalRead(pld_Ent[8])
  #define E9 digitalRead(pld_Ent[9])
  #define E10 digitalRead(pld_Ent[10])
  #define E11 digitalRead(pld_Ent[11])
  #define S0 pld_Sal[0]
  #define S1 pld_Sal[1]
  #define S3 pld_Sal[2]
  #define S4 pld_Sal[3]
  #define S5 pld_Sal[4]
  #define S6 pld_Sal[5]
  #define S7 pld_Sal[6]
  #define S8 pld_Sal[7]
  #define S9 pld_Sal[8]
  #define S10 pld_Sal[9]
  #define S11 pld_Sal[10]
  #define S12 pld_Sal[11]

ISR(TIMER1_COMPA_vect){pld_clk = !pld_clk;}

void pld_ini()
{
  for(int i=0; i<12; i++){
    pinMode(pld_Ent[i],INPUT);
    pinMode(pld_Sal[i],OUTPUT);
  }
}

void pld_555(int16_t pld_fre)
{
  long tmp;
  tmp = (8000000/(1024*2*pld_fre))-1;
  //tmp = 2*980/pld_fre;
  noInterrupts();
  TCCR1A = 0,
  TCCR1B = 0,
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TCNT1 = 0;
  OCR1A = tmp;  //7811.5 PARA FREQ DE 1 Hz
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}
#endif
//**********************************************

//**********************************************
#if defined(ESP32)
int pld_Ent[] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13};
int pld_Sal[] = {23, 22, 3, 21, 19, 18, 5, 17, 16, 4, 2, 15};
#define E0 digitalRead(pld_Ent[0])
#define E1 digitalRead(pld_Ent[1])
#define E2 digitalRead(pld_Ent[2])
#define E3 digitalRead(pld_Ent[3])
#define E4 digitalRead(pld_Ent[4])
#define E5 digitalRead(pld_Ent[5])
#define E6 digitalRead(pld_Ent[6])
#define E7 digitalRead(pld_Ent[7])
#define E8 digitalRead(pld_Ent[8])
#define E9 digitalRead(pld_Ent[9])
#define E10 digitalRead(pld_Ent[10])
#define E11 digitalRead(pld_Ent[11])
#define E12 digitalRead(pld_Ent[12])
#define S0 pld_Sal[0]
#define S1 pld_Sal[1]
#define S3 pld_Sal[2]
#define S4 pld_Sal[3]
#define S5 pld_Sal[4]
#define S6 pld_Sal[5]
#define S7 pld_Sal[6]
#define S8 pld_Sal[7]
#define S9 pld_Sal[8]
#define S10 pld_Sal[9]
#define S11 pld_Sal[10]
#define S12 pld_Sal[11]

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void pld_555(float dato){
    if(dato==0)
      timerAlarmDisable(timer);
    else{
      timerAlarmWrite(timer, 1000000/(2*dato), true);
      timerAlarmEnable(timer);
    }
}

void IRAM_ATTR pld_Timer(){
  portENTER_CRITICAL_ISR(&timerMux);
  portEXIT_CRITICAL_ISR(&timerMux);
  pld_clk = !pld_clk;
}

void pld_ini(){
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &pld_Timer, true);
  for(int i=0; i<12; i++){
    pinMode(pld_Ent[i],INPUT);
    pinMode(pld_Sal[i],OUTPUT);
  }
}

void pld_output_clear(){
  for(int i=0;i<=11;i++)
  digitalWrite(i,LOW);
}

#endif
//**********************************************

