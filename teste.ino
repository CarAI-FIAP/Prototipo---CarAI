#include <Wire.h> //biblioteca para comunicação com MPU6050
#include "SoftwareSerial.h" //biblioteca que emula pinos serial (TX e RX) 


#define EXIST_MPU6050 1 //define a existencia do MPU6050

#define EXIST_ACCX (EXIST_MPU6050 && 0)  //define a existencia de printar os valores do acelerometro
#define EXIST_ACCX_FILTRO (EXIST_ACCX && EXIST_FILTRO && 1)  //define a existencia de filtro do acelerometro

#define EXIST_GYROZ (EXIST_MPU6050 && 1) //define a existencia de printar os valores do giroscopio
#define EXIST_GYROZ_FILTRO (EXIST_GYROZ && EXIST_FILTRO && 1) //define a existencia de foltro do giroscopio

#define EXIST_Ultrassonico 1 //define a existencia do sensor ultrassonico.
#define EXIST_Ultrassonico_FILTRO (EXIST_Ultrassonico && EXIST_FILTRO && 1) //define a existencia do filtro para sensor ultrassonico.

#define EXIST_MOTOR (EXIST_Ultrassonico && 1) //define a existencia dos motores

#define EXIST_FILTRO 1 //define a existencia de filtro
#define EXIST_COMPARACAO (!EXIST_FILTRO || 0)  //define a existencia de comparação entre os valores do filtro

#define EXIST_BLUE_VEL 0 //define a existencia do modulo bluetooth

#define EXIST_AJUSTE_GRAFICO 0 //ajusta o grafico do serial plotter para uma melhor visualização



/*******************************************************************/
#if EXIST_MPU6050
//biblioteca do MPU6050:
const uint8_t IMUAddress = 0x68;
const uint8_t I2C_TIMEOUT = 1000;
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop){
  return i2cWrite(registerAddress, &data, 1, sendStop);
}
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  if(rcode){
    Serial.print(F("i2cWrite Failed: "));
    Serial.println(rcode);
  }
  return rcode;
}
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes){
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false);
  if(rcode){
    Serial.print(F("i2cRead Failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for(uint8_t i = 0; i< nbytes; i++){
    if(Wire.available())
      data[i]=Wire.read();
    else{
      timeOutTimer = micros();
      while(((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if(Wire.available())
        data[i] = Wire.read();
      else{
        Serial.println(F("i2cRead timeout"));
        return 5;
      }
    }
  }
  return 0;
}
uint8_t i2c_data[14]; //configuração do mpu6050
#endif
//Fim da biblioteca do MPU6050
/*******************************************************************/

#define MEDIA_PARA_CHAO 300  //media para zerar o sinal do acelerometro
#define MEDIA_PARA_GIRO 3000 //media para zerar o angulo do giroscopio

#define NUMERO_FILTROS_ULTRA 1  //numero de filtros do sensor ultrasonico
#define NUMERO_FILTROS_GIRO 2  //numero de filtros do giroscopio
#define NUMERO_FILTROS_ACEL 1  //numero de filtros do acelerometro

#define INTERVALO_MEDIA_ULTRA 50 //intervalo de valores da media movel (filtro)
#define INTERVALO_MEDIA_GIRO 100 //intervalo de valores da media movel (filtro)
#define INTERVALO_MEDIA_ACEL 100 //intervalo de valores da media movel (filtro)

#define DISTANCIA_PARA 5 //distancia minima (em cm) para o carro parar

#define TEMPO_AJUSTE_MEDIA 2000 //tempo para a media movel se ajustar
#define TEMPO_PARADO 2000 //tempo do carro parado antes de fazer algum movimento

//**********************************************************

#define ECHO 7 //define o pino echo do sensor ultrassonico 
#define TRIG 8 //define o pino trig do sensor ultrassonico 

#define MD_1 12 //define o pino 1 de controle do motor direito
#define MD_2 11 //define o pino 2 de controle do motor direito
#define PWM_D A1 //define o pino de controle pwm do motor esquerdo

#define ME_1 10 //define o pino 1 de controle do motor esquerdo
#define ME_2 9 //define o pino 2 de controle do motor esquerdo
#define PWM_E A2 //define o pino de controle pwm do motor direito

//**********************************************************

#if EXIST_BLUE_VEL
SoftwareSerial bluetooth(5, 6); //define os pinos TX, RX do bluetooth
char msg_blue; //recebe os dados do bluetooth
#endif

#if EXIST_Ultrassonico
float tempoEcho = 0;  //armazena o tempo de pulso sonoro do sensor 
float distancia;  //distancia vinda do sensor ultrassonico em cm
const float velocidadeSom = 0.00034029; //velocidade do som em metros/microsegundo
#endif

#if EXIST_ACCX
double accX;  //armazena a aceleração no eixo x
double ref_chao_accX; //armazena o valor de referencia do acelerometro
#endif

#if EXIST_GYROZ
double gyroX; //armazenam os angulos brutos do giroscopio
double gyroXoffset; //armazena o valor excedente de referencia do giroscopio
double angleX, angulo_x; //armazenam os angulos do giroscopio depois de tratado
double angulo_zero; //armazena o valor de referencia (0) do giroscopio
float interval; //armazena a variação do angulo em um determinado tempo
long preInterval; // tempo de variação do angulo do giroscopio
#endif

#if EXIST_MOTOR
bool trava_angulo = true; //trava para definir um angulo inicial
bool trava_volta = true;  //trava feita para fazer o carro girar
bool trava_deteccao = false; //trava para fazer a media movel se ajustar
#endif

bool trava_chao = true; //trava para fazer o ajuste de referencia dos sinais do mpu6050
bool trava_inicio = true; //trava para fazer o modulo bluetooth e decção funcionarem
bool trava_ajuste_pwm = false; //trava para ajustar o pwm 
int vel_max = 255; //velocidade maxima defull dos motores
String dados_print = ""; //armazena os dados que serão printados no monitor serial 

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// inicio da classe responsavel por aplicar os filtros
#if EXIST_FILTRO
class Filtro {
  float** sinal;
  int intervalo_media_m;
  int numero_filtros;
 public:
  Filtro(int intervalo_media, int numero){
    intervalo_media_m = intervalo_media;
    numero_filtros = numero;
    sinal = new float*[numero_filtros];

    for (int i = 0; i < numero_filtros; i++) {
      sinal[i] = new float[intervalo_media_m];
      memset(sinal[i], 0, sizeof(float) * intervalo_media_m);
    }
  }

  ~Filtro() {
    for (int i = 0; i < numero_filtros; i++) {
      delete[] sinal[i];
    }
    delete[] sinal;
  }

  float filtro_media_movel(float sinal_) {
    float k = 0;
    for (int i = 0; i < numero_filtros; i++) {
    for (int x = intervalo_media_m - 1; x > 0; x--) {
      sinal[i][x] = sinal[i][x - 1];
    }
    sinal[i][0] = sinal_;
    k = 0;
    for (int x = 0; x < intervalo_media_m; x++) {
      k += sinal[i][x];
    }
    }
    return k / intervalo_media_m;

  }  
};
#if EXIST_GYROZ_FILTRO
Filtro MPU6050_giro(INTERVALO_MEDIA_GIRO, NUMERO_FILTROS_GIRO);
#endif
#if EXIST_ACCX_FILTRO
Filtro MPU6050_acel(INTERVALO_MEDIA_ACEL, NUMERO_FILTROS_ACEL);
#endif
#if EXIST_Ultrassonico_FILTRO
Filtro Sensor_ultra(INTERVALO_MEDIA_ULTRA, NUMERO_FILTROS_ULTRA);
#endif
#endif
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// inicio da classe responsavel pela rotação dos motores
#if EXIST_MOTOR
class Motores {
  int pin_m1;
  int pin_m2;
  int pin_pwm;
 public:
  Motores(const int pwm, int m1, int m2){
  pin_m1 = m1;
  pin_m2 = m2;
  pin_pwm = pwm;
  pinMode(pin_m1, OUTPUT);  
  pinMode(pin_m2, OUTPUT);  
  pinMode(pin_pwm, OUTPUT);  
  }

  void frente(int pwm) {
  analogWrite(pin_pwm,pwm);
  digitalWrite(pin_m1,HIGH);
  digitalWrite(pin_m2,LOW);
  }

  void tras(int pwm) {
  analogWrite(pin_pwm,pwm);
  digitalWrite(pin_m1,LOW);
  digitalWrite(pin_m2,HIGH);
  }
  
  void para() {
  analogWrite(pin_pwm,0);
  digitalWrite(pin_m1,LOW);
  digitalWrite(pin_m2,LOW);
  }
};

Motores motor_d(PWM_D, MD_1, MD_2);
Motores motor_e(PWM_E, ME_1, ME_2);

#endif
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// inicio da classe responsavel por contar a variação de tempo em um determinado intervalo

class Contador_tempo {
  unsigned long intervalo;
  unsigned long ultima_atualizacao;
  bool trava_chamado;
  
public:
  Contador_tempo(unsigned long intervalo_ms){
    intervalo = intervalo_ms;
    trava_chamado = true;
  }
  
  bool atingiu_tempo() {
    unsigned long tempo_atual = millis();
    if(trava_chamado){
      ultima_atualizacao = tempo_atual;
      trava_chamado = false;
    }   
    if(tempo_atual - ultima_atualizacao >= intervalo){
      trava_chamado = true;      
      return true;
    }
    return false;
  }
};

Contador_tempo ajuste_media_movel(TEMPO_AJUSTE_MEDIA); 
Contador_tempo parado(TEMPO_PARADO); 

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*******************************************************************/
void setup() {

  Serial.begin(115200);

  #if EXIST_BLUE_VEL
  bluetooth.begin(9600);
  trava_ajuste_pwm = true;
  trava_inicio = false;
  #endif

  #if EXIST_MPU6050
  Wire.begin();
  Wire.setClock(4000000UL);
  i2c_data[0] = 7;
  for(int i = 1; i<4; i++){
    i2c_data[i] = 0x00;
  }
  while(i2cWrite(0x19, i2c_data, 4, false));
  while(i2cWrite(0x6B, 0x01, true));
  while(i2cRead(0x75, i2c_data, 1));
  if(i2c_data[0]!= 0x68){
    Serial.print("Erro. Placa desconhecida!\n");
    while(1){
      Serial.println("Erro. Conecte a MPU6050 no barramento I2C.\n");
    }
  }
  delay(100);
  #endif  

  #if EXIST_Ultrassonico
  pinMode(ECHO, INPUT); 
  pinMode(TRIG, OUTPUT);   
  digitalWrite(TRIG, LOW);
  #endif
}
//**********************************************************************************
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void loop() {
  if(trava_ajuste_pwm){
    #if EXIST_BLUE_VEL
    Ajuste_pwm_max();
    #endif    
  }else{
    
    #if EXIST_MPU6050
    if(trava_chao){
      Call_ref_chao();
      trava_chao = false;       
    }
    #endif 

    #if EXIST_BLUE_VEL
    Restaura_ajuste_pwm();
    #endif
    
    #if EXIST_ACCX
    Acelerometro();
    #endif

    #if EXIST_GYROZ
    Giroscopio();
    #endif

    #if EXIST_Ultrassonico
    Distancia_Sensor();
    #endif
      
    #if EXIST_MOTOR
    if(trava_deteccao){
      Deteccao();
    }   
    if(ajuste_media_movel.atingiu_tempo()){  
      trava_deteccao = true;
    }    
    #endif

    Prints();

  }
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//**********************************************************************************
#if EXIST_BLUE_VEL
void Restaura_ajuste_pwm(){
  if (bluetooth.available()) {
    msg_blue = bluetooth.read();
    if(msg_blue == 'B'){
      #if EXIST_MOTOR
      motor_d.para();
      motor_e.para();
      #endif
      bluetooth.println("redefinindo o PWM maximo");
      trava_ajuste_pwm = 0;
    }
  }
}
//----------------------------------------------------------------------------------
void Ajuste_pwm_max(){
  if (bluetooth.available()) {
    msg_blue = bluetooth.read();
    if(msg_blue == 'A'){
      trava_ajuste_pwm = false;
      trava_inicio = false;
      #if EXIST_MOTOR
      motor_d.para();
      motor_e.para();
      #endif      
      bluetooth.print("PWM maximo definido de: ");
      bluetooth.println(String(vel_max));
    }else{
      if(msg_blue == '1'){
        if(vel_max < 255){
          vel_max = vel_max + 1;           
        }
        msg_blue = 0;      
      }else if (msg_blue == '2'){
        if(vel_max > 0){
          vel_max = vel_max - 1; 
        }        
        msg_blue = 0;         
      }else if (msg_blue == '3'){
        if(vel_max < 245){
          vel_max = vel_max + 10; 
        }        
        msg_blue = 0;         
      }else if (msg_blue == '4'){
        if(vel_max > 10){
          vel_max = vel_max - 10; 
        }        
        msg_blue = 0;         
      }
      bluetooth.print("PWM =: ");
      bluetooth.println(String(vel_max));
      #if EXIST_MOTOR
      motor_d.frente(vel_max);
      motor_e.frente(vel_max);
      #endif
    }
  }
}
#endif
//**********************************************************************************
//**********************************************************************************

void Call_ref_chao(){
  
  #if EXIST_ACCX
  double zc_x = 0;
  double zcx = 0;
  for (int x = 0; x < MEDIA_PARA_CHAO; x++) {
    while(i2cRead(0x3B, i2c_data, 14));
      zc_x = (int16_t) ((i2c_data[0] << 8) | i2c_data[1]);
      zcx = zcx + zc_x;
  }
  ref_chao_accX = zcx / MEDIA_PARA_CHAO;
  #endif
  
  #if EXIST_GYROZ
  double zgx = 0;
  double x = 0;
  preInterval = millis();
  for(int i = 0; i < MEDIA_PARA_GIRO; i++){
    if(i % 1000 == 0){
      Serial.println("...");
      #if EXIST_BLUE_VEL
      bluetooth.println("...");
      #endif
    }
    while(i2cRead(0x3B, i2c_data, 14));
      zgx = (int16_t) ((i2c_data[8] << 8) | i2c_data[9]);
      x += ((float)zgx) / 131.0;
      gyroXoffset = x/MEDIA_PARA_GIRO;
  }
  Serial.println("Concluido");
  #if EXIST_BLUE_VEL
  bluetooth.println("Concluido");
  #endif
  #endif
}
//**********************************************************************************
//**********************************************************************************

#if EXIST_ACCX
void Acelerometro(){
  double zc_x = 0;
  double zcx = 0;
  while(i2cRead(0x3B, i2c_data, 14));
    zc_x = (int16_t) ((i2c_data[0] << 8) | i2c_data[1]);
    accX =  zc_x - ref_chao_accX;    
  accX = accX / 16383.5;
  accX = accX * 9.8;
  #if EXIST_COMPARACAO 
  dados_print += String(accX);
  dados_print += "\t";
  #endif
  #if EXIST_ACCX_FILTRO
  accX =  MPU6050_acel.filtro_media_movel(accX);
  dados_print += String(accX);
  dados_print += "\t";
  #endif
}
#endif
//**********************************************************************************
//**********************************************************************************

#if EXIST_GYROZ
void Giroscopio(){
  while(i2cRead(0x3B, i2c_data, 14));
    gyroX = (int16_t) ((i2c_data[8] << 8) | i2c_data[9]);
    gyroX= gyroX/131.0;
  
  gyroX -= gyroXoffset;
  interval = (millis() - preInterval) * 0.001;
  angleX = (angleX + gyroX * interval);
  angulo_x = angleX;
  preInterval = millis();

  #if EXIST_COMPARACAO 
  dados_print += String(angleX);
  dados_print += "\t";
  #endif

  #if EXIST_GYROZ_FILTRO
  angulo_x = MPU6050_giro.filtro_media_movel(angleX);
  dados_print += String(angulo_x);
  dados_print += "\t";
  #endif

}
#endif
//**********************************************************************************
//**********************************************************************************

#if EXIST_Ultrassonico
void Distancia_Sensor(){
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  tempoEcho = pulseIn(ECHO, HIGH);
  distancia = Calcula_Distancia(tempoEcho)*100;
  #if EXIST_COMPARACAO 
  dados_print += String(distancia);
  dados_print += "\t";
  #endif
  #if EXIST_Ultrassonico_FILTRO
  distancia = Sensor_ultra.filtro_media_movel(distancia);
  dados_print += String(distancia);
  dados_print += "\t";
  #endif
}
//----------------------------------------------------------------------------------
float Calcula_Distancia(float tempoMicrossegundos){
  return((tempoMicrossegundos*velocidadeSom)/2); // velocidade do som em m/microssegundo
}
#endif
//**********************************************************************************
//**********************************************************************************

#if EXIST_MOTOR
void Deteccao(){
  #if EXIST_BLUE_VEL
  if (bluetooth.available()) {
    msg_blue = bluetooth.read();
    if(msg_blue == 'C'){
      trava_inicio = true;
    }
  }
  #endif  
  if(trava_inicio){
    if(distancia < DISTANCIA_PARA){
      if(trava_volta){
        motor_d.para();
        motor_e.para();
        dados_print += "parado";
        dados_print += "\t";
        trava_volta = false;  
      }
      if(parado.atingiu_tempo()){
        motor_d.tras(vel_max);
        motor_e.tras(vel_max);
        dados_print += "ré";
        dados_print += "\t";
      }
    }else{
      if(trava_volta){
        motor_d.frente(vel_max);
        motor_e.frente(vel_max);
        dados_print += "andando";
        dados_print += "\t";
      }else{
        if(trava_angulo){
          angulo_zero = angulo_x;
          trava_angulo = false; 
        }
        if(abs(angulo_x-angulo_zero) < 180){                
          motor_d.frente(vel_max);
          motor_e.tras(vel_max);
          dados_print += "girando";
          dados_print += "\t";
        }else{
          motor_d.para();
          motor_e.para();
          dados_print += "parado";
          dados_print += "\t";
          delay(200);
          trava_angulo = true;
          trava_volta = true;
        }
      }
    }
  }
}
#endif
//**********************************************************************************
//**********************************************************************************

void Prints(){
  #if EXIST_AJUSTE_GRAFICO
  dados_print += String(50);
  dados_print += "\t";
  dados_print += String(0);
  dados_print += "\t";
  dados_print += String(-50);
  dados_print += "\t";
  #endif

  Serial.println(dados_print);

  #if EXIST_BLUE_VEL
  bluetooth.println(dados_print);
  #endif
  
  dados_print = " ";

}











