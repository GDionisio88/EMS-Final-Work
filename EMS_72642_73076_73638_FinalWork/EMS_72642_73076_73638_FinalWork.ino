#include <arduinoFFT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define ADC_PIN  32  //Pino ADC (HB100)
#define PEAK_PIN 35  //Peak detector Pin

//FFT
#define N         256   //} Número de amostras
#define Fs        2000  //} Frequência de amostragem

//Ultrasonic Sensor PINS
#define trigPin 19
#define echoPin 18

//Leds active/deactive PINS
#define greenPin  22
#define redPin    23


//Ultrasonic Sensor Values
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701


//FFT Variables/Constants
#define THRESHOLD 0.5   
#define C_LIGHT  3e8
#define F_RADAR  10.525e9
#define LAMBDA   (C_LIGHT / F_RADAR)
#define K_VEL    (LAMBDA / 2.0)
float vReal[N];
float vImag[N];
ArduinoFFT<float> FFT(vReal, vImag, N, Fs);
const unsigned long Ts_us = 1000000UL / Fs;
enum State {
  IDLE,
  ACQUIRE,
  PROCESS
};
State state = IDLE;
unsigned long lastSampleTime = 0;
uint16_t sampleIndex = 0;
float velocity = 0;



//Ultrasonic Variables
unsigned long trigTime = 0;
unsigned long echoStart = 0;
unsigned long echoEnd = 0;
bool waitingEcho = false;
float distanceCm = 0;



//Auxiliar Variables
unsigned long lastUltraTrigger = 0;
float counterAux = 0;
float counterAux1 = 0;
int flag = 0;
float maxSpeed = 1.5; //Km/h



//IO Connection (GUI)
enum Mode {
  MODE_IDLE,
  MODE_VELOCIDADE,
  MODE_DISTANCIA,
  MODE_LED,
  MODE_RADAR,
  MODE_PEAK,
};
Mode currentMode = MODE_IDLE;



//WIFI configuration 
const char* ssid = "AndroidAP";
const char* password = "civw6968";

const char* mqtt_server = "192.168.2.1";   // onde corre o Mosquitto
const int mqtt_port = 1883;
const char* clientName = "Grupo_8";

WiFiClient espClient;
PubSubClient client(espClient);

void connectWiFi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected.");
  Serial.print("IP address: ");     //IP do Esp32
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
   while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientName)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishVelocity(float velocity) {
  char payload[128];

  // measurement = motion
  // field = velocity
  // unit = m/s
  snprintf(payload, sizeof(payload),
           "ems velocidade=%.4f",
           velocity);

  client.publish("ems/p1/g8", payload);
}



void publishDistance(float distance) {
  char payload[128];

  // measurement = motion
  // field = velocity
  // unit = m/s
  snprintf(payload, sizeof(payload),
           "ems distancia=%.4f",
           distance);

  client.publish("ems/p1/g8", payload);
}





void setup() {
  Serial.begin(115200);
  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  connectMQTT();
}



void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();  
  checkSerial();                          //Está constantemente à procura das instruções provenientes da GUI
  if(currentMode == MODE_VELOCIDADE){
    taskVelocity();                       //Task associada ao sensor HB100
  }
  if (currentMode == MODE_DISTANCIA) {
    taskUltrasonic();                     //Task associada ao sensor ultrasónico
  }
  if(currentMode == MODE_LED){
    taskLED();                          //Task associada a um semáforo simples. Se a velocidade registada for superior à velocidade limite, o led vermelho ativa-se (comportamento de um semáforo com sensor de velocidade)
  }
  if(currentMode == MODE_RADAR){
    taskUltrasonic();                   //Task associada ao modo radar. permite perceber se um objeto está a aproximar-se ou afastar-se do sensor
  }
  if(currentMode == MODE_PEAK){
    peakDetector();                     //Task associada à leitura do peak detector
  } 
  if(currentMode == MODE_IDLE){
    digitalWrite(redPin, LOW);          //Quando o utilizador cancela qualquer ação, os LEDs vão a low
    digitalWrite(greenPin, LOW);
  }
  counterAux = micros();                //Regista o tempo atual em counterAux de forma a ser usado numa função posterior
}

/* ================= TASK VELOCIDADE ================= */
                                                    
void taskVelocity() {                                    
  //A task de leitura de velocidade tem três estados fundamentais
  //1-> IDLE) o estado está em aberto, ou seja, o software procura por leituras chamando detectActivity()
  //2-> ACQUIRE) Quando o sistema adquire uma leitura, ele passa por acquireSamples() de forma a compor um conjunto de amostrar para o processamento;
  //3-> PROCESS) Tendo as amostras recolhidas, o sistema processa as mesmas usando a FFT                 
  switch (state) {

    case IDLE:
      detectActivity();
      break;

    case ACQUIRE:
      acquireSamples();
      break;

    case PROCESS:
      processFFT();
      state = IDLE;
      break;
  }
}

/* ================= TASK ULTRASSÓNICO ================= */

void taskUltrasonic() {
  unsigned long now = micros();

  /* Dispara trigger a cada 60 ms */
  if (now - lastUltraTrigger > 600000 && !waitingEcho) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    waitingEcho = true;
    lastUltraTrigger = now;
  }

  /* Deteta subida do echo */
  if (waitingEcho && digitalRead(echoPin) == HIGH) {
    echoStart = micros();
    waitingEcho = false;
  }

                                                                            /* Deteta descida do echo */
  if (!waitingEcho && digitalRead(echoPin) == LOW && echoStart != 0) {
    echoEnd = micros();

    unsigned long duration = echoEnd - echoStart;
    distanceCm = duration * SOUND_SPEED / 2.0;

    publishDistance(distanceCm);
    
    Serial.print("DIST=");
    Serial.println(distanceCm, 1);

    echoStart = 0;
  }
                                                                              //delay(1000);
}




/* ================= TASK SEMÁFORO================= */
void taskLED(){
  if(flag == 0){
      //flag = 0 indica que a velocidade não foi ainda excedida
      digitalWrite(greenPin, HIGH);
      taskVelocity();
  }

  if(velocity*3.6 > maxSpeed && flag == 0){
    //A se velocidade foi excedida (em Km/h), encerra-se o processamento de registo velocidade e aciona-se o processo do semáforo
    counterAux1 = micros();     //Indica o momento da contraordenação
    flag = 1;
    velocity = 0; 
  }
  if(flag == 1){
    if(counterAux - counterAux1 >= 3000000){
      //Após três segundos da contraordenação, o led verde desliga-se e o vermelho vai a On durante 10 segundos
      digitalWrite(greenPin, LOW);
      digitalWrite(redPin, HIGH);
      flag = 2;       //Flag = 2 indica uma nova fase do processo do semáforo
      counterAux1 = micros();   //Indica o momento onde o semáforo passou para vermelho
    }
  }

  if(flag == 2){
    if(counterAux - counterAux1 >= 10000000){
      digitalWrite(redPin, LOW);      
      flag = 0;             //Reset das variáveis associadas ao processo
      counterAux1 = 0;      //
    }
  }
  
}



void peakDetector(){
    //Detecta o pico
    float peak = analogRead(PEAK_PIN) * (3.3 / 4095.0);
    Serial.print("PEAK=");
    Serial.println(peak);
    delay(500);
}




/* ===================== DETEÇÃO ===================== */

void detectActivity() {
  float sample = analogRead(ADC_PIN) * (3.3 / 4095.0);

  static float mean = 0.0;                                      //Média de amostras para melhor precisão
  mean = 0.995 * mean + 0.005 * sample;                         //

  float ac = fabs(sample - mean);

  if (ac > THRESHOLD) {
                                                                //Se o valor ac for superior ao THRESHOLD, acontece o processamento (tentar ao máximo remover qualquer tipo de imperfeição)
    sampleIndex = 0;        
    lastSampleTime = micros();  //Tempo da ultima amostra
    state = ACQUIRE;
  }
}

/* ===================== AQUISIÇÃO ===================== */

void acquireSamples() {   
 
  unsigned long now = micros();
  
  if (now - lastSampleTime >= Ts_us) {
                                                                        //verificação do tempo entre amostras do sinal e comparação com o período de amostra (inverso da frequência de amostragem)
    lastSampleTime += Ts_us;

    vReal[sampleIndex] = analogRead(ADC_PIN) * (3.3 / 4095.0);  
    vImag[sampleIndex] = 0.0;

    sampleIndex++;

    if (sampleIndex >= N) {
      state = PROCESS;                                                    //Apenas processa quando obtiver amostras suficientes
    }
  }
}

/* ===================== FFT + VELOCIDADE ===================== */

//Processo adaptado tendo em conta alguns exemplos que podem ser encontrados em: https://github.com/kosme/arduinoFFT/tree/master/Examples
//Retirar o valor médio do sinal
void processFFT() {
  float mean = 0.0;
  for (int i = 0; i < N; i++) mean += vReal[i];
  mean /= N;
  for (int i = 0; i < N; i++) vReal[i] -= mean;

                                                                                                    //Processamento do sinal usando a FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

                                                                                                                  //fD é a frequência com maior pico
  float fD = FFT.majorPeak();

                                                                                                      //Alguns ajustes de forma a obter valores mais agradáveis (filtragem de ruído e valores negativos esquisitos)
  if (isnan(fD) || isinf(fD)) fD = 0.0;
  if (fD < 0.0) fD = 0.0;          

                                                                                                        //Conversão para velocidade
  velocity = K_VEL * fD;

                                                                                                        //Enviar os dados da velocidade para a DB
  publishVelocity(velocity*3.6);

  Serial.print("VEL=");
  Serial.println(velocity*3.6,3);
}




void checkSerial() {
                                                              //Processo para averiguar os diferentes comandos da GUI
  if (Serial.available() > 0) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "READ_VELOC") {
      currentMode = MODE_VELOCIDADE;
    }
    else if (cmd == "READ_DIST") {
      currentMode = MODE_DISTANCIA;
    }
    else if (cmd == "IDLE") {
      currentMode = MODE_IDLE;
    }
    else if (cmd == "ACTIVATE_LED"){
      currentMode = MODE_LED;
    }
    else if(cmd == "RADAR"){
      currentMode = MODE_RADAR;
    }
    else if(cmd == "PEAK"){
      currentMode = MODE_PEAK;
    }
  }
}
