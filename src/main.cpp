#include <Arduino.h>
#include <DC-Motor-Arduino.h>
#include <QTRSensors.h>
#include <pitches.h>

// #define LEFT_BUZZER
#define CALIBRATE_MELODY
#define RIGHT_BUZZER
#define CALIBRATE_BUZZER

// sensor frontal deverá ser o QTR-8RC
#define BUZZER_PIN 13

// motor pins
#define MOTOR_PIN1_E A5
#define MOTOR_PIN2_E A4
#define PWM_E A0
#define MOTOR_PIN1_D A2
#define MOTOR_PIN2_D A3
#define PWM_D A1
DC_Motor m1(PWM_E, MOTOR_PIN1_E, MOTOR_PIN2_E);
DC_Motor m2(PWM_D, MOTOR_PIN1_D, MOTOR_PIN2_D);

// sensors pins
#define RIGHT_SENSOR_PIN 2
#define LEFT_SENSOR_PIN 12
#define EMIT_PIN 11 // if used, the qtr emmiter is only turned on when reading
const uint8_t qtr_pins[] { 10, 9, 8, 7, 6, 5, 4, 3 };
const uint8_t num_sensors = sizeof(qtr_pins);

QTRSensors qtr;
uint16_t sensors[8];

enum Estado { EM_CORRIDA, CURVA_ESQ, CURVA_DIR, RETA, PARANDO };
Estado estadoAtual = EM_CORRIDA;

float kp = 0.2;

int velocidadeBase = 200;

bool linhaVisivel = true; // indica se o sensor frontal ainda vê a linha

void controlarMotores(int correcao);
void pararMotores();
void calibra_sensores();
void lerSensorDireito();
void lerSensorEsquerdo();
void atualizaEstado(int erro);
bool verificaLinhaVisivel(uint16_t sensorValues[]);

#ifdef CALIBRATE_MELODY
// HP melody
int melody[] = { REST, NOTE_D4, NOTE_G4, NOTE_AS4, NOTE_A4, NOTE_G4, NOTE_D5, NOTE_C5, NOTE_A4,
  NOTE_G4, NOTE_AS4, NOTE_A4, NOTE_F4, NOTE_GS4, NOTE_D4, NOTE_D4, NOTE_G4, NOTE_AS4, NOTE_A4,
  NOTE_G4, NOTE_D5, NOTE_F5, NOTE_E5, NOTE_DS5, NOTE_B4, NOTE_DS5, NOTE_D5, NOTE_CS5, NOTE_CS4,
  NOTE_B4, NOTE_G4 };
int durations[] = { 2, 4, 4, 8, 4, 2, 4, 2, 2, 4, 8, 4, 2, 4, 1, 4, 4, 8, 4, 2, 4, 2, 4, 2, 4, 4, 8,
  4, 2, 4, 1 };
#endif

void setup()
{
  Serial.begin(9600);

  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // qtr.setTypeAnalog();
  qtr.setTypeRC();
  qtr.setSensorPins(qtr_pins, num_sensors);
  qtr.setEmitterPin(EMIT_PIN);

  calibra_sensores();

  Serial.println("Sistema iniciado...");
  delay(300);
}

// Nossa lógica principal
void loop()
{
  lerSensorDireito();

  uint16_t sensorValues[num_sensors];
  int posicao = qtr.readLineWhite(sensorValues); // posição entre 0 e 7000 aprox.
  int erro = posicao - 3500; // centro ideal = 3500
  int correcao = kp * erro;

  Serial.print("posicao: ");
  Serial.println(posicao);

  atualizaEstado(erro);

  // aqui que usamos a nossa máquina de estados
  if (estadoAtual == CURVA_ESQ || estadoAtual == CURVA_DIR) {
    controlarMotores(correcao);
  } else {
    controlarMotores(correcao);
  }

  // para registrar as curvas e retas, mantemos isso
  lerSensorEsquerdo();
}

//==========Nossas Funções===========//
// função simples para controlar motores com correcao proporcional
void controlarMotores(int correcao)
{
  int velEsq = velocidadeBase - correcao;
  int velDir = velocidadeBase + correcao;

  // Prevenção de valores negativos - constrain
  velEsq = constrain(velEsq, 0, 255);
  velDir = constrain(velDir, 0, 255);

  m1.forward(velEsq);
  m2.forward(velDir);

  Serial.print("m1.forward(");
  Serial.print(velEsq);
  Serial.print(")\n");

  Serial.print("m2.forward(");
  Serial.print(velDir);
  Serial.print(")\n\n");
}

void pararMotores()
{
  m1.stop();
  m2.stop();
}

// Se frequncia eh 0, buzzer eh desligado
// se ms eh diferente de zero, buzzer eh ligado por ms milisegundos (trava programa)
void setBuzzer(int pin, int freq, int ms)
{
  if (freq != 0) {
    tone(pin, freq);
  } else {
    noTone(pin);
    return;
  }

  if (ms != 0) {
    delay(ms);
    noTone(pin);
  }
}

void calibra_sensores()
{

  //// aciona led ao calibrar sensor
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(300);
  // digitalWrite(LED_BUILTIN, LOW);

#ifdef CALIBRATE_MELODY
  delay(3000);
  Serial.println("Calibrando sensor frontal");
  int size = sizeof(durations) / sizeof(int);
  for (int note = 0; note < size; note++) {
    // to calculate the note duration, take one second divided by the note type.
    // e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 800 / durations[note];
    tone(BUZZER_PIN, melody[note], duration);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);

    qtr.calibrate();

    //  stop the tone playing:
    noTone(BUZZER_PIN);
  }
#else
  Serial.println("Calibrando sensor frontal");
#ifdef CALIBRATE_BUZZER
  setBuzzer(BUZZER_PIN, 600, 1000);
#endif
  for (int i = 0; i < 200; i++)
    qtr.calibrate();
#ifdef CALIBRATE_BUZZER
  setBuzzer(BUZZER_PIN, 600, 100);
  delay(100);
  setBuzzer(BUZZER_PIN, 600, 100);
  delay(100);
  setBuzzer(BUZZER_PIN, 600, 100);
  delay(100);
#endif
#endif

  Serial.println("Fim da calibração");
}

void lerSensorDireito()
{
  bool right_sensor_active = (digitalRead(RIGHT_SENSOR_PIN) == LOW);
  // bool left_sensor_active = (digitalRead(LEFT_SENSOR_PIN) == LOW);
  // bool leitura = (!left_sensor_active && right_sensor_active);
  bool leitura = right_sensor_active;

#ifdef RIGHT_BUZZER
  if (leitura == HIGH) {
    Serial.println("Sensor direito ativado");
    setBuzzer(BUZZER_PIN, 600, 0);
  } else
    setBuzzer(BUZZER_PIN, 0, 0);
#endif
}

// usaremos a leitura do sensor esquerdo para sabermos as curvas e retas
void lerSensorEsquerdo()
{
  bool left_sensor_active = (digitalRead(LEFT_SENSOR_PIN) == LOW);
  // bool right_sensor_active = (digitalRead(RIGHT_SENSOR_PIN) == LOW);
  // bool leitura = (left_sensor_active && !right_sensor_active);
  bool leitura = left_sensor_active;

#ifdef LEFT_BUZZER
  if (leitura != HIGH) {
    Serial.println("Sensor esquerdo ativado");
    setBuzzer(BUZZER_PIN, 600, 0);
  } else
    setBuzzer(BUZZER_PIN, 0, 0);
#endif
}

// verifica se estamos numa reta ou curva com base no erro do sensor frontal
void atualizaEstado(int erro)
{
  if (abs(erro) < 400)
    estadoAtual = RETA;
  else if (erro > 0)
    estadoAtual = CURVA_DIR;
  else
    estadoAtual = CURVA_ESQ;
}

bool verificaLinhaVisivel(uint16_t sensorValues[])
{
  for (int i = 0; i < num_sensors; i++)
    if (sensorValues[i] < 800)
      return true;
  return false;
}
