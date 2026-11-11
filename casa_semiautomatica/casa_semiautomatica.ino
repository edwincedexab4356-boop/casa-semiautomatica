/*
  CASA SEMIAUTOMATICA - Arduino UNO
  ---------------------------------
  - Aplauso (sensor de sonido)  -> enciende / apaga la matriz 8x8
                                   (corazon, apagado, carita, apagado... en bucle)
  - Vibracion moderada (SW-420) -> buzzer activo continuo por 10 segundos
  - Fotoresistencia en el piso  -> si un auto la tapa (oscuro) el servo abre
                                   la compuerta a 90 grados; se cierra sola
                                   cuando el auto se va
  - HC-05 (Bluetooth) y USB     -> control total desde la pagina web

  CONEXIONES
  Matriz MAX7219 : VCC 5V, GND, DIN D7, CS D6, CLK D5
  Sensor sonido  : VCC 5V, GND, DO  D2
  Sensor vibra.  : VCC 5V, GND, DO  D3
  Buzzer activo  : +  D4, -  GND
  Servo          : rojo 5V, cafe GND, naranja D9
  Fotoresistencia: una pata a 5V, la otra a A0  y  resistencia 10k de A0 a GND
  HC-05          : VCC 5V, GND, TXD -> D10, RXD <- D11 (con divisor 1k / 2k)

  Protocolo (9600, una orden por linea):
    MATRIZ:1/0  BUZZER:1/0  PUERTA:1/0  AUTO:1/0  SONIDO:1/0  VIBRA:1/0
    UMBRAL:n (0-1023)  SENSVIB:n (1-20)  BRILLO:n (0-15)  CAL  ESTADO
    DIAG:1/0 -> DG,sonido,muestras_sonido,vibra,cambios_vibra,luz_cruda (cada 250 ms)
  El Arduino responde cada 500 ms:
    ST,matriz,cuadro,buzzer,seg_restantes,puerta,auto,luz,umbral,sonido,vibra,sensvib,brillo,angulo
  y avisos:  EV,texto
*/

#include <Servo.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// ---------- pines ----------
const byte P_DIN = 7, P_CS = 6, P_CLK = 5;
const byte P_SONIDO = 2;
const byte P_VIBRA  = 3;
const byte P_BUZZER = 4;
const byte P_SERVO  = 9;
const byte P_BT_RX  = 10;   // aqui llega el TXD del HC-05
const byte P_BT_TX  = 11;   // va al RXD del HC-05 (con divisor)
const byte P_LDR    = A0;

// ---------- ajustes ----------
// El nivel del sensor de sonido en silencio se aprende solo al encender
// (unos modulos marcan HIGH al escuchar y otros LOW). No hagas ruido al encender.
int SONIDO_ACTIVO = HIGH;
// Si la figura sale de cabeza en tu matriz, cambia a 1
#define VOLTEAR_MATRIZ 0

const byte ANG_CERRADO = 0;
const byte ANG_ABIERTO = 90;
const unsigned long BUZZER_MS      = 10000UL; // 10 segundos
const unsigned long CUADRO_MS      = 700;     // tiempo de cada figura
const unsigned long BLOQUEO_APLAUSO = 600;    // evita contar dos veces el mismo aplauso
const unsigned long VENTANA_VIBRA  = 300;     // ventana para medir la vibracion
const unsigned long CIERRE_MS      = 3000;    // se cierra 3 s despues de que el auto se va
const unsigned long OSCURO_MS      = 400;     // tiene que estar oscuro 0,4 s seguidos

SoftwareSerial bt(P_BT_RX, P_BT_TX);
Servo compuerta;

// ---------- figuras ----------
const byte CORAZON[8] = {
  B00000000,
  B01100110,
  B11111111,
  B11111111,
  B01111110,
  B00111100,
  B00011000,
  B00000000
};
const byte CARITA[8] = {
  B00111100,
  B01000010,
  B10100101,
  B10000001,
  B10100101,
  B10011001,
  B01000010,
  B00111100
};

// ---------- estado ----------
bool matrizOn = false;
byte cuadro = 0;               // 0 corazon, 1 apagado, 2 carita, 3 apagado
unsigned long tCuadro = 0;

bool buzzerOn = false;
unsigned long tBuzzer = 0;

bool puertaAbierta = false;
bool modoAuto = true;
int  anguloActual = ANG_CERRADO;
int  anguloMeta = ANG_CERRADO;
unsigned long tServo = 0;
unsigned long tMovio = 0;      // ultima vez que se movio el servo
unsigned long tOscuro = 0, tClaro = 0;
bool estabaOscuro = false;

bool sonidoOn = true, vibraOn = true;
int  umbral = 300;             // por debajo de esto = oscuro (auto encima)
byte sensVib = 4;              // cambios necesarios en 300 ms = vibracion moderada
byte brillo = 6;
int  luz = 1023;

int  ultimoSonido = !SONIDO_ACTIVO;
unsigned long tAplauso = 0;

int  ultimoVibra = LOW;
byte cambiosVibra = 0;
unsigned long tVentana = 0;

unsigned long tEstado = 0;
bool diag = false;             // DIAG:1 muestra las lecturas crudas
unsigned int dgSonido = 0, dgVibra = 0;
String linUsb, linBt;

// ================= MAX7219 =================
void mx(byte reg, byte val) {
  digitalWrite(P_CS, LOW);
  shiftOut(P_DIN, P_CLK, MSBFIRST, reg);
  shiftOut(P_DIN, P_CLK, MSBFIRST, val);
  digitalWrite(P_CS, HIGH);
}
void mxIniciar() {
  pinMode(P_DIN, OUTPUT); pinMode(P_CS, OUTPUT); pinMode(P_CLK, OUTPUT);
  digitalWrite(P_CS, HIGH);
  mx(0x0F, 0);       // sin modo prueba
  mx(0x09, 0);       // sin decodificar
  mx(0x0B, 7);       // 8 filas
  mx(0x0A, brillo);
  mx(0x0C, 1);       // encendido
}
void mxDibujar(const byte *f) {
  for (byte i = 0; i < 8; i++) {
    byte fila = VOLTEAR_MATRIZ ? 7 - i : i;
    mx(i + 1, f ? f[fila] : 0);
  }
}
void mostrarCuadro() {
  if (!matrizOn) { mxDibujar(0); return; }
  if (cuadro == 0) mxDibujar(CORAZON);
  else if (cuadro == 2) mxDibujar(CARITA);
  else mxDibujar(0);
}

// ================= salida a USB y Bluetooth =================
void enviar(const String &s) { Serial.println(s); bt.println(s); }
void evento(const String &s) { enviar("EV," + s); }

void enviarEstado() {
  String s = "ST,";
  s += matrizOn; s += ','; s += cuadro; s += ',';
  s += buzzerOn; s += ',';
  long resta = buzzerOn ? (long)(BUZZER_MS - (millis() - tBuzzer) + 999) / 1000 : 0;
  s += resta; s += ',';
  s += puertaAbierta; s += ','; s += modoAuto; s += ',';
  s += luz; s += ','; s += umbral; s += ',';
  s += sonidoOn; s += ','; s += vibraOn; s += ',';
  s += sensVib; s += ','; s += brillo; s += ','; s += anguloActual;
  enviar(s);
}

// ================= acciones =================
void ponerMatriz(bool on, const char *origen) {
  matrizOn = on; cuadro = 0; tCuadro = millis();
  mostrarCuadro();
  evento(String(on ? "Matriz encendida (" : "Matriz apagada (") + origen + ")");
}
void ponerBuzzer(bool on, const char *origen) {
  buzzerOn = on; tBuzzer = millis();
  digitalWrite(P_BUZZER, on ? HIGH : LOW);
  evento(String(on ? "Buzzer 10 s (" : "Buzzer apagado (") + origen + ")");
}
void ponerPuerta(bool abrir, const char *origen) {
  if (abrir == puertaAbierta) return;
  puertaAbierta = abrir;
  anguloMeta = abrir ? ANG_ABIERTO : ANG_CERRADO;
  if (abrir) tClaro = millis();   // si se abre a mano, da 3 s antes de cerrar sola
  evento(String(abrir ? "Compuerta abriendo (" : "Compuerta cerrando (") + origen + ")");
}

void guardar() {
  EEPROM.put(0, (byte)0xC5);
  EEPROM.put(1, umbral);
  EEPROM.put(3, sensVib);
  EEPROM.put(4, brillo);
  EEPROM.put(5, (byte)modoAuto);
}
bool sinAjustes = false;
void cargar() {
  byte firma; EEPROM.get(0, firma);
  if (firma != 0xC5) { sinAjustes = true; return; }
  EEPROM.get(1, umbral);
  EEPROM.get(3, sensVib);
  EEPROM.get(4, brillo);
  byte a; EEPROM.get(5, a); modoAuto = a;
  umbral = constrain(umbral, 0, 1023);
  sensVib = constrain(sensVib, 1, 20);
  brillo = constrain(brillo, 0, 15);
}

// ================= ordenes =================
void orden(String s) {
  s.trim(); s.toUpperCase();
  if (s.length() == 0) return;
  int dos = s.indexOf(':');
  String cmd = dos < 0 ? s : s.substring(0, dos);
  int val = dos < 0 ? -1 : s.substring(dos + 1).toInt();

  if (cmd == "MATRIZ")       ponerMatriz(val == 1, "app");
  else if (cmd == "BUZZER")  ponerBuzzer(val == 1, "app");
  else if (cmd == "PUERTA")  ponerPuerta(val == 1, "app");
  else if (cmd == "AUTO")    { modoAuto = val == 1; guardar(); evento(modoAuto ? "Compuerta en automatico" : "Compuerta en manual"); }
  else if (cmd == "SONIDO")  { sonidoOn = val == 1; evento(sonidoOn ? "Aplausos activados" : "Aplausos desactivados"); }
  else if (cmd == "VIBRA")   { vibraOn = val == 1; evento(vibraOn ? "Sensor de vibracion activado" : "Sensor de vibracion desactivado"); }
  else if (cmd == "UMBRAL" && val >= 0)  { umbral = constrain(val, 0, 1023); guardar(); evento("Umbral de luz = " + String(umbral)); }
  else if (cmd == "SENSVIB" && val >= 0) { sensVib = constrain(val, 1, 20); guardar(); evento("Sensibilidad vibracion = " + String(sensVib)); }
  else if (cmd == "BRILLO" && val >= 0)  { brillo = constrain(val, 0, 15); mx(0x0A, brillo); guardar(); }
  else if (cmd == "CAL") {
    // calibra con la luz que hay ahora: oscuro = 55 % de la luz normal
    long suma = 0;
    for (byte i = 0; i < 16; i++) { suma += analogRead(P_LDR); delay(5); }
    umbral = (suma / 16) * 55 / 100;
    guardar();
    evento("Calibrado: umbral = " + String(umbral));
  }
  else if (cmd == "DIAG") { diag = val == 1; return; }
  else if (cmd == "ESTADO") { /* se responde abajo */ }
  else { enviar("ERR," + s); return; }
  enviarEstado();
}
void leer(Stream &p, String &lin) {
  while (p.available()) {
    char c = p.read();
    if (c == '\n' || c == '\r') { if (lin.length()) orden(lin); lin = ""; }
    else if (lin.length() < 32) lin += c;
  }
}

// ================= setup / loop =================
void setup() {
  Serial.begin(9600);
  bt.begin(9600);
  pinMode(P_SONIDO, INPUT);
  pinMode(P_VIBRA, INPUT);
  pinMode(P_BUZZER, OUTPUT);
  digitalWrite(P_BUZZER, LOW);
  cargar();
  // aprender el nivel de silencio del sensor de sonido (0,6 s)
  int altos = 0;
  for (int i = 0; i < 300; i++) { altos += digitalRead(P_SONIDO); delay(2); }
  SONIDO_ACTIVO = altos > 150 ? LOW : HIGH;
  ultimoSonido = !SONIDO_ACTIVO;
  ultimoVibra = digitalRead(P_VIBRA);
  luz = analogRead(P_LDR);
  mxIniciar();
  mxDibujar(0);
  compuerta.attach(P_SERVO);
  compuerta.write(ANG_CERRADO);
  tMovio = millis();
  if (sinAjustes) orden("CAL"); // la primera vez se calibra solo con la luz del lugar
  evento("Casa semiautomatica lista");
  evento(String("Sensor de sonido: activo en ") + (SONIDO_ACTIVO ? "HIGH" : "LOW") + ", silencio " + String(altos / 3) + "% alto");
  enviarEstado();
}

void loop() {
  unsigned long ahora = millis();
  leer(Serial, linUsb);
  leer(bt, linBt);

  // ignorar los sensores mientras el servo se mueve (su ruido y su temblor los engañan)
  // (y los primeros 2 s tras encender, porque el servo da un tiron)
  bool servoQuieto = (ahora - tMovio) > 800 && ahora > 2000;

  // ---- aplauso ----
  int s = digitalRead(P_SONIDO);
  if (s == SONIDO_ACTIVO && ultimoSonido != SONIDO_ACTIVO) {
    if (sonidoOn && !buzzerOn && servoQuieto && ahora - tAplauso > BLOQUEO_APLAUSO) {
      tAplauso = ahora;
      ponerMatriz(!matrizOn, "aplauso");
    }
  }
  ultimoSonido = s;
  if (s == SONIDO_ACTIVO) dgSonido++;

  // ---- animacion de la matriz ----
  if (matrizOn && ahora - tCuadro >= CUADRO_MS) {
    tCuadro = ahora;
    cuadro = (cuadro + 1) % 4;
    mostrarCuadro();
  }

  // ---- vibracion moderada ----
  int v = digitalRead(P_VIBRA);
  if (v != ultimoVibra) { cambiosVibra++; dgVibra++; ultimoVibra = v; }
  if (ahora - tVentana >= VENTANA_VIBRA) {
    if (vibraOn && !buzzerOn && servoQuieto && cambiosVibra >= sensVib) {
      ponerBuzzer(true, "vibracion");
    }
    cambiosVibra = 0;
    tVentana = ahora;
  }

  // ---- buzzer 10 s ----
  if (buzzerOn && ahora - tBuzzer >= BUZZER_MS) ponerBuzzer(false, "fin 10 s");

  // ---- fotoresistencia y compuerta ----
  static unsigned long tLuz = 0;
  if (ahora - tLuz >= 50) {
    tLuz = ahora;
    luz = (luz * 3 + analogRead(P_LDR)) / 4;      // suavizado
    bool oscuro = luz < umbral;
    if (oscuro != estabaOscuro) { estabaOscuro = oscuro; if (oscuro) tOscuro = ahora; else tClaro = ahora; }
    if (modoAuto) {
      if (oscuro && !puertaAbierta && ahora - tOscuro >= OSCURO_MS) ponerPuerta(true, "auto detectado");
      if (!oscuro && puertaAbierta && ahora - tClaro >= CIERRE_MS) ponerPuerta(false, "auto ya paso");
    }
  }

  // ---- movimiento suave del servo ----
  if (anguloActual != anguloMeta && ahora - tServo >= 12) {
    tServo = ahora;
    anguloActual += anguloMeta > anguloActual ? 2 : -2;
    if (abs(anguloActual - anguloMeta) < 2) anguloActual = anguloMeta;
    compuerta.write(anguloActual);
    tMovio = ahora;
  }

  // ---- diagnostico ----
  static unsigned long tDiag = 0;
  if (diag && ahora - tDiag >= 250) {
    tDiag = ahora;
    enviar("DG," + String(digitalRead(P_SONIDO)) + "," + String(dgSonido) + "," + String(digitalRead(P_VIBRA)) + "," + String(dgVibra) + "," + String(analogRead(P_LDR)));
    dgSonido = 0; dgVibra = 0;
  }

  // ---- estado ----
  if (ahora - tEstado >= 500) { tEstado = ahora; enviarEstado(); }
}
