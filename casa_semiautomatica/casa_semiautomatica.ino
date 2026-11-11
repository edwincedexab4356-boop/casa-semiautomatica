/*
  CASA SEMIAUTOMATICA - Arduino UNO
  ---------------------------------
  - Aplauso (sensor de sonido)  -> enciende / apaga la matriz 8x8
                                   (corazon, apagado, carita, apagado... en bucle)
  - Vibracion moderada (SW-420) -> buzzer activo continuo por 10 segundos
  - Sensor ultrasonico HC-SR04  -> si un auto se acerca a menos del umbral (cm)
                                   el servo abre la compuerta a 90 grados; se
                                   cierra sola cuando el auto se va
  - HC-05 (Bluetooth) y USB     -> control total desde la pagina web

  CONEXIONES
  Matriz MAX7219 : VCC 5V, GND, DIN D7, CS D6, CLK D5
  Sensor sonido  : VCC 5V, GND, DO  D2
  Sensor vibra.  : VCC 5V, GND, DO  D3
  Buzzer activo  : +  D4, -  GND
  Servo          : rojo 5V, cafe GND, naranja D9
  Ultrasonico    : VCC 5V, GND, TRIG D8, ECHO D12
  HC-05          : VCC 5V, GND, TXD -> D10, RXD <- D11 (con divisor 1k / 2k)

  Protocolo (9600, una orden por linea):
    MATRIZ:1/0  BUZZER:1/0  PUERTA:1/0  AUTO:1/0  SONIDO:1/0  VIBRA:1/0
    FIG1:hex16 FIG2:hex16 (dibujos)  FIGS
    UMBRAL:n (cm, 2-200)  SENSVIB:n (1-20)  BRILLO:n (0-15)  CAL (umbral = 60 % de la distancia sin auto)  PRUEBA  ESTADO
    DIAG:1/0 -> DG,sonido,muestras_sonido,vibra,cambios_vibra,distancia_cm (cada 250 ms)
  El Arduino responde cada 500 ms:
    ST,matriz,cuadro,buzzer,seg_restantes,puerta,auto,distancia_cm,umbral_cm,sonido,vibra,sensvib,brillo,angulo
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
const byte P_TRIG   = 8;    // ultrasonico HC-SR04
const byte P_ECHO   = 12;

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
const unsigned long CERCA_MS       = 300;     // el auto tiene que estar 0,3 s seguidos
const unsigned long SOLTAR_SERVO   = 500;     // suelta el servo 0,5 s despues de llegar (no zumba ni tiembla)
const int  DIST_MAX = 300;                    // sin eco = lejos (300 cm)

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

byte figs[2][8];               // las 2 figuras que se ven (editables desde la pagina)

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
unsigned long tCerca = 0, tClaro = 0;
bool estabaCerca = false;

bool sonidoOn = true, vibraOn = true;
int  umbral = 10;              // cm: mas cerca que esto = hay auto
unsigned long pruebaHasta = 0; // prueba de la matriz (todas las luces)
bool servoPegado = false;
byte sensVib = 4;              // cambios necesarios en 300 ms = vibracion moderada
byte brillo = 6;
int  dist = DIST_MAX;           // cm medidos por el ultrasonico

unsigned long tAplauso = 0;
volatile bool aplausoNuevo = false;   // lo marcan las interrupciones (no se pierde nada
volatile byte cambiosVibra = 0;       // mientras el ultrasonico mide)
unsigned long tVentana = 0;

unsigned long tEstado = 0;
bool diag = false;             // DIAG:1 muestra las lecturas crudas
volatile unsigned int dgSonido = 0, dgVibra = 0;
volatile unsigned long tSonIni = 0, tRafaga = 0;
volatile byte pulsosRafaga = 0;
void isrSonido() {
  unsigned long t = micros();
  if (digitalRead(P_SONIDO) == SONIDO_ACTIVO) tSonIni = t | 1;
  else if (tSonIni && t - tSonIni >= 100) {
    // un aplauso da una rafaga de pulsos; un ruido suelto da uno solo
    if (t - tRafaga > 60000UL) { tRafaga = t; pulsosRafaga = 0; }
    if (++pulsosRafaga == 3) aplausoNuevo = true;
    dgSonido++; tSonIni = 0;
  }
  else tSonIni = 0;
}
void isrVibra()  { if (cambiosVibra < 250) cambiosVibra++; dgVibra++; }
String linUsb, linBt;

// ================= MAX7219 =================
// envio lento: con cables largos o ruido del servo no se pierden bits
void mxByte(byte b) {
  for (byte i = 0; i < 8; i++) {
    digitalWrite(P_CLK, LOW);
    digitalWrite(P_DIN, (b & 0x80) ? HIGH : LOW);
    b <<= 1;
    delayMicroseconds(2);
    digitalWrite(P_CLK, HIGH);
    delayMicroseconds(2);
  }
}
void mx(byte reg, byte val) {

  digitalWrite(P_CS, LOW);
  mxByte(reg);
  mxByte(val);
  digitalWrite(P_CLK, LOW);
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
  if (cuadro == 0) mxDibujar(figs[0]);
  else if (cuadro == 2) mxDibujar(figs[1]);
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
  s += dist; s += ','; s += umbral; s += ',';
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
  EEPROM.put(0, (byte)0xC7);
  EEPROM.put(1, umbral);
  EEPROM.put(3, sensVib);
  EEPROM.put(4, brillo);
  EEPROM.put(5, (byte)modoAuto);
}
const char HEX_D[] = "0123456789ABCDEF";
void enviarFigs() {
  for (byte k = 0; k < 2; k++) {
    String s = "FG," + String(k + 1) + ",";
    for (byte i = 0; i < 8; i++) { s += HEX_D[figs[k][i] >> 4]; s += HEX_D[figs[k][i] & 15]; }
    enviar(s);
  }
}
void cargarFigs() {
  memcpy(figs[0], CORAZON, 8); memcpy(figs[1], CARITA, 8);
  byte f; EEPROM.get(10, f);
  if (f == 0xA7) for (byte i = 0; i < 16; i++) figs[i / 8][i % 8] = EEPROM.read(11 + i);
}
void guardarFigs() {
  EEPROM.update(10, 0xA7);
  for (byte i = 0; i < 16; i++) EEPROM.update(11 + i, figs[i / 8][i % 8]);
}
int hexVal(char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1; }
bool sinAjustes = false;
void cargar() {
  byte firma; EEPROM.get(0, firma);
  if (firma != 0xC7) { sinAjustes = true; return; }
  EEPROM.get(1, umbral);
  EEPROM.get(3, sensVib);
  EEPROM.get(4, brillo);
  byte a; EEPROM.get(5, a); modoAuto = a;
  umbral = constrain(umbral, 2, 200);
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
  else if (cmd == "UMBRAL" && val >= 0)  { umbral = constrain(val, 2, 200); guardar(); evento("Distancia para abrir = " + String(umbral) + " cm"); }
  else if (cmd == "SENSVIB" && val >= 0) { sensVib = constrain(val, 1, 20); guardar(); evento("Sensibilidad vibracion = " + String(sensVib)); }
  else if (cmd == "BRILLO" && val >= 0)  { brillo = constrain(val, 0, 15); mx(0x0A, brillo); guardar(); }
  else if (cmd == "CAL") {
    // calibra con el camino vacio: abre si algo esta a menos del 60 % de esa distancia
    int d[5];
    for (byte i = 0; i < 5; i++) { d[i] = medir(); delay(40); }
    for (byte i = 0; i < 5; i++) for (byte j = i + 1; j < 5; j++) if (d[j] < d[i]) { int t = d[i]; d[i] = d[j]; d[j] = t; }
    if (d[2] >= DIST_MAX) {
      umbral = 15; guardar();
      evento("El ultrasonico no da eco (revisa TRIG D8 y ECHO D12). Abre a menos de 15 cm");
    } else {
      umbral = constrain((long)d[2] * 60 / 100, 3, 100);
      guardar();
      evento("Calibrado: sin auto " + String(d[2]) + " cm, abre a menos de " + String(umbral) + " cm");
    }
  }
  else if ((cmd == "FIG1" || cmd == "FIG2") && dos > 0) {
    // FIG1:0066FFFF7E3C1800 = 8 filas en hexadecimal (bit alto = columna izquierda)
    String h = s.substring(dos + 1);
    if (h.length() != 16) { enviar("ERR," + s); return; }
    byte nueva[8];
    for (byte i = 0; i < 8; i++) {
      int a = hexVal(h[i * 2]), b = hexVal(h[i * 2 + 1]);
      if (a < 0 || b < 0) { enviar("ERR," + s); return; }
      nueva[i] = a * 16 + b;
    }
    byte k = cmd == "FIG1" ? 0 : 1;
    memcpy(figs[k], nueva, 8); guardarFigs();
    // la muestra 2 s en la matriz para verla
    mx(0x0F, 0); mxDibujar(figs[k]); pruebaHasta = millis() + 2000;
    evento(String("Figura ") + (k + 1) + " guardada");
    enviarFigs();
  }
  else if (cmd == "FIGS") { enviarFigs(); return; }
  else if (cmd == "PRUEBA") { mx(0x0F, 1); pruebaHasta = millis() + 3000; evento("Prueba de la matriz: 64 luces prendidas 3 s"); }
  else if (cmd == "DIAG") { diag = val == 1; return; }
  else if (cmd == "ESTADO") { enviarFigs(); }
  else { enviar("ERR," + s); return; }
  enviarEstado();
}
int medir() {   // distancia en cm (DIST_MAX si no hay eco)
  digitalWrite(P_TRIG, LOW); delayMicroseconds(3);
  digitalWrite(P_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(P_TRIG, LOW);
  unsigned long us = pulseIn(P_ECHO, HIGH, 18000UL);   // 18 ms = unos 3 m
  if (us == 0) return DIST_MAX;
  return min((long)(us / 58), (long)DIST_MAX);
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
  cargarFigs();
  // aprender el nivel de silencio del sensor de sonido (0,6 s)
  int altos = 0;
  for (int i = 0; i < 300; i++) { altos += digitalRead(P_SONIDO); delay(2); }
  SONIDO_ACTIVO = altos > 150 ? LOW : HIGH;
  pinMode(P_TRIG, OUTPUT); digitalWrite(P_TRIG, LOW);
  pinMode(P_ECHO, INPUT);
  dist = medir();
  attachInterrupt(digitalPinToInterrupt(P_SONIDO), isrSonido, CHANGE);
  attachInterrupt(digitalPinToInterrupt(P_VIBRA), isrVibra, CHANGE);
  mxIniciar();
  mxDibujar(0);
  pinMode(P_SERVO, OUTPUT);
  compuerta.attach(P_SERVO); servoPegado = true;
  compuerta.write(ANG_CERRADO);
  tMovio = millis();
  // prueba al encender: las 64 luces prendidas 1,5 s (si alguna no prende, es la matriz o sus cables)
  mx(0x0F, 1); pruebaHasta = millis() + 1500;
  if (sinAjustes) orden("CAL"); // la primera vez se calibra solo con el camino vacio
  evento("Casa semiautomatica lista");
  evento(String("Sensor de sonido: activo en ") + (SONIDO_ACTIVO ? "HIGH" : "LOW") + ", silencio " + String(altos / 3) + "% alto");
  enviarFigs();
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
  if (aplausoNuevo) {
    aplausoNuevo = false;
    if (sonidoOn && !buzzerOn && servoQuieto && ahora - tAplauso > BLOQUEO_APLAUSO) {
      tAplauso = ahora;
      ponerMatriz(!matrizOn, "aplauso");
    }
  }

  // ---- animacion de la matriz ----
  if (matrizOn && ahora - tCuadro >= CUADRO_MS) {
    tCuadro = ahora;
    cuadro = (cuadro + 1) % 4;
    mostrarCuadro();
  }

  // ---- vibracion moderada ----
  if (ahora - tVentana >= VENTANA_VIBRA) {
    if (vibraOn && !buzzerOn && servoQuieto && cambiosVibra >= sensVib) {
      ponerBuzzer(true, "vibracion");
    }
    cambiosVibra = 0;
    tVentana = ahora;
  }

  // ---- buzzer 10 s ----
  if (buzzerOn && ahora - tBuzzer >= BUZZER_MS) ponerBuzzer(false, "fin 10 s");

  // ---- ultrasonico y compuerta ----
  static unsigned long tDist = 0;
  static int m[3] = {DIST_MAX, DIST_MAX, DIST_MAX};
  static byte im = 0;
  if (ahora - tDist >= 70) {
    tDist = ahora;
    m[im] = medir(); im = (im + 1) % 3;
    // mediana de las 3 ultimas: un eco falso suelto no abre la compuerta
    int a = m[0], b = m[1], c = m[2];
    dist = max(min(a, b), min(max(a, b), c));
    bool cerca = dist < umbral;
    if (cerca != estabaCerca) { estabaCerca = cerca; if (cerca) tCerca = ahora; else tClaro = ahora; }
    if (modoAuto) {
      if (cerca && !puertaAbierta && ahora - tCerca >= CERCA_MS) ponerPuerta(true, "auto detectado");
      if (!cerca && puertaAbierta && ahora - tClaro >= CIERRE_MS) ponerPuerta(false, "auto ya paso");
    }
  }

  // ---- movimiento suave del servo ----
  if (anguloActual != anguloMeta && ahora - tServo >= 12) {
    tServo = ahora;
    anguloActual += anguloMeta > anguloActual ? 2 : -2;
    if (abs(anguloActual - anguloMeta) < 2) anguloActual = anguloMeta;
    if (!servoPegado) { compuerta.attach(P_SERVO); servoPegado = true; }
    compuerta.write(anguloActual);
    tMovio = ahora;
  }
  if (servoPegado && anguloActual == anguloMeta && ahora - tMovio >= SOLTAR_SERVO) {
    compuerta.detach(); servoPegado = false;   // sin senal el servo queda quieto y en silencio
    digitalWrite(P_SERVO, LOW);
  }

  // ---- la matriz se reconfigura cada segundo (el ruido del servo la puede desordenar) ----
  static unsigned long tMx = 0;
  if (pruebaHasta && ahora >= pruebaHasta) { pruebaHasta = 0; mx(0x0F, 0); mostrarCuadro(); }
  if (!pruebaHasta && ahora - tMx >= 300) { tMx = ahora; mxIniciar(); mostrarCuadro(); }

  // ---- diagnostico ----
  static unsigned long tDiag = 0;
  if (diag && ahora - tDiag >= 250) {
    tDiag = ahora;
    enviar("DG," + String(digitalRead(P_SONIDO)) + "," + String(dgSonido) + "," + String(digitalRead(P_VIBRA)) + "," + String(dgVibra) + "," + String(dist));
    dgSonido = 0; dgVibra = 0;
  }

  // ---- estado ----
  if (ahora - tEstado >= 500) { tEstado = ahora; enviarEstado(); }
}
