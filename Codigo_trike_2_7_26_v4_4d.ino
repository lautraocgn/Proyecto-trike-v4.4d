//==============================================================================
// Proyecto : Selector electrónico VW Autostick
// Hardware : Arduino Nano ATmega328P
// Firmware : v4.4d
// Estado   : Pruebas en banco
// Fecha    : 10/07/2026
//==============================================================================

#include <EEPROM.h>

// =============================================================================
// CONSTANTES CONFIGURABLES
// =============================================================================

// — Tiempos de botones —
const uint16_t DEBOUNCE_MS               = 20;
const uint16_t BLOQUEO_MS                = 500;
const uint16_t TIEMPO_PULSACION_LARGA_MS = 600;

// — Tiempos de operación —
const uint16_t TIMEOUT_MS                = 3000;
const uint16_t RETARDO_RELE_MS           = 50;
const uint16_t ESPERA_FC_S_MS            = 150;
const uint16_t TIEMPO_LECTURA_POT        = 20;

// — Tolerancia de posición —
const uint16_t TOLERANCIA_ADC            = 30;

// — Modo aprendizaje —
const uint16_t PASO_APRENDIZAJE_MS       = 100;

// — Patrones de parpadeo —
const uint16_t PARPADEO_ERROR_ON         = 250;
const uint16_t PARPADEO_ERROR_OFF        = 250;
const uint16_t PARPADEO_APREND_ON        = 150;
const uint16_t PARPADEO_APREND_OFF       = 150;
const uint16_t PARPADEO_APREND_PAUSA     = 600;
const uint16_t PARPADEO_CONFIRM_MS       = 500;
const uint16_t PARPADEO_AVISO_EEPROM_MS  = 5000;

// — EEPROM —
const uint8_t  EEPROM_FIRMA_ADDR         = 0;
const uint8_t  EEPROM_FIRMA_VAL          = 0xA5;
const uint8_t  EEPROM_POS_BASE           = 2;

// — Valores ADC por defecto —
const uint16_t DEFAULT_POS_R             = 228;
const uint16_t DEFAULT_POS_1             = 294;
const uint16_t DEFAULT_POS_N             = 500;
const uint16_t DEFAULT_POS_2             = 644;

//======================================================
// DEBUG
//======================================================

#define DEBUG 1

#if DEBUG

  #define DBG(x)        Serial.print(x)
  #define DBGLN(x)      Serial.println(x)

#else

  #define DBG(x)
  #define DBGLN(x)

#endif

uint32_t ultimoDebugEstado = 0;
const uint16_t DEBUG_ESTADO_MS = 100;

// =============================================================================
// PINOUT
// =============================================================================

const uint8_t PIN_BTN_UP       = 3;
const uint8_t PIN_BTN_DOWN     = 2;
const uint8_t PIN_REL_IN       = 4;
const uint8_t PIN_REL_OUT      = 5;
const uint8_t PIN_K2           = 6;
const uint8_t PIN_LED_R        = 7;
const uint8_t PIN_LED_N        = 8;
const uint8_t PIN_LED_1        = 9;
const uint8_t PIN_LED_2        = 10;
const uint8_t PIN_BTN_MODO     = 11;
const uint8_t PIN_BTN_CONF     = 12;
const uint8_t PIN_K1           = 13;
const uint8_t PIN_POT          = A0;
const uint8_t PIN_FC_S         = A1;
const uint8_t PIN_FC_C         = A2;

// =============================================================================
// ENUMERACIONES
// =============================================================================

enum Marcha {
  MARCHA_R = 0,
  MARCHA_N = 1,
  MARCHA_1 = 2,
  MARCHA_2 = 3
};

enum Estado {
  ARRANQUE,
  REPOSO,
  ESPERANDO_FC_C,
  MOVIENDO,
  ESPERANDO_FC_S,
  ESPERA_FC_S_RETORNO,
  EMERGENCIA_A_N,
  RECUPERANDO_A_N,
  ERROR_LEVE,
  ERROR_GRAVE,
  MODO_APRENDIZAJE,
  APRENDIZAJE_MOVIENDO,
  APRENDIZAJE_CONFIRMANDO
};

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================

Estado estadoActual     = ARRANQUE;
Marcha marchaActual     = MARCHA_N;
Marcha marchaDestino    = MARCHA_N;
Marcha marchaOrigen     = MARCHA_N;
Marcha marchaError      = MARCHA_N;

uint16_t posADC[4];

uint32_t tiempoInicio   = 0;
uint32_t tiempoAux      = 0;

uint32_t ultimaLecturaPot = 0;
uint16_t valorPot         = 0;

bool relInActivo  = false;
bool relOutActivo = false;
uint32_t tiempoApagadoRele = 0;
bool esperandoRetardoRele  = false;
bool relPendienteIn        = false;
bool relPendienteOut       = false;
bool cambioCarrilPendiente = false;

struct Boton {
  uint8_t  pin;
  bool     estadoAnterior;
  bool     estadoActual;
  bool     presionado;
  bool     soltado;
  uint32_t tiempoPresion;
  uint32_t ultimoCambio;
  bool     bloqueado;
};

Boton btnUp   = {PIN_BTN_UP,   true, true, false, false, 0, 0, false};
Boton btnDown = {PIN_BTN_DOWN, true, true, false, false, 0, 0, false};
Boton btnModo = {PIN_BTN_MODO, true, true, false, false, 0, 0, false};
Boton btnConf = {PIN_BTN_CONF, true, true, false, false, 0, 0, false};

bool     bloqueoCambio     = false;
uint32_t tiempoBloqueo     = 0;

uint32_t tiempoLed         = 0;
bool     ledEstado         = false;
uint8_t  contadorDestellos = 0;
uint8_t  faseParpadeo      = 0;

Marcha marchaAprendizaje   = MARCHA_N;

bool   avisandoEEPROM      = false;
uint32_t tiempoAvisoEEPROM = 0;

Estado ultimoEstadoLog     = ARRANQUE;
uint32_t ultimoLogPot      = 0;

// =============================================================================
// PROTOTIPOS
// =============================================================================

void leerBotones();
void actualizarBoton(Boton &btn);
bool upDownSimultaneos();
void activarReleIn();
void activarReleOut();
void apagarActuador();
void gestionarRetardoRele();
void activarK1();
void desactivarK1();
void activarK2();
void desactivarK2();
void apagarTodoReles();
uint16_t leerPot();
bool enPosicion(uint16_t pos);
void moverHacia(uint16_t pos);
void guardarPosEnEEPROM(Marcha m, uint16_t valor);
void cargarPosiciones();
uint8_t pinLED(Marcha m);
void apagarTodosLEDs();
void encenderLED(Marcha m);
void manejarParpadeoSimple(uint8_t pin);
void manejarParpadeoAprendizaje(uint8_t pin);
void manejarParpadeoGrave();
void estadoArranque();
void estadoReposo();
void estadoEsperandoFCC();
void estadoMoviendo();
void estadoEsperandoFCS();
void estadoEsperaFCSRetorno();
void estadoEmergenciaN();
void estadoRecuperandoN();
void estadoErrorLeve();
void estadoErrorGrave();
void estadoModoAprendizaje();
void estadoAprendizajeMoviendo();
void estadoAprendizajeConfirmando();
void iniciarCambioA(Marcha destino);
void entrarErrorGrave();
const char* nombreMarcha(Marcha m);
const char* nombreEstado(Estado e);
void logEstado();
void logPot();
void logAccion(const char* accion);

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(9600);
    delay(2000);

  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_MODO, INPUT_PULLUP);
  pinMode(PIN_BTN_CONF, INPUT_PULLUP);
  pinMode(PIN_FC_S,     INPUT_PULLUP);
  pinMode(PIN_FC_C,     INPUT_PULLUP);

  pinMode(PIN_REL_IN,  OUTPUT); digitalWrite(PIN_REL_IN,  LOW);
  pinMode(PIN_REL_OUT, OUTPUT); digitalWrite(PIN_REL_OUT, LOW);
  pinMode(PIN_K1,      OUTPUT); digitalWrite(PIN_K1,      LOW);
  pinMode(PIN_K2,      OUTPUT); digitalWrite(PIN_K2,      LOW);
  pinMode(PIN_LED_R,   OUTPUT); digitalWrite(PIN_LED_R,   LOW);
  pinMode(PIN_LED_N,   OUTPUT); digitalWrite(PIN_LED_N,   LOW);
  pinMode(PIN_LED_1,   OUTPUT); digitalWrite(PIN_LED_1,   LOW);
  pinMode(PIN_LED_2,   OUTPUT); digitalWrite(PIN_LED_2,   LOW);

  cargarPosiciones();

  estadoActual = ARRANQUE;
  tiempoInicio = millis();

#if DEBUG
  Serial.println();
  Serial.println(F("================================="));
  Serial.println(F(" TRIKE AUTOSTICK  -  VERSION 4.4d "));
  Serial.println(F("================================="));
  Serial.println(F("Inicializando sistema..."));
  Serial.println();

  Serial.print(F("EEPROM R : "));
  Serial.println(posADC[MARCHA_R]);

  Serial.print(F("EEPROM N : "));
  Serial.println(posADC[MARCHA_N]);

  Serial.print(F("EEPROM 1 : "));
  Serial.println(posADC[MARCHA_1]);

  Serial.print(F("EEPROM 2 : "));
  Serial.println(posADC[MARCHA_2]);

  Serial.print(F("Potenciómetro actual : "));
  Serial.println(analogRead(PIN_POT));

  Serial.println(F("Entradas:"));

  Serial.print(F("A2 FC_C : "));
  Serial.println(digitalRead(PIN_FC_C) ? F("HIGH") : F("LOW"));

  Serial.print(F("A1 FC_S : "));
  Serial.println(digitalRead(PIN_FC_S) ? F("HIGH") : F("LOW"));

  Serial.println();

Serial.print(F("ADC FC_C : "));
Serial.print(analogRead(PIN_FC_C));
Serial.println("  (lectura analógica)");

Serial.print(F("ADC FC_S : "));
Serial.print(analogRead(PIN_FC_S));
Serial.println("  (lectura analógica)");

Serial.println();

  Serial.println();
  Serial.println(F("Entrando en estado ARRANQUE"));
  Serial.println(F("================================="));
  Serial.println();
#endif
}
// =============================================================================
// LOOP PRINCIPAL
// =============================================================================

void loop() {
  uint32_t ahora = millis();

  leerBotones();
  gestionarRetardoRele();
  logEstado();
  // logPot();   // Se elimina el registro continuo. El potenciómetro
                // se mostrará únicamente en eventos importantes.

  bool modoAprendizajeActivo = (digitalRead(PIN_BTN_MODO) == LOW);

  if (modoAprendizajeActivo &&
      estadoActual != MODO_APRENDIZAJE &&
      estadoActual != APRENDIZAJE_MOVIENDO &&
      estadoActual != APRENDIZAJE_CONFIRMANDO &&
      estadoActual != ERROR_GRAVE) {
    logAccion("Entrando en MODO APRENDIZAJE");
    apagarTodoReles();
    estadoActual      = MODO_APRENDIZAJE;
    marchaAprendizaje = MARCHA_N;
    faseParpadeo      = 0;
    contadorDestellos = 0;
    tiempoLed         = ahora;
    valorPot = leerPot();
    if (!enPosicion(posADC[MARCHA_N])) {
      moverHacia(posADC[MARCHA_N]);
    }
    return;
  }

  if (!modoAprendizajeActivo &&
      (estadoActual == MODO_APRENDIZAJE ||
       estadoActual == APRENDIZAJE_MOVIENDO ||
       estadoActual == APRENDIZAJE_CONFIRMANDO)) {
    logAccion("Saliendo de MODO APRENDIZAJE -> ARRANQUE");
    apagarTodoReles();
    estadoActual = ARRANQUE;
    tiempoInicio = ahora;
    return;
  }

  switch (estadoActual) {
    case ARRANQUE:                estadoArranque();              break;
    case REPOSO:                  estadoReposo();                break;
    case ESPERANDO_FC_C:          estadoEsperandoFCC();          break;
    case MOVIENDO:                estadoMoviendo();              break;
    case ESPERANDO_FC_S:          estadoEsperandoFCS();          break;
    case ESPERA_FC_S_RETORNO:     estadoEsperaFCSRetorno();      break;
    case EMERGENCIA_A_N:          estadoEmergenciaN();           break;
    case RECUPERANDO_A_N:         estadoRecuperandoN();          break;
    case ERROR_LEVE:              estadoErrorLeve();             break;
    case ERROR_GRAVE:             estadoErrorGrave();            break;
    case MODO_APRENDIZAJE:        estadoModoAprendizaje();       break;
    case APRENDIZAJE_MOVIENDO:    estadoAprendizajeMoviendo();   break;
    case APRENDIZAJE_CONFIRMANDO: estadoAprendizajeConfirmando();break;
  }
}

// =============================================================================
// MONITOR SERIE
// =============================================================================

const char* nombreMarcha(Marcha m) {
  switch (m) {
    case MARCHA_R: return "R";
    case MARCHA_N: return "N";
    case MARCHA_1: return "1";
    case MARCHA_2: return "2";
  }
  return "?";
}

const char* nombreEstado(Estado e) {
  switch (e) {
    case ARRANQUE:                return "ARRANQUE";
    case REPOSO:                  return "REPOSO";
    case ESPERANDO_FC_C:          return "ESPERANDO_FC_C";
    case MOVIENDO:                return "MOVIENDO";
    case ESPERANDO_FC_S:          return "ESPERANDO_FC_S";
    case ESPERA_FC_S_RETORNO:     return "ESPERA_FC_S_RETORNO";
    case EMERGENCIA_A_N:          return "EMERGENCIA_A_N";
    case RECUPERANDO_A_N:         return "RECUPERANDO_A_N";
    case ERROR_LEVE:              return "ERROR_LEVE";
    case ERROR_GRAVE:             return "ERROR_GRAVE";
    case MODO_APRENDIZAJE:        return "MODO_APRENDIZAJE";
    case APRENDIZAJE_MOVIENDO:    return "APRENDIZAJE_MOVIENDO";
    case APRENDIZAJE_CONFIRMANDO: return "APRENDIZAJE_CONFIRMANDO";
  }
  return "?";
}

void logEstado() {
  if (estadoActual != ultimoEstadoLog) {
    Serial.print(F("["));
    Serial.print(millis());
    Serial.print(F(" ms] Estado: "));
    Serial.print(nombreEstado(ultimoEstadoLog));
    Serial.print(F(" -> "));
    Serial.println(nombreEstado(estadoActual));
    ultimoEstadoLog = estadoActual;
  }
}

void logPot() {

  static int ultimoValor = -1000;

  if (abs(valorPot - ultimoValor) >= 5) {

    Serial.print(F("Potenciómetro: "));
    Serial.println(valorPot);

    ultimoValor = valorPot;
  }
}

void logAccion(const char* accion) {

  Serial.print(F("["));
  Serial.print(millis());
  Serial.print(F(" ms] "));
  Serial.println(accion);

  Serial.print(F("Potenciómetro: "));
  Serial.println(valorPot);
}

// =============================================================================
// BOTONES
// =============================================================================

void leerBotones() {
  actualizarBoton(btnUp);
  actualizarBoton(btnDown);
  actualizarBoton(btnModo);
  actualizarBoton(btnConf);
}

void actualizarBoton(Boton &btn) {
  uint32_t ahora   = millis();
  bool     lectura = (digitalRead(btn.pin) == LOW);

  btn.presionado = false;
  btn.soltado    = false;

  if (lectura != btn.estadoAnterior) {
    if ((ahora - btn.ultimoCambio) >= DEBOUNCE_MS) {
      btn.estadoAnterior = lectura;
      btn.ultimoCambio   = ahora;
      if (lectura) {
        btn.tiempoPresion = ahora;
        btn.bloqueado     = false;
      } else {
        btn.soltado   = true;
        btn.bloqueado = false;
      }
    }
  }

  btn.estadoActual = lectura;

  if (btn.soltado && !btn.bloqueado) {
    btn.presionado = true;
    btn.bloqueado  = true;
  }
}

bool upDownSimultaneos() {
  return (btnUp.estadoActual && btnDown.estadoActual);
}

// =============================================================================
// RELÉS
// =============================================================================

void activarReleIn() {
  if (relInActivo) return;
  if (relOutActivo) {
    digitalWrite(PIN_REL_OUT, LOW);
    relOutActivo         = false;
    tiempoApagadoRele    = millis();
    esperandoRetardoRele = true;
    relPendienteIn       = true;
    relPendienteOut      = false;
    return;
  }
  digitalWrite(PIN_REL_IN, HIGH);
  relInActivo = true;
  logAccion("REL_IN activado (actuador IN)");
}

void activarReleOut() {
  if (relOutActivo) return;
  if (relInActivo) {
    digitalWrite(PIN_REL_IN, LOW);
    relInActivo          = false;
    tiempoApagadoRele    = millis();
    esperandoRetardoRele = true;
    relPendienteOut      = true;
    relPendienteIn       = false;
    return;
  }
  digitalWrite(PIN_REL_OUT, HIGH);
  relOutActivo = true;
  logAccion("REL_OUT activado (actuador OUT)");
}

void apagarActuador() {
  bool habia = relInActivo || relOutActivo;
  digitalWrite(PIN_REL_IN,  LOW);
  digitalWrite(PIN_REL_OUT, LOW);
  relInActivo          = false;
  relOutActivo         = false;
  esperandoRetardoRele = false;
  relPendienteIn       = false;
  relPendienteOut      = false;
  if (habia) logAccion("Actuador detenido");
}

void gestionarRetardoRele() {
  if (!esperandoRetardoRele) return;
  if ((millis() - tiempoApagadoRele) >= RETARDO_RELE_MS) {
    esperandoRetardoRele = false;
    if (relPendienteIn) {
      relPendienteIn = false;
      digitalWrite(PIN_REL_IN, HIGH);
      relInActivo = true;
      logAccion("REL_IN activado tras retardo");
    } else if (relPendienteOut) {
      relPendienteOut = false;
      digitalWrite(PIN_REL_OUT, HIGH);
      relOutActivo = true;
      logAccion("REL_OUT activado tras retardo");
    }
  }
}

void activarK1() {
  digitalWrite(PIN_K1, HIGH);
  logAccion("K1 activado (embrague)");
}

void desactivarK1() {
  digitalWrite(PIN_K1, LOW);
  logAccion("K1 desactivado (embrague)");
}

void activarK2() {
  digitalWrite(PIN_K2, HIGH);
  logAccion("K2 activado (servo carril)");
}

void desactivarK2() {
  digitalWrite(PIN_K2, LOW);
  logAccion("K2 desactivado (servo carril)");
}

void apagarTodoReles() {
  apagarActuador();
  digitalWrite(PIN_K1, LOW);
  digitalWrite(PIN_K2, LOW);
  logAccion("Todos los relés apagados");
}

// =============================================================================
// POTENCIÓMETRO
// =============================================================================

uint16_t leerPot() {
  return analogRead(PIN_POT);
}

bool enPosicion(uint16_t pos) {
  int16_t diff = (int16_t)valorPot - (int16_t)pos;
  return (diff >= -(int16_t)TOLERANCIA_ADC && diff <= (int16_t)TOLERANCIA_ADC);
}

void moverHacia(uint16_t pos)
{
    if (enPosicion(pos))
    {
        apagarActuador();
        return;
    }

    if (valorPot < pos)
    {
        activarReleOut();   // subir ADC
    }
    else
    {
        activarReleIn();    // bajar ADC
    }
}

// =============================================================================
// EEPROM
// =============================================================================

void cargarPosiciones() {
  if (EEPROM.read(EEPROM_FIRMA_ADDR) == EEPROM_FIRMA_VAL) {
    for (uint8_t i = 0; i < 4; i++) {
      uint16_t val;
      EEPROM.get(EEPROM_POS_BASE + i * 2, val);
      posADC[i] = val;
    }
    avisandoEEPROM = false;
    Serial.println(F("EEPROM: datos validos cargados"));
  } else {
    posADC[MARCHA_R] = DEFAULT_POS_R;
    posADC[MARCHA_N] = DEFAULT_POS_N;
    posADC[MARCHA_1] = DEFAULT_POS_1;
    posADC[MARCHA_2] = DEFAULT_POS_2;
    avisandoEEPROM    = true;
    tiempoAvisoEEPROM = millis();
    Serial.println(F("### EEPROM DEFAULT ###"));
  }
}

void guardarPosEnEEPROM(Marcha m, uint16_t valor) {
  posADC[m] = valor;
  EEPROM.put(EEPROM_POS_BASE + m * 2, valor);
  EEPROM.write(EEPROM_FIRMA_ADDR, EEPROM_FIRMA_VAL);
  Serial.print(F("EEPROM: guardado marcha "));
  Serial.print(nombreMarcha(m));
  Serial.print(F(" = "));
  Serial.println(valor);
}

// =============================================================================
// LEDs
// =============================================================================

uint8_t pinLED(Marcha m) {
  switch (m) {
    case MARCHA_R: return PIN_LED_R;
    case MARCHA_N: return PIN_LED_N;
    case MARCHA_1: return PIN_LED_1;
    case MARCHA_2: return PIN_LED_2;
  }
  return PIN_LED_N;
}

void apagarTodosLEDs() {
  digitalWrite(PIN_LED_R, LOW);
  digitalWrite(PIN_LED_N, LOW);
  digitalWrite(PIN_LED_1, LOW);
  digitalWrite(PIN_LED_2, LOW);
}

void encenderLED(Marcha m) {
  apagarTodosLEDs();
  digitalWrite(pinLED(m), HIGH);
}

void manejarParpadeoSimple(uint8_t pin) {
  uint32_t ahora    = millis();
  uint16_t intervalo = ledEstado ? PARPADEO_ERROR_ON : PARPADEO_ERROR_OFF;
  if ((ahora - tiempoLed) >= intervalo) {
    ledEstado = !ledEstado;
    digitalWrite(pin, ledEstado ? HIGH : LOW);
    tiempoLed = ahora;
  }
}

void manejarParpadeoAprendizaje(uint8_t pin) {
  uint32_t ahora = millis();
  bool     encender;
  uint16_t intervalo;

  if (faseParpadeo < 6) {
    encender  = (faseParpadeo % 2 == 0);
    intervalo = encender ? PARPADEO_APREND_ON : PARPADEO_APREND_OFF;
  } else {
    encender  = false;
    intervalo = PARPADEO_APREND_PAUSA;
  }

  if ((ahora - tiempoLed) >= intervalo) {
    tiempoLed = ahora;
    faseParpadeo++;
    if (faseParpadeo > 6) faseParpadeo = 0;
    digitalWrite(pin, encender ? HIGH : LOW);
  }
}

void manejarParpadeoGrave() {
  uint32_t ahora    = millis();
  uint16_t intervalo = ledEstado ? PARPADEO_ERROR_ON : PARPADEO_ERROR_OFF;
  if ((ahora - tiempoLed) >= intervalo) {
    ledEstado = !ledEstado;
    uint8_t val = ledEstado ? HIGH : LOW;
    digitalWrite(PIN_LED_R, val);
    digitalWrite(PIN_LED_N, val);
    digitalWrite(PIN_LED_1, val);
    digitalWrite(PIN_LED_2, val);
    tiempoLed = ahora;
  }
}

// =============================================================================
// ESTADOS
// =============================================================================

void estadoArranque() {

  uint32_t ahora = millis();

  // Aviso EEPROM sin bloquear el centrado
  if (avisandoEEPROM) {
    manejarParpadeoAprendizaje(PIN_LED_N);

    if ((ahora - tiempoAvisoEEPROM) >= PARPADEO_AVISO_EEPROM_MS) {
      avisandoEEPROM = false;
      apagarTodosLEDs();
      faseParpadeo = 0;
      Serial.println(F("Aviso EEPROM finalizado"));
    }
  }

  // Actualizar lectura del potenciómetro
  if ((ahora - ultimaLecturaPot) >= TIEMPO_LECTURA_POT) {
    ultimaLecturaPot = ahora;
    valorPot = leerPot();
  }

#if DEBUG
  static bool mensajeInicio = false;
  static bool ordenMovimientoMostrada = false;

  if (!mensajeInicio) {

    mensajeInicio = true;

    Serial.println();
    Serial.println(F("=========== ARRANQUE ==========="));

    Serial.print(F("Potenciómetro : "));
    Serial.println(valorPot);

    if (enPosicion(posADC[MARCHA_N])) {
      Serial.println(F("Marcha detectada : N"));
    } else {
      Serial.println(F("Marcha detectada : Desconocida"));
      Serial.println(F("Buscando N..."));
    }

    Serial.println(F("================================"));
    Serial.println();
  }
#endif

  if (enPosicion(posADC[MARCHA_N])) {

    apagarActuador();

    marchaActual = MARCHA_N;

    encenderLED(MARCHA_N);

    bloqueoCambio = false;

#if DEBUG
    Serial.println();
    Serial.println(F("===== ARRANQUE COMPLETADO ====="));

    Serial.println(F("Marcha : N"));

    Serial.print(F("Potenciómetro : "));
    Serial.println(valorPot);

    Serial.println(F("==============================="));
    Serial.println();
#endif

    estadoActual = REPOSO;

    logEstado();

    return;
  }

#if DEBUG
  if (!ordenMovimientoMostrada) {
    Serial.println(F("Moviendo actuador hacia N"));
    ordenMovimientoMostrada = true;
  }
#endif

  moverHacia(posADC[MARCHA_N]);

  if ((ahora - tiempoInicio) >= TIMEOUT_MS) {

#if DEBUG
    Serial.println();
    Serial.println(F("=========== ERROR GRAVE ==========="));
    Serial.println(F("No se pudo alcanzar N"));

    Serial.print(F("Potenciómetro : "));
    Serial.println(valorPot);

    Serial.println(F("Timeout de movimiento"));
    Serial.println();
#endif

    logAccion("TIMEOUT en arranque -> ERROR GRAVE");

    entrarErrorGrave();
  }
}

void estadoReposo() {

  uint32_t ahora = millis();

  // Fin del bloqueo entre cambios
  if (bloqueoCambio && (ahora - tiempoBloqueo) >= BLOQUEO_MS)
    bloqueoCambio = false;

  if (bloqueoCambio)
    return;

  // Emergencia: UP + DOWN siempre vuelve a N
  if (upDownSimultaneos() && marchaActual != MARCHA_N) {

#if DEBUG
    Serial.println();
    Serial.println(F("===== EMERGENCIA ====="));
    Serial.println(F("UP + DOWN pulsados"));
    Serial.println(F("Destino : N"));
    Serial.println();
#endif

    logAccion("EMERGENCIA: UP+DOWN -> N");

    marchaOrigen  = marchaActual;
    marchaDestino = MARCHA_N;

    tiempoInicio = ahora;

    activarK1();

    estadoActual = EMERGENCIA_A_N;

    return;
  }

  Marcha sig = marchaActual;

  // -------------------------
  // BOTON UP
  // -------------------------

  if (btnUp.presionado) {

    switch (marchaActual) {

      case MARCHA_N: sig = MARCHA_1; break;
      case MARCHA_1: sig = MARCHA_2; break;
      case MARCHA_R: sig = MARCHA_N; break;

      case MARCHA_2:
      default:
        manejarParpadeoSimple(pinLED(marchaActual));
        return;
    }

#if DEBUG
    Serial.println();
    Serial.println(F("===== CAMBIO SOLICITADO ====="));
    Serial.print(F("Marcha actual : "));
    Serial.println(nombreMarcha(marchaActual));
    Serial.print(F("Marcha destino: "));
    Serial.println(nombreMarcha(sig));
    Serial.print(F("Potenciómetro : "));
    Serial.println(valorPot);
    Serial.println(F("============================="));
    Serial.println();
#endif

    iniciarCambioA(sig);

    return;
  }

  // -------------------------
  // BOTON DOWN
  // -------------------------

  if (btnDown.presionado) {

    switch (marchaActual) {

      case MARCHA_N: sig = MARCHA_R; break;
      case MARCHA_1: sig = MARCHA_N; break;
      case MARCHA_2: sig = MARCHA_1; break;

      case MARCHA_R:
      default:
        manejarParpadeoSimple(pinLED(marchaActual));
        return;
    }

#if DEBUG
    Serial.println();
    Serial.println(F("===== CAMBIO SOLICITADO ====="));
    Serial.print(F("Marcha actual : "));
    Serial.println(nombreMarcha(marchaActual));
    Serial.print(F("Marcha destino: "));
    Serial.println(nombreMarcha(sig));
    Serial.print(F("Potenciómetro : "));
    Serial.println(valorPot);
    Serial.println(F("============================="));
    Serial.println();
#endif

    iniciarCambioA(sig);

    return;
  }
}

void iniciarCambioA(Marcha destino)
{
    marchaOrigen  = marchaActual;
    marchaDestino = destino;

    tiempoInicio = millis();

    // Siempre partimos sin cambio de carril pendiente.
    // Será la máquina de estados quien decida posteriormente
    // si hay que activarlo (únicamente al entrar en R).
    cambioCarrilPendiente = false;

    activarK1();

    estadoActual = ESPERANDO_FC_C;
    logEstado();
}

void estadoEsperandoFCC()
{
  static bool mensajeMostrado = false;

  if (!mensajeMostrado)
  {
    Serial.println();
    Serial.println(F("=========== ESPERANDO FC_C ==========="));
    Serial.println(F("Esperando confirmacion del embrague..."));
    Serial.println();

    mensajeMostrado = true;
  }

  // Esperar a que el embrague confirme
  if (digitalRead(PIN_FC_C) == HIGH)
  {
    if (millis() - tiempoInicio > TIMEOUT_MS)
    {
      Serial.println();
      Serial.println(F("***** ERROR *****"));
      Serial.println(F("Timeout esperando FC_C"));
      Serial.println();

      mensajeMostrado = false;

      logAccion("Timeout esperando FC_C");

      marchaDestino = MARCHA_N;
      tiempoInicio = millis();

      estadoActual = RECUPERANDO_A_N;
      logEstado();
    }

    return;
  }

  mensajeMostrado = false;

  Serial.println(F("FC_C confirmado"));

  tiempoInicio = millis();

  if (marchaDestino == MARCHA_R)
  {
    Serial.println(F("Cambio hacia R"));
    Serial.println(F("Activando K2"));
    Serial.println(F("Moviendo primero hacia N"));

    // Entrada a R:
    // activar cambio de carril y dirigirse primero a N.
    activarK2();
    cambioCarrilPendiente = true;

    moverHacia(posADC[MARCHA_N]);
  }
  else
  {
    Serial.print(F("Moviendo hacia "));
    Serial.println(nombreMarcha(marchaDestino));

    // Cambio normal
    cambioCarrilPendiente = false;

    moverHacia(posADC[marchaDestino]);
  }

  estadoActual = MOVIENDO;
  logEstado();
}

void estadoMoviendo() {

  uint32_t ahora = millis();

  // Emergencia: UP + DOWN
  if (upDownSimultaneos()) {

    Serial.println();
    Serial.println(F("===== EMERGENCIA ====="));
    Serial.println(F("UP + DOWN durante movimiento"));
    Serial.println(F("Destino forzado: N"));
    Serial.println();

    logAccion("EMERGENCIA durante MOVIENDO");

    apagarActuador();
    desactivarK2();

    marchaDestino = MARCHA_N;
    tiempoInicio = ahora;

    estadoActual = EMERGENCIA_A_N;
    return;
  }

  // Actualizar lectura del potenciómetro
  if ((ahora - ultimaLecturaPot) >= TIEMPO_LECTURA_POT) {
    ultimaLecturaPot = ahora;
    valorPot = leerPot();
  }

  // ==========================================================
  // Entrada en R
  // ==========================================================
  if (cambioCarrilPendiente) {

    moverHacia(posADC[MARCHA_N]);

    if (analogRead(PIN_FC_S) < 100) {

      Serial.println(F("FC_S detectado"));
      Serial.println(F("Continuando hacia R"));

      logAccion("FC_S activo -> continuar hacia R");

      cambioCarrilPendiente = false;

      tiempoInicio = ahora;

      moverHacia(posADC[MARCHA_R]);
    }

    if ((ahora - tiempoInicio) >= TIMEOUT_MS) {

      Serial.println();
      Serial.println(F("***** ERROR *****"));
      Serial.println(F("Timeout esperando FC_S"));
      Serial.println();

      logAccion("TIMEOUT esperando FC_S");

      apagarActuador();
      desactivarK2();

      marchaError = MARCHA_R;

      tiempoInicio = ahora;

      estadoActual = RECUPERANDO_A_N;
    }

    return;
  }

  // ==========================================================
  // Movimiento normal
  // ==========================================================

  moverHacia(posADC[marchaDestino]);

  if (enPosicion(posADC[marchaDestino])) {

    apagarActuador();

    // Salida desde R hacia N
    if (marchaActual == MARCHA_R && marchaDestino == MARCHA_N) {

      Serial.println(F("N alcanzada"));
      Serial.println(F("Esperando liberacion de FC_S"));

      tiempoAux = ahora;

      estadoActual = ESPERA_FC_S_RETORNO;
      logEstado();
      return;
    }

    desactivarK2();
    desactivarK1();

    marchaActual = marchaDestino;

    encenderLED(marchaActual);

    Serial.println();
    Serial.println(F("===== CAMBIO COMPLETADO ====="));

    Serial.print(F("Marcha: "));
    Serial.println(nombreMarcha(marchaActual));

    Serial.print(F("Potenciómetro: "));
    Serial.println(valorPot);

    Serial.println(F("============================="));
    Serial.println();

    bloqueoCambio = true;
    tiempoBloqueo = ahora;

    estadoActual = REPOSO;
    return;
  }

  if ((ahora - tiempoInicio) >= TIMEOUT_MS) {

    Serial.println();
    Serial.println(F("***** ERROR *****"));
    Serial.println(F("Timeout durante movimiento"));

    Serial.print(F("Destino: "));
    Serial.println(nombreMarcha(marchaDestino));

    Serial.print(F("Potenciómetro: "));
    Serial.println(valorPot);
    Serial.println();

    logAccion("TIMEOUT en MOVIENDO");

    apagarActuador();
    desactivarK2();

    marchaError = marchaDestino;

    tiempoInicio = ahora;

    estadoActual = RECUPERANDO_A_N;
  }
}

void estadoEsperandoFCS() {

  // Este estado ha quedado obsoleto.
  // La lógica del cambio de carril se realiza ahora íntegramente
  // dentro de estadoMoviendo().

  estadoActual = MOVIENDO;
  logEstado();
}

void estadoEsperaFCSRetorno() {

  uint32_t ahora = millis();

  static bool mensajeMostrado = false;

  if (!mensajeMostrado) {

    Serial.println();
    Serial.println(F("===== SALIENDO DE R ====="));
    Serial.println(F("Esperando retorno de FC_S..."));
    Serial.println();

    mensajeMostrado = true;
  }

  // Salir en cuanto FC_S vuelva a HIGH
  if (analogRead(PIN_FC_S) > 900) {

    Serial.println(F("FC_S liberado"));
    Serial.println(F("Cambio finalizado"));

    logAccion("FC_S retorno correcto");

    mensajeMostrado = false;

    desactivarK1();

    marchaActual = MARCHA_N;

    encenderLED(MARCHA_N);

    Serial.println();
    Serial.println(F("===== CAMBIO COMPLETADO ====="));
    Serial.println(F("Marcha : N"));

    Serial.print(F("Potenciómetro : "));
    Serial.println(valorPot);

    Serial.println(F("============================="));
    Serial.println();

    bloqueoCambio = true;
    tiempoBloqueo = ahora;

    estadoActual = REPOSO;
    logEstado();

    return;
  }

  // Máximo 1 segundo de espera
  if ((ahora - tiempoAux) >= 1000) {

    Serial.println();
    Serial.println(F("***** ERROR GRAVE *****"));
    
    Serial.print(F("FC_C : "));
    Serial.println(digitalRead(PIN_FC_C) ? F("OFF") : F("ON"));
    Serial.print(F("FC_S : "));
    Serial.println(digitalRead(PIN_FC_S) ? F("OFF") : F("ON"));
    
    Serial.println(F("FC_S no ha regresado"));
    Serial.println(F("Sistema bloqueado"));
    Serial.println();

    logAccion("ERROR: FC_S no retorno");

    mensajeMostrado = false;

    entrarErrorGrave();
  }
}

void estadoEmergenciaN() {
  uint32_t ahora = millis();

  if ((ahora - ultimaLecturaPot) >= TIEMPO_LECTURA_POT) {
    ultimaLecturaPot = ahora;
    valorPot = leerPot();
  }

  if (enPosicion(posADC[MARCHA_N])) {
    apagarActuador();
    desactivarK1();
    desactivarK2();
    marchaActual = MARCHA_N;
    encenderLED(MARCHA_N);
    bloqueoCambio = false;
    estadoActual = REPOSO;
    logAccion("Emergencia completada — en N");
    return;
  }

  if ((ahora - tiempoInicio) >= TIMEOUT_MS) {
    logAccion("TIMEOUT en EMERGENCIA_A_N -> ERROR GRAVE");
    entrarErrorGrave();
    return;
  }

  moverHacia(posADC[MARCHA_N]);
}

void estadoRecuperandoN() {

  uint32_t ahora = millis();

  // Actualizar lectura del potenciómetro
  if ((ahora - ultimaLecturaPot) >= TIEMPO_LECTURA_POT) {
    ultimaLecturaPot = ahora;
    valorPot = leerPot();
  }

  // Seguir intentando llegar a N
  moverHacia(posADC[MARCHA_N]);

  // ¿Ya hemos llegado?
  if (enPosicion(posADC[MARCHA_N])) {

    apagarActuador();

    desactivarK1();
    desactivarK2();

    marchaActual = MARCHA_N;

    apagarTodosLEDs();
    encenderLED(MARCHA_N);

    ledEstado = false;
    tiempoLed = ahora;

    estadoActual = ERROR_LEVE;

    Serial.print(F("Recuperado en N tras fallo en "));
    Serial.println(nombreMarcha(marchaError));

    return;
  }

  // Si tampoco consigue volver a N → ERROR GRAVE
  if ((ahora - tiempoInicio) >= TIMEOUT_MS) {

    logAccion("TIMEOUT recuperando N -> ERROR GRAVE");

    apagarActuador();
    desactivarK1();
    desactivarK2();

    entrarErrorGrave();
  }
}

void estadoErrorLeve() {

  static bool mensajeEntrada = false;

  // Mostrar el mensaje solo una vez al entrar
  if (!mensajeEntrada) {

    Serial.println();
    Serial.println(F("=========== ERROR LEVE ==========="));
    
    Serial.print(F("FC_C : "));
    Serial.println(digitalRead(PIN_FC_C) ? F("OFF") : F("ON"));
    Serial.print(F("FC_S : "));
    Serial.println(digitalRead(PIN_FC_S) ? F("OFF") : F("ON"));

    Serial.print(F("Cambio fallido : "));
    Serial.println(nombreMarcha(marchaError));

    Serial.print(F("Marcha actual  : "));
    Serial.println(nombreMarcha(marchaActual));

    Serial.print(F("Potenciómetro  : "));
    Serial.println(valorPot);

    Serial.println(F("Sistema operativo"));
    Serial.println(F("UP + DOWN para reconocer el error"));
    Serial.println();

    mensajeEntrada = true;
  }

  // Mantener visible el error
  manejarParpadeoSimple(pinLED(marchaError));

  // Reconocimiento del error
  if (upDownSimultaneos()) {

    Serial.println();
    Serial.println(F("ERROR LEVE RECONOCIDO"));
    Serial.println(F("Volviendo a REPOSO"));
    Serial.println();

    logAccion("ERROR_LEVE reconocido");

    apagarTodosLEDs();
    encenderLED(marchaActual);

    mensajeEntrada = false;

    estadoActual = REPOSO;
    logEstado();

    return;
  }

  // Evitar cambios demasiado seguidos
  uint32_t ahora = millis();

  if (bloqueoCambio && (ahora - tiempoBloqueo) >= BLOQUEO_MS)
    bloqueoCambio = false;

  if (bloqueoCambio)
    return;

  Marcha sig = marchaActual;

  // -------------------------
  // BOTON UP
  // -------------------------
  if (btnUp.presionado) {

    switch (marchaActual) {

      case MARCHA_N: sig = MARCHA_1; break;
      case MARCHA_1: sig = MARCHA_2; break;
      case MARCHA_R: sig = MARCHA_N; break;

      case MARCHA_2:
      default:
        return;
    }

    Serial.println();
    Serial.println(F("===== CAMBIO SOLICITADO ====="));

    Serial.print(F("Marcha actual : "));
    Serial.println(nombreMarcha(marchaActual));

    Serial.print(F("Marcha destino: "));
    Serial.println(nombreMarcha(sig));

    Serial.print(F("Potenciómetro : "));
    Serial.println(valorPot);

    Serial.println();

    mensajeEntrada = false;

    iniciarCambioA(sig);

    return;
  }

  // -------------------------
  // BOTON DOWN
  // -------------------------
  if (btnDown.presionado) {

    switch (marchaActual) {

      case MARCHA_N: sig = MARCHA_R; break;
      case MARCHA_1: sig = MARCHA_N; break;
      case MARCHA_2: sig = MARCHA_1; break;

      case MARCHA_R:
      default:
        return;
    }

    Serial.println();
    Serial.println(F("===== CAMBIO SOLICITADO ====="));

    Serial.print(F("Marcha actual : "));
    Serial.println(nombreMarcha(marchaActual));

    Serial.print(F("Marcha destino: "));
    Serial.println(nombreMarcha(sig));

    Serial.print(F("Potenciómetro : "));
    Serial.println(valorPot);

    Serial.println();

    mensajeEntrada = false;

    iniciarCambioA(sig);

    return;
  }
}

void estadoErrorGrave() {
  manejarParpadeoGrave();
}

void entrarErrorGrave() {
  apagarTodoReles();
  apagarTodosLEDs();
  ledEstado    = false;
  tiempoLed    = millis();
  estadoActual = ERROR_GRAVE;
  logAccion("*** ERROR GRAVE - sistema bloqueado - requiere reset físico ***");
}

//  =============================================================================
// MODO APRENDIZAJE
// =============================================================================

void estadoModoAprendizaje() {
  uint32_t ahora = millis();

  apagarTodosLEDs();
  manejarParpadeoAprendizaje(pinLED(marchaAprendizaje));

  if (btnUp.estadoActual && !btnUp.bloqueado &&
      (ahora - btnUp.tiempoPresion) >= TIEMPO_PULSACION_LARGA_MS) {
    btnUp.bloqueado = true;
    Marcha anterior = marchaAprendizaje;
    if      (marchaAprendizaje == MARCHA_2) marchaAprendizaje = MARCHA_1;
    else if (marchaAprendizaje == MARCHA_1) marchaAprendizaje = MARCHA_N;
    else if (marchaAprendizaje == MARCHA_N) marchaAprendizaje = MARCHA_R;
    if (marchaAprendizaje != anterior) {
      Serial.print(F("Aprendizaje: navegando a marcha "));
      Serial.println(nombreMarcha(marchaAprendizaje));
      faseParpadeo  = 0;
      tiempoLed     = ahora;
      tiempoInicio  = ahora;
      estadoActual  = APRENDIZAJE_MOVIENDO;
      marchaDestino = marchaAprendizaje;
      moverHacia(posADC[marchaAprendizaje]);
    }
    return;
  }

  if (btnDown.estadoActual && !btnDown.bloqueado &&
      (ahora - btnDown.tiempoPresion) >= TIEMPO_PULSACION_LARGA_MS) {
    btnDown.bloqueado = true;
    Marcha anterior = marchaAprendizaje;
    if      (marchaAprendizaje == MARCHA_R) marchaAprendizaje = MARCHA_N;
    else if (marchaAprendizaje == MARCHA_N) marchaAprendizaje = MARCHA_1;
    else if (marchaAprendizaje == MARCHA_1) marchaAprendizaje = MARCHA_2;
    if (marchaAprendizaje != anterior) {
      Serial.print(F("Aprendizaje: navegando a marcha "));
      Serial.println(nombreMarcha(marchaAprendizaje));
      faseParpadeo  = 0;
      tiempoLed     = ahora;
      tiempoInicio  = ahora;
      estadoActual  = APRENDIZAJE_MOVIENDO;
      marchaDestino = marchaAprendizaje;
      moverHacia(posADC[marchaAprendizaje]);
    }
    return;
  }

  if (btnUp.presionado) {
    Serial.print(F("Aprendizaje: paso IN — Pot:"));
    Serial.println(analogRead(PIN_POT));
    tiempoInicio  = ahora;
    estadoActual  = APRENDIZAJE_MOVIENDO;
    marchaDestino = MARCHA_R;
    activarReleIn();
    return;
  }

  if (btnDown.presionado) {
    Serial.print(F("Aprendizaje: paso OUT — Pot:"));
    Serial.println(analogRead(PIN_POT));
    tiempoInicio  = ahora;
    estadoActual  = APRENDIZAJE_MOVIENDO;
    marchaDestino = MARCHA_2;
    activarReleOut();
    return;
  }

  if (btnConf.presionado) {
    uint16_t valorActual = analogRead(PIN_POT);
    guardarPosEnEEPROM(marchaAprendizaje, valorActual);
    apagarTodosLEDs();
    digitalWrite(pinLED(marchaAprendizaje), HIGH);
    tiempoAux    = ahora;
    estadoActual = APRENDIZAJE_CONFIRMANDO;
  }
}

void estadoAprendizajeConfirmando() {
  uint32_t ahora = millis();
  if ((ahora - tiempoAux) >= PARPADEO_CONFIRM_MS) {
    apagarTodosLEDs();
    faseParpadeo = 0;
    tiempoLed    = ahora;
    estadoActual = MODO_APRENDIZAJE;
  }
}

void estadoAprendizajeMoviendo() {
  uint32_t ahora = millis();

  apagarTodosLEDs();
  manejarParpadeoAprendizaje(pinLED(marchaAprendizaje));

  if ((ahora - ultimaLecturaPot) >= TIEMPO_LECTURA_POT) {
    ultimaLecturaPot = ahora;
    valorPot = leerPot();
  }

  bool esNavegacion = (marchaDestino == marchaAprendizaje);

  if (esNavegacion) {
    if (enPosicion(posADC[marchaAprendizaje])) {
      apagarActuador();
      Serial.print(F("Aprendizaje: llegue a posicion de marcha "));
      Serial.print(nombreMarcha(marchaAprendizaje));
      Serial.print(F(" Pot:"));
      Serial.println(valorPot);
      estadoActual = MODO_APRENDIZAJE;
      return;
    }
    if ((ahora - tiempoInicio) >= TIMEOUT_MS) {
      apagarActuador();
      logAccion("Aprendizaje: timeout navegacion");
      estadoActual = MODO_APRENDIZAJE;
      return;
    }
    moverHacia(posADC[marchaAprendizaje]);
  } else {
    if ((ahora - tiempoInicio) >= PASO_APRENDIZAJE_MS) {
      apagarActuador();
      Serial.print(F("Aprendizaje: paso fin — Pot:"));
      Serial.println(valorPot);
      estadoActual = MODO_APRENDIZAJE;
    }
  }
}
