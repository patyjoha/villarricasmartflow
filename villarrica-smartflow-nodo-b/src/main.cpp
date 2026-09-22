// Villarrica Smartflow - Nodo B: semaforo con cruce peatonal accesible

// Banderas de compilacion
#define USAR_OLED   0   // 1 = con pantalla, 0 = sin pantalla (compila mas rapido)
#define MODO_PRUEBA 1   // 1 = banco de pruebas automatico, 0 = operacion normal

#include <WiFi.h>
#include <PubSubClient.h>
#if USAR_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

#define WIFI_SSID       "Wokwi-GUEST"
#define WIFI_PASS       ""
#define MQTT_SERVIDOR   "mqtt.eu.thingsboard.cloud"
#define MQTT_PUERTO     1883
#define TB_ACCESS_TOKEN "9UdK3YqzxFGPXlEqU3Hh"
#define ID_DISPOSITIVO  "4f07b6b0-b305-11f1-b747-f73439e59bea"

// ---- Pines ----
static const uint8_t PIN_VEH_ROJO     = 27;
static const uint8_t PIN_VEH_AMARILLO = 26;
static const uint8_t PIN_VEH_VERDE    = 25;
static const uint8_t PIN_PEA_ROJO     = 33;
static const uint8_t PIN_PEA_VERDE    = 32;
static const uint8_t PIN_BOTON        = 4;   // a GND, con pull-up interno
static const uint8_t PIN_LED_BOTON    = 5;   // testigo de solicitud registrada
static const uint8_t PIN_BUZZER       = 13;
static const uint8_t PIN_DET_VEHIC    = 14;  // detector que apunta a la calzada

// ---- Geometria del cruce ----
constexpr float ANCHO_CALZADA_M      = 8.0f;   // [MEDIR EN EL CRUCE]
constexpr float VELOCIDAD_MARCHA_M_S = 0.85f;  // velocidad de diseno del peaton

// ---- Tiempos (milisegundos; UL = unsigned long, mismo tipo que millis()) ----
static const uint32_t T_CICLO_FIJO_MS          = 60000UL; // verde de autos sin solicitud antes del cruce automatico [DEFINIR]
static const uint32_t T_VEH_VERDE_MIN_MS       = 20000UL; // verde minimo de autos si hay demanda
static const uint32_t T_VEH_VERDE_MIN_LIBRE_MS =  5000UL; // verde minimo de autos si la calle esta libre
static const uint32_t T_SIN_VEHICULOS_MS       =  3000UL; // sin detecciones por este tiempo = calle libre
static const uint32_t T_VEH_AMARILLO_MS        =  3000UL; // aviso al conductor
static const uint32_t T_TODO_ROJO_MS           =  1500UL; // todos en rojo mientras sale el ultimo auto
static const uint32_t T_PEA_VERDE_MS           =  7000UL; // tiempo para EMPEZAR a cruzar
static const uint32_t T_PEA_DESPEJE_MS =                   // tiempo para TERMINAR de cruzar
      (uint32_t)((ANCHO_CALZADA_M / VELOCIDAD_MARCHA_M_S) * 1000.0f);
static const uint32_t T_ENFRIAMIENTO_MS        = 30000UL; // pausa minima entre dos cruces pedidos
static const uint32_t T_ESPERA_MAXIMA_MS       = 90000UL; // espera maxima garantizada al peaton
static const uint32_t ANTIRREBOTE_MS           =    50UL; // estabilizacion del contacto del boton
static const uint32_t T_ESTADO_MAXIMO_MS       = 180000UL;// si un estado dura mas, se asume falla
static const uint32_t T_MUERTO_VEHIC_MS        =  1000UL; // separacion minima entre dos autos [CALIBRAR]
static const uint32_t PERIODO_TELEMETRIA_MS    = 30000UL; // cada cuanto se publica telemetria

// ---- Senales acusticas: CICLO = cada cuanto se repite, PULSO = cuanto suena ----
static const uint16_t HZ_LOCALIZADOR  = 880;   // "el boton esta aqui"
static const uint16_t HZ_CONFIRMACION = 1200;  // "tu pedido se registro"
static const uint16_t HZ_CRUCE        = 1500;  // "podes cruzar"
static const uint32_t MS_LOCALIZADOR_CICLO = 1000UL, MS_LOCALIZADOR_PULSO = 60UL;  // lento
static const uint32_t MS_CRUCE_CICLO       =  200UL, MS_CRUCE_PULSO       = 90UL;  // rapido
static const uint32_t MS_DESPEJE_CICLO     =  600UL, MS_DESPEJE_PULSO     = 60UL;  // se afloja
static const uint32_t MS_CONFIRMACION      =  120UL;                               // chasquido

// ---- Banco de pruebas ----
#if MODO_PRUEBA
static const uint16_t SERIE_VEHICULOS[] = { 30, 50, 100 };
static const uint16_t SERIE_PEATONES[]  = {  5, 10,  20 };
static const uint8_t  CANT_SERIES        = 3;
static const uint8_t  SERIE_UNICA        = 255;     // 0, 1 o 2 = una serie; 255 = las tres
static const uint32_t INTERVALO_VEHIC_MS = 1500UL;  // separacion entre autos simulados
#endif

// ---- Estados del ciclo ----
enum Estado : uint8_t {
  VEH_VERDE,      // pasan los autos
  SOLICITUD,      // hay pedido de cruce, corre el verde minimo
  VEH_AMARILLO,   // aviso al conductor
  TODO_ROJO,      // nadie avanza
  PEA_VERDE,      // el peaton puede empezar a cruzar
  PEA_DESPEJE,    // el peaton que ya entro termina
  FALLA           // modo seguro
};

const char *nombreEstado(Estado e) {
  switch (e) {
    case VEH_VERDE:    return "VEH_VERDE";
    case SOLICITUD:    return "SOLICITUD";
    case VEH_AMARILLO: return "VEH_AMARILLO";
    case TODO_ROJO:    return "TODO_ROJO";
    case PEA_VERDE:    return "PEA_VERDE";
    case PEA_DESPEJE:  return "PEA_DESPEJE";
    default:           return "FALLA";
  }
}

void publicarTelemetria(bool incluirCiclo);
void transicion(Estado nuevo);
void entrarEnFalla(const char *motivo);

// ---- Estado interno (las variables t* guardan un instante de millis()) ----
Estado   estado            = VEH_VERDE;
uint32_t tEntradaEstado    = 0;      // inicio del estado actual
uint32_t tSolicitud        = 0;      // momento en que se acepto el pedido
uint32_t tFinCicloPeatonal = 0;      // fin del ultimo cruce pedido
bool     cicloAutomatico   = false;  // true si el cruce en curso no lo pidio nadie

// Contadores publicados como telemetria
uint32_t ciclosPeatonales       = 0;  // cruces pedidos con el boton
uint32_t ciclosAutomaticos      = 0;  // cruces dados por tiempo, sin pedido
uint32_t pulsacionesDescartadas = 0;
uint32_t comandosRechazados     = 0;
uint32_t verdesEnConflicto      = 0;  // debe quedar en 0
uint32_t esperaPeatonalUltima   = 0;  // segundos

// Detector vehicular
uint32_t tUltimoVehiculo     = 0;
uint32_t vehiculosDetectados = 0;
int      estadoDetPrevio     = LOW;   // lectura anterior, para detectar el cambio

// Pulsador
int      lecturaBotonPrevia = HIGH;
uint32_t tCambioBoton       = 0;
bool     botonEstable       = HIGH;   // lectura confirmada tras el antirrebote

// Audio
uint32_t tAudioCiclo        = 0;      // inicio del patron sonoro actual
bool     audioSonando       = false;
uint32_t tConfirmacionHasta = 0;      // fin del chasquido de confirmacion

// Red
WiFiClient   espClient;
PubSubClient mqtt(espClient);
uint32_t     tUltimoIntentoMqtt = 0;
uint32_t     tUltimaTelemetria  = 0;
bool         redHabilitada      = true;  // se alterna con el comando W

#if USAR_OLED
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
#endif
bool oledOk = false;

#if MODO_PRUEBA
uint32_t pVehInyectados = 0, pVehContados = 0;
uint32_t pPeaInyectadas = 0, pPeaAceptadas = 0, pPeaDescartadas = 0;
uint32_t pCiclosCompletos = 0, pEsperaMax = 0, pEsperaSuma = 0;
uint8_t  serieActual = 0;
uint32_t tInicioSerie = 0;
#endif

// Enciende las cinco luces con la combinacion de cada estado.
void escribirLuces(bool vR, bool vA, bool vV, bool pR, bool pV) {
  digitalWrite(PIN_VEH_ROJO,     vR);
  digitalWrite(PIN_VEH_AMARILLO, vA);
  digitalWrite(PIN_VEH_VERDE,    vV);
  digitalWrite(PIN_PEA_ROJO,     pR);
  digitalWrite(PIN_PEA_VERDE,    pV);
}

// Lee los pines reales: detecta si los dos verdes quedaron encendidos a la vez.
bool hayConflicto() {
  return digitalRead(PIN_VEH_VERDE) == HIGH &&
         digitalRead(PIN_PEA_VERDE) == HIGH;
}

// Modo seguro: amarillo intermitente y buzzer en silencio.
void entrarEnFalla(const char *motivo) {
  Serial.print(F("[FALLA] "));
  Serial.println(motivo);
  estado = FALLA;
  tEntradaEstado = millis();
  noTone(PIN_BUZZER);
  audioSonando = false;
  digitalWrite(PIN_LED_BOTON, LOW);
}

// El sonido depende solo del estado, asi nunca contradice a las luces.
void actualizarAudio() {
  uint32_t ahora = millis();

  if (ahora < tConfirmacionHasta) {                 // el acuse del boton tiene prioridad
    if (!audioSonando) { tone(PIN_BUZZER, HZ_CONFIRMACION); audioSonando = true; }
    return;
  }

  uint16_t hz = 0;
  uint32_t ciclo = 0, pulso = 0;

  switch (estado) {
    case VEH_VERDE:
    case SOLICITUD:
      hz = HZ_LOCALIZADOR; ciclo = MS_LOCALIZADOR_CICLO; pulso = MS_LOCALIZADOR_PULSO;
      break;
    case PEA_VERDE:
      hz = HZ_CRUCE; ciclo = MS_CRUCE_CICLO; pulso = MS_CRUCE_PULSO;
      break;
    case PEA_DESPEJE:
      hz = HZ_CRUCE; ciclo = MS_DESPEJE_CICLO; pulso = MS_DESPEJE_PULSO;
      break;
    default:                                        // amarillo, todo rojo y falla: silencio
      if (audioSonando) { noTone(PIN_BUZZER); audioSonando = false; }
      return;
  }

  // fase va de 0 a ciclo y vuelve a empezar; suena mientras sea menor que pulso
  uint32_t fase = (ahora - tAudioCiclo) % ciclo;
  if (fase < pulso) {
    if (!audioSonando) { tone(PIN_BUZZER, hz); audioSonando = true; }
  } else {
    if (audioSonando) { noTone(PIN_BUZZER); audioSonando = false; }
  }
}

// Suma un auto si paso la separacion minima desde el anterior.
void registrarVehiculo() {
  if (millis() - tUltimoVehiculo < T_MUERTO_VEHIC_MS) return;
  vehiculosDetectados++;
  tUltimoVehiculo = millis();
#if MODO_PRUEBA
  pVehContados++;
#endif
  Serial.print(F("[VEHIC] deteccion. Total: "));
  Serial.println(vehiculosDetectados);
}

// Cuenta solo el instante en que el sensor pasa de apagado a encendido.
void leerDetectorVehicular() {
  int lectura = digitalRead(PIN_DET_VEHIC);
  if (lectura == HIGH && estadoDetPrevio == LOW) registrarVehiculo();
  estadoDetPrevio = lectura;
}

// true si no pasa ningun auto desde hace T_SIN_VEHICULOS_MS.
bool calzadaLibre() {
  if (tUltimoVehiculo == 0) return true;
  return (millis() - tUltimoVehiculo) >= T_SIN_VEHICULOS_MS;
}

// Verde minimo que corresponde segun lo que mide el detector.
uint32_t verdeMinimoVigente() {
  return calzadaLibre() ? T_VEH_VERDE_MIN_LIBRE_MS : T_VEH_VERDE_MIN_MS;
}

// Atiende una pulsacion: confirma siempre, acepta solo si corresponde.
void procesarPulsacion() {
  tConfirmacionHasta = millis() + MS_CONFIRMACION;
  digitalWrite(PIN_LED_BOTON, HIGH);

  bool enfriando = (ciclosPeatonales > 0) &&
                   (millis() - tFinCicloPeatonal < T_ENFRIAMIENTO_MS);

  if (estado == VEH_VERDE && !enfriando) {          // se acepta solo con autos en verde y sin pausa
    estado = SOLICITUD;
    tEntradaEstado = millis();
    tSolicitud = millis();
    cicloAutomatico = false;
#if MODO_PRUEBA
    pPeaAceptadas++;
#endif
    Serial.println(F("[BOTON] solicitud aceptada"));
  } else {                                          // cruce en curso o en pausa: se descarta
    pulsacionesDescartadas++;
#if MODO_PRUEBA
    pPeaDescartadas++;
#endif
    Serial.print(F("[BOTON] pulsacion descartada. Total: "));
    Serial.println(pulsacionesDescartadas);
  }
}

// Filtra el rebote del contacto y reacciona solo al presionar.
void leerPulsador() {
  int lectura = digitalRead(PIN_BOTON);

  if (lectura != lecturaBotonPrevia) {
    tCambioBoton = millis();
    lecturaBotonPrevia = lectura;
  }
  if (millis() - tCambioBoton < ANTIRREBOTE_MS) return;
  if (lectura == botonEstable) return;

  botonEstable = lectura;
  if (botonEstable != LOW) return;

  procesarPulsacion();
}

// Cambia de estado y reinicia los relojes del estado y del audio.
void transicion(Estado nuevo) {
  estado = nuevo;
  tEntradaEstado = millis();
  tAudioCiclo = millis();
  Serial.print(F("[ESTADO] -> "));
  Serial.println(nombreEstado(nuevo));
}

// Maquina de estados del semaforo.
void actualizarMaquina() {
  uint32_t enEstado = millis() - tEntradaEstado;

  // Supervisor: ningun estado del ciclo puede durar indefinidamente.
  if (estado != FALLA && estado != VEH_VERDE && enEstado > T_ESTADO_MAXIMO_MS) {
    entrarEnFalla("estado excedio su duracion maxima");
    return;
  }

  switch (estado) {
    case VEH_VERDE:
      escribirLuces(LOW, LOW, HIGH, HIGH, LOW);
      // Ciclo fijo: aunque nadie pulse, los peatones tienen su cruce.
      if (enEstado >= T_CICLO_FIJO_MS) {
        cicloAutomatico = true;
        tSolicitud = millis();
        Serial.println(F("[CICLO] cruce peatonal automatico"));
        transicion(VEH_AMARILLO);
      }
      break;

    case SOLICITUD:
      escribirLuces(LOW, LOW, HIGH, HIGH, LOW);
      if (millis() - tSolicitud >= verdeMinimoVigente()) {      // cumplido el verde minimo
        Serial.print(F("[ADAPT] verde minimo aplicado: "));
        Serial.print(verdeMinimoVigente() / 1000);
        Serial.println(calzadaLibre() ? F(" s (calzada libre)")
                                      : F(" s (demanda vehicular activa)"));
        transicion(VEH_AMARILLO);
      } else if (millis() - tSolicitud >= T_ESPERA_MAXIMA_MS) { // tope de espera alcanzado
        Serial.println(F("[TOPE] espera maxima alcanzada"));
        transicion(VEH_AMARILLO);
      }
      break;

    case VEH_AMARILLO:
      escribirLuces(LOW, HIGH, LOW, HIGH, LOW);
      if (enEstado >= T_VEH_AMARILLO_MS) transicion(TODO_ROJO);
      break;

    case TODO_ROJO:
      escribirLuces(HIGH, LOW, LOW, HIGH, LOW);
      if (enEstado >= T_TODO_ROJO_MS) {
        if (!cicloAutomatico)                                  // la espera solo cuenta si alguien pidio
          esperaPeatonalUltima = (millis() - tSolicitud) / 1000UL;
        transicion(PEA_VERDE);
      }
      break;

    case PEA_VERDE:
      escribirLuces(HIGH, LOW, LOW, LOW, HIGH);
      if (enEstado >= T_PEA_VERDE_MS) transicion(PEA_DESPEJE);
      break;

    case PEA_DESPEJE:
      escribirLuces(HIGH, LOW, LOW, LOW, ((millis() / 400) % 2) == 0);  // verde intermitente
      if (enEstado >= T_PEA_DESPEJE_MS) {
        if (cicloAutomatico) {
          ciclosAutomaticos++;
        } else {
          ciclosPeatonales++;
          tFinCicloPeatonal = millis();
          digitalWrite(PIN_LED_BOTON, LOW);
#if MODO_PRUEBA
          pCiclosCompletos++;
          pEsperaSuma += esperaPeatonalUltima;
          if (esperaPeatonalUltima > pEsperaMax) pEsperaMax = esperaPeatonalUltima;
#endif
        }
        publicarTelemetria(!cicloAutomatico);
        cicloAutomatico = false;
        transicion(VEH_VERDE);
      }
      break;

    case FALLA:
    default:
      escribirLuces(LOW, ((millis() / 500) % 2) == 0, LOW, LOW, LOW);   // amarillo intermitente
      break;
  }

  if (estado != FALLA && hayConflicto()) {
    verdesEnConflicto++;
    entrarEnFalla("verde vehicular y peatonal simultaneos");
  }
}

// Pantalla: cuenta regresiva durante el cruce y resumen el resto del tiempo.
void actualizarPantalla() {
#if !USAR_OLED
  return;
#else
  if (!oledOk) return;
  static uint32_t tUltima = 0;
  if (millis() - tUltima < 200) return;   // refresco cada 200 ms
  tUltima = millis();

  uint32_t enEstado = millis() - tEntradaEstado;
  int restante = 0;
  if (estado == PEA_VERDE)   restante = (int)((T_PEA_VERDE_MS   - enEstado) / 1000);
  if (estado == PEA_DESPEJE) restante = (int)((T_PEA_DESPEJE_MS - enEstado) / 1000);
  if (restante < 0) restante = 0;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("VILLARRICA SMARTFLOW"));
  oled.setCursor(0, 12);
  oled.print(F("Estado: "));
  oled.println(nombreEstado(estado));

  if (estado == PEA_VERDE || estado == PEA_DESPEJE) {
    oled.setTextSize(3);
    oled.setCursor(40, 26);
    oled.print(restante);
    oled.setTextSize(1);
    oled.setCursor(0, 56);
    oled.print(estado == PEA_VERDE ? F("CRUCE HABILITADO")
                                   : F("TERMINE DE CRUZAR"));
  } else if (estado == TODO_ROJO) {
    oled.setCursor(0, 30);
    oled.print(F("Despejando cruce..."));
    oled.setCursor(0, 44);
    oled.print(F("NO CRUZAR"));
  } else {
    oled.setCursor(0, 26);
    oled.print(F("Cruces pedidos: "));
    oled.println(ciclosPeatonales);
    oled.setCursor(0, 36);
    oled.print(F("Descartadas  : "));
    oled.println(pulsacionesDescartadas);
    oled.setCursor(0, 46);
    oled.print(F("Espera ult.  : "));
    oled.print(esperaPeatonalUltima);
    oled.println(F(" s"));
    oled.setCursor(0, 56);
    oled.print(mqtt.connected() ? F("MQTT ok") : F("MQTT sin conexion"));
  }
  oled.display();
#endif
}

// Publica la telemetria. incluirCiclo = true si el envio cierra un cruce pedido.
void publicarTelemetria(bool incluirCiclo) {
  if (!mqtt.connected()) return;
  char payload[384];
  snprintf(payload, sizeof(payload),
    "{\"estado_semaforo\":\"%s\",\"solicitud_cruce\":%s,"
    "\"espera_peatonal\":%lu,\"ciclos_peatonales\":%lu,"
    "\"ciclos_automaticos\":%lu,"
    "\"pulsaciones_descartadas\":%lu,\"verdes_en_conflicto\":%lu,"
    "\"comandos_rechazados\":%lu,\"vehiculos_detectados\":%lu,"
    "\"calzada_libre\":%s,\"verde_minimo_ms\":%lu}",
    nombreEstado(estado),
    incluirCiclo ? "true" : "false",
    (unsigned long)esperaPeatonalUltima,
    (unsigned long)ciclosPeatonales,
    (unsigned long)ciclosAutomaticos,
    (unsigned long)pulsacionesDescartadas,
    (unsigned long)verdesEnConflicto,
    (unsigned long)comandosRechazados,
    (unsigned long)vehiculosDetectados,
    calzadaLibre() ? "true" : "false",
    (unsigned long)verdeMinimoVigente());
  mqtt.publish("v1/devices/me/telemetry", payload);
  Serial.print(F("[TX] "));
  Serial.println(payload);
}

// Publica la configuracion del cruce (canal de atributos, no de telemetria).
void publicarAtributos() {
  char attrs[512];
  snprintf(attrs, sizeof(attrs),
    "{\"tipo_nodo\":\"semaforo_accesible\","
    "\"cruce_accesible\":true,\"control_adaptativo\":true,"
    "\"senal_acustica\":true,\"pulsador_confirmacion\":true,"
    "\"ancho_calzada_m\":%.1f,\"velocidad_marcha_ms\":%.2f,"
    "\"ciclo_fijo_ms\":%lu,\"walk_ms\":%lu,\"despeje_ms\":%lu,"
    "\"todo_rojo_ms\":%lu,\"amarillo_ms\":%lu,"
    "\"passage_time_ms\":%lu,\"espera_maxima_ms\":%lu}",
    ANCHO_CALZADA_M, VELOCIDAD_MARCHA_M_S,
    (unsigned long)T_CICLO_FIJO_MS,
    (unsigned long)T_PEA_VERDE_MS,
    (unsigned long)T_PEA_DESPEJE_MS,
    (unsigned long)T_TODO_ROJO_MS,
    (unsigned long)T_VEH_AMARILLO_MS,
    (unsigned long)T_SIN_VEHICULOS_MS,
    (unsigned long)T_ESPERA_MAXIMA_MS);
  mqtt.publish("v1/devices/me/attributes", attrs);
  Serial.print(F("[ATTR] "));
  Serial.println(attrs);
}

// Toda orden remota se rechaza, se responde y se contabiliza.
void alRecibirMensaje(char *topic, byte *payload, unsigned int length) {
  comandosRechazados++;
  Serial.print(F("[RECHAZO] comando remoto en "));
  Serial.print(topic);
  Serial.print(F(" -> "));
  for (unsigned int i = 0; i < length && i < 80; i++) Serial.print((char)payload[i]);
  Serial.println();

  String t(topic);
  int p = t.lastIndexOf('/');
  if (p > 0) {
    String resp = "v1/devices/me/rpc/response/" + t.substring(p + 1);
    mqtt.publish(resp.c_str(),
      "{\"aceptado\":false,\"motivo\":\"la fase del semaforo no es controlable por red\"}");
  }
  publicarTelemetria(false);
}

// Conecta a la plataforma con reintento cada 5 s, sin bloquear el semaforo.
bool conectarMQTT() {
  if (!redHabilitada) return false;
  if (mqtt.connected()) return true;
  if (millis() - tUltimoIntentoMqtt < 5000UL) return false;
  tUltimoIntentoMqtt = millis();

  if (mqtt.connect(ID_DISPOSITIVO, TB_ACCESS_TOKEN, NULL)) {
    Serial.println(F("[MQTT] conectado"));
    mqtt.subscribe("v1/devices/me/rpc/request/+");
    publicarAtributos();
    return true;
  }
  Serial.print(F("[MQTT] fallo rc="));
  Serial.println(mqtt.state());
  return false;
}

#if MODO_PRUEBA
// Resumen de cada serie y veredicto.
void imprimirResumen() {
  uint32_t dur = (millis() - tInicioSerie) / 1000UL;
  uint32_t prom = pCiclosCompletos ? (pEsperaSuma / pCiclosCompletos) : 0;
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.print  (F(" RESUMEN SERIE ")); Serial.println(serieActual + 1);
  Serial.println(F("--------------------------------------------------"));
  Serial.print  (F(" Vehiculos inyectados   : ")); Serial.println(pVehInyectados);
  Serial.print  (F(" Vehiculos contados     : ")); Serial.println(pVehContados);
  Serial.print  (F(" Pulsaciones inyectadas : ")); Serial.println(pPeaInyectadas);
  Serial.print  (F("   aceptadas            : ")); Serial.println(pPeaAceptadas);
  Serial.print  (F("   descartadas          : ")); Serial.println(pPeaDescartadas);
  Serial.print  (F(" Cruces pedidos         : ")); Serial.println(pCiclosCompletos);
  Serial.print  (F(" Espera peatonal maxima : ")); Serial.print(pEsperaMax); Serial.println(F(" s"));
  Serial.print  (F(" Espera peatonal media  : ")); Serial.print(prom);       Serial.println(F(" s"));
  Serial.print  (F(" Verdes en conflicto    : ")); Serial.println(verdesEnConflicto);
  Serial.print  (F(" Duracion (s simulados) : ")); Serial.println(dur);
  Serial.print  (F(" RESULTADO              : "));
  bool ok = (verdesEnConflicto == 0) &&
            (pPeaAceptadas + pPeaDescartadas == pPeaInyectadas) &&
            (pCiclosCompletos == pPeaAceptadas) &&
            (pEsperaMax <= T_ESPERA_MAXIMA_MS / 1000UL);
  Serial.println(ok ? F("APROBADO") : F("NO APROBADO"));
  Serial.println(F("=================================================="));
  Serial.println();
}

// Inyecta autos y pulsaciones simuladas en lugar de leer los pines.
void ejecutarBancoDePruebas() {
  static uint32_t tUltimaInyeccion = 0;
  static bool     finalizado = false;
  static bool     iniciado   = false;

  if (finalizado) return;

  if (!iniciado) {                                  // arranque de una serie
    if (SERIE_UNICA != 255) serieActual = SERIE_UNICA;
    iniciado = true;
    tInicioSerie = millis();
    Serial.print(F("\n>>> INICIO SERIE ")); Serial.print(serieActual + 1);
    Serial.print(F(": ")); Serial.print(SERIE_VEHICULOS[serieActual]);
    Serial.print(F(" vehiculos y ")); Serial.print(SERIE_PEATONES[serieActual]);
    Serial.println(F(" pulsaciones peatonales\n"));
  }

  // Serie completa y semaforo de vuelta en verde: resumen y siguiente serie.
  if (pVehInyectados >= SERIE_VEHICULOS[serieActual] &&
      pPeaInyectadas >= SERIE_PEATONES[serieActual] &&
      estado == VEH_VERDE) {
    imprimirResumen();
    if (SERIE_UNICA != 255 || serieActual + 1 >= CANT_SERIES) {
      Serial.println(F(">>> BANCO DE PRUEBAS FINALIZADO <<<"));
      finalizado = true;
      return;
    }
    serieActual++;
    pVehInyectados = pVehContados = 0;
    pPeaInyectadas = pPeaAceptadas = pPeaDescartadas = 0;
    pCiclosCompletos = pEsperaMax = pEsperaSuma = 0;
    iniciado = false;
    return;
  }

  if (millis() - tUltimaInyeccion < INTERVALO_VEHIC_MS) return;
  tUltimaInyeccion = millis();

  // Las pulsaciones se reparten de forma pareja entre los autos.
  uint16_t cadaCuantos = SERIE_VEHICULOS[serieActual] / SERIE_PEATONES[serieActual];
  if (cadaCuantos < 1) cadaCuantos = 1;

  bool tocaPulsar = false;
  if (pVehInyectados < SERIE_VEHICULOS[serieActual]) {
    pVehInyectados++;
    registrarVehiculo();
    tocaPulsar = (pVehInyectados % cadaCuantos == 0);
  } else {
    tocaPulsar = true;                              // autos terminados: quedan pulsaciones
  }

  if (tocaPulsar && pPeaInyectadas < SERIE_PEATONES[serieActual]) {
    pPeaInyectadas++;
    Serial.print(F("[PRUEBA] pulsacion peatonal "));
    Serial.print(pPeaInyectadas); Serial.print(F(" de "));
    Serial.println(SERIE_PEATONES[serieActual]);
    procesarPulsacion();
  }
}
#endif

// Comandos de prueba: F = falla, R = reinicio, W = corta/restablece la red.
void leerConsola() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'F' || c == 'f') {
    entrarEnFalla("falla forzada desde consola");
  } else if (c == 'R' || c == 'r') {
    Serial.println(F("[CONSOLA] reinicio del ciclo"));
    transicion(VEH_VERDE);
  } else if (c == 'W' || c == 'w') {
    redHabilitada = !redHabilitada;
    if (!redHabilitada) mqtt.disconnect();
    Serial.print(F("[CONSOLA] red "));
    Serial.println(redHabilitada ? F("habilitada") : F("deshabilitada"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n== Nodo B - semaforo con cruce accesible =="));
#if MODO_PRUEBA
  Serial.println(F("*** MODO PRUEBA ACTIVO: estimulo sintetico ***"));
#endif
  Serial.print(F("Ciclo fijo sin pedido : ")); Serial.print(T_CICLO_FIJO_MS / 1000);         Serial.println(F(" s"));
  Serial.print(F("Verde peatonal        : ")); Serial.print(T_PEA_VERDE_MS / 1000);          Serial.println(F(" s"));
  Serial.print(F("Despeje peatonal      : ")); Serial.print(T_PEA_DESPEJE_MS / 1000.0, 1);   Serial.println(F(" s"));
  Serial.print(F("Todo en rojo          : ")); Serial.print(T_TODO_ROJO_MS / 1000.0, 1);     Serial.println(F(" s"));

  pinMode(PIN_VEH_ROJO, OUTPUT);
  pinMode(PIN_VEH_AMARILLO, OUTPUT);
  pinMode(PIN_VEH_VERDE, OUTPUT);
  pinMode(PIN_PEA_ROJO, OUTPUT);
  pinMode(PIN_PEA_VERDE, OUTPUT);
  pinMode(PIN_LED_BOTON, OUTPUT);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_DET_VEHIC, INPUT);

  escribirLuces(LOW, LOW, HIGH, HIGH, LOW);

#if USAR_OLED
  Wire.begin(21, 22);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) Serial.println(F("[OLED] no detectado; el nodo sigue operando"));
#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000UL) delay(250);  // espera maxima 12 s
  Serial.println(WiFi.status() == WL_CONNECTED ? F("[WiFi] ok")
                                               : F("[WiFi] sin red; el ciclo funciona igual"));

  mqtt.setServer(MQTT_SERVIDOR, MQTT_PUERTO);
  mqtt.setBufferSize(512);
  mqtt.setCallback(alRecibirMensaje);

  tEntradaEstado = millis();
  tAudioCiclo    = millis();
}

void loop() {
  // Primero lo que afecta a las personas; al final, lo que depende de la red.
#if MODO_PRUEBA
  ejecutarBancoDePruebas();
#else
  leerDetectorVehicular();
  leerPulsador();
#endif
  actualizarMaquina();
  actualizarAudio();
  actualizarPantalla();
  leerConsola();

  if (WiFi.status() == WL_CONNECTED && conectarMQTT()) {
    mqtt.loop();
    if (millis() - tUltimaTelemetria >= PERIODO_TELEMETRIA_MS) {
      tUltimaTelemetria = millis();
      publicarTelemetria(false);
    }
  }
}
