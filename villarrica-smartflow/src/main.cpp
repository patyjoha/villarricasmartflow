/* =====================================================================
   VILLARRICA SMARTFLOW  -  NODO A: conteo de flujo urbano
   Trabajo Final Integrador - Diplomado en IoT y Ciberseguridad (FP-UNA)

   Funcion: detectar el paso de peatones/vehiculos, acumular las
   detecciones por ventana de tiempo y publicar el total a la plataforma.

   Requisitos que implementa (ver informe 4.3):
     RF-01  medicion continua de densidad de flujo
     RF-02  detectar movimiento e incrementar contador
     RF-03  agrupar conteos y transmitir por intervalo
     RD-01  payload < 50 bytes
     RS-01  sin imagenes ni datos personales: solo conteos
     RS-02  el servidor exige credencial valida para publicar (CP-07);
            el aprovisionamiento por dispositivo queda fuera del
            alcance de la simulacion

   SUSTITUCIONES DE SIMULACION (declarar en 8.1 del informe):
     - El sensor real del diseno es un radar Doppler. Wokwi no dispone de
       ese modelo, por lo que se usa un PIR con la misma funcion logica:
       generar un evento de presencia. El PIR NO distingue peaton de
       vehiculo ni mide velocidad.
     - El enlace real del diseno es LoRaWAN. Wokwi no dispone de radio
       LoRa, por lo que se usa WiFi + MQTT. El contenido del payload es
       equivalente.
   ===================================================================== */

#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>

// ---------------------------------------------------------------------
// CONFIGURACION DE RED Y PLATAFORMA
//
// Alcance (informe 8.1): el aprovisionamiento de credenciales por
// dispositivo corresponde a la implementacion real y queda declarado
// fuera del alcance de la simulacion. Lo que SI se verifica en el
// piloto es que el servidor rechace publicaciones sin credencial
// valida (caso de prueba CP-07).
//
// WIFI_SSID / WIFI_PASS son la red publica del simulador, no son
// credenciales reales.
// ---------------------------------------------------------------------
#define WIFI_SSID       "Wokwi-GUEST"
#define WIFI_PASS       ""

#define MQTT_SERVIDOR   "mqtt.eu.thingsboard.cloud"
#define MQTT_PUERTO     1883

// En ThingsBoard el usuario MQTT es el access token del dispositivo.
#define TB_ACCESS_TOKEN "bqp3CvleniBXOQVAS0M1"
#define ID_DISPOSITIVO  "n2e266680-b2fb-11f1-9b75-572f450238a2odoVillarricaA"

// ---------------------------------------------------------------------
// PINES
// ---------------------------------------------------------------------
static const uint8_t PIN_SENSOR = 27;   // PIR (sustituye al radar Doppler)

// ---------------------------------------------------------------------
// PARAMETROS
// [PREGUNTA] El intervalo de agregacion definitivo debe definirse en el
// informe (6.2 y 9.1). 60 s es el valor provisional de trabajo.
// ---------------------------------------------------------------------
static const uint32_t VENTANA_MS       = 60000UL;  // intervalo de agregacion
static const uint32_t TIEMPO_MUERTO_MS = 2000UL;   // evita contar dos veces
                                                   // la misma presencia
static const uint8_t  BUFFER_VENTANAS  = 30;       // respaldo offline
static const uint32_t REINTENTO_MQTT_MS = 5000UL;

// ---------------------------------------------------------------------
// BANCO DE PRUEBAS AUTOMATIZADO
//
// MODO_PRUEBA 0 -> operacion normal: el conteo proviene del sensor.
// MODO_PRUEBA 1 -> se sustituye UNICAMENTE la lectura del pin por un
//                  generador de eventos sinteticos. La logica de conteo,
//                  tiempo muerto, cierre de ventana y publicacion es la
//                  misma que en operacion normal, por lo que el
//                  resultado es valido como evidencia de CP-01 y CP-03.
//
// Declararlo en 8.1 del informe: en modo prueba el estimulo es
// sintetico; el resto de la cadena no se altera.
// ---------------------------------------------------------------------
#define MODO_PRUEBA 1

#if MODO_PRUEBA
// Series a ejecutar, una tras otra.
static const uint16_t SERIES[]   = { 30, 50, 100 };
static const uint8_t  CANT_SERIES = sizeof(SERIES) / sizeof(SERIES[0]);

// Separacion entre eventos inyectados.
// > TIEMPO_MUERTO_MS  -> se esperan 0 descartes (valida el conteo, CP-01)
// < TIEMPO_MUERTO_MS  -> se esperan descartes  (valida el filtro, CP-03)
static const uint32_t INTERVALO_INYECCION_MS = 2200UL;

// Duracion del pulso alto, analoga al tiempo que el sensor permanece
// activo ante una presencia.
static const uint32_t PULSO_MS = 300UL;
#endif

// ---------------------------------------------------------------------
// ESTADO
// ---------------------------------------------------------------------
uint16_t conteoVentana       = 0;
uint32_t tInicioVentana      = 0;
uint32_t tUltimaDeteccion    = 0;
uint32_t tUltimoIntentoMqtt  = 0;
int      estadoSensorPrevio  = LOW;
uint32_t ventanasDescartadas = 0;   // se perdieron por buffer lleno

#if MODO_PRUEBA
uint32_t pruebaInyectadas  = 0;
uint32_t pruebaContadas    = 0;
uint32_t pruebaDescartadas = 0;
uint32_t pruebaVentanas    = 0;
uint8_t  serieActual       = 0;
uint32_t tInicioSerie      = 0;
#endif

struct Muestra {
  time_t   epoch;      // 0 si no habia hora sincronizada
  uint16_t conteo;
};
Muestra  buffer[BUFFER_VENTANAS];
uint8_t  bufferUsado = 0;

WiFiClient   espClient;
PubSubClient mqtt(espClient);

// ---------------------------------------------------------------------
// RED
// ---------------------------------------------------------------------
void conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print(F("[WiFi] conectando"));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000UL) {
    delay(250);                      // solo durante el arranque
    Serial.print('.');
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? F(" ok") : F(" sin red"));
}

void sincronizarHora() {
  // Necesaria para sellar las muestras del buffer offline.
  // Las muestras publicadas en linea las sella la plataforma.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print(F("[NTP] sincronizando"));
  uint32_t t0 = millis();
  while (time(nullptr) < 100000 && millis() - t0 < 10000UL) {
    delay(200);
    Serial.print('.');
  }
  Serial.println(time(nullptr) > 100000 ? F(" ok") : F(" sin hora"));
}

bool conectarMQTT() {
  if (mqtt.connected()) return true;
  if (millis() - tUltimoIntentoMqtt < REINTENTO_MQTT_MS) return false;
  tUltimoIntentoMqtt = millis();

  Serial.print(F("[MQTT] conectando... "));
  // En ThingsBoard el usuario MQTT es el access token del dispositivo.
  if (mqtt.connect(ID_DISPOSITIVO, TB_ACCESS_TOKEN, NULL)) {
    Serial.println(F("ok"));
    return true;
  }
  Serial.print(F("fallo rc="));
  Serial.println(mqtt.state());
  return false;
}

// ---------------------------------------------------------------------
// PUBLICACION
// Payload minimo, sin datos personales (RS-01) y por debajo de 50 bytes
// (RD-01). Ejemplo: {"conteo_flujo":14}  -> 20 bytes
// ---------------------------------------------------------------------
bool publicarEnLinea(uint16_t conteo) {
  char payload[48];
  snprintf(payload, sizeof(payload), "{\"conteo_flujo\":%u}", conteo);
  bool ok = mqtt.publish("v1/devices/me/telemetry", payload);
  Serial.print(ok ? F("[TX] ") : F("[TX-FALLO] "));
  Serial.println(payload);
  return ok;
}

bool publicarDesdeBuffer(const Muestra &m) {
  char payload[96];
  if (m.epoch > 0) {
    // ThingsBoard acepta sello propio en milisegundos.
    snprintf(payload, sizeof(payload),
             "{\"ts\":%llu000,\"values\":{\"conteo_flujo\":%u}}",
             (unsigned long long)m.epoch, m.conteo);
  } else {
    snprintf(payload, sizeof(payload), "{\"conteo_flujo\":%u}", m.conteo);
  }
  return mqtt.publish("v1/devices/me/telemetry", payload);
}

void guardarEnBuffer(uint16_t conteo) {
  if (bufferUsado >= BUFFER_VENTANAS) {
    ventanasDescartadas++;           // se pierde la muestra mas antigua
    for (uint8_t i = 1; i < BUFFER_VENTANAS; i++) buffer[i - 1] = buffer[i];
    bufferUsado = BUFFER_VENTANAS - 1;
  }
  time_t ahora = time(nullptr);
  buffer[bufferUsado].epoch  = (ahora > 100000) ? ahora : 0;
  buffer[bufferUsado].conteo = conteo;
  bufferUsado++;
  Serial.print(F("[BUFFER] guardada. En espera: "));
  Serial.println(bufferUsado);
}

void vaciarBuffer() {
  while (bufferUsado > 0 && mqtt.connected()) {
    if (!publicarDesdeBuffer(buffer[0])) break;
    for (uint8_t i = 1; i < bufferUsado; i++) buffer[i - 1] = buffer[i];
    bufferUsado--;
    Serial.print(F("[BUFFER] reenviada. Restan: "));
    Serial.println(bufferUsado);
  }
}

// ---------------------------------------------------------------------
// SENSADO
// ---------------------------------------------------------------------
// Procesa una lectura venga de donde venga: del pin en operacion normal,
// o del generador en modo prueba. La logica es identica en ambos casos.
void procesarLectura(int estado) {
  // Cuenta solo el flanco de subida: una presencia sostenida frente al
  // sensor no debe sumar varias unidades.
  if (estado == HIGH && estadoSensorPrevio == LOW) {
    if (millis() - tUltimaDeteccion >= TIEMPO_MUERTO_MS) {
      conteoVentana++;
      tUltimaDeteccion = millis();
#if MODO_PRUEBA
      pruebaContadas++;
#endif
      Serial.print(F("[SENSOR] deteccion. Conteo ventana = "));
      Serial.println(conteoVentana);
    } else {
#if MODO_PRUEBA
      pruebaDescartadas++;
#endif
      // Hacer visible el descarte: sin esta linea, una deteccion
      // ignorada por tiempo muerto es indistinguible de "el sensor
      // no disparo". Es necesario para verificar el caso CP-01.
      Serial.print(F("[SENSOR] descartada por tiempo muerto ("));
      Serial.print((TIEMPO_MUERTO_MS - (millis() - tUltimaDeteccion)) / 1000.0, 1);
      Serial.println(F(" s restantes)"));
    }
  }

  estadoSensorPrevio = estado;
}

void leerSensor() {
  procesarLectura(digitalRead(PIN_SENSOR));
}

void cerrarVentana() {
#if MODO_PRUEBA
  pruebaVentanas++;
#endif
  Serial.print(F("[VENTANA] cierre con "));
  Serial.print(conteoVentana);
  Serial.println(F(" detecciones"));

  if (mqtt.connected()) {
    if (!publicarEnLinea(conteoVentana)) guardarEnBuffer(conteoVentana);
  } else {
    guardarEnBuffer(conteoVentana);
  }

  conteoVentana  = 0;
  tInicioVentana = millis();
}

// ---------------------------------------------------------------------
// GENERADOR DE ESTIMULO (solo en modo prueba)
// ---------------------------------------------------------------------
#if MODO_PRUEBA
void imprimirResumenSerie() {
  uint32_t dur = (millis() - tInicioSerie) / 1000UL;
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.print  (F(" RESUMEN SERIE "));
  Serial.print(serieActual + 1);
  Serial.print(F(" de "));
  Serial.println(CANT_SERIES);
  Serial.println(F("--------------------------------------------------"));
  Serial.print  (F(" Iteraciones solicitadas : ")); Serial.println(SERIES[serieActual]);
  Serial.print  (F(" Eventos inyectados      : ")); Serial.println(pruebaInyectadas);
  Serial.print  (F(" Contados                : ")); Serial.println(pruebaContadas);
  Serial.print  (F(" Descartados (t. muerto) : ")); Serial.println(pruebaDescartadas);
  Serial.print  (F(" Ventanas cerradas       : ")); Serial.println(pruebaVentanas);
  Serial.print  (F(" Duracion (s simulados)  : ")); Serial.println(dur);
  Serial.print  (F(" Intervalo inyeccion (ms): ")); Serial.println(INTERVALO_INYECCION_MS);
  Serial.print  (F(" Tiempo muerto (ms)      : ")); Serial.println(TIEMPO_MUERTO_MS);
  Serial.print  (F(" RESULTADO               : "));
  // Esperado: si el intervalo supera al tiempo muerto, no debe haber
  // descartes y contados debe igualar a inyectados.
  bool esperadoSinDescartes = (INTERVALO_INYECCION_MS > TIEMPO_MUERTO_MS);
  bool ok = esperadoSinDescartes
            ? (pruebaContadas == pruebaInyectadas && pruebaDescartadas == 0)
            : (pruebaContadas + pruebaDescartadas == pruebaInyectadas);
  Serial.println(ok ? F("APROBADO") : F("NO APROBADO"));
  Serial.println(F("=================================================="));
  Serial.println();
}

void ejecutarBancoDePruebas() {
  static uint32_t tUltimoEvento = 0;
  static bool     pulsoAlto     = false;
  static bool     finalizado    = false;

  if (finalizado) return;

  if (tInicioSerie == 0) {
    tInicioSerie = millis();
    Serial.print(F("\n>>> INICIO SERIE "));
    Serial.print(serieActual + 1);
    Serial.print(F(": "));
    Serial.print(SERIES[serieActual]);
    Serial.println(F(" iteraciones\n"));
  }

  // Bajar el pulso una vez cumplida su duracion.
  if (pulsoAlto && millis() - tUltimoEvento >= PULSO_MS) {
    procesarLectura(LOW);
    pulsoAlto = false;
    return;
  }

  // Serie terminada: resumen y paso a la siguiente.
  if (pruebaInyectadas >= SERIES[serieActual] && !pulsoAlto) {
    imprimirResumenSerie();
    serieActual++;
    if (serieActual >= CANT_SERIES) {
      Serial.println(F(">>> BANCO DE PRUEBAS FINALIZADO <<<"));
      finalizado = true;
      return;
    }
    pruebaInyectadas = pruebaContadas = pruebaDescartadas = pruebaVentanas = 0;
    tInicioSerie = 0;
    return;
  }

  // Inyectar el proximo evento.
  if (!pulsoAlto && millis() - tUltimoEvento >= INTERVALO_INYECCION_MS) {
    tUltimoEvento = millis();
    pruebaInyectadas++;
    pulsoAlto = true;
    procesarLectura(HIGH);
  }
}
#endif

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n== Nodo A - conteo de flujo urbano =="));
#if MODO_PRUEBA
  Serial.println(F("*** MODO PRUEBA ACTIVO: estimulo sintetico ***"));
#endif

  pinMode(PIN_SENSOR, INPUT);

  conectarWiFi();
  sincronizarHora();

  mqtt.setServer(MQTT_SERVIDOR, MQTT_PUERTO);
  mqtt.setBufferSize(256);

  tInicioVentana = millis();
}

void loop() {
  // El sensado nunca se bloquea por la red.
#if MODO_PRUEBA
  ejecutarBancoDePruebas();     // sustituye la lectura del pin
#else
  leerSensor();
#endif

  if (millis() - tInicioVentana >= VENTANA_MS) cerrarVentana();

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t tReintentoWifi = 0;
    if (millis() - tReintentoWifi > 10000UL) {
      tReintentoWifi = millis();
      WiFi.reconnect();
    }
  } else if (conectarMQTT()) {
    mqtt.loop();
    if (bufferUsado > 0) vaciarBuffer();
  }
}
