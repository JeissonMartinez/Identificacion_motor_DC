/*
 * identificacion_motor.ino — Banco de identificación de los motores del balancín
 * Control Inteligente II (168418) · Universidad de Pamplona · Ing. Jeisson Martínez Flórez
 * Versión 1.0 · noviembre de 2026
 *
 * Qué hace:
 *   - Lee los dos encoders en cuadratura (x4) con interrupciones.
 *   - Aplica secuencias de PWM a los motores (escalón y escalera) con el robot en un soporte.
 *   - Guarda cada muestra en la RAM durante el ensayo y la imprime al final como CSV.
 *   - Entrega una estimación rápida del modelo del motor:
 *       RPM ≈ K·PWM − B (recta estática), zona muerta y constante de tiempo τ.
 *
 * Funciona con el núcleo Arduino-ESP32 2.x y 3.x (Arduino IDE o PlatformIO), placa ESP32-S3.
 * Solo se modifica config.h. Escriba "h" en el monitor serie para ver los comandos.
 *
 * SEGURIDAD: todos los ensayos con motores se hacen con el robot sobre un soporte y las
 * ruedas en el aire. Cualquier tecla detiene los motores.
 */
#include <Arduino.h>
#include <esp_arduino_version.h>
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "config.h"

#define VERSION_SKETCH "1.0"

// ============================================================================
// 0. DATOS: tipos y variables globales
// ============================================================================
struct Encoder {
  int pinA, pinB;              // pinB = -1: modo de un canal
  int8_t signo;                // corrige el sentido de conteo
  float cuentasVuelta;         // cuentas por vuelta de rueda
  volatile int32_t cuentas;    // contador crudo (lo escribe solo la interrupción)
  volatile uint32_t flancosA, flancosB, errores;
  volatile uint8_t estado;     // último estado (A<<1)|B
  volatile int8_t sentido;     // modo de un canal: sentido del último PWM aplicado
  int32_t cero;                // valor crudo en el último "z"
};

struct Motor {
  int pinPwm, pinIn1, pinIn2;
  int8_t signo;
  uint8_t canal;               // canal LEDC (solo núcleo 2.x)
  Encoder *enc;
};

// Un ensayo es una lista de tramos: [PWM izquierdo, PWM derecho, duración en ms].
struct Tramo {
  int16_t uL, uR;
  uint16_t ms;
};

struct Muestra {
  uint32_t t_us;   // tiempo desde el inicio del ensayo
  int16_t uL, uR;  // PWM aplicado desde esta muestra
  int32_t cL, cR;  // cuentas leídas al inicio de la muestra
};

Encoder encL = {ENC_L_A, ENC_L_B, ENC_L_SIGNO, CUENTAS_VUELTA_L, 0, 0, 0, 0, 0, 1, 0};
Encoder encR = {ENC_R_A, ENC_R_B, ENC_R_SIGNO, CUENTAS_VUELTA_R, 0, 0, 0, 0, 0, 1, 0};
Motor motL = {MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_SIGNO, 0, &encL};
Motor motR = {MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_SIGNO, 1, &encR};

static Muestra muestras[MAX_MUESTRAS];
static int nMuestras = 0;
static int inicioTramo[24]; // índice de la primera muestra de cada tramo (+1 al final)
static float bateriaV = 0;

// ============================================================================
// 1. ENCODERS
// ============================================================================
// Un encoder en cuadratura tiene dos señales, A y B, desfasadas un cuarto de ciclo.
// El estado es el par (A, B): 00 → 01 → 11 → 10 → 00 en un sentido y al revés en el otro.
// Cada cambio de A o de B dispara una interrupción. La tabla dice cuánto sumar según el
// estado anterior y el actual: +1, −1, 0 (sin cambio) o 2 (salto imposible = ruido).
// Así se cuentan los cuatro flancos de cada ciclo (decodificación x4).
//
//   índice = estado_anterior·4 + estado_actual
static DRAM_ATTR const int8_t TABLA_CUADRATURA[16] = {
  //  actual: 00   01   10   11
  /* 00 */     0,  +1,  -1,   2,
  /* 01 */    -1,   0,   2,  +1,
  /* 10 */    +1,   2,   0,  -1,
  /* 11 */     2,  -1,  +1,   0,
};

// Lectura directa del registro de entradas: más rápida que digitalRead dentro de la interrupción.
static inline __attribute__((always_inline)) int leerPin(int pin)
{
  if (pin < 32)
    return (REG_READ(GPIO_IN_REG) >> pin) & 1;
  return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}

static void IRAM_ATTR pasoEncoder(Encoder &e)
{
  if (e.pinB < 0)
  {
    // Un solo canal: cada flanco de A vale 2 cuentas (x2) y el sentido lo da el PWM.
    e.cuentas += 2 * e.sentido;
    e.flancosA++;
    return;
  }
  const uint8_t actual = (leerPin(e.pinA) << 1) | leerPin(e.pinB);
  const uint8_t anterior = e.estado;
  if ((actual ^ anterior) & 0b10)
    e.flancosA++;
  if ((actual ^ anterior) & 0b01)
    e.flancosB++;
  const int8_t paso = TABLA_CUADRATURA[(anterior << 2) | actual];
  if (paso == 2)
    e.errores++;
  else
    e.cuentas += paso;
  e.estado = actual;
}

void IRAM_ATTR isrEncoderL() { pasoEncoder(encL); }
void IRAM_ATTR isrEncoderR() { pasoEncoder(encR); }

static void iniciarEncoder(Encoder &e, void (*isr)())
{
  pinMode(e.pinA, INPUT_PULLUP);
  if (e.pinB >= 0)
    pinMode(e.pinB, INPUT_PULLUP);
  e.estado = (leerPin(e.pinA) << 1) | (e.pinB >= 0 ? leerPin(e.pinB) : 0);
  attachInterrupt(digitalPinToInterrupt(e.pinA), isr, CHANGE);
  if (e.pinB >= 0)
    attachInterrupt(digitalPinToInterrupt(e.pinB), isr, CHANGE);
}

// Cuentas desde el último "z", con el signo corregido (avanzar = positivo).
static int32_t leerCuentas(const Encoder &e)
{
  const int8_t s = (e.pinB < 0) ? 1 : e.signo; // en un canal el sentido ya viene del PWM
  return s * (e.cuentas - e.cero);
}

static void ponerCero(Encoder &e)
{
  e.cero = e.cuentas;
  e.flancosA = e.flancosB = e.errores = 0;
}

// ============================================================================
// 2. MOTORES — TB6612FNG
// ============================================================================
// IN1/IN2 fijan el sentido y el PWM la magnitud. Con PWM = 0 y el driver habilitado el
// TB6612 frena en corto (el motor se frena con su propia fuerza contraelectromotriz).

// El generador de PWM (LEDC) cambió de interfaz en el núcleo 3.x.
static void pwmIniciar(Motor &m)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(m.pinPwm, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(m.canal, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(m.pinPwm, m.canal);
#endif
}

static void pwmEscribir(Motor &m, uint32_t duty)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(m.pinPwm, duty);
#else
  ledcWrite(m.canal, duty);
#endif
}

static void iniciarMotor(Motor &m)
{
  pinMode(m.pinIn1, OUTPUT);
  pinMode(m.pinIn2, OUTPUT);
  digitalWrite(m.pinIn1, LOW);
  digitalWrite(m.pinIn2, LOW);
  pwmIniciar(m);
  pwmEscribir(m, 0);
}

// u en [-PWM_MAX, PWM_MAX]; positivo = adelante (después de corregir con MOTOR_x_SIGNO).
static void aplicarPWM(Motor &m, int u)
{
  u = constrain(u, -PWM_MAX, PWM_MAX);
  if (u != 0)
    m.enc->sentido = (u > 0) ? 1 : -1; // lo usa el encoder de un canal
  const int v = u * m.signo;           // sentido eléctrico
  if (v > 0)
  {
    digitalWrite(m.pinIn1, HIGH);
    digitalWrite(m.pinIn2, LOW);
  }
  else if (v < 0)
  {
    digitalWrite(m.pinIn1, LOW);
    digitalWrite(m.pinIn2, HIGH);
  }
  pwmEscribir(m, abs(v));
}

static void habilitarDriver(bool on)
{
  if (PIN_STBY >= 0)
    digitalWrite(PIN_STBY, on ? HIGH : LOW);
}

static void detenerMotores()
{
  aplicarPWM(motL, 0);
  aplicarPWM(motR, 0);
  habilitarDriver(false);
}

// ============================================================================
// 3. ENSAYO: una lista de tramos [PWM izquierdo, PWM derecho, duración]
// ============================================================================
// Durante el ensayo no se imprime nada: cada muestra se guarda en la RAM y se imprime al
// final. Imprimir en vivo retrasaría el muestreo y a 2 ms no cabe por el puerto serie.
// En cada muestra k primero se leen las cuentas y luego se aplica el PWM del tramo, que
// queda fijo hasta la muestra k+1.

static void vaciarEntrada()
{
  delay(20);
  while (Serial.available())
    Serial.read();
}

// Devuelve false si el usuario lo detuvo o si no cabe en la RAM.
static bool ejecutarEnsayo(const Tramo *tr, int nTr, uint32_t ts_us)
{
  long total = 0;
  for (int j = 0; j < nTr; j++)
    total += (long)tr[j].ms * 1000L / ts_us;
  if (total + 1 > MAX_MUESTRAS || nTr > 23)
  {
    Serial.printf("El ensayo necesita %ld muestras y caben %d. Acórtelo.\n", total + 1, MAX_MUESTRAS);
    return false;
  }

  ponerCero(encL);
  ponerCero(encR);
  habilitarDriver(true);
  int k = 0;
  const uint32_t t0 = micros();
  uint32_t proxima = t0;
  for (int j = 0; j < nTr; j++)
  {
    inicioTramo[j] = k;
    const int nj = (long)tr[j].ms * 1000L / ts_us;
    for (int i = 0; i < nj; i++)
    {
      while ((int32_t)(micros() - proxima) < 0)
      {
        // espera activa hasta el instante de la muestra: periodo fijo
      }
      Muestra &m = muestras[k++];
      m.t_us = micros() - t0;
      m.cL = leerCuentas(encL);
      m.cR = leerCuentas(encR);
      aplicarPWM(motL, tr[j].uL);
      aplicarPWM(motR, tr[j].uR);
      m.uL = tr[j].uL;
      m.uR = tr[j].uR;
      proxima += ts_us;
      if (Serial.available())
      {
        detenerMotores();
        nMuestras = k;
        vaciarEntrada();
        Serial.println("Ensayo detenido por el usuario. Motores apagados.");
        return false;
      }
    }
  }
  // Muestra final: cuentas al terminar el último tramo, mando en 0.
  inicioTramo[nTr] = k;
  while ((int32_t)(micros() - proxima) < 0)
  {
  }
  Muestra &m = muestras[k++];
  m.t_us = micros() - t0;
  m.cL = leerCuentas(encL);
  m.cR = leerCuentas(encR);
  m.uL = m.uR = 0;
  detenerMotores();
  nMuestras = k;
  return true;
}

// ============================================================================
// 4. ANÁLISIS RÁPIDO (el ajuste fino se hace en el notebook con el CSV)
// ============================================================================
// RPM media entre dos muestras: cuentas giradas / cuentas por vuelta / tiempo.
// Se calcula con las cuentas acumuladas y no derivando, así no amplifica el ruido.
static float rpmEntre(int i0, int i1, char lado)
{
  const float cpv = (lado == 'L') ? encL.cuentasVuelta : encR.cuentasVuelta;
  const int32_t dc = (lado == 'L') ? muestras[i1].cL - muestras[i0].cL : muestras[i1].cR - muestras[i0].cR;
  const float dt = (muestras[i1].t_us - muestras[i0].t_us) * 1e-6f;
  return (dt > 0) ? dc / cpv / dt * 60.0f : 0.0f;
}

// Constante de tiempo por el método del área.
// Si la rueda saltara de inmediato de ra a rb, en T segundos giraría rb·T.
// Lo que giró de verdad (las cuentas) es menos. La diferencia, dividida por el salto (rb − ra),
// es τ (más el retardo, si lo hay). Usa solo cuentas acumuladas: no necesita derivar.
static float tauArea(int iS, char lado, float ra, float rb, uint32_t ts_us)
{
  const int nT = 250000UL / ts_us; // ventana de 250 ms: más de 10 veces τ
  int iT = iS + nT;
  if (iT >= nMuestras)
    iT = nMuestras - 1;
  const float T = (muestras[iT].t_us - muestras[iS].t_us) * 1e-6f;
  const float integral = rpmEntre(iS, iT, lado) * T; // ∫ RPM dt  [RPM·s]
  if (fabsf(rb - ra) < 2.0f)
    return NAN;
  return (rb * T - integral) / (rb - ra) * 1000.0f; // [ms]
}

static void analizarEscalon(char lado, int u0, int u1)
{
  const uint32_t ts = ESCALON_TS_US;
  const int iAntes = inicioTramo[0], iDurante = inicioTramo[1], iDespues = inicioTramo[2], iFin = inicioTramo[3];
  const int n60 = 60000UL / ts;
  const float r0 = rpmEntre(max(iAntes, iDurante - n60), iDurante, lado);
  const float r1 = rpmEntre(iDurante + (iDespues - iDurante) / 2, iDespues, lado);
  const float r2 = rpmEntre(iDespues + (iFin - iDespues) / 2, iFin, lado);
  const float K = (u1 != u0) ? (r1 - r0) / (float)(u1 - u0) : NAN;
  const float tauSube = tauArea(iDurante, lado, r0, r1, ts);
  const float tauBaja = tauArea(iDespues, lado, r1, r2, ts);
  Serial.printf("  Rueda %c: RPM %.2f -> %.2f -> %.2f\n", lado, r0, r1, r2);
  Serial.printf("    K = %.3f RPM/PWM (incremental)\n", K);
  Serial.printf("    tau al cambiar a %d = %.1f ms  |  tau al volver a %d = %.1f ms\n", u1, tauSube, u0, tauBaja);
  if (u0 == 0)
    Serial.println("    (al volver a PWM 0 el TB6612 frena en corto: esa tau sale menor)");
}

static void analizarEscalera(int nPeldanos, bool usarL, bool usarR)
{
  Serial.println("\n  Peldaño |  PWM  | RPM izq. | RPM der.");
  float sx[2] = {0, 0}, sy[2] = {0, 0}, sxx[2] = {0, 0}, sxy[2] = {0, 0};
  int n[2] = {0, 0}, zonaMuerta[2] = {-1, -1};
  for (int j = 0; j < nPeldanos; j++)
  {
    const int i0 = inicioTramo[j] + (inicioTramo[j + 1] - inicioTramo[j]) / 2; // segunda mitad
    const int i1 = inicioTramo[j + 1];
    const int u = (muestras[inicioTramo[j]].uL != 0) ? muestras[inicioTramo[j]].uL : muestras[inicioTramo[j]].uR;
    const float rpm[2] = {rpmEntre(i0, i1, 'L'), rpmEntre(i0, i1, 'R')};
    Serial.printf("  %7d | %5d | %8.2f | %8.2f\n", j + 1, u, rpm[0], rpm[1]);
    for (int w = 0; w < 2; w++)
    {
      if ((w == 0 && !usarL) || (w == 1 && !usarR))
        continue;
      const float x = abs(u), y = fabsf(rpm[w]);
      if (y >= 0.5f && zonaMuerta[w] < 0)
        zonaMuerta[w] = abs(u);
      if (y >= 1.0f) // solo los peldaños donde la rueda gira de forma sostenida
      {
        sx[w] += x;
        sy[w] += y;
        sxx[w] += x * x;
        sxy[w] += x * y;
        n[w]++;
      }
    }
  }
  // Recta por mínimos cuadrados: |RPM| = K·|PWM| − B
  for (int w = 0; w < 2; w++)
  {
    if ((w == 0 && !usarL) || (w == 1 && !usarR))
      continue;
    const char lado = (w == 0) ? 'L' : 'R';
    const float d = n[w] * sxx[w] - sx[w] * sx[w];
    if (n[w] < 3 || d == 0)
    {
      Serial.printf("  Rueda %c: menos de 3 peldaños con giro; amplíe la escalera.\n", lado);
      continue;
    }
    const float K = (n[w] * sxy[w] - sx[w] * sy[w]) / d;
    const float B = -(sy[w] - K * sx[w]) / n[w];
    Serial.printf("  Rueda %c: RPM = %.3f * PWM - %.2f  (%d peldaños)  |  arranca con PWM >= %d\n", lado, K, B, n[w],
                  zonaMuerta[w]);
  }
}

// ============================================================================
// 5. SALIDA CSV
// ============================================================================
// Entre las marcas INICIO_CSV y FIN_CSV. Copie ese bloque a un archivo .csv para el notebook.
static void imprimirCSV(const char *descripcion, uint32_t ts_us)
{
  Serial.println("INICIO_CSV");
  Serial.printf("# %s ts_us=%lu bateria_V=%.2f cuentas_vuelta_L=%.1f cuentas_vuelta_R=%.1f "
                "errores_L=%lu errores_R=%lu sketch=%s\n",
                descripcion, (unsigned long)ts_us, bateriaV, encL.cuentasVuelta, encR.cuentasVuelta,
                (unsigned long)encL.errores, (unsigned long)encR.errores, VERSION_SKETCH);
  Serial.println("t_us,uL,uR,cL,cR");
  for (int k = 0; k < nMuestras; k++)
  {
    const Muestra &m = muestras[k];
    Serial.printf("%lu,%d,%d,%ld,%ld\n", (unsigned long)m.t_us, m.uL, m.uR, (long)m.cL, (long)m.cR);
  }
  Serial.println("FIN_CSV");
}

// ============================================================================
// 6. COMANDOS
// ============================================================================
static bool cuentaRegresiva()
{
  Serial.println("\n>> Robot SOBRE EL SOPORTE, ruedas en el aire. Arranca en 3 s. Cualquier tecla cancela.");
  vaciarEntrada();
  const uint32_t t = millis();
  while (millis() - t < 3000)
  {
    if (Serial.available())
    {
      vaciarEntrada();
      Serial.println("Cancelado.");
      return false;
    }
    delay(10);
  }
  return true;
}

// L o I = izquierdo, R o D = derecho, A = ambos
static bool leerMotor(char c, bool &usarL, bool &usarR)
{
  c = toupper(c);
  usarL = (c == 'L' || c == 'I' || c == 'A');
  usarR = (c == 'R' || c == 'D' || c == 'A');
  if (!usarL && !usarR)
  {
    Serial.println("Motor no válido: use L (izquierdo), R (derecho) o A (ambos).");
    return false;
  }
  return true;
}

static void ayuda()
{
  Serial.println(
      "\n=== Identificación de motores · Control Inteligente II ===\n"
      " e                 Monitor de encoders (gire las ruedas a mano). Cualquier tecla sale.\n"
      " z                 Poner en cero las cuentas.\n"
      " c N               Cuentas por vuelta: tras 'z', gire cada rueda N vueltas a mano.\n"
      " d                 Prueba de sentido de motores y encoders.\n"
      " x M ini paso n    Escalera de PWM. Ej.: x R 10 5 14   (atrás: x R -10 -5 14)\n"
      " s M pwm [base]    Escalón base -> pwm -> base. Ej.: s R 120   o   s R 120 60\n"
      " b V               Anotar el voltaje de la batería medido con el multímetro. Ej.: b 7.9\n"
      " h                 Esta ayuda.\n"
      " M = L (izquierdo), R (derecho) o A (ambos).\n"
      " Ensayos con motores: robot en el soporte. Cualquier tecla detiene los motores.");
}

static void monitorEncoders()
{
  Serial.println("Gire las ruedas a mano. Cualquier tecla sale.");
  vaciarEntrada();
  while (!Serial.available())
  {
    for (Encoder *e : {&encL, &encR})
    {
      const int32_t c = leerCuentas(*e);
      Serial.printf("%c: cuentas %7ld  vueltas %7.3f  A=%d B=%d  flancos A %lu B %lu  errores %lu   ",
                    e == &encL ? 'L' : 'R', (long)c, c / e->cuentasVuelta, leerPin(e->pinA),
                    e->pinB >= 0 ? leerPin(e->pinB) : -1, (unsigned long)e->flancosA,
                    (unsigned long)e->flancosB, (unsigned long)e->errores);
    }
    Serial.println();
    delay(200);
  }
  vaciarEntrada();
}

static void cuentasPorVuelta(float vueltas)
{
  if (vueltas <= 0)
  {
    Serial.println("Uso: c N   (N = vueltas dadas a mano después de 'z')");
    return;
  }
  for (Encoder *e : {&encL, &encR})
  {
    const char lado = (e == &encL) ? 'L' : 'R';
    const float cpv = leerCuentas(*e) / vueltas;
    Serial.printf("%c: %ld cuentas en %.1f vueltas = %.1f cuentas por vuelta\n", lado, (long)leerCuentas(*e), vueltas, cpv);
    if (fabsf(cpv) > 10)
    {
      e->cuentasVuelta = fabsf(cpv);
      Serial.printf("   Copie en config.h:  #define CUENTAS_VUELTA_%c  %.1ff\n", lado, fabsf(cpv));
    }
    if (cpv < 0)
      Serial.printf("   Contó negativo: si giró la rueda hacia ADELANTE, cambie ENC_%c_SIGNO.\n", lado);
  }
}

static void pruebaSentido()
{
  if (!cuentaRegresiva())
    return;
  for (Motor *m : {&motL, &motR})
  {
    const char lado = (m == &motL) ? 'L' : 'R';
    Encoder &propio = *m->enc;
    Encoder &otro = (m == &motL) ? encR : encL;
    ponerCero(encL);
    ponerCero(encR);
    Serial.printf("\nMotor %c a PWM +%d durante 0.8 s. MIRE HACIA DÓNDE GIRA LA RUEDA.\n", lado, PWM_SENTIDO);
    habilitarDriver(true);
    aplicarPWM(*m, PWM_SENTIDO);
    delay(800);
    detenerMotores();
    delay(400);
    const int32_t cp = leerCuentas(propio), co = leerCuentas(otro);
    const int32_t umbral = propio.cuentasVuelta * 0.02f;
    Serial.printf("  Encoder %c: %ld cuentas | encoder de la otra rueda: %ld | errores: %lu\n", lado, (long)cp,
                  (long)co, (unsigned long)propio.errores);
    if (abs(cp) < umbral && abs(co) >= umbral)
      Serial.println("  -> Se movió el encoder de la OTRA rueda: los encoders están cruzados en config.h.");
    else if (abs(cp) < umbral)
      Serial.println("  -> No llegaron pulsos. ¿Giró la rueda? No: revise STBY, IN1/IN2/PWM y batería. "
                     "Sí: revise alimentación y pines del encoder.");
    else if (propio.pinB < 0)
      Serial.printf("  -> Encoder de un canal: si la rueda giró hacia ATRÁS, cambie MOTOR_%c_SIGNO.\n", lado);
    else if (cp > 0)
      Serial.printf("  -> Contó positivo. Rueda hacia ADELANTE: todo bien. Hacia ATRÁS: cambie MOTOR_%c_SIGNO "
                    "y ENC_%c_SIGNO.\n", lado, lado);
    else
      Serial.printf("  -> Contó negativo. Rueda hacia ADELANTE: cambie ENC_%c_SIGNO. Hacia ATRÁS: cambie "
                    "MOTOR_%c_SIGNO.\n", lado, lado);
    if (propio.errores > (uint32_t)abs(cp) / 100 + 2)
      Serial.println("  -> Muchas transiciones inválidas: ruido o un cable flojo en el encoder.");
  }
}

static void ensayoEscalera(bool usarL, bool usarR, int ini, int paso, int n)
{
  if (n < 2 || n > 22 || paso == 0)
  {
    Serial.println("Uso: x M ini paso n   (2 a 22 peldaños). Ej.: x R 10 5 14");
    return;
  }
  Tramo tr[23];
  for (int j = 0; j < n; j++)
  {
    const int u = constrain(ini + j * paso, -PWM_MAX, PWM_MAX);
    tr[j] = {(int16_t)(usarL ? u : 0), (int16_t)(usarR ? u : 0), ESCALERA_PELDANO_MS};
  }
  Serial.printf("Escalera: PWM %d a %d, paso %d, %d peldaños de %d ms.\n", ini, ini + (n - 1) * paso, paso, n,
                ESCALERA_PELDANO_MS);
  if (!cuentaRegresiva() || !ejecutarEnsayo(tr, n, ESCALERA_TS_US))
    return;
  char desc[96];
  snprintf(desc, sizeof(desc), "ensayo=escalera motor=%s ini=%d paso=%d peldanos=%d peldano_ms=%d",
           usarL && usarR ? "A" : (usarL ? "L" : "R"), ini, paso, n, ESCALERA_PELDANO_MS);
  imprimirCSV(desc, ESCALERA_TS_US);
  Serial.println("\nResultado rápido (escalera):");
  analizarEscalera(n, usarL, usarR);
}

static void ensayoEscalon(bool usarL, bool usarR, int u1, int u0)
{
  u1 = constrain(u1, -PWM_MAX, PWM_MAX);
  u0 = constrain(u0, -PWM_MAX, PWM_MAX);
  const Tramo tr[3] = {
      {(int16_t)(usarL ? u0 : 0), (int16_t)(usarR ? u0 : 0), ESCALON_ANTES_MS},
      {(int16_t)(usarL ? u1 : 0), (int16_t)(usarR ? u1 : 0), ESCALON_DURANTE_MS},
      {(int16_t)(usarL ? u0 : 0), (int16_t)(usarR ? u0 : 0), ESCALON_DESPUES_MS},
  };
  Serial.printf("Escalón: PWM %d -> %d -> %d (%d / %d / %d ms), muestreo %d us.\n", u0, u1, u0, ESCALON_ANTES_MS,
                ESCALON_DURANTE_MS, ESCALON_DESPUES_MS, ESCALON_TS_US);
  if (!cuentaRegresiva() || !ejecutarEnsayo(tr, 3, ESCALON_TS_US))
    return;
  char desc[96];
  snprintf(desc, sizeof(desc), "ensayo=escalon motor=%s base=%d escalon=%d", usarL && usarR ? "A" : (usarL ? "L" : "R"),
           u0, u1);
  imprimirCSV(desc, ESCALON_TS_US);
  Serial.printf("\nResultado rápido (escalón, batería %.2f V):\n", bateriaV);
  if (usarL)
    analizarEscalon('L', u0, u1);
  if (usarR)
    analizarEscalon('R', u0, u1);
}

static void procesar(String linea)
{
  linea.trim();
  if (linea.length() == 0)
    return;
  const char cmd = tolower(linea[0]);
  const char *arg = linea.c_str() + 1;
  char m = 0;
  long a = 0, b = 0, c = 0;
  float f = 0;
  bool usarL = false, usarR = false;

  switch (cmd)
  {
  case 'h':
  case '?':
    ayuda();
    break;
  case 'e':
    monitorEncoders();
    break;
  case 'z':
    ponerCero(encL);
    ponerCero(encR);
    Serial.println("Cuentas en cero.");
    break;
  case 'c':
    if (sscanf(arg, "%f", &f) == 1)
      cuentasPorVuelta(f);
    else
      cuentasPorVuelta(0);
    break;
  case 'b':
    if (sscanf(arg, "%f", &f) == 1 && f > 0 && f < 20)
    {
      bateriaV = f;
      Serial.printf("Batería: %.2f V (se anota en cada CSV).\n", bateriaV);
    }
    else
      Serial.println("Uso: b V   Ej.: b 7.9");
    break;
  case 'd':
    pruebaSentido();
    break;
  case 'x':
  {
    const int n = sscanf(arg, " %c %ld %ld %ld", &m, &a, &b, &c);
    if (n >= 1 && leerMotor(m, usarL, usarR))
      ensayoEscalera(usarL, usarR, n >= 2 ? a : 10, n >= 3 ? b : 5, n >= 4 ? c : 14);
    else if (n < 1)
      Serial.println("Uso: x M ini paso n   Ej.: x R 10 5 14");
    break;
  }
  case 's':
  {
    const int n = sscanf(arg, " %c %ld %ld", &m, &a, &b);
    if (n >= 2 && leerMotor(m, usarL, usarR))
      ensayoEscalon(usarL, usarR, a, n >= 3 ? b : 0);
    else if (n < 2)
      Serial.println("Uso: s M pwm [base]   Ej.: s R 120");
    break;
  }
  default:
    Serial.println("Comando desconocido. Escriba h para ver la ayuda.");
  }
}

// ============================================================================
// 7. SETUP Y LOOP
// ============================================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(100); // así funciona con o sin fin de línea en el monitor
  const uint32_t t = millis();
  while (!Serial && millis() - t < 2000)
  {
  }
  if (PIN_STBY >= 0)
  {
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, LOW);
  }
  iniciarMotor(motL);
  iniciarMotor(motR);
  iniciarEncoder(encL, isrEncoderL);
  iniciarEncoder(encR, isrEncoderR);

  Serial.printf("\nIdentificación de motores v%s · núcleo Arduino-ESP32 %d.%d.%d\n", VERSION_SKETCH,
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Serial.printf("Encoder L: A=%d B=%d (%s) · encoder R: A=%d B=%d (%s)\n", ENC_L_A, ENC_L_B,
                ENC_L_B < 0 ? "un canal, x2" : "x4", ENC_R_A, ENC_R_B, ENC_R_B < 0 ? "un canal, x2" : "x4");
  Serial.printf("Cuentas por vuelta: L %.1f · R %.1f · PWM %d Hz, %d bits\n", encL.cuentasVuelta, encR.cuentasVuelta,
                PWM_FREQ_HZ, PWM_BITS);
  ayuda();
}

void loop()
{
  if (Serial.available())
    procesar(Serial.readStringUntil('\n'));
}
