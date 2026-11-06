/*
 * lazo_rpm — Verificación del lazo de RPM de cada rueda, diseñado desde el modelo del motor
 * Control Inteligente II (168418) · Universidad de Pamplona · Ing. Jeisson Martínez Flórez
 * Versión 1.0 · noviembre de 2026
 *
 * Qué hace:
 *   - Cierra un lazo de RPM en cada rueda, cada 5 ms, con el modelo del Lab 1:
 *       u = u_ff + Kp·e + I        e = r_f − y
 *       u_ff = signo(r)·(|r| + N0·s)/K_m      (inversa de la recta; N0/K_m compensa la zona muerta)
 *       PI por cancelación de polo: Ti = τ, Kp = τ/(K_m·(λ + L_total)), Ki = Kp/τ
 *       Integración condicional: no integra si el mando está saturado en el sentido del error.
 *   - Aplica un escalón de consigna (base → escalón → base) con el robot en el soporte.
 *   - Guarda cada muestra en la RAM y la imprime al final como CSV, igual que el sketch de
 *     identificación. El notebook del Taller 2 compara la medida con el modelo en lazo cerrado.
 *
 * Archivos que se modifican:
 *   - config.h: el MISMO del Lab 1 (pines, signos y cuentas por vuelta). Cópielo tal cual.
 *   - lazo_config.h: el modelo de sus motores y el diseño. Lo genera el notebook del Taller 2.
 *
 * SEGURIDAD: robot sobre el soporte, ruedas en el aire. Enter detiene los motores.
 */
#include <Arduino.h>
#include <esp_arduino_version.h>
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "config.h"
#include "lazo_config.h"

#define VERSION_SKETCH "lazo-1.0"

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

// Recta estática de un sentido: RPM ≈ K_m·PWM − N0, válida por encima de la zona muerta.
struct Recta {
  float Km, N0;
  int zm;
};

// Una rueda: su modelo, su diseño y el estado de su lazo.
struct Rueda {
  char lado;
  Motor *mot;
  Recta adelante, atras;
  float tau, retardo;          // [s]
  float kp[2], ki[2];          // [0] consigna positiva, [1] consigna negativa
  // Estado del lazo
  float integ, refF, u, uFF, uP, y;
};

// Un tramo del ensayo: consigna de cada rueda [RPM] y duración.
struct Tramo {
  float rL, rR;
  uint16_t ms;
};

struct Muestra {
  uint32_t t_us;     // tiempo desde el inicio del ensayo
  int16_t rL, rR;    // consigna ×10 [0,1 RPM]
  int16_t uL, uR;    // PWM aplicado desde esta muestra
  int32_t cL, cR;    // cuentas leídas al inicio de la muestra
};

Encoder encL = {ENC_L_A, ENC_L_B, ENC_L_SIGNO, CUENTAS_VUELTA_L, 0, 0, 0, 0, 0, 1, 0};
Encoder encR = {ENC_R_A, ENC_R_B, ENC_R_SIGNO, CUENTAS_VUELTA_R, 0, 0, 0, 0, 0, 1, 0};
Motor motL = {MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_SIGNO, 0, &encL};
Motor motR = {MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_SIGNO, 1, &encR};

Rueda ruedaL = {'L', &motL, {KM_L_ADELANTE, N0_L_ADELANTE, ZM_L_ADELANTE}, {KM_L_ATRAS, N0_L_ATRAS, ZM_L_ATRAS},
                TAU_L_S, RETARDO_L_S};
Rueda ruedaR = {'R', &motR, {KM_R_ADELANTE, N0_R_ADELANTE, ZM_R_ADELANTE}, {KM_R_ATRAS, N0_R_ATRAS, ZM_R_ATRAS},
                TAU_R_S, RETARDO_R_S};

// Diseño vigente (se cambia con comandos)
static float lambdaMs = LAMBDA_MS;
static int criterioK = CRITERIO_K;      // 0 = por sentido, 1 = promediada
static bool conFF = PREALIMENTACION;
static float refTauMs = REF_TAU_MS;

static Muestra muestras[MAX_MUESTRAS];
static int nMuestras = 0;
static int inicioTramo[8];
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
// casiQuieta: en el lazo, la rueda gira a menos de 2 RPM. El encoder de un canal toma el sentido
// del PWM, pero solo lo cambia con la rueda casi detenida: si el PWM cambia de signo mientras la
// rueda todavía gira en el sentido anterior, esas cuentas siguen siendo del sentido anterior.
static void aplicarPWM(Motor &m, int u, bool casiQuieta = true)
{
  u = constrain(u, -PWM_MAX, PWM_MAX);
  if (u != 0 && casiQuieta)
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
// 3. DISEÑO DEL PI DESDE EL MODELO
// ============================================================================
// Ti = τ: el cero del PI cancela el polo del motor.
// Kp = τ / (K_m·(λ + L_total)): λ es la constante de tiempo que se quiere en lazo cerrado.
// L_total = retardo del motor + medio periodo del lazo + media ventana de medición.

static const char *nombreCriterio() { return criterioK == 0 ? "por_sentido" : "promediada"; }

// Recta que usa la prealimentación para una consigna de signo s (+1 o −1), según el criterio.
static Recta rectaDe(const Rueda &w, int s)
{
  if (criterioK == 1)
    return {0.5f * (w.adelante.Km + w.atras.Km), 0.5f * (w.adelante.N0 + w.atras.N0),
            (w.adelante.zm + w.atras.zm + 1) / 2};
  return (s >= 0) ? w.adelante : w.atras;
}

static float lTotal(const Rueda &w)
{
  return w.retardo + 0.5f * TS_LAZO_US * 1e-6f + 0.5f * VENTANA_MS * 1e-3f;
}

static void disenarPI(Rueda &w)
{
  for (int i = 0; i < 2; i++)
  {
    const Recta rc = rectaDe(w, i == 0 ? +1 : -1);
    w.kp[i] = w.tau / (rc.Km * (lambdaMs * 1e-3f + lTotal(w)));
    w.ki[i] = w.kp[i] / w.tau;
  }
}

static void disenar()
{
  disenarPI(ruedaL);
  disenarPI(ruedaR);
}

static void mostrarDiseno()
{
  Serial.printf("\nDiseño: λ = %.0f ms · criterio de K: %s · prealimentación: %s · modelo de referencia: %s\n", lambdaMs,
                nombreCriterio(), conFF ? "sí" : "no", refTauMs > 0 ? "sí" : "no");
  if (refTauMs > 0)
    Serial.printf("        τ del modelo de referencia = %.0f ms\n", refTauMs);
  Serial.printf("        Ts = %d ms · ventana = %d ms · batería anotada = %.2f V\n", TS_LAZO_US / 1000, VENTANA_MS,
                bateriaV);
  for (Rueda *w : {&ruedaL, &ruedaR})
  {
    Serial.printf(" Rueda %c: τ = %.1f ms, retardo = %.1f ms, L_total = %.1f ms\n", w->lado, w->tau * 1e3f,
                  w->retardo * 1e3f, lTotal(*w) * 1e3f);
    for (int i = 0; i < 2; i++)
    {
      const Recta rc = rectaDe(*w, i == 0 ? +1 : -1);
      Serial.printf("   %-8s RPM = %.3f·PWM − %.2f · N0/K_m = %4.1f PWM (arranca con %d) · Kp = %.2f · Ki = %.1f\n",
                    i == 0 ? "adelante" : "atrás", rc.Km, rc.N0, rc.N0 / rc.Km, rc.zm, w->kp[i], w->ki[i]);
    }
  }
}

// ============================================================================
// 4. LAZO DE RPM (un paso, cada Ts)
// ============================================================================
static inline float signo(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }

static void reiniciarLazo(Rueda &w)
{
  w.integ = w.refF = w.u = w.uFF = w.uP = 0;
}

// r: consigna [RPM]; y: RPM medida [RPM]; dt: periodo [s]. Devuelve el PWM.
static float pasoLazo(Rueda &w, float r, float y, float dt)
{
  w.y = y;
  if (r == 0)
  {
    reiniciarLazo(w); // consigna 0: freno en corto, sin integral que empuje
    return 0;
  }
  // Modelo de referencia (opcional): la respuesta que se espera del motor a la prealimentación
  if (refTauMs > 0)
    w.refF += (r - w.refF) * (1.0f - expf(-dt / (refTauMs * 1e-3f)));
  else
    w.refF = r;
  const float e = w.refF - y;
  const int i = (r > 0) ? 0 : 1;
  const Recta rc = rectaDe(w, r > 0 ? +1 : -1);
  // Prealimentación: inversa de la recta. El término N0/K_m es la compensación de la zona muerta;
  // crece desde 0 hasta |r| = R_SUAVE_RPM para que una consigna que cruza 0 no dé golpes.
  const float s = fminf(1.0f, fabsf(r) / R_SUAVE_RPM);
  w.uFF = conFF ? signo(r) * (fabsf(r) + rc.N0 * s) / rc.Km : 0.0f;
  w.uP = w.kp[i] * e;
  // Integración condicional: no integrar si el mando ya está saturado en el sentido del error
  const float candidato = constrain(w.integ + w.ki[i] * e * dt, -I_MAX_PWM, I_MAX_PWM);
  const float crudo = w.uFF + w.uP + candidato;
  if (!((crudo > PWM_MAX && e > 0) || (crudo < -PWM_MAX && e < 0)))
    w.integ = candidato;
  w.u = constrain(w.uFF + w.uP + w.integ, -PWM_MAX, PWM_MAX);
  return w.u;
}

// ============================================================================
// 5. ENSAYO: una lista de tramos [consigna izquierda, consigna derecha, duración]
// ============================================================================
// En cada muestra k: se leen las cuentas, se mide la RPM con la ventana, se calcula el PWM de
// cada lazo y se aplica. El PWM queda fijo hasta la muestra k+1. Nada se imprime durante el
// ensayo: las muestras se guardan en la RAM.

static void vaciarEntrada()
{
  delay(20);
  while (Serial.available())
    Serial.read();
}

static bool ejecutarEnsayo(const Tramo *tr, int nTr)
{
  const uint32_t ts_us = TS_LAZO_US;
  const int nV = max(1, VENTANA_MS * 1000 / TS_LAZO_US); // muestras de la ventana
  long total = 0;
  for (int j = 0; j < nTr; j++)
    total += (long)tr[j].ms * 1000L / ts_us;
  if (total + 1 > MAX_MUESTRAS || nTr > 7)
  {
    Serial.printf("El ensayo necesita %ld muestras y caben %d. Acórtelo.\n", total + 1, MAX_MUESTRAS);
    return false;
  }

  ponerCero(encL);
  ponerCero(encR);
  reiniciarLazo(ruedaL);
  reiniciarLazo(ruedaR);
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
      Muestra &m = muestras[k];
      m.t_us = micros() - t0;
      m.cL = leerCuentas(encL);
      m.cR = leerCuentas(encR);
      // RPM medida: cuentas en la ventana / cuentas por vuelta / tiempo
      float yL = 0, yR = 0;
      if (k >= nV)
      {
        const Muestra &a = muestras[k - nV];
        const float dtv = (m.t_us - a.t_us) * 1e-6f;
        yL = (m.cL - a.cL) / encL.cuentasVuelta / dtv * 60.0f;
        yR = (m.cR - a.cR) / encR.cuentasVuelta / dtv * 60.0f;
      }
      const float dt = ts_us * 1e-6f;
      const float uL = pasoLazo(ruedaL, tr[j].rL, yL, dt);
      const float uR = pasoLazo(ruedaR, tr[j].rR, yR, dt);
      aplicarPWM(motL, (int)lroundf(uL), k < nV || fabsf(yL) < 2.0f); // al inicio la rueda está quieta
      aplicarPWM(motR, (int)lroundf(uR), k < nV || fabsf(yR) < 2.0f);
      m.rL = (int16_t)lroundf(tr[j].rL * 10);
      m.rR = (int16_t)lroundf(tr[j].rR * 10);
      m.uL = (int16_t)lroundf(uL);
      m.uR = (int16_t)lroundf(uR);
      k++;
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
  m.rL = m.rR = 0;
  m.uL = m.uR = 0;
  detenerMotores();
  nMuestras = k;
  return true;
}

// ============================================================================
// 6. ANÁLISIS RÁPIDO (la comparación con el modelo se hace en el notebook)
// ============================================================================
// RPM media entre dos muestras, con las cuentas acumuladas.
static float rpmEntre(int i0, int i1, char lado)
{
  const float cpv = (lado == 'L') ? encL.cuentasVuelta : encR.cuentasVuelta;
  const int32_t dc = (lado == 'L') ? muestras[i1].cL - muestras[i0].cL : muestras[i1].cR - muestras[i0].cR;
  const float dt = (muestras[i1].t_us - muestras[i0].t_us) * 1e-6f;
  return (dt > 0) ? dc / cpv / dt * 60.0f : 0.0f;
}

// Subida 10–90 %, sobrepico y error en régimen del escalón r0 → r1, con RPM en ventana de 20 ms.
static void analizarEscalon(char lado, float r0, float r1)
{
  const int iDurante = inicioTramo[1], iDespues = inicioTramo[2];
  const int h = max(1, 10000 / TS_LAZO_US); // media ventana de 10 ms
  const float salto = r1 - r0;
  float t10 = NAN, t90 = NAN, pico = 0;
  for (int k = iDurante + h; k < iDespues - h; k++)
  {
    const float y = rpmEntre(k - h, k + h, lado);
    const float avance = (y - r0) / salto; // 0 al inicio, 1 en la consigna
    const float t = (muestras[k].t_us - muestras[iDurante].t_us) * 1e-3f;
    if (isnan(t10) && avance >= 0.1f)
      t10 = t;
    if (isnan(t90) && avance >= 0.9f)
      t90 = t;
    pico = fmaxf(pico, avance - 1.0f);
  }
  const float yReg = rpmEntre(iDurante + (iDespues - iDurante) / 2, iDespues, lado);
  Serial.printf("  Rueda %c: consigna %.1f -> %.1f RPM · régimen %.2f RPM (error %+.2f)\n", lado, r0, r1, yReg,
                r1 - yReg);
  Serial.printf("    subida 10-90 %% = %.0f ms · sobrepico = %.0f %%\n", t90 - t10, 100.0f * pico);
}

// ============================================================================
// 7. SALIDA CSV
// ============================================================================
static void imprimirCSV(const char *descripcion)
{
  Serial.println("INICIO_CSV");
  Serial.printf("# %s ts_us=%d ventana_ms=%d lambda_ms=%.0f criterio=%s ff=%d ref_tau_ms=%.0f bateria_V=%.2f "
                "cuentas_vuelta_L=%.1f cuentas_vuelta_R=%.1f errores_L=%lu errores_R=%lu "
                "kp_L_ad=%.4f ki_L_ad=%.3f kp_L_at=%.4f ki_L_at=%.3f kp_R_ad=%.4f ki_R_ad=%.3f kp_R_at=%.4f "
                "ki_R_at=%.3f sketch=%s\n",
                descripcion, TS_LAZO_US, VENTANA_MS, lambdaMs, nombreCriterio(), conFF ? 1 : 0, refTauMs, bateriaV,
                encL.cuentasVuelta, encR.cuentasVuelta, (unsigned long)encL.errores, (unsigned long)encR.errores,
                ruedaL.kp[0], ruedaL.ki[0], ruedaL.kp[1], ruedaL.ki[1], ruedaR.kp[0], ruedaR.ki[0], ruedaR.kp[1],
                ruedaR.ki[1], VERSION_SKETCH);
  Serial.println("t_us,rL,rR,uL,uR,cL,cR");
  for (int k = 0; k < nMuestras; k++)
  {
    const Muestra &m = muestras[k];
    Serial.printf("%lu,%.1f,%.1f,%d,%d,%ld,%ld\n", (unsigned long)m.t_us, m.rL / 10.0f, m.rR / 10.0f, m.uL, m.uR,
                  (long)m.cL, (long)m.cR);
  }
  Serial.println("FIN_CSV");
}

// ============================================================================
// 8. COMANDOS
// ============================================================================
static bool cuentaRegresiva()
{
  Serial.println("\n>> Robot SOBRE EL SOPORTE, ruedas en el aire. Arranca en 3 s. Enter cancela.");
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
    Serial.println("Motor no válido: use L (izquierdo), R (derecho) o A (ambos). Ej.: r A 30 10");
    return false;
  }
  return true;
}

static void ayuda()
{
  Serial.println(
      "\n=== Lazo de RPM · Control Inteligente II ===\n"
      " Escriba el comando con letras y números reales y pulse Enter. Ej.: r A 30 10\n"
      " r M rpm [base]    Escalón de consigna base -> rpm -> base. Ej.: r A 30 10   (atrás: r A -30 -10)\n"
      "                   Cruce por cero: r A -10 10. Desde parado: r A 10\n"
      " l ms              λ del diseño (recalcula Kp y Ki). Ej.: l 50   o   l 30\n"
      " k s | k p         Criterio de K: s = por sentido, p = promediada (para comparar)\n"
      " f 1 | f 0         Prealimentación encendida o apagada (solo PI)\n"
      " m ms              Modelo de referencia: τ del filtro de la consigna. Ej.: m 20. m 0 lo apaga\n"
      " b V               Anotar el voltaje de la batería medido con el multímetro. Ej.: b 7.9\n"
      " v                 Ver el modelo y el diseño vigente (Kp y Ki por rueda y sentido)\n"
      " h                 Esta ayuda.\n"
      " M = L (izquierdo), R (derecho) o A (ambos).\n"
      " Ensayos con motores: robot en el soporte. Enter detiene los motores.");
}

static void ensayoLazo(bool usarL, bool usarR, float r1, float r0)
{
  r1 = constrain(r1, -REF_MAX_RPM, REF_MAX_RPM);
  r0 = constrain(r0, -REF_MAX_RPM, REF_MAX_RPM);
  const Tramo tr[3] = {
      {usarL ? r0 : 0, usarR ? r0 : 0, LAZO_ANTES_MS},
      {usarL ? r1 : 0, usarR ? r1 : 0, LAZO_DURANTE_MS},
      {usarL ? r0 : 0, usarR ? r0 : 0, LAZO_DESPUES_MS},
  };
  Serial.printf("Escalón de consigna: %.1f -> %.1f -> %.1f RPM (%d / %d / %d ms), Ts = %d ms.\n", r0, r1, r0,
                LAZO_ANTES_MS, LAZO_DURANTE_MS, LAZO_DESPUES_MS, TS_LAZO_US / 1000);
  if (!cuentaRegresiva() || !ejecutarEnsayo(tr, 3))
    return;
  char desc[96];
  snprintf(desc, sizeof(desc), "ensayo=lazo motor=%s base=%.1f escalon=%.1f",
           usarL && usarR ? "A" : (usarL ? "L" : "R"), r0, r1);
  imprimirCSV(desc);
  Serial.printf("\nResultado rápido (lazo, λ = %.0f ms, criterio %s, batería %.2f V):\n", lambdaMs, nombreCriterio(),
                bateriaV);
  if (usarL)
    analizarEscalon('L', r0, r1);
  if (usarR)
    analizarEscalon('R', r0, r1);
}

static void procesar(String linea)
{
  linea.trim();
  if (linea.length() == 0)
    return;
  const char cmd = tolower(linea[0]);
  const char *arg = linea.c_str() + 1;
  char m = 0;
  float a = 0, b = 0;
  bool usarL = false, usarR = false;

  switch (cmd)
  {
  case 'h':
  case '?':
    ayuda();
    break;
  case 'v':
    mostrarDiseno();
    break;
  case 'b':
    if (sscanf(arg, "%f", &a) == 1 && a > 0 && a < 20)
    {
      bateriaV = a;
      Serial.printf("Batería: %.2f V (se anota en cada CSV).\n", bateriaV);
    }
    else
      Serial.println("Uso: b V   Ej.: b 7.9");
    break;
  case 'l':
    if (sscanf(arg, "%f", &a) == 1 && a >= 5 && a <= 500)
    {
      lambdaMs = a;
      disenar();
      mostrarDiseno();
    }
    else
      Serial.println("Uso: l ms   (5 a 500). Ej.: l 50");
    break;
  case 'k':
    if (sscanf(arg, " %c", &m) == 1 && (tolower(m) == 's' || tolower(m) == 'p'))
    {
      criterioK = (tolower(m) == 's') ? 0 : 1;
      disenar();
      mostrarDiseno();
    }
    else
      Serial.println("Uso: k s (por sentido)  o  k p (promediada)");
    break;
  case 'f':
    if (sscanf(arg, "%f", &a) == 1)
    {
      conFF = (a != 0);
      Serial.printf("Prealimentación %s.\n", conFF ? "encendida" : "apagada (solo PI)");
    }
    else
      Serial.println("Uso: f 1  o  f 0");
    break;
  case 'm':
    if (sscanf(arg, "%f", &a) == 1 && a >= 0 && a <= 200)
    {
      refTauMs = a;
      if (a > 0)
        Serial.printf("Modelo de referencia: la consigna se filtra con τ = %.0f ms.\n", a);
      else
        Serial.println("Modelo de referencia apagado.");
    }
    else
      Serial.println("Uso: m ms   (0 a 200). Ej.: m 20");
    break;
  case 'r':
  {
    const int n = sscanf(arg, " %c %f %f", &m, &a, &b);
    if (n >= 2 && leerMotor(m, usarL, usarR))
      ensayoLazo(usarL, usarR, a, n >= 3 ? b : 0);
    else if (n < 2)
      Serial.println("Uso: r M rpm [base]   Ej.: r A 30 10");
    break;
  }
  default:
    Serial.printf("Comando desconocido: \"%s\". Escriba h para ver la ayuda.\n", linea.c_str());
  }
}

// ============================================================================
// 9. SETUP Y LOOP
// ============================================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
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
  disenar();

  Serial.printf("\nLazo de RPM %s · núcleo Arduino-ESP32 %d.%d.%d\n", VERSION_SKETCH, ESP_ARDUINO_VERSION_MAJOR,
                ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Serial.printf("Encoder L: A=%d B=%d (%s) · encoder R: A=%d B=%d (%s)\n", ENC_L_A, ENC_L_B,
                ENC_L_B < 0 ? "un canal, x2" : "x4", ENC_R_A, ENC_R_B, ENC_R_B < 0 ? "un canal, x2" : "x4");
  Serial.printf("Cuentas por vuelta: L %.1f · R %.1f · PWM %d Hz, %d bits\n", encL.cuentasVuelta, encR.cuentasVuelta,
                PWM_FREQ_HZ, PWM_BITS);
  mostrarDiseno();
  ayuda();
}

// Junta los caracteres hasta el fin de línea (igual que el sketch de identificación).
void loop()
{
  static String linea;
  while (Serial.available())
  {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r')
    {
      procesar(linea);
      linea = "";
    }
    else if (linea.length() < 64)
      linea += ch;
  }
}
