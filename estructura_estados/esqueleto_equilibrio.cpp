/*
 * esqueleto_equilibrio.cpp — Cómo se arma un ciclo de 5 ms en SU firmware
 * Control Inteligente II (168418) · Universidad de Pamplona · Ing. Jeisson Martínez Flórez
 *
 * Esto es un ESQUELETO. Compila, pero no mueve el robot hasta que su grupo escriba las partes
 * marcadas con TODO, con el hardware y el código que ya tiene.
 *
 * Un ciclo, cada 5 ms (200 Hz), siempre en este orden:
 *   1. IMU → θ y θ̇
 *   2. Encoders → vueltas y RPM medida de cada rueda
 *   3. Seguridad y lazo de equilibrio → RPM deseada
 *   4. Lazo de RPM de cada rueda → PWM
 *   5. Copia para la telemetría
 *
 * El control corre en su propia tarea. loop() solo atiende comandos e imprime la telemetría,
 * para que imprimir nunca retrase el control.
 */
#include <Arduino.h>
#include "config.h"             // pines, signos y cuentas por vuelta del Lab 1
#include "lazo_config.h"        // el que generó el notebook del Taller 2 para su grupo
#include "lazo_rpm_nucleo.h"    // ruedaL, ruedaR, disenar(), pasoLazo()
#include "estructura_estados.h" // Ganancias, Equilibrio, filtroComplementario(), pasoEquilibrio()

// ============================================================================
// 1. AJUSTES DEL GRUPO
// ============================================================================
#define SIGNO_THETA   +1     // que θ salga POSITIVO con el robot inclinado hacia ADELANTE
#define SIGNO_OMEGA   +1     // que θ̇ salga POSITIVA mientras se inclina hacia ADELANTE
#define TAU_FILTRO_S  0.5f   // constante de tiempo del filtro complementario
#define RPM_MAX       50.0f  // tope de la RPM deseada (deje margen bajo la RPM máxima del motor)

static Ganancias K = {80.8f, 10.9f, 3.21f, 129.4f}; // TODO: las de SU Taller 2 (unidades del firmware)
static float theta0 = 0.0f;                         // [°] ángulo de equilibrio (guía, paso B2)

// ============================================================================
// 2. HARDWARE (TODO: lo escribe cada grupo)
//    Si trabajan en src/ del repositorio, peguen aquí las secciones 1 (ENCODERS) y
//    2 (MOTORES) de control_rpm.cpp, con los struct Encoder y Motor y las variables
//    encL, encR, motL y motR. NO copien Recta, Rueda, ruedaL ni ruedaR: esos vienen
//    de lazo_rpm_nucleo.h. Vea la sección 4.4 de la guía.
// ============================================================================
// thetaAcc: ángulo de inclinación que da el ACELERÓMETRO [°] (lee la gravedad).
// omega:    velocidad de inclinación que da el GIROSCOPIO [°/s], menos el sesgo.
// Los dos sin corregir el signo: eso lo hacen SIGNO_THETA y SIGNO_OMEGA.
static bool leerIMU(float &thetaAcc, float &omega)
{
  thetaAcc = 0; // TODO: atan2(a_x, a_z) en grados, con los ejes de SU montaje
  omega = 0;    // TODO: giroscopio del eje de cabeceo [°/s] − sesgo medido al arrancar
  return false; // TODO: true cuando la lectura sea válida
}

// Vueltas de cada rueda desde que arrancó el programa, positivas al avanzar.
// En src/:  vueltasL = leerCuentas(encL) / encL.cuentasVuelta;  (igual con R)
static void leerVueltas(float &vueltasL, float &vueltasR)
{
  vueltasL = 0; // TODO
  vueltasR = 0; // TODO
}

// PWM con signo: positivo = adelante (el signo de cada motor ya corregido).
// En src/:  aplicarPWM(motL, uL);  aplicarPWM(motR, uR);
static void moverMotores(int uL, int uR)
{
  (void)uL; // TODO
  (void)uR; // TODO
}

// ============================================================================
// 3. ESTADO COMPARTIDO ENTRE LA TAREA DE CONTROL Y loop()
// ============================================================================
enum Modo { PARADO, PRUEBA_RPM, EQUILIBRIO };
static volatile Modo modo = PARADO;
static volatile float rpmPrueba = 0; // consigna fija en el modo PRUEBA_RPM

struct Telemetria
{
  uint32_t t_ms;
  float theta, omega, rpmRef, yL, yR, x;
  int uL, uR;
  uint32_t ciclo_us; // lo que tardó el ciclo
};
static Telemetria tele;
static portMUX_TYPE muxTele = portMUX_INITIALIZER_UNLOCKED;

static Equilibrio eq;

// ============================================================================
// 4. EL CICLO DE CONTROL (cada 5 ms)
// ============================================================================
static void tareaControl(void *)
{
  const float dt = TS_LAZO_US * 1e-6f;
  const int nV = VENTANA_MS * 1000 / TS_LAZO_US; // muestras de la ventana de RPM (2)
  float histL[8] = {0}, histR[8] = {0};          // vueltas de los últimos ciclos
  int k = 0;
  float theta = 0;
  TickType_t t0 = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(TS_LAZO_US / 1000));
    const uint32_t inicio = micros();

    // 1. IMU → θ [°] y θ̇ [°/s], con la convención de signos del curso
    float thetaAcc, omega;
    const bool imuOk = leerIMU(thetaAcc, omega);
    omega *= SIGNO_OMEGA;
    theta = filtroComplementario(theta, SIGNO_THETA * thetaAcc, omega, dt, TAU_FILTRO_S);
    const float th = theta - theta0;

    // 2. Encoders → vueltas y RPM medida (cuentas de los últimos 10 ms)
    float vL, vR;
    leerVueltas(vL, vR);
    histL[k % 8] = vL;
    histR[k % 8] = vR;
    float yL = 0, yR = 0;
    if (k >= nV)
    {
      yL = (vL - histL[(k - nV) % 8]) / (nV * dt) * 60.0f;
      yR = (vR - histR[(k - nV) % 8]) / (nV * dt) * 60.0f;
    }
    k++;

    // 3. Consigna de RPM según el modo (r = 0 frena y deja los integradores en cero)
    float r = 0;
    if (modo != EQUILIBRIO && eq.armado)
      desarmar(eq);
    if (modo == PRUEBA_RPM)
      r = rpmPrueba;
    else if (modo == EQUILIBRIO && imuOk && revisarSeguridad(eq, th, vL, vR))
      r = pasoEquilibrio(eq, K, th, omega, vL, vR, dt, RPM_MAX);

    // 4. Lazo de RPM de cada rueda → PWM
    const int uL = (int)lroundf(pasoLazo(ruedaL, r, yL, dt));
    const int uR = (int)lroundf(pasoLazo(ruedaR, r, yR, dt));
    moverMotores(uL, uR);

    // 5. Telemetría
    portENTER_CRITICAL(&muxTele);
    const float xTele = eq.armado ? eq.x : 0.5f * (vL + vR); // sin armar: vueltas desde el arranque
    tele = {millis(), th, omega, r, yL, yR, xTele, uL, uR, micros() - inicio};
    portEXIT_CRITICAL(&muxTele);
  }
}

// ============================================================================
// 5. COMANDOS Y TELEMETRÍA (en loop, nunca dentro del ciclo de control)
// ============================================================================
//   e         equilibrio (se enciende solo cuando |θ| < 5°)
//   p         parar
//   r 20      prueba de los lazos de RPM con el robot en el soporte: las dos ruedas a 20 RPM
//   k Kθ Kω Kv Kx   cambiar ganancias, con el robot parado.  Ej.: k 80.8 10.9 3.21 129.4
//   o 1.5     θ0 en grados, con el robot parado
static void procesar(String s)
{
  s.trim();
  if (s.length() == 0)
    return;
  float a, b, c, d;
  switch (s[0])
  {
  case 'e': modo = EQUILIBRIO; Serial.println("Equilibrio: se enciende con |θ| < 5°"); break;
  case 'p': modo = PARADO; Serial.println("Parado"); break;
  case 'r':
    if (sscanf(s.c_str() + 1, "%f", &a) == 1) { rpmPrueba = a; modo = PRUEBA_RPM; }
    break;
  case 'k':
    if (modo == PARADO && sscanf(s.c_str() + 1, "%f %f %f %f", &a, &b, &c, &d) == 4) K = {a, b, c, d};
    Serial.printf("K = %.2f %.2f %.3f %.2f\n", K.kTheta, K.kOmega, K.kV, K.kX);
    break;
  case 'o':
    if (modo == PARADO && sscanf(s.c_str() + 1, "%f", &a) == 1) theta0 = a;
    Serial.printf("θ0 = %.2f°\n", theta0);
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  // TODO: iniciar la IMU y medir el sesgo del giroscopio con el robot quieto (promedio de ~500 lecturas).
  // TODO: iniciar encoders y motores. En src/: STBY como salida, iniciarMotor(motL), iniciarMotor(motR),
  //       iniciarEncoder(encL, isrEncoderL), iniciarEncoder(encR, isrEncoderR) y habilitarDriver(true).
  disenar(); // Kp y Ki de cada rueda desde el modelo (lazo_config.h)
  xTaskCreatePinnedToCore(tareaControl, "control", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
  Serial.println("t_ms,theta,omega,rpmRef,yL,yR,uL,uR,x,ciclo_us");
}

void loop()
{
  static String linea;
  while (Serial.available())
  {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { procesar(linea); linea = ""; }
    else if (linea.length() < 64) linea += ch;
  }
  static uint32_t ultima = 0;
  if (millis() - ultima >= 20) // 50 Hz
  {
    ultima = millis();
    Telemetria t;
    portENTER_CRITICAL(&muxTele);
    t = tele;
    portEXIT_CRITICAL(&muxTele);
    Serial.printf("%lu,%.2f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.3f,%lu\n", (unsigned long)t.t_ms, t.theta, t.omega, t.rpmRef,
                  t.yL, t.yR, t.uL, t.uR, t.x, (unsigned long)t.ciclo_us);
  }
}
