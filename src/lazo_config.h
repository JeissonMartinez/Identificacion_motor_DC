/*
 * lazo_config.h — Modelo de los motores y diseño del lazo de RPM
 * Control Inteligente II (168418) · Universidad de Pamplona · Ing. Jeisson Martínez Flórez
 *
 * Este archivo lo genera la parte A del notebook del Taller 2 (notebook/taller2_diseno.ipynb)
 * a partir de resultados/parametros_motor_G<n>.json. También se puede llenar a mano.
 *
 * LOS VALORES DE ABAJO SON LOS DEL EJEMPLO SINTÉTICO. Reemplácelos por los de su grupo.
 *
 * Nombres (para no confundirlos con la constante B del balancín ni con el vector K):
 *   RPM ≈ K_m·PWM − N0, una recta por sentido.
 *   K_m = "K_adelante" / "K_atras" del JSON. N0 = "B_adelante_rpm" / "B_atras_rpm" del JSON.
 */
#pragma once

// ============================================================================
// 1. MODELO DE CADA MOTOR (del JSON)
// ============================================================================
// Rueda izquierda (L)
#define KM_L_ADELANTE   0.23992f  // [RPM/PWM]
#define N0_L_ADELANTE   2.05699f  // [RPM]
#define ZM_L_ADELANTE   20        // [PWM] zona muerta (solo informativa)
#define KM_L_ATRAS      0.23898f
#define N0_L_ATRAS      1.98980f
#define ZM_L_ATRAS      25
#define TAU_L_S         0.02117f  // [s] constante de tiempo (sin freno)
#define RETARDO_L_S     0.00269f  // [s] retardo (sin freno)

// Rueda derecha (R)
#define KM_R_ADELANTE   0.24849f
#define N0_R_ADELANTE   1.70988f
#define ZM_R_ADELANTE   20
#define KM_R_ATRAS      0.25154f
#define N0_R_ATRAS      1.85452f
#define ZM_R_ATRAS      20
#define TAU_R_S         0.01960f
#define RETARDO_R_S     0.00230f

// ============================================================================
// 2. DISEÑO DEL LAZO (valores iniciales; se cambian en vivo con comandos)
// ============================================================================
// PI por cancelación de polo: Ti = τ, Kp = τ / (K_m·(λ + L_total)), Ki = Kp / τ
// L_total = retardo + Ts/2 + ventana/2
#define LAMBDA_MS        50     // constante de tiempo deseada en lazo cerrado (comando "l")
#define CRITERIO_K        0     // 0 = por sentido, 1 = promediada (comando "k")
#define PREALIMENTACION   1     // 1 = con prealimentación, 0 = solo PI (comando "f")
#define REF_TAU_MS        0     // modelo de referencia: τ del filtro de la consigna, 0 = apagado (comando "m")

#define TS_LAZO_US       5000   // periodo del lazo: 5 ms
#define VENTANA_MS         10   // ventana de medición de la RPM (múltiplo de Ts)
#define R_SUAVE_RPM      1.0f   // el término N0 crece desde 0 hasta esta |consigna| (sin golpes en 0)
#define I_MAX_PWM       255.0f  // tope del integrador [PWM]
#define REF_MAX_RPM      60.0f  // consigna máxima permitida [RPM]

// ============================================================================
// 3. ENSAYO DE ESCALÓN DE CONSIGNA ("r")
// ============================================================================
#define LAZO_ANTES_MS     500   // consigna base
#define LAZO_DURANTE_MS  1000   // consigna del escalón
#define LAZO_DESPUES_MS   700   // vuelve a la base
