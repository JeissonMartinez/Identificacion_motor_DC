/*
 * main.cpp — Selector del programa que se compila y se sube al ESP32-S3
 * Control Inteligente II (168418) · Universidad de Pamplona · Ing. Jeisson Martínez Flórez
 *
 * Programas disponibles (en esta misma carpeta src/):
 *   - identificacion.cpp : banco de identificación de los motores (Lab 1).
 *   - control_rpm.cpp    : verificación del lazo de RPM diseñado desde el modelo (Taller 2).
 *
 * Para cambiar de programa, modifique PROGRAMA abajo, compile y suba.
 *
 * Los dos usan el mismo config.h (pines, signos y cuentas por vuelta). control_rpm usa
 * además lazo_config.h.
 *
 * Nota: platformio.ini solo compila este archivo (build_src_filter); el programa elegido
 * entra aquí con #include. Así no se duplican setup(), loop() ni las variables globales.
 */

#define PROG_IDENTIFICACION  1
#define PROG_CONTROL_RPM     2

// ============================================================================
// Elija el programa
// ============================================================================
#define PROGRAMA  PROG_CONTROL_RPM
//PROG_IDENTIFICACION

#if PROGRAMA == PROG_IDENTIFICACION
  #include "identificacion.cpp"
#elif PROGRAMA == PROG_CONTROL_RPM
  #include "control_rpm.cpp"
#else
  #error "PROGRAMA debe ser PROG_IDENTIFICACION o PROG_CONTROL_RPM"
#endif
