# Identificación de motores · Control Inteligente II (168418)

Sketch de banco para identificar el modelo de cada motor del balancín:

- la **recta estática** RPM ≈ K·PWM − B;
- la **zona muerta** (PWM mínimo con el que arranca la rueda);
- la **constante de tiempo** τ.

Placa ESP32-S3 con driver TB6612FNG y encoders en cuadratura (también funciona con un encoder de un solo canal). Funciona con el núcleo Arduino-ESP32 2.x y 3.x.

## Archivos

| Archivo | Qué es | ¿Se modifica? |
|---|---|---|
| `src/config.h` | Pines, signos, cuentas por vuelta y tiempos de los ensayos | **Sí, cada grupo con los pines de su PCB** |
| `src/main.cpp` | El programa | No |
| `platformio.ini` | Placa, velocidad y configuración del monitor serie | No |
| `GUIA.md` | **Guía del laboratorio**: fundamento, procedimiento, preguntas y entrega | No |
| `notebook/ajuste_motor.ipynb` | Ajuste del modelo a partir de los CSV | Solo la celda 0 (grupo y carpeta de datos) |
| `datos/ejemplo_sintetico/` | Log de ejemplo generado con un motor simulado, para probar el notebook | No |
| `resultados/` | Aquí el notebook guarda `parametros_motor_<grupo>.json` | Lo llena el notebook |

**PlatformIO (recomendado):** descargue el repositorio (`git clone` o *Code → Download ZIP*) y abra la carpeta en VS Code. El monitor serie ya queda configurado.

**Arduino IDE:** cree una carpeta `identificacion_motor`, copie ahí `src/config.h` y `src/main.cpp`, y cambie el nombre de `main.cpp` a `identificacion_motor.ino`.

## Antes de empezar

1. Llene `config.h` con los pines de su PCB. Si un canal B del encoder no existe o está dañado, escriba `-1`.
2. Cargue el programa. Abra el monitor serie a **115200** baudios.
   - **Arduino IDE:** en la lista de fin de línea elija *Nueva línea* (no *Sin ajuste de línea*).
3. Mida la batería con el multímetro y anótela con `b`.
4. **Todos los ensayos con motores se hacen con el robot sobre un soporte y las ruedas en el aire.** Pulse **Enter** para detener los motores.

## Cómo se escriben los comandos

Escriba el comando completo y pulse **Enter**. Las letras de la tabla son huecos que se reemplazan por valores:

```
x M ini paso n      ← plantilla (así NO)
x R 10 5 14         ← comando (así SÍ): motor R, empieza en 10, sube de 5 en 5, 14 peldaños
```

`M` es el motor: `L` izquierdo, `R` derecho o `A` ambos. Los números negativos mueven el motor hacia atrás.

## Comandos

| Comando | Qué hace |
|---|---|
| `h` | Ayuda |
| `e` | Monitor de encoders. Gire las ruedas a mano y mire cuentas, nivel de A y B, flancos y errores. Enter sale |
| `z` | Cuentas en cero |
| `c N` | Cuentas por vuelta: después de `z`, gire cada rueda N vueltas a mano y escriba `c N`. Ej.: `c 10` |
| `d` | Prueba de sentido: mueve cada motor 0,8 s y dice qué signo corregir en `config.h` |
| `b V` | Anota el voltaje de la batería en cada CSV. Ej.: `b 7.9` |
| `x M ini paso n` | Escalera de PWM (2 a 22 peldaños). Ej.: `x R 10 5 14`. Hacia atrás: `x R -10 -5 14`. Solo `x R` usa 10 5 14 |
| `s M pwm [base]` | Escalón base → pwm → base. Ej.: `s R 120` (desde parado) o `s R 120 60` (girando) |

## Orden de trabajo

| Paso | Comando | Qué debe ver |
|---|---|---|
| 1. Encoders | `e`, girando cada rueda despacio a mano | A y B cambian alternados. Las cuentas suben al girar hacia adelante. Los flancos de A y B crecen parecido. Errores en 0 |
| 2. Cuentas por vuelta | `z`, marcar la rueda, girar 10 vueltas, `c 10` | Copie el valor a `CUENTAS_VUELTA_L` y `_R` |
| 3. Sentido | `d` | Corrija los signos que indique y vuelva a cargar el programa |
| 4. Recta y zona muerta | `x L 10 5 14`, `x L -10 -5 14` y lo mismo con `R` | Tabla de RPM por peldaño, recta K y B, PWM de arranque |
| 5. Dinámica | `s R 60`, `s R 120`, `s R 200`, `s R 120 60`, `s R -120`, `s R -120 -60` y lo mismo con `L` | K incremental y τ al subir y al bajar |

## Los datos

Cada ensayo imprime un bloque entre `INICIO_CSV` y `FIN_CSV`. Copie ese bloque a un archivo con este nombre:

`G<grupo>_<motor>_<ensayo>_<pwm>.csv`, por ejemplo `G3_R_escalon_120.csv`.

- La primera línea (`#`) guarda el ensayo, la batería y las cuentas por vuelta.
- Las columnas son el tiempo en µs, el PWM de cada motor y las cuentas de cada encoder.
- Para guardar todo lo que imprime el monitor en un archivo, agregue `log2file` a `monitor_filters` en `platformio.ini` (queda en la carpeta `logs/`). También sirve un terminal que guarde a archivo (CoolTerm, PuTTY).

El «resultado rápido» que imprime el sketch es una estimación. El ajuste fino del modelo se hace con estos CSV.

## Ajuste del modelo (notebook)

1. Instale las dependencias: `pip install -r notebook/requirements.txt` (o abra el notebook en Google Colab y suba la carpeta de datos).
2. Ponga sus CSV, o el log completo del monitor, en `datos/G<n>/`.
3. Abra `notebook/ajuste_motor.ipynb`, cambie `GRUPO` y `RUTA_DATOS` en la celda 0 y ejecute todo.

El notebook entrega la recta estática y la zona muerta de cada sentido, τ y el retardo de cada escalón, una validación del modelo contra las escaleras, el punto de partida del lazo de RPM y el archivo `resultados/parametros_motor_<grupo>.json`.

Los datos de `datos/ejemplo_sintetico/` salen de un motor simulado. Sirven para probar el notebook antes de tener datos propios, no son medidas de un robot real.

El procedimiento completo está en [GUIA.md](GUIA.md).

### Cambios en el notebook después del video 2 (5 de noviembre de 2026)

El video 2 muestra la versión anterior de las secciones 5, 6 y 7. Estos son los cambios:

| Qué | Antes | Ahora |
|---|---|---|
| Retardo `retardo_s` | Mediana de todas las transiciones, incluidas las que frenan a PWM 0 | Mediana de las transiciones que **no** terminan en PWM 0, igual que τ. El retardo de freno queda en `retardo_freno_s` |
| Criterio de K | El JSON guardaba `K_rpm_por_pwm` (promedio de los dos sentidos), pero el PI y la validación usaban la K de adelante | **Una recta por sentido** en la prealimentación, el PI y la validación. La recta promediada (`K_promedio`, `B_promedio_rpm`) queda en el JSON solo para comparar |
| Validación | Solo con la recta de cada sentido | Con los dos criterios, con el error RMS de cada uno |
| PI de partida | Con la K de adelante | Un Kp por sentido, y la fila promediada para comparar |
| JSON | Sin versión | `"version": 2` y `"criterio_K_firmware": "por_sentido"` |

Si su JSON no dice `"version": 2`, vuelva a ejecutar el notebook con sus mismos datos (*Run All*). El Taller 2 lee las dos versiones, pero con la nueva el retardo y el criterio de K quedan bien.

## Problemas comunes

| Síntoma | Causa | Solución |
|---|---|---|
| No pasa nada al escribir un comando | El monitor no envía fin de línea | Arduino IDE: elija *Nueva línea*. Otros terminales: active el envío de CR o LF |
| `Motor no válido` | Escribió la plantilla (`x M ...`) | Use `L`, `R` o `A` y números. Ej.: `x R 10 5 14` |
| No carga el programa | El encoder conectado a GPIO 46 (pin de arranque) | Desconéctelo para cargar y vuelva a conectarlo |
| Carga, pero el monitor no muestra nada | El cable está en el puerto USB nativo y el firmware usa el puerto UART (`ARDUINO_USB_CDC_ON_BOOT=0`) | Conecte el cable al puerto marcado UART o COM |
| El notebook dice «0 ensayos» | `RUTA_DATOS` no apunta a la carpeta, o falta la línea `# ensayo=` en el bloque copiado | Revise la ruta y copie el bloque completo |
