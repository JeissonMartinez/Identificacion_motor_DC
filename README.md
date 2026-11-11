# Identificación de motores · Control Inteligente II (168418)

Firmware de banco para identificar el modelo de cada motor del balancín y verificar el lazo de RPM diseñado con ese modelo. El programa de identificación entrega:

- la **recta estática** RPM ≈ K·PWM − B;
- la **zona muerta** (PWM mínimo con el que arranca la rueda);
- la **constante de tiempo** τ.

Placa ESP32-S3 con driver TB6612FNG y encoders en cuadratura (también funciona con un encoder de un solo canal). Funciona con el núcleo Arduino-ESP32 2.x y 3.x.

## Archivos

| Archivo | Qué es | ¿Se modifica? |
|---|---|---|
| `src/config.h` | Pines, signos, cuentas por vuelta y tiempos de los ensayos | **Sí, cada grupo con los pines de su PCB** |
| `src/main.cpp` | **Selector**: elige qué programa se compila (`PROGRAMA`) | Solo la línea `#define PROGRAMA` |
| `src/identificacion.cpp` | Programa de identificación de los motores (Lab 1) | No |
| `src/control_rpm.cpp` | Programa del lazo de RPM de cada rueda (Taller 2) | No |
| `src/lazo_config.h` | Modelo de los motores y diseño del lazo. Lo usa `control_rpm` | Lo genera el notebook del Taller 2 |
| `platformio.ini` | Placa, velocidad, monitor serie y filtro de compilación (solo compila `main.cpp`) | No |
| `GUIA.md` | **Guía del laboratorio**: fundamento, procedimiento, preguntas y entrega | No |
| `notebook/ajuste_motor.ipynb` | Ajuste del modelo a partir de los CSV | Solo la celda 0 (grupo y carpeta de datos) |
| `datos/ejemplo_sintetico/` | Log de ejemplo generado con un motor simulado, para probar el notebook | No |
| `resultados/` | Aquí el notebook guarda `parametros_motor_<grupo>.json` | Lo llena el notebook |
| `datos/profesor/` | Log del monitor con los motores del prototipo del profesor (6 de noviembre de 2026) | No |
| `lazo_rpm/` | Versión anterior del Taller 2 como proyecto de PlatformIO aparte. Ahora el mismo programa está en `src/control_rpm.cpp` | No |
| `GUIA_LAZO_RPM.md` | **Guía del laboratorio del lazo de RPM** | No |
| `GUIA_ESTRUCTURA_ESTADOS.md` | **Guía del laboratorio del 11 de noviembre**: primer equilibrio con la estructura de estados | No |
| `estructura_estados/` | Esqueleto para el firmware de cada grupo: el lazo de RPM de una rueda (`lazo_rpm_nucleo.h`), el lazo de equilibrio (`estructura_estados.h`) y el ciclo de 5 ms (`esqueleto_equilibrio.cpp`) | Solo los `TODO` del esqueleto y las ganancias |
| `notebook/taller2_diseno.ipynb` | Taller 2: revisión del JSON, PI por λ, modelo en lazo cerrado frente a la medida, planta con α, ubicación de polos y simulación | Solo la celda 0 |
| `datos/ejemplo_sintetico_lazo/` | Log de ejemplo del programa `control_rpm`, generado con un motor simulado | No |

**PlatformIO (recomendado):** descargue el repositorio (`git clone` o *Code → Download ZIP*) y abra la carpeta en VS Code. El monitor serie ya queda configurado.

**Arduino IDE:** cree una carpeta con el nombre del programa, por ejemplo `identificacion`, copie ahí `src/config.h` y `src/identificacion.cpp`, y cambie el nombre de `identificacion.cpp` a `identificacion.ino`. Para el lazo de RPM: carpeta `control_rpm` con `config.h`, `lazo_config.h` y `control_rpm.cpp` renombrado a `control_rpm.ino`.

## Elegir el programa

Los dos programas están en `src/` y comparten `config.h`. En `src/main.cpp` cambie esta línea, compile y cargue:

```cpp
#define PROGRAMA  PROG_IDENTIFICACION   // identificación de los motores (Lab 1)
#define PROGRAMA  PROG_CONTROL_RPM      // lazo de RPM (Taller 2)
```

(Deje solo una de las dos.) `platformio.ini` compila solo `main.cpp` (`build_src_filter = -<*> +<main.cpp>`), y `main.cpp` incluye el programa elegido. Así no se repiten `setup()`, `loop()` ni las variables globales. Si agrega otro `.cpp` a `src/`, inclúyalo desde `main.cpp` o agréguelo al filtro.

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
- `platformio.ini` ya trae el filtro `log2file`: todo lo que imprime el monitor queda guardado en la carpeta `logs/` (git no la sube). Copie ese log a `datos/G<n>/` para el notebook. Con el Arduino IDE sirve un terminal que guarde a archivo (CoolTerm, PuTTY).

El «resultado rápido» que imprime el sketch es una estimación. El ajuste fino del modelo se hace con estos CSV.

## Ajuste del modelo (notebook)

1. Instale las dependencias: `pip install -r notebook/requirements.txt` (o abra el notebook en Google Colab y suba la carpeta de datos).
2. Ponga sus CSV, o el log completo del monitor, en `datos/G<n>/`.
3. Abra `notebook/ajuste_motor.ipynb`, cambie `GRUPO` y `RUTA_DATOS` en la celda 0 y ejecute todo.

El notebook entrega la recta estática y la zona muerta de cada sentido, τ y el retardo de cada escalón, una validación del modelo contra las escaleras, el punto de partida del lazo de RPM y el archivo `resultados/parametros_motor_<grupo>.json`.

Los datos de `datos/ejemplo_sintetico/` salen de un motor simulado. Sirven para probar el notebook antes de tener datos propios, no son medidas de un robot real. Los de `datos/profesor/` sí son medidas: los motores del prototipo del profesor, con su resultado en `resultados/parametros_motor_Profesor.json`.

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

## Lazo de RPM (Taller 2)

El programa `src/control_rpm.cpp` cierra un lazo de RPM en cada rueda, cada 5 ms, con el modelo del Lab 1. Usa una prealimentación con la inversa de la recta, un PI por cancelación de polo (Ti = τ, Kp = τ/(K_m·(λ + L_total))) e integración condicional. El procedimiento está en [GUIA_LAZO_RPM.md](GUIA_LAZO_RPM.md).

1. Use el mismo `src/config.h` del Lab 1 (pines, signos y cuentas por vuelta). No hay que copiarlo: los dos programas lo comparten.
2. Genere `src/lazo_config.h` con la sección A.2 de `notebook/taller2_diseno.ipynb`. El que viene en el repositorio tiene los valores del ejemplo sintético.
3. En `src/main.cpp` ponga `#define PROGRAMA  PROG_CONTROL_RPM`, cargue y abra el monitor.

| Comando | Qué hace |
|---|---|
| `r M rpm [base]` | Escalón de consigna base → rpm → base. Ej.: `r A 30 10`. Atrás: `r A -30 -10`. Cruce por cero: `r A -10 10` |
| `l ms` | λ del diseño. Recalcula Kp y Ki. Ej.: `l 30` |
| `k s` / `k p` | Criterio de K: por sentido (el del firmware) o promediado (para comparar) |
| `f 1` / `f 0` | Prealimentación encendida o apagada |
| `m ms` | Modelo de referencia: filtra la consigna con esa τ. `m 0` lo apaga |
| `b V` | Anota la batería en cada CSV |
| `v` | Muestra el modelo y el diseño vigente (Kp y Ki por rueda y sentido) |
| `h` | Ayuda |

Cada ensayo imprime un bloque CSV con las columnas `t_us, rL, rR, uL, uR, cL, cR`: tiempo, consigna, PWM y cuentas de cada rueda. La línea `#` guarda el diseño con que se corrió.

## Estructura de estados (11 de noviembre)

El lazo de equilibrio pide una aceleración de rueda con los cuatro estados y la integra en la RPM deseada de las dos ruedas:

$$
\alpha = K_\theta\,\theta + K_\omega\,\dot\theta + K_v\,v + K_x\,x, \qquad v \leftarrow v + \alpha\,T_s
$$

La carpeta `estructura_estados/` no se compila con este proyecto. Es un esqueleto para el firmware de cada grupo: se copian los dos `.h` y el `lazo_config.h` del Taller 2, y se completan los `TODO` del esqueleto con la IMU, los encoders y los motores de cada robot. En este repositorio se agrega como tercer programa: `src/equilibrio.cpp` y `PROGRAMA = PROG_EQUILIBRIO` en `main.cpp` (sección 4.5 de la guía). El procedimiento está en [GUIA_ESTRUCTURA_ESTADOS.md](GUIA_ESTRUCTURA_ESTADOS.md).

| Comando del esqueleto | Qué hace |
|---|---|
| `r 20` | Prueba de los dos lazos de RPM en el soporte: las dos ruedas a 20 RPM |
| `e` | Equilibrio. Se arma solo con \|θ\| < 5° y se apaga con \|θ\| > 40° |
| `p` | Parar |
| `k Kθ Kω Kv Kx` | Ganancias en unidades del firmware, con el robot parado. Ej.: `k 80.8 10.9 3.21 129.4` |
| `o 1.5` | Ángulo de equilibrio θ0 en grados, con el robot parado |

La telemetría sale a 50 Hz con las columnas `t_ms, theta, omega, rpmRef, yL, yR, uL, uR, x, ciclo_us`.

## Problemas comunes

| Síntoma | Causa | Solución |
|---|---|---|
| No pasa nada al escribir un comando | El monitor no envía fin de línea | Arduino IDE: elija *Nueva línea*. Otros terminales: active el envío de CR o LF |
| `Motor no válido` | Escribió la plantilla (`x M ...`) | Use `L`, `R` o `A` y números. Ej.: `x R 10 5 14` |
| No carga el programa | El encoder conectado a GPIO 46 (pin de arranque) | Desconéctelo para cargar y vuelva a conectarlo |
| Carga, pero el monitor no muestra nada | El cable está en el puerto USB nativo y el firmware usa el puerto UART (`ARDUINO_USB_CDC_ON_BOOT=0`) | Conecte el cable al puerto marcado UART o COM |
| El notebook dice «0 ensayos» | `RUTA_DATOS` no apunta a la carpeta, o falta la línea `# ensayo=` en el bloque copiado | Revise la ruta y copie el bloque completo |
