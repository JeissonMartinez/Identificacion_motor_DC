# Guía de laboratorio · Equilibrio con la estructura de estados

**Control Inteligente II (168418) · Ingeniería Mecatrónica · Universidad de Pamplona**\
**Ing. Jeisson Martínez Flórez · Semestre 2026-2**

---

## 1. Datos generales

| Campo | Valor |
|---|---|
| Actividad | Laboratorio · Primer equilibrio con las ganancias del modelo |
| Sesión | 11A · miércoles 11 de noviembre de 2026 · 8:00 a 10:00 |
| Modalidad | Grupal (equipo del proyecto) |
| Entran con | Las cuatro ganancias del Taller 2 y los dos lazos de RPM probados en el soporte |
| Archivos | `estructura_estados/` de este repositorio y el firmware de su grupo |
| Entrega | En el repositorio del proyecto, antes del Lab 2 (miércoles 18 de noviembre) |

---

## 2. Qué vamos a hacer hoy

Hasta ahora el robot equilibra con la cascada del Corte 1. Hoy cambiamos los dos lazos de afuera por un solo bloque, con las ganancias que sacaron del modelo en el Taller 2. El lazo de RPM de cada rueda, que es la parte amarilla del esquema, se queda igual.

![De la cascada a la estructura de estados](img/estructura/cascada_a_estados.png)

### 2.1 ¿Qué control es este y por qué cambiamos la cascada?

Es una **realimentación de estados**. El controlador mira a la vez los cuatro estados del robot ($\theta$, $\dot\theta$, $v$ y $x$) y con una sola ecuación decide la aceleración de las ruedas. Las cuatro ganancias salen por **ubicación de polos**: ustedes escogen qué tan rápido debe volver el robot (los polos) y el modelo dice qué ganancias lo logran. Eso fue lo que hicieron en el Taller 2.

| | Cascada del Corte 1 | Estructura de estados |
|---|---|---|
| Cuántos controladores | Tres lazos, uno dentro del otro | Un bloque con cuatro ganancias, más el lazo de RPM |
| De dónde salen las ganancias | Prueba y error, lazo por lazo | Del modelo, las cuatro a la vez |
| Si se mueve una ganancia | Cambia el comportamiento de los otros lazos | Se sabe dónde quedan los polos |
| Qué se sabe antes de probar | Casi nada | Si es estable, el $K_\theta$ mínimo y el efecto del retardo, en la simulación |
| Qué sale del lazo de ángulo | PWM directo: la batería y la zona muerta lo cambian todo | Una aceleración en RPM/s. El lazo de RPM se encarga de la batería y la zona muerta |

La cascada no está mal. En el fondo usa las mismas cuatro señales. La diferencia es cómo se escogen las ganancias. Además, esta estructura es la que usa el LQR (20 de noviembre) y la que la red neuronal va a imitar en el Corte 3. En el Lab 2 se comparan las dos con datos.

---

## 3. La idea, paso a paso

### 3.1 El robot mide cuatro cosas

| Símbolo | Qué es | De dónde sale | Unidad |
|---|---|---|---|
| $\theta$ | Inclinación del robot | IMU (filtro complementario) | ° |
| $\dot\theta$ | Qué tan rápido se inclina | Giroscopio, directo, sin derivar | °/s |
| $v$ | Velocidad de las ruedas | Es la RPM deseada. No se mide, la calcula el mismo lazo | RPM |
| $x$ | Cuánto se ha movido el robot | Encoders: promedio de las dos ruedas | vueltas |

Los signos son los mismos para todos. Inclinado hacia adelante es $\theta > 0$. Ruedas hacia adelante es RPM $> 0$. Robot adelante de donde arrancó es $x > 0$.

### 3.2 Con esas cuatro cosas pide una aceleración

$$
\alpha = K_\theta\,\theta + K_\omega\,\dot\theta + K_v\,v + K_x\,x
$$

$\alpha$ es la aceleración que se le pide a las ruedas, en RPM/s. Cada término se lee así, con los números del ejemplo:

| Término | Ganancia | Se lee |
|---|---|---|
| $K_\theta\,\theta$ | 80,8 RPM/s por grado | Si el robot se inclina 1°, las ruedas aceleran 80,8 RPM/s **hacia el lado de la caída** |
| $K_\omega\,\dot\theta$ | 10,9 RPM/s por °/s | Si además se está cayendo rápido, acelera más |
| $K_v\,v$ | 3,21 1/s | Si ya va rodando hacia adelante, pide un poco más hacia adelante |
| $K_x\,x$ | 129,4 RPM/s por vuelta | Si ya se adelantó, pide un poco más hacia adelante |

Las dos últimas parecen al revés. Para frenar, el robot primero tiene que inclinarse hacia atrás, y para eso las ruedas tienen que adelantarse un poco al cuerpo. Por eso **las cuatro ganancias son positivas**.

Las de ustedes están en la tabla «En las unidades del firmware» del Taller 2 (sección B.3).

### 3.3 La aceleración se vuelve RPM deseada

Cada 5 ms se suma lo que se pidió:

$$
v \leftarrow v + \alpha\,T_s, \qquad T_s = 0{,}005\ \mathrm{s}
$$

Esa $v$ es la RPM deseada, y va a las dos ruedas. Tiene un tope (por ejemplo, ±50 RPM) para dejar margen bajo la RPM máxima del motor.

Así se ve en la simulación del Taller 2. El robot se suelta inclinado 8° hacia adelante:

![El robot se recupera desde 8°](img/estructura/equilibrio_8grados.gif)

Fíjense en tres cosas:
1. Las ruedas arrancan hacia adelante, hacia el lado de la caída.
2. El robot pasa un poco al otro lado (unos −2°) y vuelve.
3. Al final regresa a donde se armó. Eso lo hace $K_x$.

---

## 4. Firmware

### 4.1 ¿Por qué cada 5 ms?

En el paso 16 de la guía de modelado sacamos el periodo de muestreo con la regla de 20 muestras dentro de la constante de tiempo del polo inestable:

$$
T_s \le \frac{1}{20}\cdot\frac{1}{p}
$$

Con el par como entrada, $p = 17{,}26\ \mathrm{rad/s}$, y daba $T_s \le 2{,}9\ \mathrm{ms}$. Ahora la entrada es $\alpha$, porque el lazo de RPM está cerrado, y el polo es más lento (sección B.1 del Taller 2):

$$
T_s \le \frac{1}{20}\cdot\frac{1}{7{,}37\ \mathrm{rad/s}} = 6{,}8\ \mathrm{ms}
$$

El lazo de RPM corre en el mismo ciclo, y también necesita varias muestras dentro de la $\tau$ del motor. Con $\tau \approx 20\ \mathrm{ms}$ y $T_s = 5\ \mathrm{ms}$ hay 4 muestras por cada $\tau$. Ir más rápido no ayuda, porque la RPM se mide con las cuentas de los últimos 10 ms, y en menos tiempo cada cuenta pesa más.

Por eso 5 ms sirve para los dos lazos.

![Las escalas de tiempo del robot](img/estructura/escalas_de_tiempo.png)

Si cambian $T_s$, también cambian $K_p$ y $K_i$ del lazo de RPM, porque $L_{total}$ incluye $T_s/2$.

### 4.2 Un ciclo de 5 ms

Cada 5 ms y siempre en este orden:

1. IMU → $\theta$ y $\dot\theta$.
2. Encoders → vueltas y RPM medida de cada rueda.
3. Lazo de equilibrio → $\alpha$ y la RPM deseada $v$.
4. Lazo de RPM de cada rueda → PWM.
5. Se guarda una copia para la telemetría.

El control corre en una **tarea** de FreeRTOS que se despierta cada 5 ms. Imprimir por el puerto serie se hace **fuera**, en `loop()`. Si se imprime adentro, el ciclo se demora y el control pierde el paso.

![Un ciclo de 5 ms](img/estructura/ciclo_5ms.gif)

### 4.3 Cómo está organizado el código

| Archivo | Qué trae | ¿Lo modifican? |
|---|---|---|
| `lazo_config.h` | El modelo de sus motores. Lo generó el notebook del Taller 2 | No |
| `estructura_estados/lazo_rpm_nucleo.h` | El lazo de RPM de una rueda, igual al del Taller 2 | No |
| `estructura_estados/estructura_estados.h` | El lazo de equilibrio: la ecuación de 3.2, el tope y la seguridad | No |
| `estructura_estados/esqueleto_equilibrio.cpp` | El ciclo de 5 ms, los comandos y la telemetría | Sí: los `TODO` y las ganancias |

**`lazo_rpm_nucleo.h`** es el lazo del Taller 2, sin hardware:

- `ruedaL` y `ruedaR` guardan el modelo de cada rueda ($K_m$, $N_0$, $\tau$, $L$).
- `disenar()` calcula $K_p$ y $K_i$ desde el modelo, una vez al arrancar.
- `pasoLazo(rueda, r, y, dt)` recibe la RPM deseada `r` y la medida `y`, y devuelve el PWM.

**`estructura_estados.h`** es el lazo de equilibrio. Su corazón son estas líneas de `pasoEquilibrio()`:

```cpp
e.x = 0.5f * ((vueltasL - e.origenL) + (vueltasR - e.origenR));                 // x [vueltas]
e.alfa = k.kTheta * theta + k.kOmega * omega + k.kV * e.rpmRef + k.kX * e.x;    // α [RPM/s]
e.rpmRef += e.alfa * dt;                                                        // v ← v + α·Ts
```

Además trae `filtroComplementario()` y `revisarSeguridad()`, que desarma el lazo con $|\theta| > 40°$ y lo vuelve a armar con $|\theta| < 5°$.

**`esqueleto_equilibrio.cpp`** tiene cinco secciones:

| Sección | Qué tiene |
|---|---|
| 1. Ajustes | `SIGNO_THETA`, `SIGNO_OMEGA`, el tope de la RPM deseada, las ganancias `K` y `theta0` |
| 2. Hardware | Las tres funciones que escriben ustedes (`TODO`) |
| 3. Estado compartido | El modo (parado, prueba de RPM, equilibrio) y la copia para la telemetría |
| 4. Ciclo de control | `tareaControl()`: los cinco pasos de 4.2 |
| 5. Comandos y telemetría | `procesar()`, `setup()` y `loop()` |

En pocas líneas, la tarea de control hace esto (versión resumida):

```cpp
leerIMU(thetaAcc, omega);                                                   // 1. IMU
theta = filtroComplementario(theta, thetaAcc, omega, dt, TAU_FILTRO_S);
leerVueltas(vL, vR);                                                        // 2. encoders
r = pasoEquilibrio(eq, K, theta - theta0, omega, vL, vR, dt, RPM_MAX);      // 3. equilibrio
moverMotores(pasoLazo(ruedaL, r, yL, dt), pasoLazo(ruedaR, r, yR, dt));     // 4. lazos de RPM
```

### 4.4 Lo que ustedes escriben (los `TODO`)

**`thetaAcc` y `omega`.** La IMU entrega dos medidas distintas del mismo movimiento:

- `thetaAcc` es el ángulo que da el **acelerómetro**. Quieto, el acelerómetro solo siente la gravedad. Si el robot se inclina, la gravedad se reparte entre los ejes $a_x$ y $a_z$ del sensor, y el ángulo sale de $\theta_{acc} = \mathrm{atan2}(a_x, a_z)$. En promedio es correcto, pero es ruidoso: cada vibración del motor y cada aceleración del robot lo mueven.
- `omega` es lo que da el **giroscopio**: qué tan rápido gira el robot, en °/s. Es limpio, pero tiene un sesgo. Quieto no da exactamente 0, y si uno lo integra, el ángulo se va corriendo.

El filtro complementario toma lo mejor de cada uno:

$$
\theta \leftarrow a\,(\theta + \omega\,T_s) + (1-a)\,\theta_{acc}, \qquad a = \frac{\tau}{\tau + T_s} = \frac{0{,}5}{0{,}505} \approx 0{,}99
$$

En cada ciclo, el 99 % sale del giroscopio y el 1 % del acelerómetro, que corrige la deriva poco a poco.

![Acelerómetro, giroscopio y filtro complementario](img/estructura/imu_theta_omega.gif)

Si su IMU es una MPU6050, `leerIMU()` puede quedar así. Ojo con los ejes: depende de cómo está montada. El eje del giroscopio es el que gira cuando el robot se inclina hacia adelante, y los del acelerómetro son los dos que quedan en el plano del movimiento.

```cpp
#include <Wire.h>
static float sesgoGy = 0;   // se mide en setup(), con el robot quieto

static bool leerIMU(float &thetaAcc, float &omega)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                                    // desde ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x68, 14) != 14)
    return false;
  int16_t d[7];                                        // ax, ay, az, temperatura, gx, gy, gz
  for (int i = 0; i < 7; i++)
    d[i] = (Wire.read() << 8) | Wire.read();
  thetaAcc = atan2f(d[0], d[2]) * RAD_TO_DEG;          // atan2(a_x, a_z) en grados
  omega = d[5] / 131.0f - sesgoGy;                     // gy en °/s (escala de ±250 °/s)
  return true;
}
```

**`leerVueltas()` y `moverMotores()`.** Con el código de `src/` son dos líneas cada una:

```cpp
vueltasL = leerCuentas(encL) / encL.cuentasVuelta;     // en leerVueltas()
vueltasR = leerCuentas(encR) / encR.cuentasVuelta;

aplicarPWM(motL, uL);                                  // en moverMotores()
aplicarPWM(motR, uR);
```

**`setup()`.** Arrancar la IMU, medir el sesgo con el robot quieto e iniciar encoders y motores:

```cpp
Wire.begin();                       // o Wire.begin(SDA, SCL) con sus pines
Wire.setClock(400000);
// despertar la MPU6050 (0x6B = 0) y poner su filtro interno en unos 44 Hz (0x1A = 3)
Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(3); Wire.endTransmission();
float suma = 0, a, w;
for (int i = 0; i < 500; i++) { leerIMU(a, w); suma += w; delay(2); }
sesgoGy = suma / 500;               // el robot NO se mueve durante este segundo

if (PIN_STBY >= 0) pinMode(PIN_STBY, OUTPUT);
iniciarMotor(motL);  iniciarMotor(motR);
iniciarEncoder(encL, isrEncoderL);  iniciarEncoder(encR, isrEncoderR);
habilitarDriver(true);
```

### 4.5 Cómo se integra con lo que ya está en `src/`

Lo más simple es agregar un **tercer programa** al proyecto del curso. Así reutilizan los pines, los signos y las cuentas por vuelta que ya probaron en el Lab 1, y el lazo de RPM del Taller 2.

![El programa nuevo dentro de src/](img/estructura/archivos_firmware.png)

| Paso | Qué hacer |
|---|---|
| 1 | Copien `estructura_estados/esqueleto_equilibrio.cpp` a `src/equilibrio.cpp`, y los dos `.h` de `estructura_estados/` a `src/` |
| 2 | En la sección 2 de `equilibrio.cpp`, peguen de `control_rpm.cpp` los `struct Encoder` y `struct Motor`, las variables `encL`, `encR`, `motL` y `motR`, y las secciones `1. ENCODERS` y `2. MOTORES`. Agreguen arriba `#include "soc/gpio_reg.h"` y `#include "soc/soc.h"` |
| 3 | **No** copien `Recta`, `Rueda`, `ruedaL` ni `ruedaR`. Esos ya vienen en `lazo_rpm_nucleo.h`, y si se repiten no compila |
| 4 | Completen los `TODO` como en 4.4 |
| 5 | En `src/main.cpp` agreguen el tercer programa (abajo) |

```cpp
#define PROG_IDENTIFICACION  1
#define PROG_CONTROL_RPM     2
#define PROG_EQUILIBRIO      3

#define PROGRAMA  PROG_EQUILIBRIO

#if PROGRAMA == PROG_IDENTIFICACION
  #include "identificacion.cpp"
#elif PROGRAMA == PROG_CONTROL_RPM
  #include "control_rpm.cpp"
#elif PROGRAMA == PROG_EQUILIBRIO
  #include "equilibrio.cpp"
#endif
```

Nosotros lo armamos así, con una MPU6050, y compila en el ESP32-S3. Si su grupo prefiere trabajar en su propio firmware, el camino es el mismo: copien `lazo_config.h` y los dos `.h`, y usen sus propias funciones de encoders y motores en los `TODO`.

---

## 5. Procedimiento

**Importante.** Si algo gira al revés, se corrige el **signo de la medida** (`SIGNO_THETA`, `SIGNO_OMEGA` o los signos del Lab 1). Las ganancias nunca se cambian de signo.

### Parte 0 · Solo si el viernes no terminaron el lazo de RPM (25 min)

Hagan los pasos B1, B2 y B4 de `GUIA_LAZO_RPM.md`. Sin el lazo de RPM probado, la parte C no va a funcionar.

### Parte A · Los dos lazos de RPM en su firmware (20 min)

Robot en el soporte, ruedas en el aire.

| Paso | Qué hacer | Qué deben ver |
|---|---|---|
| A1 | Armen `src/equilibrio.cpp` como en 4.5 | Compila |
| A2 | Completen los `TODO` como en 4.4: leer la IMU, leer las vueltas de cada rueda y mover los motores | — |
| A3 | Carguen y escriban `r 20` | Las dos ruedas a 20 RPM: columnas `yL` y `yR` cerca de 20 |
| A4 | `r -20` y luego `p` | Las dos hacia atrás. Con `p` se detienen |
| A5 | Miren la columna `ciclo_us` | Siempre menos de 2000 µs. El ciclo dura 5000 µs y necesita margen |

### Parte B · Signos y ángulo de equilibrio (15 min)

Robot en la mano.

| Paso | Qué hacer | Qué deben ver | Si no |
|---|---|---|---|
| B1 | Inclínenlo hacia adelante | `theta` positivo | `SIGNO_THETA = -1` |
| B2 | Inclínenlo hacia adelante, moviéndolo | `omega` positivo mientras se mueve | `SIGNO_OMEGA = -1` |
| B3 | Con `p`, rueden el robot hacia adelante sobre la mesa | `x` crece | Revisen los signos de los encoders del Lab 1 |
| B4 | Busquen con la mano el punto donde el robot «no se quiere caer» y lean `theta` unos segundos | Un valor estable, por ejemplo 1,5° | — |
| B5 | Escriban ese valor con `o`, por ejemplo `o 1.5` | Desde ahí, en ese punto `theta` da 0 | — |

**Prueba en el aire (B6).** Escriban `k 5 0 0 0` (solo un $K_\theta$ pequeño) y luego `e`. Sostengan el robot vertical y después inclínenlo hacia adelante:

![Prueba de signos en el aire](img/estructura/signos_en_el_aire.gif)

- Las ruedas deben girar **hacia adelante** y acelerar mientras esté inclinado.
- Al volver a la vertical siguen girando. Es normal, porque la RPM deseada es una integral. Paren con `p`.
- Repitan hacia atrás. Deben girar hacia atrás.

### Parte C · Primer equilibrio (30 min)

Robot en el piso. Una persona con las manos cerca, **sin tocarlo**. Cambien las ganancias siempre con `p` primero.

| Paso | Qué hacer | Qué deben ver |
|---|---|---|
| C1 | `k Kθ Kω Kv 0` (las suyas, con $K_x = 0$) y `e` con el robot vertical | Se sostiene solo y rueda a velocidad constante. Entre mejor esté $\theta_0$, más despacio rueda |
| C2 | `p`, `k Kθ Kω Kv Kx` (las cuatro) y `e` | Se sostiene y se queda cerca de donde se armó |
| C3 | 30 s sin tocarlo. Guarden el registro (`log2file`) | `theta` cerca de 0 y `x` cerca de 0 |
| C4 | Dos empujoncitos suaves con un dedo, con 5 s entre ellos. Guarden el registro | Se recupera y vuelve a su sitio |

Si el robot se queda quieto pero corrido de donde se armó, y `theta` no da 0, el problema es $\theta_0$:

![θ0 mal puesto](img/estructura/theta0_desplazamiento.png)

En reposo el robot no se mueve ($v = 0$, $\dot\theta = 0$, $\alpha = 0$). Entonces la ecuación de 3.2 queda $K_\theta\,\Delta\theta + K_x\,x = 0$, y el robot se corre

$$
|x| = \frac{K_\theta\,|\Delta\theta|}{K_x}
$$

Con las ganancias del ejemplo, 1° de error son unos 13 cm. Lean la inclinación media de C3, súmenla a $\theta_0$ y repitan.

### Parte D · Guardar (5 min)

Copien los registros de C3 y C4 a `datos/` y anoten las ganancias y el $\theta_0$ que usaron.

---

## 6. Si algo sale mal

| Lo que ven | Lo más probable | Qué hacer |
|---|---|---|
| Las ruedas empujan hacia el lado contrario de la caída | El signo de $\theta$ | Repitan B1 y B6 |
| Vibra rápido o zumba | Ruido en $\dot\theta$ o mucho retardo en la IMU | Bajen el filtro de la IMU o escojan polos más lentos en el Taller 2 (sección B.5) |
| Se mece despacio, cada vez más | $\theta_0$ mal puesto o $K_x$ grande para su $K_\theta$ | Corrijan $\theta_0$. Si sigue, polos más lentos |
| Se sostiene, pero se va hasta caerse, con la RPM deseada en el tope | Falta $K_x$ o el signo de $x$ está mal | Repitan B3 y C2 |
| Gira sobre su eje | Una rueda no sigue bien su RPM | Repitan A3 con esa rueda. El rumbo se corrige el 20 de noviembre |
| `ciclo_us` alto o irregular | Impresiones o esperas dentro del ciclo, o I2C lento | Saquen los `Serial.print` del ciclo. Pongan el I2C a 400 kHz |
| Se cae con un empujón fuerte, con la RPM deseada en el tope | El motor llegó a su límite | Es normal. Empujen más suave |

---

## 7. Criterios de éxito

- [ ] Las dos ruedas siguen `r 20` y `r -20` en el soporte.
- [ ] `ciclo_us` menor que 2000 µs.
- [ ] En el aire, las ruedas aceleran hacia el lado de la inclinación.
- [ ] El robot se sostiene 30 s solo, con las cuatro ganancias del modelo.
- [ ] Después de un empujoncito vuelve a menos de 5 cm de donde se armó.
- [ ] Los registros de C3 y C4 están guardados con las ganancias y el $\theta_0$ que se usaron.

---

## 8. Entrega (antes del Lab 2, miércoles 18 de noviembre)

En el repositorio del proyecto, carpeta `equilibrio/`:

| Archivo | Contenido |
|---|---|
| El firmware | Con la estructura de estados funcionando |
| `datos/` | Los registros de C3 y C4 |
| `RESULTADOS.md` | Las ganancias (unidades del firmware), $\theta_0$ y las respuestas a las tres preguntas |

Preguntas, con los números de su robot:

1. En C3, ¿cuánto se corrió el robot y qué inclinación media tuvo? ¿Cuadra con la fórmula de $\theta_0$?
2. En C4, ¿a cuánto llegó la RPM deseada? Compárenlo con la simulación del Taller 2 (sección B.4).
3. ¿Tuvieron que cambiar alguna ganancia? ¿Cuál, cuánto y por qué?

Guarden también el firmware de la cascada del Corte 1. En el Lab 2 se comparan los dos con datos.

---

## 9. Recursos

- `notebook/taller2_diseno.ipynb`, sección B: la planta con $\alpha$, las ganancias y la simulación.
- `GUIA_LAZO_RPM.md`: el lazo de RPM de cada rueda.
- Guía de modelado del balancín: las constantes A, B y C.
- `estructura_estados/`: el esqueleto y los dos archivos de cabecera.
