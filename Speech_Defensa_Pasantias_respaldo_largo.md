# Guion de Defensa — Informe de Pasantía

**Título:** Diseño e implementación de un sistema IoT con Microcontroladores para el Monitoreo Remoto de Equipos Fiscales

**Autora:** Lorena Del Valle Rojas Chams El Dein — C.I. 19-10469 — Ingeniería Electrónica, Universidad Simón Bolívar
**Institución:** The Factory HKA
**Tutor académico:** Prof. Miguel Díaz
**Tutores industriales:** Ing. Pablo Arguinzones y Ing. Luis Muñoz
**Duración:** 35 minutos (rango objetivo: 30–40 min) + ronda de preguntas

> **Versión alineada con el libro corregido (v2).** Este guion ya incorpora las correcciones del profesor y la terminología del informe final: el enlace local es una interfaz **síncrona SPI a 100 kHz** (no confundir con el RS-232/UART asíncrono de la etapa inicial); el conflicto de concurrencia se denomina **acceso concurrente al bus compartido (*hardware lock*)**, arbitrado con un **mutex de software de FreeRTOS**; y el **Bloque 3 está reenfocado a resultados**, no a los problemas encontrados durante el desarrollo.

---

## Cómo usar este guion

- El texto en **negrita entre corchetes** son acotaciones para la ponente: `[PAUSA]`, `[SEÑALAR]`, `[MOSTRAR]`. **No se leen en voz alta.**
- El discurso está redactado en **primera persona del plural (nosotros)**, que es el tono que se usará para toda la exposición técnica. Se reserva la primera persona del singular para la presentación personal inicial, los agradecimientos y la reflexión final.
- Los tiempos por bloque son acumulativos y estimados a un ritmo de 130–145 palabras por minuto. Ensaya con cronómetro y ajusta: si te sobra tiempo, expande el Bloque 3 (es el que más margen tiene); si te falta, recorta la diapositiva 9 (justificación tecnológica).
- Las frases de transición están marcadas al final de cada bloque: son clave para que el jurado sienta continuidad y no saltos.

---

## Mapa de diapositivas (24 en total)

| # | Diapositiva | Bloque |
|---|---|---|
| 1 | Portada: título, autora, tutores, institución | 1 |
| 2 | Contexto: The Factory HKA y el parque de impresoras fiscales | 1 |
| 3 | Planteamiento del problema | 1 |
| 4 | Objetivos: general y específicos | 1 |
| 5 | Arquitectura IIoT de tres niveles (Edge / Gateway / Cloud) | 2 |
| 6 | Nivel Edge: captura del *Status S1* por interfaz síncrona SPI | 2 |
| 7 | Nivel Gateway: ESP32, ESP-IDF y kernel concurrente (FreeRTOS) | 2 |
| 8 | Nivel Cloud: MQTT v5.0 / TLS 1.2 y Django + HTMX | 2 |
| 9 | Justificación tecnológica: Arduino vs. ESP-IDF · HTTP vs. MQTT | 2 |
| 10 | Metodología de validación: banco Hardware-in-the-Loop (HIL) | 3 |
| 11 | **Resultado 1** — Captura íntegra y transparente del *Status S1* | 3 |
| 12 | **Resultado 2** — Integridad de la telemetría de extremo a extremo | 3 |
| 13 | **Resultado 3** — Robustez e inmunidad del bus local | 3 |
| 14 | **Resultado 4** — Eficiencia de red: ahorro del 80,2 % | 3 |
| 15 | **Resultado 5** — Resiliencia: cero pérdida de datos | 3 |
| 16 | **Resultado 6** — Latencia End-to-End (T_E2E) | 3 |
| 17 | **Resultado 7** — Actualización remota OTA de doble objetivo | 3 |
| 18 | **Resultado 8** — Rendimiento global: memoria, energía y trazabilidad | 3 |
| 19 | **Resultado 9** — Comparativa: estado previo vs. estado actual | 3 |
| 20 | Verificación de cumplimiento de objetivos | 4 |
| 21 | Aporte y beneficio para la organización | 4 |
| 22 | Limitaciones encontradas y trabajo futuro | 4 |
| 23 | Escalabilidad y recomendaciones | 4 |
| 24 | Cierre, agradecimientos y paso a preguntas | 5 |

---

# BLOQUE 1 — Introducción y Marco del Problema
**Tiempo estimado: 5 min (acumulado 0:00 – 5:00) · Diapositivas 1 a 4**

## Diapositiva 1 — Portada (0:00 – 0:30)

[Diapositiva en pantalla. Habla pausado, mirando al jurado.]

Primero que nada, buenos días. Nos gustaría comenzar extendiéndoles un cordial saludo a todos los presentes y presentándonos: mi nombre es Lorena Rojas. El día de hoy estamos aquí para presentarles la defensa de mi Proyecto de Pasantía, titulado *Diseño e implementación de un sistema IoT con Microcontroladores para el Monitoreo Remoto de Equipos Fiscales*.

Este trabajo fue desarrollado en la empresa The Factory HKA, bajo la tutoría académica del profesor Miguel Díaz y la tutoría industrial del ingeniero Pablo Arguinzones, a quienes agradezco desde ya su acompañamiento durante todo el proceso. [PAUSA BREVE]

## Diapositiva 2 — Contexto (0:30 – 2:00)

[SEÑALAR la imagen del parque de impresoras o el logo de HKA.]

The Factory HKA es una empresa líder en el sector de equipos fiscales en Venezuela. Su actividad gira en torno a las impresoras fiscales: los dispositivos que, por normativa, registran y certifican cada transacción comercial de un contribuyente.

Históricamente, la fiscalización en nuestro país ha dependido de equipos físicos que operan bajo protocolos de comunicación punto a punto, predominantemente el estándar serial RS-232. Estos equipos han evolucionado en capacidad de almacenamiento de memorias fiscales, pero su arquitectura de comunicación se ha mantenido aislada de las redes corporativas.

En la práctica, esto significa que la única forma de interactuar con una impresora fiscal para diagnóstico o auditoría es la conexión física de un técnico, con una herramienta de depuración serial local, equipo por equipo. Por eso la empresa ha iniciado una transición hacia la digitalización de estos procesos, buscando centralizar información que hoy reside únicamente en el hardware físico de cada contribuyente. Ese es el contexto en el que se enmarca la pasantía.

## Diapositiva 3 — Planteamiento del problema (2:00 – 3:30)

[PAUSA. Mira al jurado.]

El laboratorio de desarrollo y la unidad de soporte de The Factory HKA enfrentan limitaciones operativas concretas. El monitoreo de los estados de la impresora —por ejemplo, el registro *Status S1*— depende de la presencia física del técnico o de conexiones seriales punto a punto.

Esto genera tres consecuencias directas:

Primero, dificulta la detección temprana de errores críticos, como *memoria fiscal llena* o fallos mecánicos, que pueden interrumpir la actividad comercial de un contribuyente de forma imprevista.

Segundo, la visualización actual de los datos fiscales es fragmentada: los técnicos solo pueden observar el estado de un equipo a la vez, lo que impide tener una visión global y simultánea del parque de máquinas instaladas.

Y tercero, no existe un historial de auditoría consultable. La información se observa en el momento, pero no queda trazada de forma sistemática.

[PAUSA] En síntesis, el problema es este: **equipos fiscales con datos valiosos, pero aislados, sin telemetría centralizada y sin trazabilidad histórica.**

## Diapositiva 4 — Objetivos (3:30 – 5:00)

[SEÑALAR el objetivo general.]

Frente a este problema, el objetivo general del trabajo fue: **diseñar, implementar y validar un sistema basado en microcontroladores ESP32 que permita el monitoreo remoto y en tiempo real de los datos operativos de equipos fiscales, garantizando la confiabilidad y la robustez del sistema de comunicación.**

[SEÑALAR los objetivos específicos. Léelos de forma agrupada, sin detenerte uno por uno.]

Para lograrlo se plantearon ocho objetivos específicos, que se pueden agrupar en tres frentes:

- **Diseño e integración:** definir la arquitectura del sistema IoT y desarrollar el firmware embebido para los microcontroladores ESP32, integrándolos con las interfaces del equipo fiscal para capturar los datos relevantes.
- **Comunicación y visualización:** implementar la mensajería mediante MQTT con un servidor externo, desplegar la capa de recepción y distribución de los datos, y construir un dashboard web que muestre en tiempo real el estado de los equipos.
- **Validación y documentación:** evaluar la confiabilidad de la comunicación, la latencia, la frecuencia de actualización y la robustez ante fallos de red o de energía, documentando todo el proceso con sus diagramas, scripts de prueba y resultados.

[TRANSICIÓN al Bloque 2] Con el problema y los objetivos claros, pasamos a mostrarles cómo diseñamos la solución.

---

# BLOQUE 2 — Arquitectura y Diseño de la Solución
**Tiempo estimado: 7 min (acumulado 5:00 – 12:00) · Diapositivas 5 a 9**

## Diapositiva 5 — Arquitectura IIoT de tres niveles (5:00 – 6:45)

[SEÑALAR el diagrama de arquitectura. Recórrelo de arriba hacia abajo con el puntero.]

La solución se fundamenta en una arquitectura distribuida de tres niveles, alineada con los estándares industriales del Internet Industrial de las Cosas. La denominamos arquitectura *MT-IoT*: Módulo de Transmisión IoT.

[SEÑALAR el nivel superior.]

En el **Nivel de Borde (Edge)** está la impresora fiscal HKA80. Allí se origina la información crítica: los registros de estado *Status S1* y el diagnóstico *SV2*. La captura se hace a través de una interfaz de comunicación síncrona dedicada, diseñada para operar de forma transparente, sin interferir con la máquina de estados legal del periférico.

[SEÑALAR el nivel intermedio.]

En el **Nivel de Pasarela (Gateway)** está el Módulo de Transmisión IoT, basado en el SoC ESP32. Su función es doble: actúa como maestro en la interfaz local con el periférico, y como cliente seguro hacia la red Wi-Fi. Aquí vive la lógica de empaquetado en JSON y la gestión de seguridad TLS/SSL.

[SEÑALAR el nivel inferior.]

En el **Nivel de Gestión y Visualización (Cloud)** están el *broker* HiveMQ y el servidor de aplicaciones desarrollado en Django. Este nivel gestiona la recepción asíncrona de la telemetría y su visualización en tiempo real en el dashboard, mediante un túnel de MQTT sobre WebSockets.

[SEÑALAR las flechas.]

El flujo de datos es este: desde la impresora hacia el ESP32 por el bus local; del ESP32 hacia la nube por Wi-Fi con TLS 1.2; de la nube al servidor Django por WebSockets Seguros; y del servidor a la base de datos y al panel reactivo.

## Diapositiva 6 — Nivel Edge: captura del *Status S1* (6:45 – 8:15)

[SEÑALAR el detalle del bus síncrono.]

El nivel de borde fue el más delicado de todo el proyecto, porque es donde se toca la información fiscal directamente.

Conviene aclarar una distinción importante. La impresora HKA80 se comunica con parámetros obligatorios del protocolo —9600 baudios, 8 bits de datos, paridad par y 1 bit de parada— sobre el estándar asíncrono RS-232. Ese fue el punto de partida. Sin embargo, el enlace definitivo de la solución **no es asíncrono, sino síncrono**: migramos la interconexión física local hacia una **interfaz serie síncrona dedicada operando a una frecuencia de reloj de 100 kHz**.

[SEÑALAR el diagrama maestro-esclavo.]

La diferencia es de fondo. En el estándar asíncrono no existe una señal de reloj compartida: el receptor debe sincronizarse por sí mismo, y eso lo hace vulnerable a la deriva de temporización. En la interfaz síncrona, en cambio, el ESP32 genera una señal de reloj dedicada como maestro, y el periférico muestrea los datos a ese compás. Con esto eliminamos de raíz las desviaciones por deriva de reloj —el *baud-rate drift*— y garantizamos una transmisión determinista.

[SEÑALAR el detalle de la trama.]

Para blindar la integridad de las tramas tributarias implementamos una rutina de validación por redundancia longitudinal, LRC: un lazo cerrado de operaciones XOR acumulativas sobre los bytes recibidos, que se contrasta contra el *checksum* que envía el periférico. Si la trama es válida, el módulo responde con un *ACK*; si no, responde con un *NAK* y reinicia lógicamente los registros del bus, impidiendo que datos corruptos se propaguen.

## Diapositiva 7 — Nivel Gateway: ESP32, ESP-IDF y kernel concurrente (8:15 – 10:00)

[SEÑALAR el diagrama de los dos núcleos.]

Para el nodo de transmisión seleccionamos la tarjeta ESP32 DevKitC V4, un microcontrolador de doble núcleo. La razón es la concurrencia: mientras un núcleo gestiona el sondeo de la impresora cada 5 segundos, el segundo núcleo administra la pila de red y la mensajería MQTT. Así, las latencias de internet no bloquean la captura local de datos.

[SEÑALAR el diagrama de exclusión mutua.]

Ahora bien, el corazón del diseño es el **kernel concurrente**. La coexistencia de la tarea de adquisición fiscal y la recepción de comandos inalámbricos introdujo dos conflictos que resolvimos con exclusión mutua, es decir, con semáforos de software gestionados por el propio sistema operativo de tiempo real:

- El primero es la **inconsistencia de datos por concurrencia**: la tarea de monitoreo actualizaba los acumuladores de ventas en los búferes mientras la tarea MQTT leía esos mismos campos para empaquetar el JSON, produciendo telemetría híbrida o incompleta. Lo resolvimos con un *mutex* de datos, `mqtt_data_mutex`, que protege la estructura compartida.
- El segundo es el **acceso concurrente al bus compartido**, un *hardware lock*: si una actualización OTA llegaba justo en el instante en que el sondeo iniciaba, ambas rutinas intentaban acceder simultáneamente al controlador del bus. Al no haber exclusión mutua, el sistema lanzaba un fallo de aserción y reiniciaba la placa. Lo resolvimos con un segundo *mutex*, `bus_local_mutex`, que ambas tareas deben adquirir antes de energizar el canal físico.

Además, para proteger el *heap* ante ráfagas de comandos, desacoplamos la lógica OTA en una tarea supervisora y una cola segura de comandos.

## Diapositiva 8 — Nivel Cloud: MQTT v5.0 / TLS y Django + HTMX (10:00 – 11:15)

[SEÑALAR el esquema de tópicos MQTT.]

En la capa de red adoptamos el estándar **MQTT v5.0 sobre TLS 1.2**, hacia la infraestructura de HiveMQ Cloud. Se optó por MQTT por su mínima sobrecarga de cabecera y su modelo de publicación-suscripción, y específicamente por la versión 5.0 por dos capacidades que no existían en versiones anteriores:

- Las ***User Properties***: metadatos clave-valor en la cabecera del protocolo —modelo, versión de firmware, serial y RSSI— que el servidor lee sin necesidad de deserializar el cuerpo del mensaje.
- El ***Last Will and Testament***, o testamento: una "última voluntad" que el dispositivo registra al conectar; si pierde la conexión abruptamente, el *broker* la publica automáticamente y el sistema sabe que el equipo se cayó.

La seguridad se estructuró con dos decisiones: enrutamiento forzado al puerto 8883, `secure-mqtt`, con transporte sobre SSL; y autenticación mediante un *bundle* de certificados optimizado de Espressif, que valida la cadena de confianza de HiveMQ sin quemar archivos PEM extensos en la Flash.

[SEÑALAR el diagrama del servidor.]

En la nube construimos un **ecosistema web con Django y HTMX**. La pieza clave es un *worker* MQTT independiente, `listen_mqtt.py`, que corre en segundo plano con un lazo no bloqueante. No lo acoplamos al servidor web, porque eso bloquearía el manejo de solicitudes HTTP ante ráfagas de datos. El *worker* compara cada trama contra el último estado persistido y descompone la diferencia en eventos atómicos: cambios de estado, cortes de reporte Z, emisión de facturas, reconexiones y actualizaciones de firmware.

Para la interfaz, en lugar de montar un servidor de WebSockets intermedio con Django Channels, integramos el cliente MQTT de JavaScript directamente en el navegador. El *broker* HiveMQ expone un canal de WebSockets Seguros en el puerto 8884 —distinto del 8883 que usan el nodo de borde y el *worker*—, de modo que el navegador se conecta directamente al *broker* sin intermediarios. Eso simplifica el *firewall* y elimina un eslabón pesado. Y con HTMX logramos una experiencia de aplicación de página única, intercambiando solo fragmentos del DOM, sin recargar la página.

## Diapositiva 9 — Justificación tecnológica (11:15 – 12:00)

[SEÑALAR la tabla comparativa. Sé breve, es una diapositiva de justificación.]

Permítannos justificar brevemente las dos decisiones tecnológicas más importantes.

**Arduino frente a ESP-IDF.** Aunque Arduino permite un despliegue ágil, su modelo de ejecución es un bucle secuencial único, mono-hilo. En un entorno transaccional fiscal, un `delay()` bloqueante en la pila Wi-Fi arriesgaba sobrepasar la ventana temporal de lectura del firmware legal del equipo. Por eso optamos por el framework nativo **ESP-IDF v5.5.2**, que nos dio gestión explícita de memoria en C y asignación determinista de tareas con FreeRTOS.

**HTTP frente a MQTT.** Una arquitectura HTTP REST tradicional tiene una sobrecarga de cabecera enorme para un mensaje que cambia cada 5 segundos. MQTT traslada los metadatos a la cabecera del protocolo y aligera el cuerpo. Más adelante les mostraremos la medición cuantitativa de ese ahorro.

[TRANSICIÓN al Bloque 3] Y con esto llegamos al bloque central de la presentación: la validación y los resultados experimentales.

---

# BLOQUE 3 — Validación y Resultados Experimentales
**Tiempo estimado: 16 min (acumulado 12:00 – 28:00) · Diapositivas 10 a 19**

> **Nota de enfoque:** este bloque presenta **qué se obtuvo y cómo se comprobó**, no los problemas que aparecieron en el camino. Esos hallazgos ya están documentados en el informe; aquí solo se mencionan cuando son indispensables para entender un resultado, y siempre desde la perspectiva del resultado alcanzado.

## Diapositiva 10 — Metodología de validación: banco HIL (12:00 – 13:30)

[SEÑALAR la foto del banco de pruebas.]

Antes de mostrar los resultados, explicamos brevemente **cómo se validó el sistema**, porque aquí hay una decisión metodológica importante.

La validación de un sistema fiscal **no admite ensayos destructivos sobre equipos en producción**: inducir cortes de energía, corromper tramas o abortar una reescritura de firmware sobre una impresora real comprometería registros tributarios irreversibles.

Para conciliar rigor experimental y seguridad, construimos un entorno de pruebas **Hardware-in-the-Loop, HIL**, con dos niveles de emulación:

- El **primer nivel** es un emulador en software, `impresora.py`, desarrollado con la biblioteca `pyserial`, con los mismos parámetros eléctricos del enlace fiscal. Implementa una máquina de estados de recepción, calcula el código LRC y responde con las tramas de estado y los reportes S1 y SV2, incluyendo los acuses *ACK* y *NAK*. Esto nos permitió depurar el firmware sin depender de la disponibilidad del equipo físico.
- El **segundo nivel** eleva la fidelidad: programamos un ESP32 configurado como esclavo que emula íntegramente al periférico fiscal, sobre el mismo bus y con los mismos pines. Replica la trama de estado, los reportes S1 y SV2 y, de forma crítica, el protocolo completo de reescritura remota.

Contra ese periférico simulado validamos de extremo a extremo el puente OTA, con un binario de prueba de 2500 bytes. Este enfoque nos habilitó la **inyección determinista de fallos** —paquetes fuera de secuencia, *checksums* alterados, interrupciones del enlace— y confirmó la capacidad de recuperación antes de comprometer la memoria de un equipo fiscal real.

[PAUSA] Todos los resultados que verán a continuación fueron medidos sobre este banco de pruebas.

## Diapositiva 11 — Resultado 1: captura íntegra del *Status S1* (13:30 – 15:00)

[SEÑALAR la captura de la consola serial.]

El primer resultado es la **captura íntegra y transparente del registro *Status S1***.

[SEÑALAR los datos resaltados en la captura.]

Como pueden ver en la traza, el módulo aísla de manera transparente los registros de la Memoria de Trabajo del equipo fiscal: captura exacta del RIF del contribuyente, el número de registro de la máquina y los acumuladores de ventas diarias. Y confirma el estado mecánico y lógico: modo fiscal y en espera, con el estado `0x60` y sin errores reportados por el cabezal.

Tres características hacen que este resultado sea relevante:

- **Transparencia.** La interrogación se ejecuta desde una tarea dedicada de FreeRTOS, `printer_task_main`, desacoplada de la tarea de red. Así, la adquisición nunca compite con la pila inalámbrica, y el sondeo se sostiene cada 5 segundos **sin alterar la máquina de estados del periférico ni interrumpir las transacciones en curso**.
- **Procesamiento predecible.** El análisis de la trama se hace íntegramente sobre memoria estática: dos búferes fijos, sin reservas dinámicas en la ruta crítica. Eso mantiene un tiempo de proceso acotado y constante, y evita la fragmentación de memoria.
- **Integridad.** Los 116 bytes útiles de la trama se delimitan, se separan en sus campos y se entregan listos para publicar.

## Diapositiva 12 — Resultado 2: integridad de la telemetría (15:00 – 16:30)

[SEÑALAR el ejemplo del campo de versión y el JSON publicado.]

El segundo resultado es que **la telemetría que llega a la nube es siempre válida y deserializable**.

Lo comprobamos extremo a extremo. Por el lado del borde, el campo de versión del periférico se entrega íntegro —`020507GD00`— y cada mensaje se publica como un JSON bien formado, con todos sus campos completos. Por el lado del receptor, el *worker* de Django sanea los caracteres de control del mensaje crudo antes de invocar el analizador JSON, de modo que ningún residuo llega a la base de datos.

[PAUSA] El resultado neto es que **el canal de telemetría entrega datos fiscalmente legibles y estructuralmente válidos**, condición indispensable para que el dashboard y la bitácora de auditoría operen sobre información confiable.

## Diapositiva 13 — Resultado 3: robustez e inmunidad del bus local (16:30 – 18:00)

[SEÑALAR la tabla de los tres niveles.]

El tercer resultado es la **robustez del bus local**. Como es el eslabón físicamente más expuesto del sistema, su tolerancia a perturbaciones se evaluó como requisito de primer orden.

El resultado se obtuvo con una **defensa escalonada en tres niveles**:

- **Nivel 1, defensa temporal:** toda operación de entrada y salida se ejecuta bajo ventanas de tiempo estrictas. La espera de *ACK* durante la reescritura se acota a 300 milisegundos; la recepción de respuestas de estado, a 2000; y la solicitud del reporte S1, a 3000. Al agotarse cualquiera de estas ventanas, la transacción se aborta de forma segura y se incrementa el contador de errores, con una regla de máximo 3 reintentos.
- **Nivel 2, sanitización de búfer:** antes de cada transacción se limpian de forma determinista los búferes de transmisión y recepción, para que los residuos de una operación previa no se interpreten como datos válidos. En paralelo, la validación LRC descarta cualquier trama cuya asimetría revele la alteración de un bit, respondiendo con *NAK*.
- **Nivel 3, reconstrucción estructural del bus:** cuando el conteo de errores supera el umbral, el firmware no se limita a reintentar: **desmonta y reancla por completo el controlador SPI**, mediante `spi_bus_free`, `spi_bus_initialize` y `spi_bus_add_device`.

[SEÑALAR el resultado final de la tabla.]

El resultado medido: ante la desconexión física del periférico o la inyección de tramas corruptas, el sistema **se recuperó automáticamente** una vez restablecidas las condiciones del bus, **sin reiniciar el microcontrolador y sin que una sola trama inválida alcanzara la base de datos**.

## Diapositiva 14 — Resultado 4: eficiencia de red, ahorro del 80,2 % (18:00 – 19:30)

[SEÑALAR la tabla comparativa. Esta es una de las cifras fuertes: dila con calma.]

El cuarto resultado es la **eficiencia de red**. Medimos la sobrecarga de cabecera y la masa de datos transmitida por cada ciclo de telemetría, y lo comparamos contra una arquitectura HTTP tradicional.

[SEÑALAR fila por fila.]

- **Cabecera de red:** una petición HTTP POST tradicional ronda los 450 bytes; MQTT v5.0 con *User Properties*, unos 12 bytes. Eso es una reducción del **97,33 %**.
- **Cuerpo del mensaje:** 328 bytes en JSON plano frente a 142 bytes en el empaquetado optimizado en C. Un **56,70 %** menos.
- **Consumo total por trama:** de **778 bytes en HTTP a 154 bytes en MQTT**. Hablamos de una reducción global del **80,2 %** del volumen de red.

[PAUSA. Deja que la cifra cala.] Para un parque de impresoras que reporta cada 5 segundos durante todo el día, ese 80 % se traduce directamente en un ahorro del consumo de datos del contribuyente.

## Diapositiva 15 — Resultado 5: resiliencia, cero pérdida de datos (19:30 – 21:15)

[SEÑALAR la captura de *Log Capture* y el diagrama de flujo.]

El quinto resultado es la **resiliencia**: el sistema tolera fallas de energía y de red **sin pérdida de datos**.

Cuando el enlace se pierde, al expirar el *Keep-Alive*, el *broker* HiveMQ publica la "última voluntad" que el dispositivo había registrado. Como ven en la captura, el *worker* asíncrono interceptó la alerta `offline` y persistió transaccionalmente una nueva fila con el evento clasificado como "Desconexión Abrupta (LWT)". Ese mecanismo **congeló el último estado válido** de los acumuladores sin alterar los registros históricos previos.

[SEÑALAR el diagrama de flujo.]

El ciclo completo es este: pérdida del enlace Wi-Fi, reintentos de reconexión con *backoff* exponencial, publicación del testamento por el *broker*, registro del evento por el *worker* y restauración del túnel MQTT sin pérdida de estado.

Esto se apoya en una propiedad de diseño: el módulo es un **observador de estado sin memoria tributaria propia**. Toda la información fiscal reside en el periférico y se re-consulta en cada sondeo. El *gateway* no es una fuente de verdad que pueda corromperse, sino un reflejo periódico del equipo.

[SEÑALAR la tabla de escenarios.]

En los **cuatro escenarios de falla** que evaluamos —corte abrupto de energía, micro-corte Wi-Fi, caída del *broker* y desconexión del periférico— **la pérdida de datos fue nula**. Tras un corte de energía, el primer sondeo reconstruye la telemetría completa; el único efecto observable es un hueco de una ventana de sondeo, 5 segundos. Y como el servicio es de al menos una entrega, QoS 1, y la telemetría es un estado idempotente, la repetición eventual no genera efectos adversos.

## Diapositiva 16 — Resultado 6: latencia End-to-End (21:15 – 22:45)

[SEÑALAR la tabla de latencias.]

El sexto resultado es el **desempeño temporal**. Definimos la latencia de extremo a extremo, T_E2E, como la suma de cinco etapas lógicas serializadas, desde la interrogación física del periférico hasta la actualización del DOM en el navegador.

Mediante un perfilado sobre **500 iteraciones continuas** de envío de telemetría, obtuvimos estas latencias promedio:

[SEÑALAR cada fila.]

- Sondeo síncrono del S1: **118 milisegundos**, con una desviación de 4. Dentro de la ventana fiscal.
- Empaquetado JSON: **2 milisegundos**.
- *Handshake* TLS 1.2: **45 milisegundos**.
- Publicación y tránsito por el *broker* MQTT: **85 milisegundos**, la etapa más variable.
- Renderizado del DOM con HTMX y WebSockets: **18 milisegundos**.

La **latencia total promedio es de 268 milisegundos, con una desviación estándar de 15,3**.

[SEÑALAR la cifra de porcentaje.]

Ese valor representa apenas un **5,36 % del intervalo total de sondeo**, que es de 5 segundos. Es decir, el sistema opera en tiempo real estricto sin acumular retrasos en la pila de recepción.

Un análisis complementario de varianza mostró que la etapa de red concentra por sí sola el **86,8 % de la dispersión total**. Esto identifica al enlace del proveedor de servicios de Internet como la única fuente estocástica dominante: la incertidumbre no proviene del diseño embebido, sino de un factor externo y acotado. Como referencia, este margen contrasta favorablemente con antecedentes nacionales de monitoreo IoT desarrollados en la misma Universidad Simón Bolívar, donde la regularidad del período de muestreo presentaba desviaciones de hasta un segundo.

## Diapositiva 17 — Resultado 7: OTA de doble objetivo (22:45 – 24:30)

[SEÑALAR el diagrama de flujo de la OTA.]

El séptimo resultado es la **actualización remota de firmware**, que fue además el aporte más disruptivo del trabajo. Diseñamos un protocolo OTA de **doble objetivo**, inédito en la arquitectura de la empresa.

[SEÑALAR la rama izquierda.]

El primer objetivo es el **gateway** ESP32. Al seleccionar `ismart` en la interfaz web, el módulo recibe la orden por MQTT y activa la bandera de suspensión `ota_en_progreso`, que detiene el sondeo fiscal. La autoactualización usa la API `esp_https_ota` y escribe en la **partición OTA inactiva** de un esquema dual: la Flash de 4 MB se divide en dos regiones idénticas. Si ocurre un corte de energía, el sistema descarta el sector corrupto y arranca desde la partición intacta.

[SEÑALAR la rama derecha.]

El segundo objetivo es el más complejo: **reescribir in situ el firmware del periférico fiscal**, que no tiene pila de red propia. El módulo asume el rol de *host* de programación intermedio. Como el binario supera ampliamente la capacidad del *heap* libre del ESP32, la descarga no se almacena completa, sino que se procesa a través de un **búfer circular de 1024 bytes**.

Cada fragmento se empaqueta con una cabecera de 4 bytes —identificador, índice secuencial y longitud en 16 bits— más un código de redundancia XOR, y se admite hasta 3 reintentos con confirmación *ACK*. La máquina de estados tiene tres fases: apertura, transmisión segmentada y cierre. Y el cierre **solo se emite cuando los bytes confirmados igualan la longitud declarada por el servidor**; por construcción, una transferencia parcial nunca se compromete en la memoria del periférico.

Para la resiliencia ante cortes de red implementamos **reanudación con cabeceras HTTP Range**: si el enlace titubea, el firmware no reinicia desde el byte cero, sino que reabre la conexión indicando el desplazamiento exacto para reanudar en el fragmento interrumpido.

[SEÑALAR la captura del mensaje de éxito.]

Como ven en la interfaz, la confirmación llega de forma asíncrona: "Actualización Completada: el equipo actualizó *printer* con éxito". **Ambos objetivos quedaron validados de extremo a extremo.**

## Diapositiva 18 — Resultado 8: rendimiento global (24:30 – 26:30)

[SEÑALAR la tabla de perfilado de memoria.]

El octavo resultado es el **rendimiento global del sistema**. Empecemos por la memoria, que era el recurso más restringido.

Tras la reingeniería de recursos mediante la utilidad `menuconfig` —desactivando la aceleración por IRAM de la pila Wi-Fi—, redujimos la ocupación de la memoria **IRAM del 78,39 % al 52,26 %**: un ahorro directo de 34.256 bytes, un **26,13 % menos**. Ese margen fue justamente el que hizo viable reservar el *heap* necesario para las operaciones de actualización.

[SEÑALAR las dos capturas comparadas.]

Estas son las capturas de perfilado, antes y después. El margen libre resultante fue de 62.577 bytes en IRAM —casi la mitad de esa memoria— y 143.312 bytes en DRAM.

[SEÑALAR la tabla del subclocking.]

En el plano energético, adoptamos un **subclocking a 80 MHz** frente a los 240 MHz nominales. Como la potencia dinámica de un circuito CMOS es proporcional a la frecuencia, reducirla en un factor de 3 contrae la potencia dinámica a **un tercio**; y como el salto térmico es proporcional a la potencia disipada, el sistema opera **más frío** y con menor degradación por ciclado térmico. Lo que habilita esta reducción es el **acelerador criptográfico AES por hardware**: el cifrado no se ejecuta en el núcleo, sino en un bloque dedicado. Bajo esa condición, los 80 MHz sostienen el *handshake* TLS en 45 milisegundos, dentro de la ventana de tolerancia de 100.

[SEÑALAR la tabla de SQLite.]

Finalmente, en la nube, la trazabilidad también dio un resultado medible. Con un histórico simulado de **10.000 registros**, la búsqueda indexada por serial se mantiene **por debajo de los 12 milisegundos**. La rutina de depuración automática del *worker* purga los registros mayores a 90 días sin bloquear las consultas, y la bitácora se organiza en eventos atómicos vinculados al dispositivo por su serial.

## Diapositiva 19 — Resultado 9: comparativa estado previo vs. estado actual (26:30 – 28:00)

[SEÑALAR la tabla "antes y después". Esta diapositiva cierra el bloque de resultados.]

Para sintetizar el bloque de resultados, comparemos el estado previo con el estado actual.

[SEÑALAR columna izquierda.]

**Estado previo:** el técnico debía conectar físicamente una PC al puerto serial de la impresora. Solo podía observar **un equipo a la vez**. No había visión global del parque. El diagnóstico era reactivo, posterior al fallo. No existía historial de auditoría consultable. Y la actualización de firmware requería intervención física en sitio.

[SEÑALAR columna derecha.]

**Estado actual:** el monitoreo es remoto y centralizado en un dashboard que muestra **múltiples equipos en simultáneo**. La detección de errores es temprana y automática, con alertas ante desconexiones. Existe una **bitácora de auditoría de 90 días** consultable por serial. El mantenimiento puede planificarse con datos reales, es decir, preventivo y no reactivo. Y la actualización de firmware se comanda **desde la nube**, tanto del *gateway* como del periférico fiscal.

[PAUSA] En una frase: pasamos de un equipo aislado y observado a mano, a un parque conectado, trazable y administrable de forma centralizada, con una reducción del 80,2 % en el consumo de datos y una latencia de extremo a extremo de 268 milisegundos.

[TRANSICIÓN al Bloque 4] Con esto concluimos los resultados. Pasamos ahora al impacto, las conclusiones y la escalabilidad.

---

# BLOQUE 4 — Impacto, Conclusiones y Escalabilidad
**Tiempo estimado: 5 min (acumulado 28:00 – 33:00) · Diapositivas 20 a 23**

## Diapositiva 20 — Verificación de cumplimiento de objetivos (28:00 – 29:15)

[SEÑALAR la matriz de objetivos del informe final.]

Cerramos verificando el cumplimiento de los objetivos. En el informe incluimos una **matriz de cumplimiento** que contrasta cada objetivo específico con su estado y su entregable técnico.

Los **ocho objetivos del plan se alcanzaron**, con un matiz que conviene declarar con transparencia:

- **Diseño de la arquitectura IoT:** logrado. Arquitectura de tres niveles desplegada y documentada.
- **Desarrollo del firmware embebido:** logrado. Firmware en C sobre ESP-IDF y FreeRTOS, con la reingeniería de memoria y el subclocking.
- **Integración con las interfaces del equipo fiscal:** logrado. Enlace síncrono SPI a 100 kHz y analizador de las tramas *Status* S1 y SV2.
- **Comunicación mediante MQTT:** logrado. Cliente MQTT v5.0 sobre TLS 1.2, con *bundle* de certificados, *Last Will and Testament* y *User Properties*.
- **Servidor MQTT (recepción, almacenamiento y distribución):** **parcialmente logrado**. En lugar de desplegar un *broker* propio, se delegó el servicio al *broker* gestionado HiveMQ Cloud; la capa de recepción, almacenamiento y distribución sí se implementó.
- **Dashboard en tiempo real:** logrado. Interfaz reactiva con Django y HTMX, con monitor en vivo y captura de logs.
- **Validación del sistema:** logrado. Latencia, ancho de banda, robustez y estabilidad cuantificadas sobre banco HIL.
- **Documentación:** logrado. Diagramas, flujos de datos, scripts de prueba y resultados.

[PAUSA] Adicionalmente, **ampliamos el alcance respecto al plan original** con un objetivo no previsto: el **módulo de actualización remota OTA de doble objetivo**, que resultó ser el componente más valioso del trabajo.

## Diapositiva 21 — Aporte y beneficio para la organización (29:15 – 30:45)

[SEÑALAR la infografía de beneficios.]

¿Qué significa esto para The Factory HKA? El aporte se puede resumir en cuatro beneficios directos.

Primero, **visibilidad centralizada del parque**. Lo que antes exigía recorrer físicamente cada equipo hoy se observa en un solo panel, con múltiples impresoras en simultáneo.

Segundo, **mantenimiento preventivo basado en datos**. La bitácora de 90 días permite analizar patrones de fallo y anticipar intervenciones, en lugar de reaccionar tras la avería. Eso reduce tiempos de inactividad en los puntos de venta de los contribuyentes.

Tercero, **ahorro operativo en red**. La adopción de MQTT v5.0 redujo el consumo de datos por trama en un 80,2 % frente a HTTP, lo que se traduce en menores costos de conectividad a escala de flota.

Y cuarto, **mantenimiento de firmware sin intervención en sitio**. La actualización remota de doble objetivo permite corregir o mejorar el software tanto del *gateway* como de la impresora desde la nube, algo especialmente valioso para una empresa que administra equipos dispersos geográficamente.

[PAUSA] En conjunto, el sistema convierte un parque de equipos aislados en una infraestructura de telemetría gestionable de forma centralizada.

## Diapositiva 22 — Limitaciones y trabajo futuro (30:45 – 32:00)

[SEÑALAR la lista. Aquí conviene ser claros: el jurado valora la autocrítica.]

Toda solución tiene límites, y queremos ser claros con los de este trabajo.

- **Validación en laboratorio:** la solución se validó sobre un banco Hardware-in-the-Loop. Es un prototipo funcional y documentado, pero **aún no está desplegado en campo** sobre un parque real. El siguiente paso es una prueba piloto con un conjunto reducido de contribuyentes.
- **Escala de la nube:** SQLite es adecuado para un piloto, pero para un parque distribuido a nivel nacional conviene migrar a un motor relacional con mayor concurrencia, como PostgreSQL.
- **Seguridad y cumplimiento:** la información fiscal viaja cifrada de extremo a extremo, pero antes de un despliegue masivo debe revisarse el cumplimiento con la normativa tributaria vigente sobre custodia y tratamiento de datos fiscales.
- **Dependencia de la red del contribuyente:** el sistema es resiliente, pero su desempeño depende de la calidad del enlace del cliente; el 86,8 % de la dispersión proviene de ese factor externo.

Y como **trabajo futuro** proponemos: extender la telemetría a los reportes Z y a los cierres de auditoría; incorporar analítica predictiva sobre el histórico de errores, para anticipar fallos mecánicos antes de que ocurran; y añadir aprovisionamiento masivo de dispositivos, para dar de alta un parque completo sin configuración manual.

## Diapositiva 23 — Escalabilidad y recomendaciones (32:00 – 33:00)

[SEÑALAR el diagrama de escalamiento.]

Sobre la escalabilidad, queremos destacar tres propiedades del diseño que la habilitan.

Primero, la **arquitectura de tres niveles es intrínsecamente escalable**: el nivel de borde no conoce al servidor más que por un tópico MQTT, de modo que añadir un dispositivo es añadir una suscripción.

Segundo, la **naturaleza sin estado tributario del *gateway*** facilita el despliegue masivo: no hay base de datos local que sincronizar ni estado que reconciliar; cada dispositivo es independiente y reemplazable.

Tercero, el **aprovisionamiento por tópicos jerárquicos** —con rutas por serial— permite que la plataforma administre flotas completas organizadas por grupo, sucursal y modelo, que es exactamente cómo está modelado el backend.

[PAUSA] En conjunto, la solución deja establecidas las condiciones técnicas para que The Factory HKA centralice la telemetría de un parque distribuido a nivel nacional y transforme la gestión reactiva del mantenimiento en una práctica basada en datos.

[TRANSICIÓN al Bloque 5] Con esto cerramos la presentación.

---

# BLOQUE 5 — Cierre y Agradecimientos
**Tiempo estimado: 2 min (acumulado 33:00 – 35:00) · Diapositiva 24**

## Diapositiva 24 — Cierre, agradecimientos y preguntas (33:00 – 35:00)

[Diapositiva final con el título y la frase de cierre. Baja el ritmo, cierra con seguridad.]

Para concluir: este trabajo resolvió una necesidad operativa concreta del parque de impresoras fiscales venezolano —la supervisión remota y el diagnóstico determinista de equipos que hasta ahora solo podían inspeccionarse mediante una conexión serial local— integrando en una sola arquitectura el **monitoreo**, la **trazabilidad** y la **actualización remota**.

Los resultados confirman un monitoreo de alta disponibilidad, resiliencia ante cortes de red y una reducción del 80,2 % en el consumo de datos. Y dejan sentadas las bases para un despliegue en campo.

[PAUSA]

Más allá del resultado técnico, esta pasantía me dejó una lección que considero el aprendizaje central: el valor de un enfoque de ingeniería sustentado en el **diagnóstico cuantitativo**. Perfilar la memoria, caracterizar las latencias y probar la tolerancia a fallos fue el método —medir antes de decidir— que permitió convertir un prototipo frágil en un sistema determinista y robusto.

[PAUSA. Mira al jurado.]

Quiero agradecer a la Universidad Simón Bolívar y a The Factory HKA por hacer posible este trabajo; al profesor Miguel Díaz por su tutoría académica; a los ingenieros Pablo Arguinzones y Luis Muñoz por su acompañamiento industrial; y a todas las personas que me apoyaron durante estos meses.

Con esto damos por concluida la presentación y quedamos atentos a sus preguntas. Muchas gracias.

[PAUSA. Espera. No agregues nada más.]

---

# Anexo — Preparación para la ronda de preguntas

Ten estas respuestas listas. Se dan en 3 a 5 frases cada una. Se responden en primera persona del plural, salvo cuando la pregunta es personal.

**1. ¿Por qué SPI y no I2C para el enlace local?**
Ambos son síncronos, pero SPI ofrece mayor velocidad y un esquema maestro-esclavo *full-duplex* con señal de reloj dedicada, que es lo que el periférico requiere. I2C comparte un direccionamiento y una tasa menor que no aportaban ventaja aquí. Además, el ESP32 integra controlador SPI nativo, lo que simplificó la capa de abstracción de hardware.

**2. Aclaración: ¿el RS-232 no es síncrono?**
Correcto, y es un punto importante. El RS-232/UART es un estándar **asíncrono**: no comparte señal de reloj, y el receptor se sincroniza por sí mismo. Ese fue el punto de partida del prototipo. El enlace definitivo de la solución es una interfaz **síncrona SPI**, donde el ESP32 genera el reloj a 100 kHz como maestro. La diferencia elimina la deriva de temporización.

**3. ¿Por qué MQTT y no HTTP o WebSockets puros?**
MQTT traslada los metadatos a la cabecera del protocolo y permite un cuerpo mínimo; la medición dio una reducción del 80,2 % del volumen por trama frente a HTTP POST. Además, su modelo publicación-suscripción desacopla el emisor del receptor, lo que permite que múltiples consumidores —el dashboard y el *worker* de ingesta— reciban la misma telemetría sin que el dispositivo los conozca.

**4. ¿Cómo garantizan que no se corrompe la información fiscal?**
Por tres vías. Primero, el módulo es un observador sin memoria tributaria propia: nunca escribe en la memoria fiscal, solo lee el *Status S1* y el *SV2*. Segundo, toda trama se valida con LRC antes de aceptarse. Tercero, el acceso al bus físico y a la memoria compartida está arbitrado con *mutex* de software, lo que elimina las condiciones de carrera que probamos en laboratorio.

**5. Aclaración: ¿el mutex es de hardware o de software?**
De software. Es un semáforo *mutex* gestionado por FreeRTOS, el sistema operativo de tiempo real del ESP32. Lo que protege es un recurso de hardware —el controlador del bus y una estructura de datos compartida—, pero el mecanismo de exclusión mutua en sí es lógico. Esa precisión es importante: hablamos de arbitrar el acceso a un recurso compartido, no de una propiedad eléctrica.

**6. ¿Qué pasa si se corta la energía a mitad de una actualización OTA?**
Depende del objetivo. Si es el *gateway*, la escritura ocurre sobre la partición OTA inactiva; el *bootloader* detecta el sector corrupto y arranca desde la partición intacta. Si es el periférico fiscal, el cierre de la transacción solo se emite cuando los bytes confirmados igualan la longitud declarada; una transferencia parcial nunca se compromete, y una nueva orden reinicia el proceso desde cero.

**7. ¿Por qué SQLite y no PostgreSQL?**
SQLite es suficiente para un piloto y para la escala de las pruebas: con 10.000 registros, la búsqueda indexada se mantiene por debajo de 12 milisegundos. Reconocemos que para un parque nacional conviene migrar a PostgreSQL por concurrencia; lo dejamos planteado como trabajo futuro.

**8. ¿Por qué ESP32 y no una Raspberry Pi o un PLC?**
El ESP32 ofrece el equilibrio adecuado para este caso: doble núcleo con FreeRTOS para concurrencia determinista, aceleración criptográfica AES por hardware, bajo consumo y bajo costo, en un dispositivo embebido sin sistema operativo de propósito general. Una Raspberry Pi añadiría un sistema operativo completo, mayor consumo y mayor superficie de ataque, sin aportar nada que el sistema requiera.

**9. ¿Es seguro enviar datos fiscales a la nube?**
El transporte usa TLS 1.2 de extremo a extremo, con validación de la cadena de confianza mediante *bundle* de certificados, lo que mitiga ataques de intermediario. El *payload* se formatea con `snprintf` para prevenir desbordamientos. Dicho esto, antes de un despliegue masivo debe revisarse el cumplimiento normativo sobre custodia de datos fiscales; lo señalamos como limitación.

**10. ¿Cómo comprobaron que el problema del bus era por interferencia electromagnética?**
Lo evidenciamos por sus síntomas: al operar sobre el enlace asíncrono aparecían errores de enmarcado y fallos en la verificación LRC, que desaparecieron al migrar a la interfaz síncrona con reloj dedicado. No lo caracterizamos con osciloscopio, y ese es un punto que reconocemos como mejora: una campaña de medición con instrumentación permitiría cuantificar la inmunidad del enlace.

**11. ¿Cómo escala a miles de dispositivos?**
La arquitectura de tres niveles escala por diseño: añadir un dispositivo es añadir una suscripción a un tópico. El *broker* gestiona las sesiones, y el *worker* de ingesta se desacopla del servidor web. Los cuellos de botella previsibles serían la base de datos y el aprovisionamiento, que atacamos con la retención de 90 días y que dejamos planteados como trabajo futuro.

**12. ¿La latencia de 268 ms es aceptable?**
Sí, y con holgura: representa el 5,36 % del intervalo de sondeo de 5 segundos. Además, el 86,8 % de la dispersión proviene del enlace del proveedor de Internet, no del diseño embebido, lo que significa que en un enlace estable el sistema sería aún más regular.

**13. ¿Por qué el objetivo del servidor MQTT quedó parcialmente logrado?**
Porque el plan contemplaba desplegar un *broker* propio, y en la arquitectura final se delegó esa función al *broker* gestionado HiveMQ Cloud, que ofrece disponibilidad y seguridad sin administración local. La capa que sí implementamos fue la de recepción, almacenamiento y distribución de la telemetría en el servidor Django. Preferimos declararlo como parcialmente logrado en lugar de forzar la lectura del objetivo.

**14. ¿Cuál fue el aporte más significativo?**
El puente síncrono heterogéneo de reprogramación inalámbrica. Es la capacidad más disruptiva porque traslada el mantenimiento de firmware a la nube y reduce drásticamente la intervención física en campo, algo crítico para una empresa que administra equipos dispersos geográficamente.

---

# Consejos de exposición

- **Ensaya en voz alta con cronómetro.** El Bloque 3 es el que más se te puede estirar; si te pasas de 40 minutos, recorta las diapositivas 9 (justificación tecnológica) y 18 (rendimiento global), que son las más prescindibles.
- **No leas las diapositivas.** Las diapositivas llevan cifras y diagramas; tu voz debe contar la historia que las conecta.
- **Ancla las cifras fuertes** (80,2 %; 78,39 % → 52,26 %; 268 ms; cero pérdida de datos) diciéndolas despacio y haciendo una pausa después. Son tus mejores argumentos.
- **Mantén el plural de forma consistente** en la parte técnica. Si en algún momento se te escapa un "yo", no lo corrijas en voz alta: sigue adelante.
- **Cuando señales un diagrama, recórrelo con el puntero** y nombra cada nivel antes de explicar su función.
- **Reserva 30 segundos de margen** entre el final del Bloque 5 y la primera pregunta; no termines y salgas corriendo.
- **Ten a mano una copia impresa** del informe con la matriz de objetivos y las tablas de resultados, por si el jurado pide ver un dato específico durante las preguntas.
