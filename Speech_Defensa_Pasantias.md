# Guion de Defensa — Informe de Pasantía

**Título:** Diseño e implementación de un sistema IoT con Microcontroladores para el Monitoreo Remoto de Equipos Fiscales

**Autora:** Lorena Del Valle Rojas Chams El Dein — C.I. 19-10469 — Ingeniería Electrónica, Universidad Simón Bolívar
**Institución:** The Factory HKA
**Tutor académico:** Prof. Miguel Díaz
**Tutores industriales:** Ing. Pablo Arguinzones y Ing. Luis Muñoz
**Duración:** 35 minutos (rango objetivo: 30–40 min) + ronda de preguntas

---

## Cómo usar este guion

- El texto en **negrita entre corchetes** son acotaciones para la ponente: `[PAUSA]`, `[SEÑALAR]`, `[MOSTRAR]`. **No se leen en voz alta.**
- El texto normal es el discurso sugerido, redactado para leerse en primera persona.
- Los tiempos por bloque son acumulativos y estimados a un ritmo de 130–145 palabras por minuto. Ensaya con cronómetro y ajusta: si te sobra tiempo, expande el Bloque 3 (es el que más margen tiene); si te falta, recorta los ejemplos de la diapositiva de decisiones tecnológicas.
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
| 6 | Nivel Edge: captura del *Status S1* por interfaz síncrona | 2 |
| 7 | Nivel Gateway: ESP32, ESP-IDF y kernel concurrente (FreeRTOS + Mutex) | 2 |
| 8 | Nivel Cloud: MQTT v5.0 / TLS 1.2 y Django + HTMX | 2 |
| 9 | Justificación tecnológica: Arduino vs. ESP-IDF · HTTP vs. MQTT | 2 |
| 10 | Entorno de pruebas Hardware-in-the-Loop (HIL) | 3 |
| 11 | Extracción transparente del *Status S1* (sin interrumpir el ciclo fiscal) | 3 |
| 12 | Sanitización del *payload* y corrección del error *Off-By-One* | 3 |
| 13 | Inmunidad electromagnética: defensa escalonada en tres niveles | 3 |
| 14 | Consumo de ancho de banda: HTTP vs. MQTT v5.0 | 3 |
| 15 | Resiliencia de red: LWT y recuperación ante micro-cortes | 3 |
| 16 | Matriz de latencias End-to-End (T_E2E) | 3 |
| 17 | Actualización remota OTA de doble objetivo | 3 |
| 18 | Perfilado de memoria IRAM y DRAM | 3 |
| 19 | Perfilado térmico-energético (80 MHz) y trazabilidad en SQLite | 3 |
| 20 | Verificación de cumplimiento de objetivos | 4 |
| 21 | Comparativa: estado previo vs. estado actual | 4 |
| 22 | Limitaciones encontradas y trabajo futuro | 4 |
| 23 | Escalabilidad y recomendaciones | 4 |
| 24 | Cierre, agradecimientos y paso a preguntas | 5 |

---

# BLOQUE 1 — Introducción y Marco del Problema
**Tiempo estimado: 5 min (acumulado 0:00 – 5:00) · Diapositivas 1 a 4**

## Diapositiva 1 — Portada (0:00 – 0:30)

[Diapositiva en pantalla. Habla pausado, mirando al jurado.]

Primero que nada buenos días, me gustaria comenzar con extenderles un cordial saludo a todos los presentes e introduciendome: Mi nombre es Lorena Rojas. Y el día de hoy estamos aquí presentes ante ustedes ver la defensa de mi Proyecto de Pasantía titulado *Diseño e implementación de un sistema IoT con Microcontroladores para el Monitoreo Remoto de Equipos Fiscales*.

Este trabajo fue desarrollado en la empresa The Factory HKA, bajo la tutoría académica del profesor Miguel Díaz y la tutoría industrial del ingeniero Pablo Arguinzones, a quienes agradezco desde ya su acompañamiento durante todo el proceso. [PAUSA BREVE]

## Diapositiva 2 — Contexto (0:30 – 2:00)

[SEÑALAR la imagen del parque de impresoras o el logo de HKA.]

The Factory HKA es una empresa líder en el sector de equipos fiscales en Venezuela. Su actividad gira en torno a las impresoras fiscales: los dispositivos que, por normativa, registran y certifican cada transacción comercial de un contribuyente.

Históricamente, la fiscalización en nuestro país ha dependido de equipos físicos que operan bajo protocolos de comunicación punto a punto, predominantemente el estándar serial RS-232. Estos equipos han evolucionado en capacidad de almacenamiento de memorias fiscales, pero su arquitectura de comunicación se ha mantenido aislada de las redes corporativas.

En la práctica, esto significa que la única forma de interactuar con una impresora fiscal para diagnóstico o auditoría es la conexión física de un técnico, con una herramienta de depuración serial local, equipo por equipo. Por eso la empresa ha iniciado una transición hacia la digitalización de estos procesos, buscando centralizar información que hoy reside únicamente en el hardware físico de cada contribuyente. Ese es el contexto en el que se enmarca mi pasantía.

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

Para lograrlo, planteé ocho objetivos específicos, que se pueden agrupar en tres frentes:

- **Diseño e integración:** definir la arquitectura del sistema IoT y desarrollar el firmware embebido para los microcontroladores ESP32, integrándolos con las interfaces del equipo fiscal para capturar los datos relevantes.
- **Comunicación y visualización:** implementar la mensajería mediante MQTT con un servidor externo, desplegar ese servidor para recibir, almacenar y distribuir los datos, y construir un dashboard web que muestre en tiempo real el estado de los equipos.
- **Validación y documentación:** evaluar la confiabilidad de la comunicación, la latencia, la frecuencia de actualización y la robustez ante fallos de red o de energía, documentando todo el proceso con sus diagramas, scripts de prueba y resultados.

[TRANSICIÓN al Bloque 2] Con el problema y los objetivos claros, paso a mostrarles cómo diseñé la solución.

---

# BLOQUE 2 — Arquitectura y Diseño de la Solución
**Tiempo estimado: 7 min (acumulado 5:00 – 12:00) · Diapositivas 5 a 9**

## Diapositiva 5 — Arquitectura IIoT de tres niveles (5:00 – 6:45)

[SEÑALAR el diagrama de arquitectura. Recórrelo de arriba hacia abajo con el puntero.]

La solución se fundamenta en una arquitectura distribuida de tres niveles, alineada con los estándares industriales del Internet Industrial de las Cosas. La llamo arquitectura *MT-IoT*: Módulo de Transmisión IoT.

[SEÑALAR el nivel superior.]

En el **Nivel de Borde (Edge)** está la impresora fiscal HKA80. Allí se origina la información crítica: los registros de estado *Status S1* y el diagnóstico *SV2*. La captura se hace a través de una interfaz de comunicación síncrona, diseñada para operar de forma transparente, sin interferir con la máquina de estados legal del periférico.

[SEÑALAR el nivel intermedio.]

En el **Nivel de Pasarela (Gateway)** está el Módulo de Transmisión IoT, basado en el SoC ESP32. Su función es doble: actúa como maestro en la interfaz local con el periférico, y como cliente seguro hacia la red Wi-Fi. Aquí vive la lógica de empaquetado en JSON y la gestión de seguridad TLS/SSL.

[SEÑALAR el nivel inferior.]

En el **Nivel de Gestión y Visualización (Cloud)** están el *broker* HiveMQ y el servidor de aplicaciones desarrollado en Django. Este nivel orquesta la recepción asíncrona de la telemetría y su visualización en tiempo real en el dashboard, mediante un túnel de MQTT sobre WebSockets.

[SEÑALAR las flechas.]

El flujo de datos es este: desde la impresora hacia el ESP32 por el bus local; del ESP32 hacia la nube por Wi-Fi con TLS 1.2; de la nube al servidor Django por WebSockets Seguros; y del servidor a la base de datos y al panel reactivo.

## Diapositiva 6 — Nivel Edge: captura del *Status S1* (6:45 – 8:15)

[SEÑALAR el detalle del bus síncrono.]

El nivel de borde fue, quizás, el más delicado de todo el proyecto, porque es donde se toca la información fiscal directamente.

La impresora HKA80 se comunica con parámetros obligatorios del protocolo: 9600 baudios, 8 bits de datos, paridad par y 1 bit de parada. En la fase inicial usé el transceptor asíncrono UART, pero encontré dos problemas serios: vulnerabilidad al ruido electromagnético y condiciones de carrera.

Por eso migré la interconexión física local hacia una **interfaz serie síncrona dedicada, operando a 100 kHz**. A diferencia de la UART, este método basa el muestreo de datos en una señal de reloj dedicada, generada por el ESP32 en rol de maestro. Eso elimina de raíz las desviaciones por deriva de reloj, el *baud-rate drift*, y garantiza una transmisión determinista.

[SEÑALAR el detalle de la trama.]

Para blindar la integridad de las tramas tributarias, implementé una rutina de validación por redundancia longitudinal, LRC: un lazo cerrado de operaciones XOR acumulativas sobre los bytes recibidos, que se contrasta contra el *checksum* que envía el periférico. Si la trama es válida, el módulo responde con un *ACK*; si no, responde con un *NAK* y reinicia lógicamente los registros del bus, impidiendo que datos corruptos se propaguen.

## Diapositiva 7 — Nivel Gateway: ESP32, ESP-IDF y kernel concurrente (8:15 – 10:00)

[SEÑALAR el diagrama de los dos núcleos.]

Para el nodo de transmisión seleccioné la tarjeta ESP32 DevKitC V4, un microcontrolador de doble núcleo. La razón es la concurrencia: mientras un núcleo gestiona el sondeo de la impresora cada 5 segundos, el segundo núcleo administra la pila de red y la mensajería MQTT. Así, las latencias de internet no bloquean la captura local de datos.

Un hallazgo temprano fue una limitación de hardware: al usar el UART1 en sus pines por defecto, GPIO 9 y 10, el sistema entraba en reinicios continuos, *bootloops*, porque esos pines están anclados al bus SPI de la memoria Flash interna del SoC. Reconfiguré la matriz de señales y moví la comunicación fiscal hacia los pines libres GPIO 18 y 19.

[SEÑALAR el diagrama de exclusión mutua.]

Ahora bien, el corazón del diseño es el **kernel concurrente**. La coexistencia de la tarea de adquisición fiscal y la recepción de comandos inalámbricos introdujo condiciones de carrera severas. Identifiqué dos, y ambas tenían solución con exclusión mutua:

- La primera es la **corrupción de RAM**: la tarea de monitoreo sobrescribía los acumuladores de ventas en los búferes mientras la tarea MQTT empaquetaba el JSON, produciendo telemetría híbrida. Lo resolví con un *mutex* de datos, `mqtt_data_mutex`, que protege la estructura compartida.
- La segunda es la **colisión eléctrica**, a la que llamé "choque de trenes": si una actualización OTA llegaba justo cuando el sondeo iniciaba, ambas rutinas polarizaban las líneas físicas al mismo tiempo, el controlador entraba en pánico y reiniciaba la placa. Lo resolví con un *mutex* de hardware, `bus_local_mutex`, que ambas tareas deben adquirir antes de energizar el canal físico.

Además, para proteger el *heap* ante ráfagas de comandos, desacoplé la lógica OTA en una tarea supervisora y una cola segura de comandos.

## Diapositiva 8 — Nivel Cloud: MQTT v5.0 / TLS y Django + HTMX (10:00 – 11:15)

[SEÑALAR el esquema de tópicos MQTT.]

En la capa de red adopté el estándar **MQTT v5.0 sobre TLS 1.2**, hacia la infraestructura de HiveMQ Cloud. Elegí MQTT por su mínima sobrecarga de cabecera y su modelo de publicación-suscripción, y elegí la versión 5.0 por dos capacidades que no existían en versiones anteriores:

- Las ***User Properties***: metadatos clave-valor en la cabecera del protocolo —modelo, versión de firmware, serial y RSSI— que el servidor lee sin necesidad de deserializar el cuerpo del mensaje.
- El ***Last Will and Testament***, o testamento: una "última voluntad" que el dispositivo registra al conectar; si pierde la conexión abruptamente, el *broker* la publica automáticamente y el sistema sabe que el equipo se cayó.

La seguridad se estructuró con dos decisiones: enrutamiento forzado al puerto 8883, `secure-mqtt`, con transporte sobre SSL; y autenticación mediante un *bundle* de certificados optimizado de Espressif, que valida la cadena de confianza de HiveMQ sin quemar archivos PEM extensos en la Flash.

[SEÑALAR el diagrama del servidor.]

En la nube construí un **ecosistema web con Django y HTMX**. La pieza clave es un *worker* MQTT independiente, `listen_mqtt.py`, que corre en segundo plano con un lazo no bloqueante. No lo acoplé al servidor web, porque eso bloquearía el manejo de solicitudes HTTP ante ráfagas de datos. El *worker* compara cada trama contra el último estado persistido y descompone la diferencia en eventos atómicos: cambios de estado, cortes de reporte Z, emisión de facturas, reconexiones y actualizaciones de firmware.

Para la interfaz, en lugar de montar un servidor de WebSockets intermedio con Django Channels, integré el cliente MQTT de JavaScript directamente en el navegador, aprovechando que HiveMQ expone un canal WSS en el puerto 8884. Eso simplifica el *firewall* y elimina un intermediario pesado. Y con HTMX logré una experiencia de aplicación de página única, intercambiando solo fragmentos del DOM, sin recargar la página.

## Diapositiva 9 — Justificación tecnológica (11:15 – 12:00)

[SEÑALAR la tabla comparativa. Sé breve, es una diapositiva de justificación.]

Permítanme justificar brevemente las dos decisiones tecnológicas más importantes.

**Arduino frente a ESP-IDF.** Aunque Arduino permite un despliegue ágil, su modelo de ejecución es un bucle secuencial único, mono-hilo. En un entorno transaccional fiscal, un `delay()` bloqueante en la pila Wi-Fi arriesgaba sobrepasar la ventana temporal de lectura del firmware legal del equipo. Por eso elegí el framework nativo **ESP-IDF v5.5.2**, que me dio gestión explícita de memoria en C y asignación determinista de tareas con FreeRTOS.

**HTTP frente a MQTT.** Una arquitectura HTTP REST tradicional tiene una sobrecarga de cabecera enorme para un mensaje que cambia cada 5 segundos. MQTT traslada los metadatos a la cabecera del protocolo y aligera el cuerpo. Más adelante les mostraré la medición cuantitativa de ese ahorro.

[TRANSICIÓN al Bloque 3] Y con esto llegamos al bloque central de la presentación: la validación y los resultados experimentales.

---

# BLOQUE 3 — Validación y Resultados Experimentales
**Tiempo estimado: 16 min (acumulado 12:00 – 28:00) · Diapositivas 10 a 19**

## Diapositiva 10 — Entorno de pruebas HIL (12:00 – 13:30)

[SEÑALAR la foto del banco de pruebas.]

Antes de mostrarles los resultados, debo explicarles cómo los obtuve, porque aquí hay una decisión metodológica importante.

La validación de un sistema fiscal **no admite ensayos destructivos sobre equipos en producción**. Inducir cortes de energía, corromper tramas o abortar una reescritura de firmware sobre una impresora real comprometería registros tributarios irreversibles.

Para conciliar rigor experimental y seguridad, construí un entorno de pruebas **Hardware-in-the-Loop, HIL**, con dos niveles de emulación:

- El **primer nivel** es un emulador en software, `impresora.py`, desarrollado con la biblioteca `pyserial`, con los mismos parámetros eléctricos del enlace fiscal. Implementa una máquina de estados de recepción, calcula el código LRC y responde con las tramas de estado y los reportes S1 y SV2, incluyendo los acuses *ACK* y *NAK*. Esto me permitió depurar el firmware sin depender de la disponibilidad del equipo físico.
- El **segundo nivel** eleva la fidelidad: programé un ESP32 configurado como esclavo que emula íntegramente al periférico fiscal, sobre el mismo bus y con los mismos pines. Replica la trama de estado, los reportes S1 y SV2 y, de forma crítica, el protocolo completo de reescritura remota.

Contra ese periférico simulado validé de extremo a extremo el puente OTA, con un binario de prueba de 2500 bytes. Este enfoque me habilitó la **inyección determinista de fallos** —paquetes fuera de secuencia, *checksums* alterados, interrupciones del enlace— y confirmó la capacidad de recuperación antes de comprometer la memoria de un equipo fiscal real.

[PAUSA] Todo lo que verán a continuación fue medido sobre este banco de pruebas.

## Diapositiva 11 — Extracción transparente del *Status S1* (13:30 – 15:30)

[SEÑALAR la captura de la consola serial.]

El primer resultado es el mecanismo de extracción transparente del registro *Status S1*. Lo abordé como un mecanismo de extracción, no como una lectura secuencial del canal.

La interrogación se ejecuta desde una tarea dedicada de FreeRTOS, `printer_task_main`, desacoplada de la tarea de red. De ese modo, la adquisición nunca compite con la pila inalámbrica por el mismo hilo, y el sondeo se sostiene cada 5 segundos **sin alterar la máquina de estados del periférico ni interrumpir las transacciones en curso**.

El desempaquetado se resuelve íntegramente en SRAM, sobre dos búferes estáticos alineados para DMA de 1056 bytes, sin memoria dinámica en la ruta crítica. Por eso el perfil es determinista y libre de fragmentación del *heap*. La localización de los delimitadores STX y ETX se hace en un único barrido, seguida de una tokenización en sitio. Como la longitud de la trama es fija, el costo se mantiene acotado en tiempo constante.

[SEÑALAR los datos resaltados en la captura.]

Como pueden ver en la traza, el módulo aísla de manera transparente los registros de la Memoria de Trabajo del equipo fiscal: captura exacta del RIF del contribuyente, el número de registro de la máquina, y los acumuladores de ventas diarias. Y confirma el estado mecánico y lógico: modo fiscal y en espera, con el estado `0x60` y sin errores reportados por el cabezal.

## Diapositiva 12 — Sanitización y error *Off-By-One* (15:30 – 17:00)

[SEÑALAR la tabla de la purga.]

Uno de los defectos más sutiles que encontré, y que quiero destacar porque muestra el tipo de problemas que aparecen en sistemas reales, fue la **contaminación del *payload* por caracteres de control invisibles**.

Las tramas delimitan sus campos con `0x0A`, y el equipo intercala un salto de línea justo antes del fin de trama. Ese carácter no se aprecia en una inspección textual, pero ocupa una posición física en el búfer y desplaza todo cálculo de longitud por posiciones fijas.

La consecuencia era directa sobre el JSON publicado: la versión del periférico llegaba **truncada**, con el valor `20507GD00` en lugar de `020507GD00`. Un error *Off-By-One* que consumía el salto de línea en lugar del último dígito.

La solución no fue cosmética, sino una manipulación explícita de punteros en RAM: el firmware recorre el *payload*, avanza un puntero auxiliar hasta situarlo tras el último delimitador, y sobrescribe cada `0x0A` con un terminador nulo. Así, la función de formateo lee una cadena limpia y cerrada.

El resultado: **integridad del *payload* recuperada**, sin campos cortados ni basura residual, y con un JSON siempre deserializable. La misma disciplina la repliqué en el receptor: el *worker* de Django elimina los caracteres de control del mensaje crudo antes de invocar el analizador JSON.

## Diapositiva 13 — Inmunidad electromagnética (17:00 – 18:30)

[SEÑALAR la tabla de los tres niveles.]

El bus local que une al módulo con la impresora es el eslabón físicamente más expuesto del sistema, así que su tolerancia a perturbaciones electromagnéticas se evaluó como requisito de primer orden. Diseñé una **defensa escalonada en tres niveles**:

- **Nivel 1, defensa temporal:** toda operación de entrada y salida se ejecuta bajo ventanas de tiempo estrictas. La espera de *ACK* durante la reescritura se acota a 300 milisegundos; la recepción de respuestas de estado, a 2000; y la solicitud del reporte S1, a 3000. Al agotarse cualquiera de estas ventanas, la transacción se aborta de forma segura y se incrementa el contador de errores, con una regla de máximo 3 reintentos.
- **Nivel 2, sanitización de búfer:** antes de cada transacción se limpian de forma determinista los búferes de transmisión y recepción, para que los residuos de una operación previa no se interpreten como datos válidos. En paralelo, la validación LRC descarta cualquier trama cuya asimetría revele la alteración de un bit, respondiendo con *NAK*.
- **Nivel 3, reconstrucción estructural del bus:** cuando el conteo de errores supera el umbral, el firmware no se limita a reintentar: **desmonta y reancla por completo el controlador SPI**, mediante `spi_bus_free`, `spi_bus_initialize` y `spi_bus_add_device`. Esta reinicialización explícita recupera el periférico de bloqueos inducidos por ruido **sin necesidad de reiniciar el microcontrolador**.

La evaluación confirmó que la combinación de los tres niveles preservó la continuidad operativa: ante la desconexión física del periférico o la inyección de tramas corruptas, el sistema transitó al estado de error y se recuperó automáticamente una vez restablecidas las condiciones del bus, sin reinicios del SoC y **sin que una sola trama inválida alcanzara la base de datos**.

## Diapositiva 14 — Consumo de ancho de banda: HTTP vs. MQTT (18:30 – 20:00)

[SEÑALAR la tabla comparativa. Esta es una de las cifras fuertes: dila con calma.]

Esta es la medición que justifica cuantitativamente la selección del protocolo. Medí la sobrecarga de cabecera y la masa de datos transmitida por cada ciclo de telemetría.

[SEÑALAR fila por fila.]

- **Cabecera de red:** una petición HTTP POST tradicional ronda los 450 bytes; MQTT v5.0 con *User Properties*, unos 12 bytes. Eso es una reducción del **97,33 %**.
- **Cuerpo del mensaje:** 328 bytes en JSON plano frente a 142 bytes en el empaquetado optimizado en C. Un **56,70 %** menos.
- **Consumo total por trama:** de **778 bytes en HTTP a 154 bytes en MQTT**. Hablamos de una reducción global del **80,2 %** del volumen de red.

[PAUSA. Deja que la cifra cala.] Para un parque de impresoras que reporta cada 5 segundos durante todo el día, ese 80 % se traduce directamente en un ahorro del consumo de datos del contribuyente.

## Diapositiva 15 — Resiliencia de red: LWT y micro-cortes (20:00 – 21:45)

[SEÑALAR la captura de *Log Capture* y el diagrama de flujo.]

La tolerancia a fallos del canal inalámbrico la evalué provocando cortes de energía arbitrarios e interrupciones de la interfaz Wi-Fi.

Cuando el enlace se pierde, al expirar el *Keep-Alive*, el *broker* HiveMQ publica la "última voluntad" que el dispositivo había registrado. Como ven en la captura, el *worker* asíncrono interceptó la alerta `offline` y persistió transaccionalmente una nueva fila con el evento clasificado como "Desconexión Abrupta (LWT)". Ese mecanismo **congeló el último estado válido** de los acumuladores sin alterar los registros históricos previos.

[SEÑALAR el diagrama de flujo.]

El ciclo completo es este: pérdida del enlace Wi-Fi, reintentos de reconexión con *backoff* exponencial, publicación del testamento por el *broker*, registro del evento por el *worker* y restauración del túnel MQTT sin pérdida de estado.

Aquí quiero detenerme en una propiedad de diseño que considero elegante: el módulo es un **observador de estado sin memoria tributaria propia**. Toda la información fiscal reside en el periférico y se re-consulta en cada sondeo. El *gateway* no es una fuente de verdad que pueda corromperse, sino un reflejo periódico del equipo.

Eso tiene una consecuencia directa: ante un **corte abrupto de energía**, la recuperación no depende de persistir datos locales. Al restablecerse el enlace, el primer sondeo reconstruye la telemetría completa; el único efecto observable es un hueco de una ventana de sondeo, 5 segundos. Y ante un **micro-corte de red**, la lectura más reciente queda retenida con el indicador de sincronización pendiente, la tarea de red omite la publicación sin descartar el dato, y al reconectar se transmite de inmediato. Como el servicio es de al menos una entrega, QoS 1, y la telemetría es un estado idempotente, la repetición eventual no genera efectos adversos.

[PAUSA] En los cuatro escenarios de falla que evalué —corte de energía, micro-corte Wi-Fi, caída del *broker* y desconexión del periférico— la **pérdida de datos fue nula**.

## Diapositiva 16 — Matriz de latencias End-to-End (21:45 – 23:30)

[SEÑALAR la tabla de latencias.]

Ahora, el desempeño temporal. Definí la latencia de extremo a extremo, T_E2E, como la suma de cinco etapas lógicas serializadas, desde la interrogación física del periférico hasta la actualización del DOM en el navegador.

Mediante un perfilado sobre **500 iteraciones continuas** de envío de telemetría, obtuve estas latencias promedio:

[SEÑALAR cada fila.]

- Sondeo síncrono del S1: **118 milisegundos**, con una desviación de 4. Dentro de la ventana fiscal.
- Empaquetado JSON: **2 milisegundos**.
- *Handshake* TLS 1.2: **45 milisegundos**.
- Publicación y tránsito por el *broker* MQTT: **85 milisegundos**, la etapa más variable.
- Renderizado del DOM con HTMX y WebSockets: **18 milisegundos**.

La **latencia total promedio es de 268 milisegundos, con una desviación estándar de 15,3**.

[SEÑALAR la cifra de porcentaje.]

Ese valor representa apenas un **5,36 % del intervalo total de sondeo**, que es de 5 segundos. Es decir, el sistema opera en tiempo real estricto sin acumular retrasos en la pila de recepción.

Un análisis complementario de varianza mostró que la etapa de red concentra por sí sola el **86,8 % de la dispersión total**. Esto identifica al enlace del proveedor de servicios de Internet como la única fuente estocástica dominante del sistema. Es un dato relevante, porque significa que la incertidumbre no proviene del diseño embebido, sino de un factor externo y acotado.

Como referencia, este margen contrasta favorablemente con antecedentes nacionales de monitoreo IoT desarrollados en la misma Universidad Simón Bolívar, donde la regularidad del período de muestreo presentaba desviaciones de hasta un segundo.

## Diapositiva 17 — Actualización remota OTA de doble objetivo (23:30 – 25:30)

[SEÑALAR el diagrama de flujo de la OTA.]

Uno de los requerimientos no funcionales más exigentes fue mantener el ciclo de vida del hardware en campo sin requerir intervención física. Para ello diseñé un protocolo de **actualización inalámbrica OTA de doble objetivo**, inédito en la arquitectura de la empresa.

[SEÑALAR la rama izquierda.]

El primer objetivo es el **gateway** ESP32. Al seleccionar `ismart` en la interfaz web, el módulo recibe la orden por MQTT y activa la bandera de suspensión `ota_en_progreso`, que actúa como un "freno de mano" y detiene el sondeo fiscal. La autoactualización usa la API `esp_https_ota` y escribe en la **partición OTA inactiva** de un esquema dual: la Flash de 4 MB se divide en dos regiones idénticas. Si ocurre un corte de energía, el sistema descarta el sector corrupto y arranca desde la partición intacta.

[SEÑALAR la rama derecha. Sube un poco la energía: aquí está el aporte más disruptivo.]

El segundo objetivo es el más complejo: **reescribir in situ el firmware del periférico fiscal**, que no tiene pila de red propia. El módulo asume el rol de *host* de programación intermedio. Como el binario supera ampliamente la capacidad del *heap* libre del ESP32, la descarga no se almacena completa, sino que se procesa a través de un **búfer circular de 1024 bytes**.

Cada fragmento se empaqueta con una cabecera de 4 bytes —identificador, índice secuencial y longitud en 16 bits— más un código de redundancia XOR, y se admite hasta 3 reintentos con confirmación *ACK*. La máquina de estados tiene tres fases: apertura, transmisión segmentada y cierre. Y el cierre **solo se emite cuando los bytes confirmados igualan la longitud declarada por el servidor**; por construcción, una transferencia parcial nunca se compromete en la memoria del periférico.

Para la resiliencia ante cortes de red implementé **reanudación con cabeceras HTTP Range**: si el enlace titubea, el firmware no reinicia desde el byte cero, sino que reabre la conexión indicando el desplazamiento exacto para reanudar en el fragmento interrumpido.

[SEÑALAR la captura del mensaje de éxito.]

Como ven en la interfaz, la confirmación llega de forma asíncrona: "Actualización Completada: el equipo actualizó *printer* con éxito".

## Diapositiva 18 — Perfilado de memoria IRAM y DRAM (25:30 – 27:00)

[SEÑALAR la tabla de perfilado.]

Pasemos al análisis de rendimiento y estabilidad global. El mayor desafío técnico del proyecto fue la gestión de memoria.

Con la implementación completa de la pila TLS/SSL y el empaquetado dinámico de JSON, el uso de la memoria **IRAM alcanzó un nivel crítico del 78,39 %**: 102.751 bytes de 131.072 disponibles. Ese escenario presentaba dos riesgos severos: un **desbordamiento de pila** ante picos de tráfico, y la **inviabilidad del sistema OTA**, porque la saturación impedía reservar el *heap* necesario para los búferes de descarga.

Apliqué entonces una reingeniería de recursos mediante la utilidad `menuconfig`, desactivando la aceleración por IRAM de la pila Wi-Fi. El resultado fue una reducción de la IRAM del **78,39 % al 52,26 %**: un ahorro directo de 34.256 bytes, un **26,13 % menos**, que liberó el margen necesario para las operaciones de actualización.

[SEÑALAR las dos capturas comparadas.]

Estas son las capturas de perfilado, antes y después. El margen libre resultante fue de 62.577 bytes en IRAM, casi la mitad de esa memoria, y 143.312 bytes en DRAM.

## Diapositiva 19 — Perfilado térmico-energético y trazabilidad SQLite (27:00 – 28:00)

[SEÑALAR la tabla del subclocking.]

Otra decisión de diseño fue el **subclocking**: reduje la frecuencia del núcleo desde los 240 MHz nominales hasta 80 MHz.

La potencia dinámica de un circuito CMOS es proporcional a la frecuencia. Como en el ESP32 la tensión del núcleo es regulada internamente y permanece casi constante, reducir la frecuencia en un factor de 3 contrae la potencia dinámica en la misma proporción. Y como el salto térmico es proporcional a la potencia disipada, el resultado es una operación más fría y una menor degradación por ciclado térmico.

Lo que habilita esta reducción es el **acelerador criptográfico AES por hardware**: el cifrado intensivo no se ejecuta en el núcleo, sino en un bloque dedicado. Bajo esa condición, los 80 MHz sostienen el *handshake* TLS en 45 milisegundos, holgadamente dentro de la ventana de tolerancia de 100.

[SEÑALAR la tabla de SQLite.]

Finalmente, en la nube medí la estabilidad del almacenamiento en SQLite. Con un histórico simulado de **10.000 registros**, la búsqueda indexada por serial se mantiene **por debajo de los 12 milisegundos**. La rutina de depuración automática del *worker* purga los registros mayores a 90 días sin bloquear las consultas de la interfaz. Y la trazabilidad se apoya en dos entidades: el dispositivo, identificado por su serial, y su bitácora, vinculada por clave foránea y ordenada cronológicamente.

[TRANSICIÓN al Bloque 4] Con esto concluyo la presentación de resultados. Paso ahora al impacto, las conclusiones y la escalabilidad.

---

# BLOQUE 4 — Impacto, Conclusiones y Escalabilidad
**Tiempo estimado: 5 min (acumulado 28:00 – 33:00) · Diapositivas 20 a 23**

## Diapositiva 20 — Cumplimiento de objetivos (28:00 – 29:30)

[SEÑALAR la matriz de objetivos. Léela como una verificación, no como una lista.]

Quiero cerrar verificando el cumplimiento de los objetivos planteados al inicio.

- **Diseño de la arquitectura IoT:** cumplido. La arquitectura de tres niveles está desplegada y documentada.
- **Desarrollo del firmware embebido:** cumplido. El Módulo de Transmisión IoT opera sobre ESP-IDF y FreeRTOS con kernel concurrente.
- **Integración con las interfaces del equipo fiscal:** cumplido. La captura del *Status S1* y del *SV2* es transparente y no interfiere con la máquina de estados legal.
- **Comunicación mediante MQTT:** cumplido. Mensajería bidireccional MQTT v5.0 sobre TLS 1.2.
- **Servidor MQTT:** cumplido. *Broker* HiveMQ con *worker* asíncrono de ingesta.
- **Dashboard en tiempo real:** cumplido. Interfaz reactiva SPA con Django y HTMX, con monitor en vivo y captura de logs.
- **Validación del sistema:** cumplido. Latencia, ancho de banda, robustez y estabilidad cuantificadas sobre banco HIL.
- **Documentación:** cumplido. Diagramas de arquitectura, flujos de datos, scripts de prueba y este informe.

[PAUSA] Los ocho objetivos específicos quedaron cubiertos, y el objetivo general —diseñar, implementar y validar el sistema— se alcanzó con una validación experimental completa.

## Diapositiva 21 — Comparativa: estado previo vs. estado actual (29:30 – 31:00)

[SEÑALAR la tabla "antes y después". Esta diapositiva responde a la pregunta "¿y esto para qué sirve?".]

Para que el impacto sea tangible, comparemos el estado previo con el estado actual.

[SEÑALAR columna izquierda.]

**Antes:** el técnico debía conectar físicamente una PC al puerto serial de la impresora. Solo podía observar **un equipo a la vez**. No había visión global del parque. El diagnóstico era reactivo, tras el fallo. No existía historial de auditoría consultable. Y la actualización de firmware requería intervención física en sitio.

[SEÑALAR columna derecha.]

**Ahora:** el monitoreo es remoto y centralizado en un dashboard que muestra **múltiples equipos en simultáneo**. La detección de errores es temprana y automática, con alertas ante desconexiones. Existe una **bitácora de auditoría de 90 días** consultable por serial. El mantenimiento puede planificarse con datos reales, es decir, preventivo y no reactivo. Y la actualización de firmware se comanda **desde la nube**, tanto del *gateway* como del periférico fiscal.

[PAUSA] En una frase: pasamos de un equipo aislado y observado a mano, a un parque conectado, trazable y administrable de forma centralizada.

## Diapositiva 22 — Limitaciones y trabajo futuro (31:00 – 32:15)

[SEÑALAR la lista. Aquí conviene ser honesta: el jurado valora la autocrítica.]

Toda solución tiene límites, y quiero ser explícita con los de este trabajo.

- **Validación en laboratorio:** la solución se validó sobre un banco Hardware-in-the-Loop. Es un prototipo funcional y documentado, pero **aún no está desplegado en campo** sobre un parque real. El siguiente paso es una prueba piloto con un conjunto reducido de contribuyentes.
- **Escala de la nube:** SQLite es adecuado para un piloto, pero para un parque distribuido a nivel nacional conviene migrar a un motor relacional con mayor concurrencia, como PostgreSQL.
- **Seguridad y cumplimiento:** la información fiscal viaja cifrada de extremo a extremo, pero antes de un despliegue masivo debe revisarse el cumplimiento con la normativa tributaria vigente sobre custodia y tratamiento de datos fiscales.
- **Dependencia de la red del contribuyente:** el sistema es resiliente, pero su desempeño depende de la calidad del enlace del cliente; el 86,8 % de la dispersión proviene de ese factor externo.

Y como **trabajo futuro**, propongo: extender la telemetría a los reportes Z y a los cierres de auditoría; incorporar analítica predictiva sobre el histórico de errores, para anticipar fallos mecánicos antes de que ocurran; y añadir aprovisionamiento masivo de dispositivos, para dar de alta un parque completo sin configuración manual.

## Diapositiva 23 — Escalabilidad (32:15 – 33:00)

[SEÑALAR el diagrama de escalamiento.]

Sobre la escalabilidad, quiero destacar tres propiedades del diseño que la habilitan.

Primero, la **arquitectura de tres niveles es intrínsecamente escalable**: el nivel de borde no conoce al servidor más que por un tópico MQTT, de modo que añadir un dispositivo es añadir una suscripción.

Segundo, la **naturaleza sin estado tributario del *gateway*** facilita el despliegue masivo: no hay base de datos local que sincronizar ni estado que reconciliar; cada dispositivo es independiente y reemplazable.

Tercero, el **aprovisionamiento por tópicos jerárquicos** —con rutas por serial— permite que la plataforma administre flotas completas organizadas por grupo, sucursal y modelo, que es exactamente cómo está modelado el backend.

[PAUSA] En conjunto, la solución deja establecidas las condiciones técnicas para que The Factory HKA centralice la telemetría de un parque distribuido a nivel nacional y transforme la gestión reactiva del mantenimiento en una práctica basada en datos.

[TRANSICIÓN al Bloque 5] Con esto cierro mi presentación.

---

# BLOQUE 5 — Cierre y Agradecimientos
**Tiempo estimado: 2 min (acumulado 33:00 – 35:00) · Diapositiva 24**

## Diapositiva 24 — Cierre, agradecimientos y preguntas (33:00 – 35:00)

[Diapositiva final con el título y la frase de cierre. Baja el ritmo, cierra con seguridad.]

Para concluir: este trabajo resolvió una necesidad operativa concreta del parque de impresoras fiscales venezolano —la supervisión remota y el diagnóstico determinista de equipos que hasta ahora solo podían inspeccionarse mediante una conexión serial local— integrando en una sola arquitectura el **monitoreo**, la **trazabilidad** y la **actualización remota**.

Los resultados confirman un monitoreo de alta disponibilidad, resiliencia ante cortes de red y una reducción del 80 % en el consumo de datos. Y deja sentadas las bases para un despliegue en campo.

[PAUSA]

Más allá del resultado técnico, esta pasantía me confirmó el valor de un enfoque de ingeniería sustentado en el **diagnóstico cuantitativo**: perfilado de memoria, caracterización de latencias y pruebas de tolerancia a fallos. Fue ese método —medir antes de decidir— el que permitió resolver los cuellos de botella reales, desde la crisis de IRAM hasta las condiciones de carrera, y convertir un prototipo frágil en un sistema determinista y robusto.

[PAUSA. Mira al jurado.]

Quiero agradecer a la Universidad Simón Bolívar y a The Factory HKA por hacer posible este trabajo; al profesor Miguel Díaz por su tutoría académica; al ingeniero Pablo Arguinzones y al ingeniero Luis Muñoz por su acompañamiento industrial; y a todas las personas que me apoyaron durante estos meses.

Con esto doy por concluida mi presentación y quedo atenta a sus preguntas. Muchas gracias.

[PAUSA. Espera. No agregues nada más.]

---

# Anexo — Preparación para la ronda de preguntas

Ten estas respuestas listas. Se dan en 3 a 5 frases cada una.

**1. ¿Por qué MQTT y no HTTP o WebSockets puros?**
MQTT traslada los metadatos a la cabecera del protocolo y permite un cuerpo mínimo; la medición dio una reducción del 80,2 % del volumen por trama frente a HTTP POST. Además, su modelo publicación-suscripción desacopla el emisor del receptor, lo que permite que múltiples consumidores (dashboard, worker de ingesta) reciban la misma telemetría sin que el dispositivo los conozca.

**2. ¿Cómo garantizan que no se corrompe la información fiscal?**
Por tres vías. Primero, el módulo es un observador sin memoria tributaria propia: nunca escribe en la memoria fiscal, solo lee el *Status S1* y el *SV2*. Segundo, toda trama se valida con LRC antes de aceptarse. Tercero, el acceso al bus físico y a la memoria compartida está serializado con *mutex*, lo que elimina las condiciones de carrera que probé en laboratorio.

**3. ¿Qué pasa si se corta la energía a mitad de una actualización OTA?**
Depende del objetivo. Si es el *gateway*, la escritura ocurre sobre la partición OTA inactiva; el *bootloader* detecta el sector corrupto y arranca desde la partición intacta. Si es el periférico fiscal, el cierre de la transacción solo se emite cuando los bytes confirmados igualan la longitud declarada; una transferencia parcial nunca se compromete, y una nueva orden reinicia el proceso desde cero.

**4. ¿Por qué SQLite y no PostgreSQL?**
SQLite es suficiente para un piloto y para la escala de las pruebas: con 10.000 registros, la búsqueda indexada se mantiene por debajo de 12 milisegundos. Reconozco que para un parque nacional conviene migrar a PostgreSQL por concurrencia; lo dejé planteado como trabajo futuro.

**5. ¿Por qué ESP32 y no una Raspberry Pi o un PLC?**
El ESP32 ofrece el equilibrio adecuado para este caso: doble núcleo con FreeRTOS para concurrencia determinista, aceleración criptográfica AES por hardware, bajo consumo y bajo costo, en un dispositivo embebido sin sistema operativo de propósito general. Una Raspberry Pi añadiría un sistema operativo completo, mayor consumo y mayor superficie de ataque, sin aportar nada que el sistema requiera.

**6. ¿Es seguro enviar datos fiscales a la nube?**
El transporte usa TLS 1.2 de extremo a extremo, con validación de la cadena de confianza mediante *bundle* de certificados, lo que mitiga ataques de intermediario. El *payload* se formatea con `snprintf` para prevenir desbordamientos. Dicho esto, antes de un despliegue masivo debe revisarse el cumplimiento normativo sobre custodia de datos fiscales; lo señalé como limitación.

**7. ¿Cómo escala a miles de dispositivos?**
La arquitectura de tres niveles escala por diseño: añadir un dispositivo es añadir una suscripción a un tópico. El *broker* gestiona las sesiones, y el *worker* de ingesta se desacopla del servidor web. Los cuellos de botella previsibles serían la base de datos y el aprovisionamiento, que ataqué con la retención de 90 días y que dejé planteado como trabajo futuro.

**8. ¿La latencia de 268 ms es aceptable?**
Sí, y con holgura: representa el 5,36 % del intervalo de sondeo de 5 segundos. Además, el 86,8 % de la dispersión proviene del enlace del proveedor de Internet, no del diseño embebido, lo que significa que en un enlace estable el sistema sería aún más regular.

**9. ¿Qué pasa si el periférico se desconecta físicamente?**
El sistema detecta el *timeout* del bus, incrementa el contador de errores y aplica la regla de 3 reintentos. Si persiste, reinicializa el controlador SPI para recuperarse de bloqueos. Cuando el periférico vuelve, el sistema se recupera automáticamente sin reiniciar el SoC.

**10. ¿Cuál fue el aporte más significativo?**
El puente síncrono heterogéneo de reprogramación inalámbrica. Es la capacidad más disruptiva porque traslada el mantenimiento de firmware a la nube y reduce drásticamente la intervención física en campo, algo crítico para una empresa que administra equipos dispersos geográficamente.

---

# Consejos de exposición

- **Ensaya en voz alta con cronómetro.** El Bloque 3 es el que más se te puede estirar; si te pasas de 40 minutos, recorta las diapositivas 9 (justificación tecnológica) y 19 (perfilado térmico), que son las más prescindibles.
- **No leas las diapositivas.** Las diapositivas llevan cifras y diagramas; tu voz debe contar la historia que las conecta.
- **Ancla las cifras fuertes** (80,2 %, 78,39 % → 52,26 %, 268 ms) diciéndolas despacio y haciendo una pausa después. Son tus mejores argumentos.
- **Cuando señales un diagrama, recórrelo con el puntero** y nombra cada nivel antes de explicar su función.
- **Reserva 30 segundos de margen** entre el final del Bloque 5 y la primera pregunta; no termines y salgas corriendo.
- **Ten a mano una copia impresa** del informe con las tablas de resultados, por si el jurado pide ver un dato específico durante las preguntas.
