# Control de navegación

`mvn_pln` conserva por ahora su paquete, ejecutable y entrada `/nav_control/goal`.
La coordinación está separada del control de la base en `simple_move` y de la
planificación geométrica en `path_planner`. No se ha creado todavía un action server.

## Goals y futura transición a navigation_manager

En `mvn_pln_node.cpp`, `accept_goal`, `process_goal` y `end_goal` delimitan el ciclo
que posteriormente podrá usar los callbacks de una action. `publish_status` es la
salida de estado del transporte actual. Cada petición crea una generación nueva:
se descartan respuestas de planes cancelados y resultados de movimientos anteriores.
No existe historial de destinos visitados ni supresión de goals repetidos.

Cada goal se compara con la pose actual:

- Posición y orientación dentro de tolerancia: éxito `already_at_goal_pose`, sin plan.
- Solo posición dentro de tolerancia: movimiento de orientación, sin solicitar A*.
- Posición fuera de tolerancia: plan nuevo desde la pose actual.
- Después de moverse: éxito `goal_pose_reached` al verificar ambas tolerancias.

El servicio interno `/navigation/path_planner/plan_navigation` usa
`path_planner/srv/PlanPath`: inicio, destino, stamp mínimo del mapa y rutas fallidas
del goal actual y puntos de bloqueo que deben existir en el mapa. Un cliente que
deje `goal_recovery_radius=0` conserva la meta exacta. El coordinador permite
alternativas sólo cuando la celda solicitada está ocupada, con un radio de
`robot_radius + near_goal_extra_margin` (0.10 m adicionales).
La respuesta distingue `start_recovery` y `goal_relocated`; el goal original se
conserva separado del destino del movimiento. Los servicios `GetPlan` existentes
siguen disponibles para otros clientes. Stop cancela la petición pendiente y
el movimiento. Un goal nuevo sustituye al anterior.

El coordinador envía `path_planner/msg/NavigationPath` a
`/navigation/simple_move/navigation_path`: ruta y región de llegada opcional en
el mismo frame. Fuera de esa región, la tolerancia de posición se estrecha según
la holgura del destino alternativo para conseguir entrar en el radio permitido.
Dentro se recupera la tolerancia normal, evitando perseguir precisión adicional
innecesaria. El éxito exige siempre estar dentro de la región y cumplir el
ángulo. Los clientes directos de
`/navigation/simple_move/goal_path` con `nav_msgs/Path` siguen usando la
configuración normal, que se restaura en cada movimiento nuevo.

La ruta de visualización se publica aparte como `nav_msgs/Path` en
`/navigation/path`, con QoS reliable/transient-local y profundidad 1. RViz puede
conectarse después del despacho y recibir la ruta activa. El fin o cancelación
del goal publica una ruta vacía para quitarla. Este tópico no es una segunda
orden de movimiento. La configuración `rviz/config.rviz` usa esta salida; una
configuración externa debe actualizar su display Path al mismo tópico.

Una meta bloqueada antes de obtener una ruta permite buscar el punto libre
alcanzable más cercano dentro de ese radio. Al llegar se comprueban orientación,
posición del destino alternativo, distancia al goal original, sensores y mapa
recientes, y 0.3 s de reposo observado por TF. El resultado terminal es único:
`SUCCEEDED` con texto `reached_near_goal: goal_blocked`. El log registra pose
solicitada, pose alcanzada y errores de posición y ángulo.

Si el bloqueo aparece durante el seguimiento, detenerse donde está el robot
requiere al menos `near_goal_min_progress=0.90` del recorrido, y tanto longitud
restante como distancia al goal menores que el radio anterior. La proyección se
limita a una ventana local para no saltarse un bucle al cruzar segmentos. Se
completa la orientación antes del éxito. Una ruta libre conserva las tolerancias
normales de llegada. Si el bloqueo desaparece se vuelve al destino original.

## Configuración y cambios durante ejecución

El launch expone `motion_mode:=omni` y `move_head:=true` por defecto.
Con el namespace predeterminado:

```bash
ros2 param set /navigation/simple_move motion_mode differential
ros2 param set /navigation/simple_move motion_mode omni
ros2 param set /navigation/simple_move move_head false
ros2 param set /navigation/simple_move move_head true
```

En diferencial nunca se ordena velocidad lateral. Cambiar de modo reinicia el
perfil de velocidad. Las órdenes relativas de distancia y distancia/ángulo
continúan disponibles y habilitan su monitor de obstáculos; una orden lateral
explícita se rechaza en diferencial.

Desactivar `move_head` deja de publicar objetivos de cabeza y elimina la espera
de alineación; no ordena una posición neutral. Al reactivarlo se comprueba la
posición medida antes de trasladar la base. El bypass de cámara es una condición
separada y no sobrescribe la preferencia `move_head`.

`head_pan_min`, `head_pan_max` y `head_tilt` corresponden a los valores configurados
en el controlador de cabeza instalado. Deben coincidir con el hardware usado.
La ausencia de feedback reciente o una alineación que excede su tiempo límite
impiden continuar la traslación con coordinación de cabeza habilitada.

Antes del primer desplazamiento, la cabeza debe estar alineada dentro de
`head_alignment_tolerance` (0.15 rad). Durante el seguimiento, se anticipa la curva
con `head_lookahead_time_sec` (0.25 s) y se reduce gradualmente la velocidad si
la cabeza se retrasa, incluido el giro de base si aumenta ese retraso. Se vuelve a detener a partir de `head_stop_error_rad`
(0.55 rad), al perder feedback o al necesitar un giro de base por límites de pan.
Se reanuda dentro de la tolerancia de alineación, evitando alternar en un único
umbral. Desactivar y reactivar `move_head` vuelve a exigir la alineación inicial.
Cuando se rebasa el límite mecánico de pan, la base corrige hasta recuperar
`head_pan_limit_margin` (0.05 rad) dentro del recorrido permitido. La traslación
no se libera exactamente en el borde que acaba de provocar la parada. Durante
esa espera no se reinicia la referencia de evasión por una proyección cercana
a la ruta.

## Seguimiento y orientación

El techo del launch sigue en 0.9 m/s y 1.3 rad/s. La anticipación revisa las curvas
antes de que entren en el lookahead para frenar con aceleración y jerk acotados,
conservando velocidad en rectas libres. Se mantienen las comprobaciones de sensores, mapa y parada crítica.

Si el segmento hacia el lookahead corta una esquina ocupada, se acorta la
anticipación hasta encontrar un conector libre. En modo omnidireccional, el giro
de base no impone un arco diferencial a la traslación ni a su frenado previsto:
se conserva la dirección de movimiento en odometría. La curvatura de la ruta
sigue limitando la velocidad antes de las curvas.

La orientación final conserva una referencia continua, independiente de cambios
de tangente y del cruce +/-pi. En los últimos 0.40 m, si existe un corredor libre
hacia el destino, la fase de captura sigue directamente el error de posición.
Se reserva tiempo para corregir el ángulo antes de consumir la distancia restante;
`angular_damping` amortigua el giro y el éxito espera una velocidad angular baja.
El límite del ajuste angular terminal es 0.65 rad/s. En traslados omnidireccionales
cortos (hasta 1 m), si el ángulo final ya coincide con el inicial dentro de 0.20 rad,
se conserva esa referencia y se evita girar hacia la tangente y luego regresar.
La base aporta el giro mínimo si la cabeza no alcanza la dirección de avance.
La referencia angular mantiene el tramo mecánico conectado con el pan actual;
no elige un ángulo equivalente que exija cruzar el sector prohibido del cuello.
Cuando se proyecta esa referencia contra un límite, se elimina la velocidad de
giro anticipada del perfil original para no volver a empujar fuera del límite.
Si el ángulo final es incompatible con los límites de cabeza durante la traslación,
se mantiene una orientación factible y se completa el giro al detenerse en destino.
La corrección angular conserva siempre las coordenadas originales del goal.

La proyección sobre segmentos y la anticipación se miden en metros. El 10 %
inicial es una preferencia adaptable: se amplía según el giro, la velocidad y
la aceleración angular disponibles. Se interpola la orientación inicial hacia
la tangente; el 10 % final interpola hacia la orientación solicitada. La ventana
final tiene un mínimo de dos tolerancias de posición y termina en el
límite de llegada. Puede adelantarse cuando la velocidad y aceleración angular
requieren más tiempo; se desacelera antes de entrar en esa ventana. En rutas cortas o restricciones de cabeza puede quedar una
corrección angular en el destino. La orientación final durante el desplazamiento
se aplica al modo omnidireccional; el diferencial termina la orientación al llegar.

El ajuste final mantiene su fase ante pequeñas variaciones de localización:
entra dentro de `fine_dist_tolerance` y reanuda la aproximación si la distancia
supera esa tolerancia más `final_position_hysteresis_m` (0.03 m), o si terminó el
giro y aún falta corregir posición. El éxito sigue exigiendo las tolerancias
originales de posición y orientación.
Una posición ligeramente fuera de tolerancia debe persistir durante
`final_position_settle_sec=0.30` antes de reabrir la aproximación una vez corregido
el ángulo. Un error mayor que la banda de histéresis reabre la corrección de
inmediato. Esto no amplía la tolerancia de éxito. Los objetivos de cabeza usan
`head_goal_deadband_rad=0.04`; a baja velocidad terminal se conserva una banda
de hasta media tolerancia de cabeza para no perseguir cambios de pocos grados.
En el perfil Takeshi AMCL, las tolerancias compartidas entre controlador y
coordinador son 0.08 m y 0.08 rad (aproximadamente 4.6 grados).

La repulsión favorece una corrección lateral acotada en relación con la velocidad
de seguimiento. No invierte ni cancela por sí sola el avance; el monitor conserva
la reducción preventiva de velocidad y la parada crítica.

## Mapas recientes y recuperación

En el perfil AMCL, `obstacle_timeout_sec=1.0` conserva las detecciones ordinarias
tras perderlas de vista. Dentro de `unobserved_obstacle_hold_radius=1.0` m,
`unobserved_obstacle_hold_sec=3.0` acota la retención adicional desde la última
observación. Deben vencer tanto ese plazo como el timeout ordinario. El tiempo
se mide desde la observación real, no desde la publicación del mapa; una nueva
detección actualiza la evidencia.

Los puntos que han entrado en el alcance mínimo de la cámara conservan una
excepción de seguridad: requieren observación libre o alejarse del radio de
retención. Se mide la distancia 3D desde el origen actual de la cámara; si falta
TF reciente no se presupone que el punto sea observable. Tres observaciones
válidas que pasan por el punto y terminan al menos 0.10 m más lejos pueden
liberarlo antes. Una nube ausente o NaN no demuestra espacio libre. Al vencer la
memoria ordinaria, el monitor sigue impidiendo movimiento si falta el sensor.
Los resets de localización/reloj limpian esta evidencia.
`remember_all_obstacles` sigue en false.

La ventana vertical se comprueba después de transformar los puntos a la base.
También se aplica a la evidencia confirmada por el monitor: ese canal evita los
filtros ordinarios de XY/densidad, pero no puede saltarse `cloud_min_z` o
`cloud_max_z`. No se usa `pc_filter` ni se cambian esos umbrales para ocultar
posibles errores de TF.

La base HSR, incluido el bumper de `hsrb_description/base_v2`, tiene un radio
envolvente de 0.228 m; los radios físicos de Takeshi usan 0.23 m. La exclusión de
puntos propios usa un cilindro independiente de 0.23 m hasta
`self_obstacle_ignore_max_z=0.30` m; por encima no se descartan objetos externos
por estar dentro de ese radio. `self_filter_boxes` permite añadir volúmenes
propios en coordenadas de la base: xmin,ymin,zmin,xmax,ymax,zmax por caja. Esto representa la base, no un brazo extendido.

La inflación estática usa `robot_radius + planning_hard_margin`: 0.25 m.
La dinámica usa `robot_radius + dynamic_planning_margin`: 0.23 m por defecto,
sin margen adicional. `planning_hard_margin_1` se conserva como alias del margen
dinámico; valores negativos se limitan al radio físico, nunca a una huella menor.
La dinámica conserva la distancia métrica sin redondearla hacia fuera otro píxel.
`static_obstacle_ignore_radius=0.08` m suprime detecciones de superficies ya
representadas en el mapa estático. El monitor inmediato sigue comprobándolas.
`collision_footprint_radius` hereda `robot_radius`; sobrescribir el primero ya no
reduce la inflación del planeador ni la exclusión del propio robot. El mapa
de entrada debe representar obstáculos sin inflar. Si se sobrescriben los radios,
deben conservar la separación física exigida por el controlador. La comodidad
adicional se expresa mediante costos suaves (`augmenter_cost_peak=15`), mientras
`collision_stop_margin=0.08` conserva el margen preventivo de frenado. Así un
pasillo puede utilizar el espacio físico disponible sin reducir la huella real.
La inflación estática de planificación redondea al tamaño de celda; las capas
de huella física y dinámica mantienen su radio métrico (resolución de 5 cm en el laboratorio).

El planeador actualiza mapa y costos cada 0.2 s y sólo acepta pares con la misma
geometría y stamp. `map_snapshot_timeout_sec` limita a 1 s la edad de la copia y
de su recepción. Un bloqueo exige un mapa construido después de la parada; el
stamp requerido permanece fijo mientras se espera para no perseguir una versión
que avanza continuamente. `waiting_for_updated_map` mantiene la base detenida y
solicita actualizar la copia, sin tratarlo como un fallo de suavizado.
`required_obstacles` añade la comprobación geométrica: una copia reciente que no
contenga esos puntos ocupados devuelve `waiting_for_obstacle_map`. La vigencia de
estos testigos se configura con `obstacle_evidence_timeout_sec` (4 s). El detector
conserva por separado la evidencia cercana de zonas no observables.

Tras dos bloqueos sin alejarse `recovery_progress_m` (0.4 m) del primer fallo,
se penalizan hasta cuatro fragmentos de rutas fallidas del goal actual. La
penalización es un costo temporal con radio `recovery_cost_radius` (0.35 m) y
pico `recovery_cost_peak` (40); nunca borra obstáculos y los atajos y el suavizado
respetan un presupuesto de costo integrado, permitiendo redondear curvas sin
cancelar el desvío. El coordinador rechaza una ruta que vuelve a recorrer un
fragmento fallido con separación menor a `recovery_path_difference_m` (0.12 m).

`max_recovery_attempts` permite tres intentos de recuperación por defecto.
Si se agotan las alternativas se termina con `recovery_exhausted`, incluso con
`patience=true`, para evitar arranques repetidos. Esperar mapas actualizados no
consume esos intentos; tampoco un rechazo exclusivo del suavizado. Cada goal nuevo limpia esta memoria; no es un historial
de destinos visitados.

La cabeza sigue la dirección de traslación prevista, incluida la repulsión. Si el
ángulo queda fuera de alcance, se permite el giro mínimo de base antes de avanzar.
La velocidad se reduce según la curvatura y el tiempo disponible para girar.
Las órdenes de velocidad tienen límites de aceleración y jerk; la parada ante
peligro y la espera de cabeza tienen prioridad sobre ese filtrado.

## Obstáculos y replanificación

La fuerza preventiva utiliza ambos ejes y se limita y filtra. El corredor de frenado
publica `potential_fields/speed_limit`: el controlador reduce su traslación para
poder detenerse dentro del espacio libre. `collision_risk` se reserva para puntos
con suficiente evidencia dentro de la huella más `collision_critical_margin`.
La reacción de frenado (`collision_reaction_time_sec`) es independiente del límite
de antigüedad de sensores; además se considera la edad real de cada observación. Observaciones vencidas
mantienen el monitor no listo y el límite de velocidad en cero. El canal `potential_fields/ready` permite esperar la
primera observación válida sin confundir el arranque de sensores con una colisión. Los campos de la nube no se desactivan automáticamente
para superar una colisión.

El coordinador revisa el corredor próximo de la ruta en `/navigation/augmented_map`.
Una intersección de la línea original da oportunidad a la evasión local, con
realimentación `local_avoidance`; no implica que todo el pasillo esté cerrado.
El controlador prueba comandos de velocidad y dirección sobre su horizonte de
frenado y conserva el lado del desvío hasta encontrar un corredor libre hacia
la ruta. `max_avoidance_deviation_m` limita ese desvío a 0.55 m. El coordinador
replanea si pasan 3 s sin avanzar 0.15 m en esa zona; si no existe comando local
seguro, el controlador mantiene velocidad cero y pide replanteo tras 1.5 s
(`local_blocked_hold_sec` en el launch).
El controlador comprueba la trayectoria local limitada, incluida la repulsión,
sobre su horizonte de frenado usando la capa reciente `clearance_map`, que
contiene la huella física; si falta, usa el mapa aumentado. No añade otra huella
completa. El mapa aumentado mantiene el margen adicional para planear la ruta.
La comprobación local agrega
medio píxel de margen para cubrir esquinas entre muestras. Si una pose libre
empieza dentro de ese margen adicional, sólo se acepta un movimiento que mantenga
o aumente la separación hasta recuperarlo. Las celdas ocupadas no se limpian.
El inicio dentro de un margen de planificación tiene una recuperación explícita
que se valida con la capa de huella física descrita abajo.

La evasión comprueba también un corredor de avance cuya longitud depende de la
velocidad, reacción y frenado, con mínimo de 0.10 m y limitada por el recorrido
restante. Esto permite negociar curvas que no tienen 0.40 m rectos por delante.
Los candidatos se comprueban también después de aplicar la misma integración
de aceleración y jerk que usará el comando publicado. Su velocidad
preferida es `local_avoidance_speed` (0.20 m/s), limitada por frenado, sensores,
cabeza y ajuste angular final. No es una velocidad mínima obligatoria. La
penalización de desvío se escala con la velocidad para no crear un mínimo local
artificial al retornar lentamente a la ruta.

Bloqueo, desviación persistente y falta de progreso detienen y permiten
replanificar. Los datos vencidos detienen la base y llevan al coordinador a
`waiting_for_obstacle_data`: no se solicitan planes mientras falte el sensor.
Tras 0.3 s de datos recuperados se planifica desde la pose real. Si no se recupera
en `sensor_wait_timeout_sec` (10 s), el goal termina con `obstacle_data_unavailable`. Los umbrales están expuestos en el launch.
La espera de alineación de cabeza o de sensores no consume el plazo de bloqueo
geométrico de la ruta. Los testigos de obstáculos usan el timestamp de la pose
con la que se calcularon para permitir su transformación al mapa.
Con `patience=true`, no encontrar una ruta válida mantiene los reintentos; con
`false`, una respuesta sin plan válido termina el goal con fallo.
`start_blocked` y `goal_blocked` consumen un presupuesto de
`max_recovery_attempts` (3) incluso con paciencia habilitada; al agotarlo terminan
con `recovery_exhausted`. Las esperas por mapas recientes no consumen ese
presupuesto. Así un bloqueo sin salida o sin alternativa no queda en un bucle.

## Suavizado

La evaluación usa intervalos métricos uniformes de hasta 10 cm y comprueba el
residuo de los puntos originales para que un zigzag de rejilla no se confunda con
una recta. Los valores por defecto son 6 m^-1 y 0.50 rad. Los reintentos añaden
costos suaves para buscar espacio de giro; no aumentan la inflación obligatoria.
`detail` en los avisos distingue curvatura/cambio de dirección, intersección con
obstáculos, presupuesto de costos y presupuesto de tiempo agotado.

Con `require_smooth_path=true`, una ruta debe satisfacer curvatura, cambios de
dirección y validación de todos los segmentos contra el mapa. Una recta original
ya puede cumplir estos criterios. Un retorno de la ruta original desde
`SmoothPath` nunca omite el control de calidad de la planificación completa.

Se prueban pesos progresivos, atajos comprobados, remuestreo y una búsqueda
alternativa con espacio adicional de giro. El número y tiempo entre intentos
están limitados por ciclo. Si no se obtiene una ruta válida, no se entrega una
ruta irregular al controlador. El siguiente ciclo de coordinación puede reintentar
con el mapa actualizado. El presupuesto se comprueba entre etapas; no interrumpe
una búsqueda A* ya iniciada.

La recuperación no borra celdas. El aumentador publica `/navigation/clearance_map`
con la huella física sin el margen de planificación, incluyendo los obstáculos
dinámicos y las zonas prohibidas. Si el robot está dentro del margen pero libre
en esa capa, una búsqueda local encuentra la salida permitida más cercana
alcanzable. La máscara tiene radio `robot_radius + start_recovery_margin`
(5 cm adicionales) y está ligada a la pose inicial de esa recuperación. Usa
memoria proporcional a la ventana local y consulta los mapas existentes sin
copiarlos. Puede rodear obstáculos físicos dentro de esa ventana; las diagonales
comprueban las celdas laterales y todos los conectores se validan físicamente.

El servicio devuelve primero sólo la recuperación, con `start_recovery=true`.
El coordinador espera su ejecución y solicita entonces el plan global desde la
pose medida, manteniendo el goal original. Se evita calcular planes globales
por cada salida candidata. La recuperación conserva la pose real como primer
punto y está sujeta a su propio límite de velocidad; no se impone suavizado de
crucero a un conector corto que abandona el margen. El controlador exige salir
realmente a una celda permitida antes de dar ese tramo por terminado.
Los fallos repetidos `start_blocked`/`goal_blocked` y las recuperaciones locales
consecutivas sin conseguir un plan global tienen un límite de tres intentos,
también con `patience=true`. Se devuelve `recovery_exhausted` con la causa;
un goal nuevo reinicia esos contadores.

`escape_clear_radius` (0.6 m, positivo) limita la búsqueda y el recorrido de
salida; se propaga como `start_escape_radius` al coordinador y al controlador.
`start_escape_speed` (0.08 m/s) limita esa fase. `escaping_inflation` identifica
su ejecución. Una vez fuera, se solicita el plan global y termina el límite
especial de velocidad; la seguridad local sigue usando la huella física.
Obstáculos físicos y desconocidos no se atraviesan, y el monitor inmediato sigue
activo. También puede corregirse sólo el ángulo dentro del margen cuando la
huella circular está libre, sin cambiar las coordenadas de la meta.

## Verificación

`test_laboratory_navigation.py` inicia el stack completo con los mapas
`maps/lab_2026` y `prohibition_maps/lab_2026`, LiDAR, base y cabeza simulados. Usa
0.9 m/s, las poses registradas, perturbaciones de 2 cm del inicio y un obstáculo
que aparece durante el recorrido. Mide separación de la base y velocidad pico.

```bash
ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 \
  python3 src/Navigation/navigation_planner/navigation_start/test/test_laboratory_navigation.py
```

Las pruebas C++ se ejecutan con `colcon test --packages-select path_planner`.
Las pruebas ROS usan una base, cabeza, sensores y planeador simulados:

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 \
  python3 src/Navigation/navigation_planner/navigation_start/test/test_coordinated_navigation.py
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 \
  python3 src/Navigation/navigation_planner/navigation_start/test/test_planning_maps.py
```

Debe usarse un dominio DDS aislado del robot. La suite no inicia hardware.
Las pruebas fijan Cyclone DDS a loopback para no heredar peers del despliegue.
Los resultados de simulación no sustituyen la calibración física de huella,
velocidades, frenado y límites de cabeza.

## Diagnóstico de obstacle_data_stale

El argumento `debug_detection:=true` habilita el diagnóstico del detector,
`simple_move` y `mvn_pln` en los cuatro launches AMCL/SLAM de Takeshi y Justina.
También puede cambiarse sin reiniciar, para cada nodo:

```bash
ros2 param set /navigation/obstacle_detector debug_detection true
ros2 param set /navigation/simple_move debug_detection true
ros2 param set /navigation/mvn_pln debug_detection true
```

`MotionDiagnostic` relaciona el ID del movimiento con pose, distancia restante,
error y referencia angular, velocidad angular medida, cabeza medida/solicitada,
motivo de espera de cabeza y velocidad nominal/solicitada/publicada. Incluye
antigüedad de datos y el motivo de rechazo local: `occupied_cell`, `unknown_cell`,
`sampling_guard`, `escape_radius`, `map_unavailable`, `map_stamp_stale`,
`outside_grid` o `insufficient_horizon`. `layer` distingue la capa física del
respaldo de planificación; celda, tiempo de simulación y separación/margen están
en metros y segundos. `tested` es la velocidad evaluada, que puede haber sido
rechazada antes de publicar cero. Los tres contadores `rejected` corresponden
al candidato nominal, comando alcanzable y corredor de salida.

Los registros incluyen las paradas que retornan antes de publicar el comando
normal. Los detalles se emiten al cambiar de fase o de las banderas de alineación,
cabeza, captura o escape; de otro modo, hasta 5 Hz en ajustes finales, evasión,
recuperación y esperas, y una vez por segundo en seguimiento normal.
`NavigationDiagnostic` y `PlanDiagnostic` relacionan el goal y movimiento con
respuesta del planeador, celdas del inicio/meta, coherencia y edad de mapas,
reintentos y la condición `near_gate` que impide la recuperación final.

`Detection[summary]` conserva sus contadores anteriores y añade las renovaciones
de evidencia ordinaria/confirmada y celdas borradas por rayos libres. `memory`
describe la última pasada de caducidad: celdas vencidas retenidas por tiempo de
espera, rango ciego, TF de robot o TF de cámara; edad, posición, distancia 3D y
frame de cámara de la más antigua. Los conteos de retención son una instantánea,
no la suma repetida de la misma celda. Una edad o distancia ausente se
representa por -1. Este diagnóstico no modifica la retención: una celda dentro
de la zona ciega todavía puede conservarse indefinidamente sin evidencia libre.
Las ganancias, tolerancias y política de seguridad mantienen su configuración.

Este estado indica que el controlador está detenido esperando observaciones válidas;
no demuestra que haya un obstáculo. `potential_fields` imprime sus tópicos al iniciar,
la habilitación del monitor y, cada dos segundos mientras esté bloqueado, el sensor
pendiente y el motivo:

- `no_messages`: el tópico configurado no entrega observaciones.
- `sensor_stamp_age`: timestamp ausente, vencido o adelantado; revisar reloj y `use_sim_time`.
- `tf_unavailable`: no existe una transformación entre el sensor y la base.
- `stale_tf_fallback`: hay TF, pero supera `tf_fallback_max_age_sec` respecto al sensor.
- `invalid_cloud_layout` / `unsupported_cloud_field`: formato de nube incompatible.

La búsqueda de TF espera hasta `tf_timeout_sec`. Si no llega la transformación exacta,
puede usar la última disponible únicamente dentro del límite de antigüedad configurado,
con la misma política que `obstacle_detector`. El launch comparte estos ajustes y
propaga `use_sim_time` también al monitor, controlador y coordinador.

`simple_move` informa además la edad de los mensajes `ready` y `collision_risk`;
`-1` significa que nunca recibió ese canal. No se libera el movimiento usando
observaciones vencidas ni desactivando automáticamente un sensor que falta.

## Nubes densas y retraso del monitor

La recepción de nube conserva solo el frame más reciente. Su procesamiento usa un
callback group separado y un executor de dos hilos; LiDAR, habilitación y publicación
de seguridad permanecen en el grupo de control. Un mutex protege únicamente copias
de estado e intercambio de resultados, nunca el recorrido de los puntos. Las
respuestas de una generación de sensor deshabilitada se descartan.

El paquete `potential_fields` usa Release por defecto cuando no se especifica otro
tipo de compilación. Los datos mantienen el límite de antigüedad original, incluyendo
el tiempo de procesamiento. `Cloud processing slow` informa del tamaño y duración
si una nube tarda más de 100 ms. No se aumenta el timeout para compensar saturación.

La prueba de carga publica nubes VGA (640×480 puntos), confirma que se procesan
observaciones completas y comprueba que monitor y LiDAR siguen respondiendo. Si DDS
pierde muestras durante la carga, el controlador debe detenerse por datos vencidos;
la ausencia de datos no se publica como si fuera una colisión geométrica.

## Frenado preventivo y parada crítica

`collision_stop_margin` (0.08 m por defecto) añade espacio para reducir velocidad.
`collision_critical_margin` (0.02 m) define la zona que exige parada inmediata,
además del radio físico configurado en `collision_footprint_radius`. El perfil Takeshi usa el radio de la base HSR; en otras plataformas se configura su propia geometría y los márgenes por separado. Las ventanas laterales heredadas `*_collision_min_y/max_y`
se aceptan por compatibilidad; la geometría de colisión se obtiene de la huella
circular y sus márgenes.

El límite de velocidad resuelve la distancia de reacción más la distancia de frenado,
con la desaceleración configurada y la separación lateral de cada punto. Se requiere
más de `*_points_threshold` observaciones: se usa el punto de orden threshold+1
para no imponer un frenado por una única muestra aislada. Sin obstáculos soportados,
el límite es infinito; con datos vencidos es cero y el controlador permanece detenido.

La predicción local también desacelera durante el horizonte de frenado, en lugar de
extender la velocidad actual hasta el final del horizonte. Las verificaciones de mapa,
la parada crítica y la calidad de ruta siguen vigentes. `Route intersects map` imprime
la celda bloqueante en coordenadas del mapa, su ocupación, la pose del robot y el stamp
del mapa; `Critical obstacle` identifica el sensor y su cantidad de puntos de riesgo.


## Profundidad, continuidad y portabilidad

`cloud_min_range` y `cloud_max_range` limitan la distancia euclídea 3D desde el
origen físico del sensor, antes de evaluar la ventana de obstáculos de la base. En Takeshi se usa 0.60 m
como mínimo fiable de la Xtion, según las pruebas físicas; no equivale a 0.60 m
horizontales desde la base. La inclinación, altura y desplazamiento de cámara se
resuelven con TF completo. La nube debe mantener su frame y coordenadas coherentes.
`cloud_sensor_frame` permite indicar el frame físico de la cámara cuando el
PointCloud2 ya se publica transformado a otro frame, por ejemplo la base. Si está
vacío se usa el frame del mensaje.
Las ventanas de obstáculos de Takeshi cubren hasta 2.5 m; la de nube sigue
la vista de la cabeza y la de LiDAR se expresa en la base; no prometen ese alcance si el sensor o el entorno no lo observan.

`potential_fields/confirmed_obstacles` contiene representantes de celdas de nube
con al menos `cloud_min_points_per_cell` puntos válidos (2 por defecto);
`obstacle_evidence_resolution` configura el tamaño de celda de esta evidencia (0.05 m). El detector los integra sin volver a exigir sus
vecinos/crop ordinarios, conservando el sello original de observación y validando
rango temporal, TF, filtro propio y mapa. Esto evita que un obstáculo aceptado por
el frenado quede excluido del mapa por un criterio de vecindad diferente. Se
conservan los umbrales configurables del monitor para detener/reducir velocidad.

La evaluación a frecuencia de control usa una caché espacial acotada por celda,
compensada mediante odometría, y reevalúa la dirección solicitada incluso estando
parado. Una nube sin puntos válidos se diagnostica como `no_valid_depth`; nunca
cuenta como confirmación de espacio libre. Si vence la última observación válida,
se mantiene la parada por datos vencidos.

Si el monitor impone velocidad casi nula, `simple_move` informa
`waiting_for_obstacle_clearance`. Exige 0.3 s de límite recuperado antes de
reanudar; si persiste durante `local_avoidance_timeout_sec`, devuelve
`obstacle_speed_blocked` al coordinador. Los testigos de bloqueo evitan aceptar
otra vez un mapa que omita el obstáculo. El coordinador propaga las fases reales
del controlador; `following_path` sólo significa que se despachó una ruta.

La traslación y su aceleración conservan su dirección en odometría mientras gira
la base; aceleración y jerk se limitan por norma vectorial. Las ventanas angulares
pueden progresar por tiempo y recorrido para no estancarse cuando la traslación
es lenta. Los atajos limitan tanto costo integrado como costo máximo; el suavizado
respeta también el costo máximo de la ruta y comprueba los segmentos adyacentes
en cada ajuste para conservar separación en las esquinas.

`hsr_head_node` es un adaptador de hardware separado: interpreta feedback por
nombre de articulación y genera un único perfil de setpoints con velocidad y
aceleración limitadas. Su callback actualiza el destino sin reiniciar trayectorias
largas; al arrancar conserva la pose medida. `simple_move` publica objetivos cuando
cambian y limita el giro usando `head_max_pan_speed`.

El núcleo no depende de ese adaptador: otro robot puede remapear `cmd_vel`,
configurar `base_link_name`, `odom_name`/`odom_frame`, `map_frame`, radios, límites
y ventanas. `head_goal_topic` y `head_feedback_topic` permiten conectar otro
adaptador; un Robotino sin cabeza usa `move_head=false`. Sensores no instalados se
deshabilitan con `use_lidar`/`use_point_cloud`. La forma propia se configura con
cilindro/cajas o se elimina en un preprocesador específico antes de estos nodos.
No se consulta el modelo o nombre del robot para decidir el comportamiento.

Pruebas adicionales, sin hardware, con el adaptador de cabeza compilado:

```bash
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/navigation_depth_ros_logs \
  python3 src/Navigation/navigation_planner/navigation_start/test/test_depth_navigation.py
```

Cubren alcance desde cámara frontal/inclinada, evidencia fina rechazada por la
vecindad ordinaria, retención en la zona ciega y liberación por observación libre,
una persona detectada sólo por profundidad, parada ante profundidad completamente NaN y avance lateral con el nodo real de
cabeza y articulaciones simuladas. Se usan frames `world`, `nav_odom` y `robot_base` para comprobar que el núcleo
no depende de los nombres de Takeshi. Las pruebas físicas siguen siendo necesarias.


## Fuente de profundidad y cabeza al iniciar

Se restauró la configuración anterior de `hardware.launch.xml` y
`takeshi_hardware.launch.xml`, incluido `depth_image_proc/point_cloud_xyz_node`
para la profundidad del robot. Se retiraron el conversor añadido y el argumento
de selección de fuente que lo acompañaba. `pc_filter` no se usa.

La cabeza recibe su objetivo aunque el monitor esté esperando profundidad. La
base conserva velocidad cero en esa espera. Se elige el pan equivalente que
requiere la menor asistencia de base y, en empate, menor recorrido mecánico de
cabeza; se evita un giro grande innecesario al cruzar +/-pi.
`head_preview_angle_rad` (0.25 rad) permite anticipar curvas, limitado por el
umbral de parada y la velocidad de pan. No se amplió `sensor_timeout_sec` para
ocultar pérdida de nube.

## Validación

Pasaron 102 casos distintos: 16 C++ del planeador, 63 de controlador y coordinación,
9 de coherencia de mapas, 9 de profundidad y 5 escenarios con los mapas del
laboratorio. Las pruebas ROS usan dominios aislados de localhost. Se cubrieron
salidas curvas desde inflación sin invadir la capa física, replanificación desde
la pose alcanzada, reintentos acotados, metas alternativas alcanzables y el
límite estricto de distancia para declarar éxito aproximado. También se
verificaron la orientación posterior al resultado del controlador, las
oscilaciones pequeñas de posición/cabeza y los clientes directos de `nav_msgs/Path`.
Los casos de diagnóstico comprueban la activación en ejecución, el motivo de
parada antes de retornar, la correlación goal/movimiento y la diferencia entre
retención por rango ciego, caducidad y borrado por rayos libres. El display de
ruta se prueba con un suscriptor que se conecta después del despacho; recibe
la ruta retenida, no genera un movimiento duplicado y se limpia al cancelar.

La prueba local de rendimiento ejecutó 100 recuperaciones sobre un mapa de un
millón de celdas en aproximadamente 0.39 ms en este equipo. Es una medición
sintética del algoritmo local; no mide latencia del stack ni del robot.

El recorrido de esquina alcanzó 0.900 m/s con separación mínima simulada de
0.248 m; el caso de obstáculo dinámico mantuvo 0.286 m. Se compilaron los nodos
modificados y se validaron XML, sintaxis Python y parámetros compartidos de
llegada. Se corrigió un punto simulado situado justo en el borde de una celda
para que la prueba de mesa exija explícitamente un inicio libre en la capa
física y bloqueado en planificación. No se enviaron comandos al hardware.
Para cargar estos cambios hay que reiniciar navegación; falta la validación física.
