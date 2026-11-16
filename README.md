# Follow the Gap en F1Tenth: pista Catalunya

Controlador reactivo Follow the Gap para el simulador F1Tenth con ROS 2 Humble. Lo desarrollé para el Proyecto del Primer Parcial, primera parte, de Vehículos Autónomos en la ESPOL. El auto recorre la pista de Catalunya usando solamente el LIDAR, sin mapa ni trayectoria guardada.

Autor: David Alejandro Acosta Lara

## Resultado

- 10 vueltas consecutivas completadas, sin ninguna colisión.
- Tiempo por vuelta de alrededor de 51 s.
- Video de evidencia: https://youtu.be/JLJOk-7RoX0

En el video se ven las 10 vueltas con el contador de vueltas, el cronómetro y el mejor tiempo. El controlador los dibuja en RViz y también los imprime en la terminal.

## 1. Enfoque

El auto tiene un LIDAR de 1080 rayos que cubre 270°, de -135° a +135°, con 0.25° entre un rayo y el siguiente. Cada rayo dice a qué distancia está el obstáculo en esa dirección.

El controlador es reactivo. En cada escaneo decide solo con lo que ve en ese momento, sin memoria ni mapa. Por eso responde rápido y no depende de la pista, pero tampoco sabe en qué sentido va la carrera. Si se choca, puede quedar mirando hacia atrás y salir en sentido contrario.

Follow the Gap significa seguir el hueco. La idea es buscar el pasillo libre más grande que hay delante del auto y apuntar hacia él. Lo fui armando por etapas, y cada una arregló un problema que vi en la anterior:

- Etapa 1. Avanzar recto a velocidad fija. El auto choca al final de la recta.
- Etapa 2. Ir hacia el rayo más lejano. Choca en la primera curva, y los rayos empatados en 10 m lo confunden.
- Etapa 3. Burbuja y gap más largo. Choca en la misma curva, porque la burbuja solo protege el obstáculo más cercano y el peligroso era otro.
- Etapa 4. Ensanchar las esquinas. Da la vuelta completa, pero rozando las paredes.
- Etapa 5. Centrar el objetivo dentro del gap. Se separa de las paredes.
- Etapa 6. Velocidad variable con una regla de frenado. Baja el tiempo de más de 195 s a unos 51 s.
- Etapa 7. Contador de vueltas, cronómetro y texto en RViz.

### Qué hace el controlador en cada escaneo

- Preprocesar. Los valores inf y NaN se reemplazan por 10 m, todo se recorta a 10 m y se descartan los rayos que miran hacia atrás, quedándome con los de -90° a +90°. El ángulo de cada rayo sale de angle_min más el número del rayo por angle_increment.
- Ensanchar esquinas. Si dos rayos vecinos difieren en más de 0.4 m, ahí hay una esquina. El rayo lejano pasa rozándola, pero el auto tiene ancho y no cabe por ahí. Del lado lejano reemplazo n rayos por la distancia corta de la esquina. Primero calculo la mitad del auto más el margen de seguridad, y n es el ángulo que ocupa esa distancia vista desde la esquina, dividido entre angle_increment y redondeado hacia arriba. Esta etapa fue la que hizo que el auto diera la vuelta sin chocar.
- Burbuja. Busco el rayo más corto y pongo en 0 los rayos que quedan dentro de 0.45 m a su alrededor.
- Gap. Busco la secuencia más larga de rayos seguidos con distancia mayor a 1.5 m.
- Objetivo. Dentro del gap tomo el punto más lejano, y si hay varios empatados el del medio. Luego lo mezclo con el centro del gap, 65% punto lejano y 35% centro. Sin esta mezcla el auto apuntaba al borde del gap y se pegaba a las paredes.
- Comando. El volante es el ángulo del objetivo, limitado a 0.4189 rad. La velocidad sale de dos reglas y se usa la menor.

### Las dos reglas de velocidad

- Regla por el volante. Con el volante recto va a v_max y con el volante muy girado baja a v_min. En el medio se reparte en proporción. Calculo la proporción como el menor valor entre 1 y el ángulo de volante dividido entre steer_for_vmin, y la velocidad es v_max menos esa proporción por la diferencia entre v_max y v_min.
- Regla por la distancia al frente. Sale de la fórmula v² = v0² + 2·a·d. Tomo la distancia mínima en una ventana de 8° a cada lado del frente y le resto front_margin, sin dejar que quede negativa. La velocidad máxima segura es la raíz cuadrada de v_min² más 2 por brake_decel por esa distancia libre.

Con las dos reglas el auto acelera en las rectas, frena antes de las curvas y las toma despacio.

### Parámetros finales

- max_range: 10.0 m. Distancia máxima que se considera.
- car_width: 0.31 m. Ancho del auto.
- safety_margin: 0.15 m. Margen extra a cada lado.
- disparity_threshold: 0.4 m. Salto entre rayos que se considera esquina.
- bubble_radius: 0.45 m. Radio de la burbuja.
- gap_threshold: 1.5 m. Distancia mínima para que un rayo cuente como libre.
- center_weight: 0.35. Peso del centro del gap en el objetivo.
- v_max: 9.5 m/s. Velocidad con el volante recto.
- v_min: 5.0 m/s. Velocidad en las curvas más cerradas.
- steer_for_vmin: 0.30 rad. Ángulo de volante al que se usa v_min.
- brake_decel: 5.5 m/s². Frenado que asumo que puede hacer el auto.
- front_margin: 2.0 m. Distancia que quiero conservar al frente.
- front_cone_deg: 8°. Mitad del ancho de la ventana frontal.

Estos valores salieron de pruebas en el simulador. Cambiaba un parámetro por prueba y repetía las 10 vueltas hasta que ninguna chocara.

## 2. Contador de vueltas y cronómetro

El nodo también se suscribe a /ego_racecar/odom para saber dónde está el auto.

- En el primer mensaje guarda la posición inicial y hacia dónde mira el auto. Esa es la línea de salida, una línea perpendicular a esa dirección que pasa por ese punto.
- En cada mensaje calcula qué tan adelante o atrás de la línea está el auto. Es el producto punto entre el desplazamiento desde la posición inicial y la dirección inicial.
- Cuenta una vuelta cuando ese valor pasa de negativo a positivo, es decir, cuando el auto cruza hacia adelante. Además exige estar a menos de 3 m de la línea y haber recorrido más de 30 m desde la vuelta anterior, para no contar vueltas falsas.
- Al contar una vuelta mide el tiempo con el reloj del nodo, actualiza el mejor tiempo y reinicia el cronómetro.

El estado se muestra de dos formas:

- En la terminal, el log imprime una línea por cada vuelta completada y, en el log continuo, el número de vueltas y el mejor tiempo.
- En RViz, el nodo publica un marcador de texto en /lap_info, de tipo MarkerArray, pegado al auto. Muestra las vueltas, el tiempo de la vuelta actual, la última vuelta y la mejor.

## 3. Estructura del código

```
src/controllers/
    setup.py                       registra el ejecutable mi_gap
    controllers/
        mi_gap_node.py             todo el controlador
src/f1tenth_gym_ros/
    config/sim.yaml                mapa y pose inicial
    maps/Catalunya_map.png         imagen de la pista
    maps/Catalunya_map.yaml        datos del mapa
```

Todo el controlador está en la clase MiGap de mi_gap_node.py:

- preprocess. Limpia el escaneo y se queda con los rayos de -90° a +90°.
- extend_disparities. Ensancha las esquinas.
- apply_bubble. Pone en 0 los rayos cercanos al punto más cercano.
- find_max_gap. Encuentra el gap más largo.
- find_best_point. Elige el objetivo, mezclando el punto lejano con el centro del gap.
- scan_cb. Se ejecuta en cada escaneo, encadena todos los pasos y publica en /drive.
- odom_cb. Detecta el cruce de la línea de salida, cuenta las vueltas y mide los tiempos.
- status_cb. Publica el texto de /lap_info para RViz.

Tópicos que usa:

- Se suscribe a /scan, de tipo LaserScan.
- Se suscribe a /ego_racecar/odom, de tipo Odometry.
- Publica en /drive, de tipo AckermannDriveStamped.
- Publica en /lap_info, de tipo MarkerArray.

## 4. Cómo ejecutarlo

Se necesita ROS 2 Humble y el simulador f1tenth_gym_ros, instalados como en el tutorial del curso. En mi caso los corro dentro de un contenedor Docker con Ubuntu 22.04 y Humble. Este repositorio ya incluye el mapa de Catalunya.

### Preparar

- Clonar el repositorio y compilar:

```bash
git clone https://github.com/Daalacos3005/f1tenth-follow-the-gap.git ~/F1Tenth-Repository
cd ~/F1Tenth-Repository
colcon build --symlink-install
source install/setup.bash
```

- Revisar el mapa. En src/f1tenth_gym_ros/config/sim.yaml, map_path debe apuntar a la ruta real de Catalunya_map en tu computadora y map_img_ext debe ser .png. La pose inicial que usé es sx 0.0, sy 0.0 y stheta -2.14.

### Correr

- En la primera terminal, lanzar el simulador:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

- En una segunda terminal, correr el controlador:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers mi_gap
```

- Para ver el contador en RViz, usar Add, luego By topic, elegir /lap_info y agregar el MarkerArray. El texto amarillo aparece junto al auto.
- Para detener el controlador, Ctrl+C. El auto se frena antes de cerrar el nodo.

## 5. Limitaciones

- Al ser reactivo no tiene memoria, así que después de un choque puede salir en sentido contrario.
- Con max_range en 10 m no se ve lo que está más lejos. Subirlo a 15 m podría ayudar a frenar antes, pero no lo probé con las 10 vueltas completas.
- Las velocidades están cerca del límite. Si cambia la carga de la computadora o la latencia, podría no completar las 10 vueltas sin chocar.

## 6. Uso de herramientas de IA

Usé Claude, de Anthropic, como asistente durante el proyecto. Me ayudó a diagnosticar errores y usar Github y mejorar ciertas partes que me estancaba con el controlador mi_gap_node.py
