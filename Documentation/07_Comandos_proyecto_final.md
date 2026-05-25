# Comandos del Proyecto de Robótica

---

## Self-control

```
ros2 launch my_robot_control my_robot_selfcontrol_holonomic.launch.xml time_to_stop:=20.0
```

---

## Wall-follower

### Wall-follower diferencial

```
ros2 launch my_robot_control my_robot_wallfollower_differential.launch.xml time_to_stop:=40.0
```

### Wall-follower holonómico

```
ros2 launch my_robot_control my_robot_wallfollower_holonomic.launch.xml time_to_stop:=40.0
```

---

## Mapping

- Poner como coordenada 0 el robot POSE actual en el mapa real:

>IMPORTANTE: Pon el 0 en el mismo lugar de siempre (la cruz central del extremo donde trabajábamos) para facilitar los siguientes pasos.

```
ros2 topic pub --once /reset_odom std_msgs/msg/Bool "{data: true}"
```

- Iniciar el cartógrafo:
```
ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=false
```

- Moverse por el mapa para cartografiarlo:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Guardar el mapa como "mapa_proyecto_final":
```
cd src/Navigation_Projects/my_robot_navigation2/map/
ros2 run nav2_map_server map_saver_cli -f mapa_proyecto_final
```

- Ver el mapa:
```
sudo apt update
sudo apt install imagemagick
```
```
display mapa_proyecto_final.pgm
```

---

## Navigation

### Navigation manual

Elige uno de estos, preferiblemente el primero si funciona:

- Con el mapa generado al momento del proyecto final:
```
ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=mapa_proyecto_final.yaml params_file:=rubot_real_lidar.yaml
```

- Con el mapa generado hace unas cuantas clases:
```
ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=grup1_map.yaml params_file:=rubot_real_lidar.yaml
```


Uso del RVIZ para navegar:

- Botón "2D-Pose estimate" para ubicar el robot en el mapa.


(Yo pasaría de esta sección de abajo, no creo que sea relevante en el todo e iba una poco raro:)
- Navigate on the MAP with Nav2:
    - Selecty 1 target point.
    - Select multiple waypoints with "Waypoint/Nav through Poses Mode" option:
        - Select different `Nav2 Goal` points in RVIZ2.
        - Choose `Start Waypoint Following` option to follow the exact `Nav2 Goal` selected points.
        - Choose `Start Nav Through Poses` option to follow an optimized unique trajectory following the different `Nav2 Goal` selected points.

### Navigation programático

Elige uno de estos, preferiblemente el primero si funciona:

- Con el mapa generado al momento del proyecto final:
```
ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=mapa_proyecto_final.yaml params_file:=rubot_real_lidar.yaml
```

- Con el mapa generado hace unas cuantas clases:
```
ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=grup1_map.yaml params_file:=rubot_real_lidar.yaml
```

Navegación programática:

```
ros2 launch my_robot_nav_control nav_waypoints.launch.py wp_file:=waypoints_sw_clase.yaml
```

---

## YOLO

- Para ver la clasificación de nuestro modelo mediante la cámara del robot:
```
cd "Documentation/Files/YOLO_model_generation"
python3 4_classify_camera_from_topic.py
```


---

# Proyecto final

## Robot Navigation

Elige uno de estos, preferiblemente el primero si funciona:

- Con el mapa generado al momento del proyecto final:
```
ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=mapa_proyecto_final.yaml params_file:=rubot_real_lidar.yaml
```

- Con el mapa generado hace unas cuantas clases:
```
ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=grup1_map.yaml params_file:=rubot_real_lidar.yaml
```

## Signal detection
```
ros2 launch my_robot_ai_identification rubot_identification_yolo.launch.py yolo_params_file:=yolo_params_real.yaml signs_file:=sign_positions_real.yaml
```

## Todo junto
Elige uno de estos, preferiblemente el primero si funciona:

- Con el mapa generado al momento del proyecto final:
```
ros2 launch my_robot_ai_identification ai_navigation.launch.py \
  map_file:=mapa_proyecto_final.yaml \
  params_file:=rubot_real_lidar.yaml \
  use_sim_time:=false \
  yolo_params:=yolo_params_real.yaml \
  nav_params:=yolo_targets_real.yaml \
  signs_file:=sign_positions_real.yaml \
  nav_start_delay:=2.0
```

- Con el mapa generado hace unas cuantas clases:
```
ros2 launch my_robot_ai_identification ai_navigation.launch.py \
  map_file:=grup1_map.yaml \
  params_file:=rubot_real_lidar.yaml \
  use_sim_time:=false \
  yolo_params:=yolo_params_real.yaml \
  nav_params:=yolo_targets_real.yaml \
  signs_file:=sign_positions_real.yaml \
  nav_start_delay:=2.0
```