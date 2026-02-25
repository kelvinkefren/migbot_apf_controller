# gazebo_scenario_v2_split_noclones

Remove todos os modelos *_clone* (que não existem no seu mundo Gazebo).

Modelos usados por padrão:
  vegetation1_buoy, vegetation3_buoy, branche3_buoy, trunk1_buoy, branche1_buoy

## Como rodar
rosrun <pkg> gazebo_scenario_A1_single_static.py

## Se seus nomes forem diferentes
Você pode sobrescrever no rosrun:
  _obstacle_names:="['nameA','nameB']"  _obstacle_radius:="[0.5,1.0]"
