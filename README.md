# uni

nel terminale di compilazione:
source /opt/ros/humble/setup.bash

colcon build

nel terminale di esecuzione:
source /opt/ros/humble/setup.bash

source install/local_setup.bash

ros2 run luxonis "nome nodo"

nomi nodo possibili:
1) applicant = invia il messaggio di scattare la foto
2) publisher = quando riceve la richiesta condivide frame RGB, depth e informazioni videocamera
3) subscriberRGB = ascolta per frame RGB e li visualizza
4) subscriberDepth = ascolta per frame Depth e li visualizza (non ancora fatto)ù
