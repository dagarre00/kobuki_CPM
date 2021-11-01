# Tutorial para usar el Kobuki Turtlebot2 en Ubuntu 18.04 con ROS melodic. 

Para usar este repositorio seguir los pasos:

1. Clonar el repositorio y actualizar los submodulos

    ```bash
    git clone https://gitlab.com/davinsony/kobuki.git
    git submodule update --init --recursive
    ```

2. Realizar las instalaciones con [`apt-get`](#apt-get)
3. Installar la libreria [`libfreenect`](#libfreenect)
4. Compilar el workspace de este repositorio

    ```bash
    catkin_make
    ```

5. Probar que todo funciona haciendo `source` y corriendo el comando:

    ```bash
    roslaunch plataforma test.launch
    ```

## 1. Instalación

Algunos de los paquetes para el Kuboki Turtlebot fueron descontinuados desde la version de ROS _kinetic_ en la version _melodic_ todavia podemos usar los paquetes principales para el Kobuki.

Para dichos paquetes algunos pueden ser instalados con `apt-get` de la siguiente forma:

### apt-get

```bash
sudo apt-get install ros-melodic-xacro 
sudo apt-get install ros-melodic-kobuki-msgs
sudo apt-get install ros-melodic-kobuki-core
sudo apt-get install ros-melodic-yujin-ocs
sudo apt-get install ros-melodic-ecl-exceptions
sudo apt-get install ros-melodic-ecl-threads
sudo apt-get install ros-melodic-ecl-streams
```

Otros paquetes deben ser compilados, dichos paquetes fueron agregados ya a este repositorio usando los submodulos de git:

```bash
# No es necesario correr estos comandos son solo de ejemplo. 

git submodule add -b melodic https://github.com/turtlebot/turtlebot.git
git submodule add -b master  https://github.com/yujinrobot/kobuki_description.git
git submodule add -b melodic https://github.com/yujinrobot/kobuki.git
git submodule add https://github.com/ros-drivers/freenect_stack.git
```

Si deseamos hacer un workspace por tu cuenta, usa los siguientes comandos: 

```bash
mkdir turtlebot-ws
mkdir turtlebot-ws/src
cd turtlebot-ws/src

git clone -b melodic https://github.com/turtlebot/turtlebot.git
git clone -b master  https://github.com/yujinrobot/kobuki_description.git
git clone -b melodic https://github.com/yujinrobot/kobuki.git

cd ..
touch .catkin_workspace
catkin_make
```

## 2. Usar el Kobuki 

Para usar el robot debemos exportar las variables del ambiente, para indicarle a los paquetes de ROS cual es el robot que estamos usando.

```bash
export TURTLEBOT_BASE=kobuki
export TURTLEBOT_STACKS=hexagons
export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_SIMULATION=false
export TURTLEBOT_SERIAL_PORT=/dev/ttyUSB0
```

Modificar el `SERIAL_PORT` de acuerdo en donde te aparezca el robot conectado. Despues de usar el comando `source`. Podremos inicializar el robot usando el siguiente comando: 

```bash
roslaunch turtlebot_bringup minimal.launch
```

Para controlarlo con el teclado usaremos en otro terminal el comando: 

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

podemos encontrar más informacón sobre el robot y los paquetes instalados en los siguientes enlaces:

- [TurtleBot2](https://www.turtlebot.com/turtlebot2/)
- [ClearPath Robotics TurtleBot2](https://www.clearpathrobotics.com/assets/guides/kinetic/turtlebot/index.html). 

## 3. Usando el Kinect 

Para usar el Kinect del Robot Kobuki, debemos instalar los controladores/drivers, esto lo lograremos compilando el código fuentes e instalando los drivers compilados. Lo logramos siguiendo los pasos presentados (encontrados [aquí](https://programmerclick.com/article/42712010003/)): 

### libfreenect
```bash
sudo apt-get update
sudo apt-get install cmake build-essential libusb-1.0-0-dev
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
mkdir build
cd build
cmake -L .. # -L lists all the project options
make
sudo make install
```

También deberemos descargar y compilar el paquete para ROS `freenect_stack`, como se muestra a continuación:

```bash
cd catkin_ws/src/
git clone https://github.com/ros-drivers/freenect_stack.git
cd catkin_ws/
catkin_make
```

Para probar que podemos usar el Kinect, usaremos el comando (despues de hacer el `source`): 

```bash
roslaunch freenect_launch freenect.launch 
# freenect es el paquete de tiempo de ejecución oficial de kinect
```

y en otro terminal (debemos tener el paquete image_view instalado): 

```bash 
rosrun image_view image_view image:=/camera/rgb/image_raw
```

