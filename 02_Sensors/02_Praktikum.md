<!--

author:   Sebastian Zug & André Dietrich & Gero Licht
email:    sebastian.zug@informatik.tu-freiberg.de & andre.dietrich@informatik.tu-freiberg.de & gero.licht@informatik.tu-freiberg.de
version:  1.0.1
language: de
narrator: Deutsch Female

comment:  Praktische Einführung in ROS Tools und Konzepte

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_Robotik/main/02_Sensors/02_Praktikum.md)

## Aufgabe 1: Laserscanner 

> Aufgabe P1: Schreiben Sie ein Programm, dass bei der Unterschreitung einer Mindestdistanz eine Warnung auf der Konsole ausgibt. (30min)

Zur Erinnerung: Wie sieht eine Laserscan-Nachricht aus?

```bash
std_msgs/msg/Header header
float angle_min
float angle_max
float angle_increment
float time_increment
float scan_time
float range_min
float range_max
float[] ranges
float[] intensities
```

Wie setze ich einen minimal subscriber um?

https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

## Aufgabe 2: Motorsteuerung 

> Aufgabe P2: Finden Sie dazu zunächst heraus, wie die Nachricht der Motorsteuerung aussieht. Schreiben Sie einen Publisher, der langsam die Geschwindigkeit des Roboters erhöht und absenkt. (30min)

### Aufgabe 3: Handfolger 

> Aufgabe P3: Schreiben Sie ein Programm, dass den Roboter Ihrer Hand folgen lässt. (20min)

## Hilfestellung

Legen Sie sich zunächt einen Workspace an und erstellen Sie ein Package. 

```bash
mkdir -p ~/00_sensors/src
cd ~/00_sensors/src
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs --node-name lasermonitor 02_package
```

```bash
.
└── 02_package
    ├── 02_package
    │   ├── lasermonitor.py
    │   └── __init__.py
    ├── package.xml
    ├── resource
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
```

Ergänzen Sie die `package.xml` um die Abhängigkeiten:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Fügen Sin in `setup.py` die `entry_points` hinzu:

```python
entry_points={
    'console_scripts': [
        'lasermonitor = 02_package.lasermonitor:main',
    ],
},
```

Ergänzen Sie diese Struktur dann um die 2 Python Dateien: `motorcontrol.py` und `handfollow.py` und implementieren Sie die Funktionalität.

