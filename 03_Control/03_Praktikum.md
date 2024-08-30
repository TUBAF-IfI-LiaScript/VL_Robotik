<!--

author:   Sebastian Zug & André Dietrich & Gero Licht
email:    sebastian.zug@informatik.tu-freiberg.de & andre.dietrich@informatik.tu-freiberg.de & gero.licht@informatik.tu-freiberg.de
version:  1.0.1
language: de
narrator: Deutsch Female

import: https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md
        https://raw.githubusercontent.com/LiaScript/CodeRunner/master/README.md

comment:  Implementierung einer Zustandsmaschine für den Roboter

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_Robotik/main/03_Control/03_Praktikum.md)

# Verhaltensmuster \
 mit Automaten
 
                          {{0-1}}
*******************************************************************************

Wir gehen in unserer heutigen Praktikumsaufgabe auf die Implementierung von Verhaltensmustern mit Hilfe von Zustandsautomaten ein. Damit finden wir uns auf der Ausführungsebene der Robotik wieder.

```ascii
                    Statusmeldungen 
     Nutzereingaben  ^                                       
                 |   |
Befehle          v   |
            +-----------------------+
            | Handlungsplanung      |  "$Strategie   $"
            +-----------------------+
                 |   ^     | | |        Folge von Aktionen     
                 v   |     v v v
            +-----------------------+
            | Ausführung            |  "$Taktik$    "           
            +-----------------------+
                     ^      | | |       Geplante Trajektorie,
Status               |      v v v       Verhalten
            +-----------------------+
            | Reaktive Überwachung  |  "$Ausführung$        "
            +-----------------------+
Sensordaten-    ^ ^ ^        | | |      Steuerbefehle an den 
erfassung       | | |        v v v      Aktuator 
            +----------+ +----------+
            | Sensoren | | Aktoren  |                               
            +----------+ +----------+
                  ^           |
                  |           v      
            .-----------------------.
            | Umgebung              |
            .-----------------------.                                              .
```

*******************************************************************************

                          {{1-2}}
*******************************************************************************

**Beispiel**

![](https://www.plantuml.com/plantuml/png/TPAzQiCm58LtFSMT2ta13sq8U6bfbveEBHsK-YGMw0Ukf53wz5LkZ6M2l5tE-Uw7P6iVa-QKdLLV3z-qsptIYpwTW07up4SmJ89NgX5IDwCKHZ2LZlwMihKWzY7eA4OPn7BWa2ZDkSGpsnlmWRNWTQPE2ZhUcbh8phiB8Kq6lK_nPu8PipyhrKgfMs1okgJ-QGDSUr_DhnpXMM-p0wm5VmHUVQet6A_KlQKUu1hSrKsLKYtxBQDB3HkON0DyCU2hPncfgxD0vILi_h_L5lMHN8wHokqzhAKpfYBUmu_PsNy0)

*******************************************************************************

## Wiederholung Zustandsmaschine 

                          {{0-1}}
*******************************************************************************

> Ein endlicher Automat (auch Zustandsmaschine, Zustandsautoma, _finite state machine_ (FSM)) ist ein Modell eines Verhaltens, bestehend aus Zuständen, Zustandsübergängen und Aktionen. 

Eine Zustandsmaschine ist gekennzeichnet durch:

+ beliebige (jedoch endliche) Menge von Zuständen
+ Zustandsübergänge in jedem Takt abhängig von Eingangssignalen
+ Ausgangssignale werden durch ein Schaltnetz generiert

Mathematisch kann ein Deterministischer Endlicher Automat als Tupel $A = (Q, \Sigma, \delta, q_0, F)$ dargestellt werden.

+ Q ist eine endliche Zustandsmenge.
+ $\Sigma$ ist das endliche Eingabealphabet, also die Menge erlaubter Eingabesymbole.
+ $\delta : Q \times \Sigma \rightarrow Q$ ist die Übergangsfunktion (oder Transitionsfunktion). Sie ordnet jedem Paar bestehend aus einem Zustand $q\in Q$ und einem Eingabesymbol $a\in \Sigma$ einen Nachfolgezustand $p\in Q$ zu.
+ $q\in Q$ ist der Startzustand (auch Anfangszustand oder Initalzustand).
+ $F\subseteq Q$ ist die Menge der akzeptierenden Zustände, die sogenannten Finalzustände (oder Endzustände).


### Beispiel I

__Interaktion eines Roboters mit einer Tür__

Darstellung als Graph

![StateMaschine](https://www.plantuml.com/plantuml/png/ZOqnJiGm44NxdEBBFITO6MrIKz565ZBs68zmPhEEtO0u2xVW0haOmH4eLD2CzDz_cdzUYisbJgdp_9Jj715OkD33nhSxMHTP2AyT06Ghpvwplhi_jImNJnYNQ6U2ndnXAJ0dNFcdtKljEBj4fk5-JFQHabOlLPdzp_2DR0rVhC4hu-OM_MxqQ04OqLHegCN5nBI-aoYOepRyqjQF4EJW3Fzn_fvxwyualaxf4m00)

Darstellung in einer Übergangstabelle

|                     | "Öffnen"            | "Schließen"               |
|---------------------|---------------------|---------------------------|
| Zustand offen       | offen (unverändert) | geschlossen               |
| Zustand geschlossen | offen               | geschlossen (unverändert) |

> Bei einem stochastischen Automaten wäre der Erfolg einer Aktion nicht deterministisch, sondern würde von einer Wahrscheinlichkeit abhängen. _Wenn "öffnen" ausgeführt wird, ist dies zu 95 Prozent erfolgreich und führt zu einem offenen Zustand._

### Beispiel II

__Statisches Umfahrung eines Hindernisses__

Der Roboter soll in der Lage sein, auf ein Hindernis mit einem vorprogrammierten Bewegungsmuster auszuweichen. Dazu soll eine Zustandsmaschine implementiert werden, die folgende Zustände kennt:

1. __Geradeausfahrt__: Der Roboter fährt geradeaus. Später wird dies durch unsere Spurverfolgung ersetzt.
2. __DrehungRechts__: Der Roboter dreht sich um 90 Grad nach rechts.
2. __DrehungLinks__: Der Roboter dreht sich um 90 Grad nach links.
3. __Streckenfahrt__: Der Roboter fährt eine Strecke (z.B. 30cm) nach vorne.

  
                              {{0-1}}
*******************************************************************************


```ascii
                                ^
                                | Fortsetzung der Geradeausfahrt
                 O--------------O                                                   
                 |                                                   
                 |       +-----------+                                            
                 |       |           |                                 
  Streckenfahrt  |       | Hindernis |                                            
     30cm        |       |           |                                            
                 |       +-----------+                                            
                 |                                                   
                 |   Streckenfahrt                                                    
                 |       30cm      Links-                               
                 O--------------O Drehung 90°                                   
             Rechts-            |                                       
             drehung            |                                       
                              Robot                                         
                                                                                               .
```

Die Zustandsmaschine könnte dann wie folgt aussehen:

```text @plantUML
@startuml
Geradeausfahrt : ""v_l=100, v_r=100""
DrehungLinks :   ""v_l=-50, v_r=50""
state Rechtsabbiegen {
  Streckenfahrt :  ""v_l=50, v_r=50""\n""count++""
  DrehungRechts :  ""v_l=50, v_r=-50""
}


note right of Rechtsabbiegen
""count=0""
end note

[*] --> Geradeausfahrt
Geradeausfahrt -> Geradeausfahrt: //Kein Hindernis//
Geradeausfahrt -> DrehungLinks : //Hindernis erkannt//
Streckenfahrt -> DrehungRechts: //30cm erreicht//\n//count kleiner 3//
DrehungLinks -> Streckenfahrt: //90 Grad erreicht//

DrehungRechts -> Streckenfahrt: //90 Grad erreicht//
Streckenfahrt --> DrehungLinks: //30cm erreicht//\n//count größer 2//
DrehungLinks --> Geradeausfahrt: //90 Grad erreicht//
@enduml
```

> Welchen Fehler sehen Sie in dem Zustandsgraphen? Welche weiteren Zustände sollten abgedeckt werden?

*******************************************************************************

                              {{1-2}}
*******************************************************************************

```text @plantUML
@startuml
Geradeausfahrt : ""v_l=100, v_r=100""\n""timeout=nan""
DrehungLinks :   ""v_l=-50, v_r=50""\n""loop++""\n""timeout=10s""
state Rechtsabbiegen {
  Streckenfahrt :  ""v_l=50, v_r=50""\n""count++""\n""timeout=20s""
  DrehungRechts :  ""v_l=50, v_r=-50""\n""timeout=10s""
  ErrorState: 
}
Error: ""v_l=0, v_r=0""

note right of Rechtsabbiegen
""count=0""
end note

note right of DrehungLinks
""loop=0""
end note

[*] --> Geradeausfahrt
Geradeausfahrt -> Geradeausfahrt: //Kein Hindernis//
Geradeausfahrt -> DrehungLinks : //Hindernis erkannt//
Streckenfahrt -> DrehungRechts: //30cm erreicht//\n//count kleiner 3//
DrehungLinks -> Streckenfahrt: //90 Grad erreicht//\n//loop == 0//

DrehungRechts -> Streckenfahrt: //90 Grad erreicht//
Streckenfahrt --> DrehungLinks: //30cm erreicht//\n//count größer 2//
DrehungLinks --> Geradeausfahrt: //90 Grad erreicht//\n//loop > 0//

DrehungLinks --> Error: //Timeout//
DrehungRechts --> ErrorState: //Timeout//
Streckenfahrt --> ErrorState: //Timeout//
ErrorState --> Error
@enduml
```

*******************************************************************************

## Umsetzung in Python

+ intuitive Implemententierung 

  ```python IntuitivSM.py
  # Beispiel einer Zustandsmaschine mit if else Anweisungen
  state = "Geradeausfahrt"
  count = 0
  timeout = 0

  while True:
      if state == "Geradeausfahrt":
          print("Geradeausfahrt")
          if Hindernis_erkannt:
              state = "DrehungLinks"
      elif state == "DrehungLinks":
          print("DrehungLinks")
          if 90 Grad erreicht:
              state = "Streckenfahrt"
      elif state == "Streckenfahrt":
          print("Streckenfahrt")
          count += 1
          if count > 2:
              state = "Geradeausfahrt"
          elif 30cm erreicht:
              state = "DrehungRechts"
      elif state == "DrehungRechts":
          print("DrehungRechts")
          if 90 Grad erreicht:
              state = "Streckenfahrt"
  ```

+ objektorientierte Umsetzung (Entwurfsmuster State Machine) vgl. zum Beispiel https://medium.com/@amirm.lavasani/design-patterns-in-python-state-8916b2f65f69

+ Implementierung mit einer Bibliothek (z.B. transitions) vgl. https://github.com/pytransitions/transitions 

##  Probleme bei der Verwendung von Zustandsmaschinen

1. Komplexität bei großen Systemen:

    Zustandsexplosion: Wenn das System viele Zustände und Übergänge hat, kann die Zustandsmaschine schnell sehr komplex und schwer zu verwalten werden. Dies führt zu einer sogenannten „Zustandsexplosion“, bei der die Anzahl der Zustände exponentiell mit der Anzahl der Variablen und möglichen Werte wächst.
    
    Schwierige Wartung: Große und komplexe Zustandsmaschinen sind schwer zu verstehen, zu dokumentieren und zu warten. Kleine Änderungen im System können dazu führen, dass viele Zustände und Übergänge angepasst werden müssen.

2. Schwierigkeiten bei nicht-diskreten Zuständen:

    Kontinuierliche Systeme: Zustandsmaschinen sind in erster Linie für diskrete Zustände geeignet. Systeme, die kontinuierliche Zustandsübergänge oder viele Zwischenschritte aufweisen, sind schwieriger zu modellieren und könnten einen übermäßigen Einsatz von Zuständen und Übergängen erfordern.

3. Eingeschränkte Flexibilität:

    Statische Struktur: Zustandsmaschinen sind im Allgemeinen statisch und schwer anzupassen, wenn sich die Systemanforderungen ändern. Jede Änderung erfordert eine explizite Anpassung der Zustands- und Übergangsdefinitionen.
    Schwierig bei parallelen Prozessen: Die Modellierung paralleler oder konkurrierender Prozesse kann komplex sein, da Zustandsmaschinen in der Regel linear und sequentiell arbeiten. Dies kann die Modellierung solcher Systeme unnötig kompliziert machen.

4. Übermäßige Abstraktion:

    Verlust an Detailtiefe: Zustandsmaschinen abstrahieren die Realität stark. Bei komplexen Systemen kann es schwierig sein, alle relevanten Details in der Zustandsmaschine zu erfassen, was zu einer ungenauen oder fehlerhaften Modellierung führen kann.

5. Leistungsprobleme:

    Laufzeitperformance: In Systemen mit vielen Zuständen und Übergängen kann die Laufzeit der Zustandsmaschine beeinträchtigt werden, insbesondere wenn viele Bedingungen bei jedem Übergang überprüft werden müssen.
    Speicherverbrauch: Komplexe Zustandsmaschinen können viel Speicher benötigen, was in eingebetteten Systemen oder ressourcenbeschränkten Umgebungen problematisch sein kann.

6. Schwierigkeit beim Testen:

    Komplexe Testfälle: Durch die hohe Anzahl möglicher Zustände und Übergänge kann das Testen eines Systems, das durch eine Zustandsmaschine modelliert wurde, sehr aufwendig sein. Es muss sichergestellt werden, dass alle möglichen Übergänge und Randfälle getestet werden.

> Wir werden in einer kommenden Veranstaltung darüber sprechen, welche Alternativen es gibt, um diese Probleme zu umgehen.

## Aufgabe

Der Roboter folgt einer Linie (Auswertung der Kamera). Sobald er auf ein Hindernis trifft dreht er sich um 180 Grad und fährt in die entgegengesetzte Richtung. Am Ende wollen wir eine Pendelbewegung zwischen zwei Hindernissen sehen.

1. __Linienverfolgung__: Der Roboter fährt geradeaus. Später wird dies durch unsere Spurverfolgung ersetzt.
2. __Drehung180__: Der Roboter dreht sich um die eigene Achse, bis er wieder auf die Linie trifft.
3. __Timeout__: Der Roboter stoppt, wenn er 10 Sekunden lang keine Linie mehr sieht.

![](https://www.plantuml.com/plantuml/png/RP1D2u9048Rl-ok6FKOodWL3eg13nsuH8ZZgeeumxjhVRumEbNlRZ_TvzjqfPb-93-DWheuTyKBIJ4CRk8KCp4ceqwLUzRFk0SnPP98Ch5tA8K_ti9IIfzOQO-wx1oJ90VuzMt4af7B4OzKnD48ECGfzjrdp5ySr2RiPIFgAsIF-Deuyj7BlCtWwKhgb9gY5DwYodxyYtjn8K_3HMl5SGLphK5y0)

**Hinweise**

1. Welche Funktionalität sollte der Knoten aufweisen?

+ Linienerkennung auf der Basis der Kameradaten 
+ Hinderniserkennung auf der Basis der Laserscannerdaten
+ Drehung um 180 Grad (Zeitgesteuert oder mit den Odometriedaten, Abbruch, wenn die Linie erreicht wird)
+ Timeout
+ Zustandslogik


2. Wie kann ich den Timeout in Python realisieren?

```python Watchdog
from threading import Timer
import sys
import time

class Watchdog(Exception):
    def __init__(self, timeout, userHandler=None):  # timeout in seconds
        self.timeout = timeout
        self.handler = userHandler if userHandler is not None else self.defaultHandler
        self.timer = Timer(self.timeout, self.handler)
        print("Timeout set to", self.timeout, "seconds")
        self.timer.start()

    def reset(self):
        self.timer.cancel()
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()

    def stop(self):
        self.timer.cancel()

    def defaultHandler(self):
        raise self

def myHandler():
  print("Timeout reached!")
  sys.exit()

watchdog = Watchdog(3, myHandler)

time.sleep(2)
watchdog.reset()
print("Watchdog was reseted after 2 seconds, we have to wait 3 seconds now")
```
@LIA.python3