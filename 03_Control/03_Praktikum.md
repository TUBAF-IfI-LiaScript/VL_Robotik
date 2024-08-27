<!--

author:   Sebastian Zug & André Dietrich & Gero Licht
email:    sebastian.zug@informatik.tu-freiberg.de & andre.dietrich@informatik.tu-freiberg.de & gero.licht@informatik.tu-freiberg.de
version:  1.0.1
language: de
narrator: Deutsch Female

import: https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md

comment:  Implementierung einer Zustandsmaschine für den Roboter

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_Robotik/main/03_Control/02_Praktikum.md)

# Sequenzen von Verhalten



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


*******************************************************************************

                          {{1-2}}
*******************************************************************************

>Beispiel I:  __Interaktion eines Roboters mit einer Tür__

Darstellung als Graph

![StateMaschine](https://www.plantuml.com/plantuml/png/ZOqnJiGm44NxdEBBFITO6MrIKz565ZBs68zmPhEEtO0u2xVW0haOmH4eLD2CzDz_cdzUYisbJgdp_9Jj715OkD33nhSxMHTP2AyT06Ghpvwplhi_jImNJnYNQ6U2ndnXAJ0dNFcdtKljEBj4fk5-JFQHabOlLPdzp_2DR0rVhC4hu-OM_MxqQ04OqLHegCN5nBI-aoYOepRyqjQF4EJW3Fzn_fvxwyualaxf4m00)

Darstellung in einer Übergangstabelle

|                     | "Öffnen"            | "Schließen"               |
|---------------------|---------------------|---------------------------|
| Zustand offen       | offen (unverändert) | geschlossen               |
| Zustand geschlossen | offen               | geschlossen (unverändert) |

> Bei einem stochastischen Automaten wäre der Erfolg einer Aktion nicht deterministisch, sondern würde von einer Wahrscheinlichkeit abhängen. _Wenn "öffnen" ausgeführt wird, ist dies zu 95 Prozent erfolgreich und führt zu einem offenen Zustand._

*******************************************************************************

                          {{2-3}}
*******************************************************************************

>Beispiel II:  


*******************************************************************************

## Umsetzung in Python 



## Aufgabe

Der Roboter soll in der Lage sein, einem Hindernis auszuweichen. Dazu soll eine Zustandsmaschine implementiert werden, die folgende Zustände kennt:

1. __Geradeausfahrt__: Der Roboter fährt geradeaus. Später wird dies durch unsere Spurverfolgung ersetzt.
2. __DrehungRechts__: Der Roboter dreht sich um 90 Grad nach rechts.
2. __DrehungLinks__: Der Roboter dreht sich um 90 Grad nach links.
3. __Streckenfahrt__: Der Roboter fährt eine Strecke (z.B. 30cm) nach vorne.

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
                                                                    
```

Die Zustandsmaschine soll folgende Übergänge kennen:

![](https://www.plantuml.com/plantuml/png/fP2nJiCm48PtFyKDdYYH35K78XLrG2e34umKLMvo95QSGphRPK0y2s_12_J5E9eMoGKf0vk7x__y_xS8qN9ClLTBP5kZJU7PjXoX1BrT-_c5CUUmNVFtGwiRnZPHS-Ue2qB0m1Ip0pCpUaMwUaaKvqABEYBSOzN6O3SRXmqIl2c0XyXOTKZ7dmRD7vR9H0j--7KG_S8BuTMxKkhnx0cAuXho8QUx13XPSUkeHYON0BcpH36RAO6yRmbN1fRYt7jUXK4lIKOdzLI6IZOlNyBKLFtHzvDQqNujT1uT8SDKPQd_LSW7ZrLeUFUv-q26o_mA-M77IooGwjJxBm00)


```text
@startuml
Geradeausfahrt : $v_l=100, v_r=100$
DrehungLinks :   $v_l=-50, v_r=50$\n$count=0$
state Rechtsabbiegen {
  Streckenfahrt :  $v_l=50, v_r=50$\n$count++$
  DrehungRechts :  $v_l=50, v_r=-50$
}

[*] --> Geradeausfahrt
Geradeausfahrt -> DrehungLinks : Hindernis erkannt
DrehungLinks -> Streckenfahrt: 90 Grad Drehwinkel erreicht
Streckenfahrt -> DrehungRechts: 30cm Strecke erreicht\ncount kleiner 3
DrehungRechts -> Streckenfahrt: 90 Grad Drehwinkel erreicht
Streckenfahrt --> DrehungLinks: 30cm Strecke erreicht\ncount größer 2
DrehungLinks --> Geradeausfahrt: 90 Grad Drehwinkel erreicht
@enduml
```

Hilfestellungen:

...