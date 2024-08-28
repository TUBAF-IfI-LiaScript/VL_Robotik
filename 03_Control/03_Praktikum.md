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

>Beispiel II:  __Statisches Umfahrung eines Hindernisses__

Der Roboter soll in der Lage sein, auf ein Hindernis mit einem vorprogrammierten Bewegungsmuster auszuweichen. Dazu soll eine Zustandsmaschine implementiert werden, die folgende Zustände kennt:

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

![](https://www.plantuml.com/plantuml/png/ZP3FIiD04CRlynJ37grXewK7XKeFGZruqgELiascoP9q0fDDB-Ap-1Q-G5_CtRg5PeNYRGxVx_ipYzuPSSEkXIM9ASaC_TRKul0UbTgls_bDdb_ZVYtXK0eUXUg1gsVBJUyr65NPBAfcGUGz7U4B5RNhpMPZgIB63q1yTK95GnmZJlGvl6AbYcvWDvaevP4O-6ls5ycEWPy0t3b2iLNjiDicsH0jvq5BN68G0y3RrJjcsGEEfwUVy89ajTPFbaiIjZsID8RPQGsdSGaLgmTgcXSx916oHUs95Mjzx8LDIvP9SAhrwCF1PMGRNEvoNFg2PqO0ewZ_KadXPEV5mfKSlWzV97YRD4x-UI5zGLmEk_O7)


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
Geradeausfahrt -> DrehungLinks : //Hindernis erkannt//
Streckenfahrt -> DrehungRechts: //30cm erreicht//\n//count kleiner 3//
DrehungLinks -> Streckenfahrt: //90 Grad erreicht//

DrehungRechts -> Streckenfahrt: //90 Grad erreicht//
Streckenfahrt --> DrehungLinks: //30cm erreicht//\n//count größer 2//
DrehungLinks --> Geradeausfahrt: //90 Grad erreicht//
@enduml
```

> Welche Erweiterungen sehen Sie?

![](https://www.plantuml.com/plantuml/png/ZPF1Ji9048Rl-nIJ7iLcAuI391WE6Znu4czYoD8ExQOmJQPRBiPdyIry0LxCQLs2Mmpnncx-_vj_VhQpsXbspMwhviWcGzFK6rEmWmb4qNwrdTuaoJNiLzp-YA8bHP6pEwmQDoKZ1tNFM3IKFragQp61jyLZRnk7Bi7KufAf3k4PqyBLPhssc2F1km9OECQqH3g6yBW-BQqQSeD1Y1vsQ00VwOT-peZ_ZWJmm5pneaqs0VMXkkEn1kyLfr9KEGIsUU6WslIMK3vRfqJAe1KhfLwltY2Exo1ikL-w27fzQlreAKCcMmDoQOYSrYfiILp1ogrhbAGxCJ1QkT5wILftkQ3SeYLa60acc2MK0DjIRXEOIy1V81LC-gUfdpTSytBUd0zVXqza6FO31tLUcdvkyt-jrY_T4pZVoulySmYLuJANbAVxaqpDa3Bvthu1)


```text @plantUML
@startuml
Geradeausfahrt : ""v_l=100, v_r=100""\n""timeout=nan""
DrehungLinks :   ""v_l=-50, v_r=50""\n""timeout=10s""
state Rechtsabbiegen {
  Streckenfahrt :  ""v_l=50, v_r=50""\n""count++""\n""timeout=20s""
  DrehungRechts :  ""v_l=50, v_r=-50""\n""timeout=10s""
  ErrorState: 
}
Error: ""v_l=0, v_r=0""

note right of Rechtsabbiegen
""count=0""
end note


[*] --> Geradeausfahrt
Geradeausfahrt -> DrehungLinks : //Hindernis erkannt//
Streckenfahrt -> DrehungRechts: //30cm erreicht//\n//count kleiner 3//
DrehungLinks -> Streckenfahrt: //90 Grad erreicht//
DrehungRechts -> Streckenfahrt: //90 Grad erreicht//
Streckenfahrt --> DrehungLinks: //30cm erreicht//\n//count größer 2//
DrehungLinks --> Geradeausfahrt: //90 Grad erreicht//

DrehungLinks --> Error: //Timeout//
DrehungRechts --> ErrorState: //Timeout//
Streckenfahrt --> ErrorState: //Timeout//
ErrorState --> Error
@enduml
```

Hilfestellungen:

...
