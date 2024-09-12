# Erläuterung zu den Bewertungsmechanismen

## Ablauf der Demosession (26.09.2024)

+ ab 14 Uhr einrichten der Technik, tunen der Präsentation, letzte Vorbereitungen
+ 15:00 Uhr Start der Präsentationen (jeweils 20min)

    + Teamvorstellung (Studiengänge, Semester, Beitrag zur Projektarbeit)
    + Strukturierte Analyse der Aufgabenstellung / Ableitung von Anforderungen (Teilaufgaben) / Charakterisierung des Roboters und seiner Komponenten
    + Vorstellung der Lösungskonzepte / erwarteten oder tatsächlichen Herausforderungen
    + Vorstellung der Umsetzung (implementierte ROS2 Nodes, verwendete Tools, Pro- und Contra verschiedener Lösungsansätze)
    + kritische Diskussion der Zielerreichung/Ursachenanalyse
    + Optional: Live-Demo mit Vorführung von (einzelnen) Funktionalitäten oder eingebettete Videos des Roboters in Aktion

  6 x 20 min = 120 min - Pause nach 3 Vorträgen. Jedes Teammitglied sollte einen eigenen Beitrag in der Präsentation haben.

+ 17:15 Uhr Start der "Wertungsläufe"

    + zwei Durchläufe für alle Teams innerhalb derer 

         + der Roboter an der Ampel auf ein grünes Signal wartet und dann startet 
         + der Roboter die gesamte (!) Strecke abfährt und dabei die Hindernisse umfährt
         + nach dem Parkschild nach einer freien Parklücke sucht und dort einparkt 

      Dazwischen sind Anpassungen an der Software und Hardware erlaubt.

   6 x 2 x 5min = 60 min

+ 17:30 Uhr Summary und Abschlussdiskussion

## Reporting / Dokumentation

Im Repository sollten die Teams folgende Inhalte vorweisen:

+ `README.md` mit einer Beschreibung des Projekts und der Teammitglieder
+ `Project.md` mit den erweiterten (!) Inhalten der Präsentation
+ Commit und Branch-Struktur 
+ Issues und Pull-Requests
+ Dokumentation der Software (Code-Kommentare, ggf. Wiki, etc.)
+ Actions mit CI/CD (ein Check der Code-Quality, ein Test eines Nodes)

## Bewertung

| Element                   | Bewertungsanteil |
| ------------------------- | ---------------- |
| Präsentation              | 50%              |
| Wertungsläufe             | 30%              |
| Reporting / Dokumentation | 20%              |