# Entwicklung einer Methode zur Verschleißdetektion von Schmiedegesenken mittels einer Prozessüberwachung

- Das Ziel dieser Arbeit ist die Ermittlung der Reststandmenge von Schmiedewerkzeugen mittels einer Prozessüberwachung. Mithilfe von Programmierwerkzeugen (bspw. Python) soll eine mathematische Methode erarbeitet werden, die eine Vorhersage der Reststandmenge des verwendeten Schmiedewerkzeugs ermöglicht. Die mathematische Methode soll über ein Graphic User Interface bedient werden können und Daten aus zwei Messystemen (optisches Messystem du Kraftmesssystem) verarbeiten. Durch die Verarbeitung der Daten der Messysteme soll eine Verschleißdetektion ermöglicht werden und eine Verschleißprognose ausgegeben werden.

- Innerhalb der Methode soll ein CAD-Modell des Schmiedeteils im Dateiformat STL oder STEP eingeladen werden. Auf Basis des CAD-Modells des Schmiedegesenks und aktuellen Bilddaten des verwendeten Schmiedegesenks soll ein SOLL/IST-Vergleich durchgeführt werden. Anhand des SOLL/IST-Vergleichs sollen Abweichungen detektiert und der Verschleißzustand erkannt werden. Aus Fertigungstoleranzen des Schmiedebauteils werden die Qualitätsgrenzen extrahiert und eine obere Verschleißgrenze definiert. Zusätzlich sollen Kraftmesswerte aufgenommen werden und Kraftanomalien detektiert werden. Innerhalb der Methode soll die Wärmeausdehung des Schmiedegesenks berücksichtigt werden.

- Die Inhalte der Arbeit sind in einer mündlichen Präsentation in Anwesenheit der Mitarbeiter des Bereichs vorzustellen.

## Teilziel 1

 Weiterentwicklung des bestehenden Programms für den SOLL/IST-Vergleich zwischen der STL-Datei aus der CAD-Datei und der Punktwolke aus dem Messsystem (teilweise neu, vieles bereits vorhanden)

- Übereinstimmung der beiden Dateien über ein ICP-Algorithmus (bisheriger Stand)
- Matchen der beiden Dateien über eine Referenzebene (neu)
- Segmentierung des Gesenks um unterschiedliche Verschleißorte zu identifizieren (Bisher möglich, aber leichte Weiterentwicklung notwendig)

## Teilziel 2

Implementierung von Kraftwerten in die Methode (neu)

- Aufbau einer Datenbank um Kraftmessdaten speichern zu können in Abhängigkeit der Produktionsmenge
- Zuweisung der Kraftwerte dem Koordinatensystem aus der optischen Überwachung
- Implementierung der Kraftwerte in die Methode mit Zuweisung einer kritischen Kraft (darf nicht unter oder überschritten werden)

## Teilziel 3

Berücksichtigung der Wärmeausdehung in die Methode

- Matchen von 2D-Wärmebildaufnahmen mit 3D Punktwolken des vermessenen Gesenks
- Zuweisung von Kraftwerten einzelnen Punkten oder Punktclustern
- Mathematische Berücksichtigung der Wärmeausdehnung in lokal unterschiedlichen Bereichen

## Frage

- Eigenes Laptop?

### Frage zum Teilziel 1

- Creo software Lizenz

- Python code von Teil 1 Übereinstimmung der beiden Datein(ICP (Iterative Closest point) ) // ANN(Approximate Nearest Neighbor)

- Wie versteht man die Referenzebene

### Frage zum Teilziel 2

- Wie funktioniert die 2. schritt: Zuweisung der Kraftwerte dem Koordinatensystem aus der optischen Überwachung

- Daten aus mechanischesensor /kraftsensor exprotieren ->  datenbank für Kraftmessdaten -> zuweisung der Kraftwert auf Punkt Cloud

### Frage zum Teilziel 3

- infrarot ? 2D Bild