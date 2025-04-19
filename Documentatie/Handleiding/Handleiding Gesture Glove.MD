# Handleiding Gesture Glove

## Inhoudsopgave
- [Handleiding Gesture Glove](#handleiding-gesture-glove)
  - [Inhoudsopgave](#inhoudsopgave)
  - [1. Introductie](#1-introductie)
  - [2. Benodigdheden](#2-benodigdheden)
    - [2.1 Hardwarecomponenten van de Gesture Glove](#21-hardwarecomponenten-van-de-gesture-glove)
    - [2.2 Hardwarecomponenten van de auto](#22-hardwarecomponenten-van-de-auto)
    - [2.3 Software](#23-software)
  - [3. Werking van de Gesture Glove](#3-werking-van-de-gesture-glove)
    - [Bewegingen](#bewegingen)
    - [Extra functionaliteiten](#extra-functionaliteiten)
    - [OLED-scherm weergave](#oled-scherm-weergave)
      - [Linkerkant:](#linkerkant)
      - [Rechterkant:](#rechterkant)
    - [Verbinden met simulatie of fysieke auto](#verbinden-met-simulatie-of-fysieke-auto)
    - [ChangeLog](#changelog)

---

## 1. Introductie

De **Gesture Glove** is een project dat handbewegingen meet en doorstuurt naar een eindapparaat, zoals een RC-auto of een simulatieprogramma. Deze methode van data-overdracht biedt een alternatieve manier om apparaten aan te sturen en opent nieuwe toepassingen.

In dit project wordt de data gebruikt om een RC-auto te besturen in zowel de fysieke als de virtuele (simulatie) wereld. In deze handleiding wordt uitgelegd hoe het systeem werkt, welke onderdelen je nodig hebt en hoe je ermee aan de slag gaat.


---

## 2. Benodigdheden

Om dit project zelf op te bouwen, heb je de volgende hardware en software nodig:

### 2.1 Hardwarecomponenten van de Gesture Glove
- LilyGO TTGO T2 ESP32-WROOM (microcontroller met ingebouwd OLED-scherm)
- MPU6500 (versnellingsmeter en gyroscoopsensor)
- Flex sensor  
- Drukknop  
- Batterij

### 2.2 Hardwarecomponenten van de auto
- 4 DC-motoren  
- 4 wielen  
- ESP32S (microcontroller)  
- L293D motor driver  
- Batterij

### 2.3 Software
- Webots (simulatieomgeving)  
- Arduino IDE (voor het programmeren van de microcontrollers)

---

## 3. Werking van de Gesture Glove

De Gesture Glove maakt gebruik van verschillende sensoren om de beweging van je hand te registreren. Deze informatie wordt draadloos via **Bluetooth (BLE)** verzonden naar zowel de simulatie als de fysieke auto.

### Bewegingen
- **Voor- en achteruit:** De MPU6500 detecteert de kanteling van je hand. Kantel je hand naar voren, dan rijdt de auto vooruit. Kantel je hand naar achter, dan rijdt hij achteruit.
- **Links/rechts:** Bij kanteling naar links of rechts beweegt de auto in dezelfde richting.

### Extra functionaliteiten
- **Flex sensor:** Detecteert of je een vuist maakt. Als je een vuist maakt, remt de auto direct.
- **Drukknop:** Schakelt alle functies uit. Na 5 seconden gaat de glove in slaapmodus. Deze vertraging voorkomt dat een per ongeluk indrukken meteen tot slaapmodus leidt.

### OLED-scherm weergave
Op het scherm van de ESP32 worden realtime gegevens getoond:

#### Linkerkant:
- 🔋 **Batterij en hand:** Percentage van de batterij van de Gesture Glove  
- 📶 **Verbinding:** Signaalsterkte tussen de glove en de auto (fysiek of simulatie)  
- 🔋 **Batterij en stuur:** Batterijniveau van de fysieke auto  
- 🚗 **Snelheidsmeter:** Toont de snelheid van het voertuig  

#### Rechterkant:
- ❌ **'X'-icoon bovenaan:** Geeft aan of de werking is uitgeschakeld door de knop (hardware interrupt)  
- 🧭 **Richting onderaan:** Richting van de hand zoals gemeten door de IMU  
  - De gekleurde cirkel geeft de status van de rem aan:
    - **Groen & klein:** Geen vuist → geen rem  
    - **Rood & groter:** Vuist → rem actief  

---

### Verbinden met simulatie of fysieke auto

- **Simulatie:** Start de Webots-wereld en run het bijbehorende Python-script om verbinding te maken via BLE.
- **Fysieke auto:** Sluit de auto aan op stroom. De verbinding met de Gesture Glove wordt automatisch opgezet via BLE.

---


### ChangeLog

| Versie | Datum       | Wijziging            |
|--------|-------------|----------------------|
| 0.1    | 14-04-2025  | Document aangemaakt  |
| 1.0    | 15-04-2025  | Informatie toegevoegd |

