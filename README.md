# Roboterarm (DE)

Dies ist der Quellcode für die praktische Umsetzung meiner Facharbeit. Der Fokus liegt auf der Herleitung eines mathematischen Algorithmus zur Berechnung von **Soll-Winkeln** basierend auf einem Zielvektor (Inverse Kinematik).

<br><br>

## Vorschau
<img src="/preview/sirius/img.jpg" alt="Vorschau Bild" width="300">
<img src="/preview/sirius/preview_box.gif" alt="Vorschau Bild" width="300">

<br><br>

## Ressourcen & Dokumentation
| Inhalt | Link |
| :--- | :--- |
| **SharePoint Ordner** | [Hier klicken](https://ebgsdinslaken-my.sharepoint.com/:f:/g/personal/kevin_moenig_ebgs_info/IgBN8jIubz5cSIsgGcK1gndsAcNVv3gbHK9tp-4K4xWqHv8?e=jFno32) |
| **Simulation (GitHub)** | [Repository ansehen](https://github.com/Buldes/Roboterarm_Simulation) |

> **Hinweis:** Der SharePoint-Ordner enthält die Facharbeit, Schaltpläne, Vorschauen und Zeichnungen.

<br><br>

## Technische Spezifikationen
Die Steuerung erfolgt über einen **ESP-32** mit MicroPython.
* **Version:** `ESP32_GENERIC-20251209-v1.27.0`


<br><br>

## Rechtliches
Zur Steuerung des OLED-Displays wird das Script `sh1106.py` von robert-hh verwendet. (Repo [hier](https://github.com/robert-hh/SH1106))

>Copyright (c) 2016 Radomir Dopieralski (@deshipu),
              2017-2021 Robert Hammelrath (@robert-hh)
              2021 Tim Weber (@scy)

<br><br>

Für die Schriftarten der Fernsteruerung in `/Client_Side/` werden Folgende Schriftarten von Google Fonts verwendet:

- Jersey10-Regular
- Sixtyfour-Regular
- Tektur-Regular

<br><br>

Für die Steuerung des IR-Sensors wird das Script `nec.py` von Peter Hinch verwendet. (Repo [hier](https://github.com/peterhinch/micropython_ir))

>Copyright Peter Hinch 2020-2024 Released under the MIT license

<br><br>


<hr style="height:10px; border:none; color:#333; background-color:#333;" />
<br><br>

# Roboterarm (EN)
This is the source code for the practical implementation of my term paper. The focus lies on the derivation of a mathematical algorithm for calculating **target angles** based on a target vector (inverse kinematics).

<br><br>

## Preview
<img src="/preview/sirius/img.jpg" alt="Vorschau Bild" width="300">
<img src="/preview/sirius/preview_box.gif" alt="Vorschau Bild" width="300">

<br><br>

## Resources & Documentation
| Inhalt | Link |
| :--- | :--- |
| **SharePoint Folder** | [Click here](https://ebgsdinslaken-my.sharepoint.com/:f:/g/personal/kevin_moenig_ebgs_info/IgBN8jIubz5cSIsgGcK1gndsAcNVv3gbHK9tp-4K4xWqHv8?e=jFno32) |
| **Simulation (GitHub)** | [View repository](https://github.com/Buldes/Roboterarm_Simulation) |

> **Note:** The SharePoint folder contains the term paper, circuit diagrams, previews, and drawings.

<br><br>

## Technical Specifications
The control is handled via an **ESP-32** running MicroPython.
* **Version:** `ESP32_GENERIC-20251209-v1.27.0`

<br><br>

## Legal
The script `sh1106.py` by robert-hh is used to control the OLED display. (Repo [here](https://github.com/robert-hh/SH1106))

>Copyright (c) 2016 Radomir Dopieralski (@deshipu),
              2017-2021 Robert Hammelrath (@robert-hh)
              2021 Tim Weber (@scy)

<br><br>

The following fonts from Google Fonts are used for the remote control interface in `/Client_Side/`.

- Jersey10-Regular
- Sixtyfour-Regular
- Tektur-Regular

<br><br>

The script `nec.py` by Peter Hinch is used to control the IR sensor. (Repo [here](https://github.com/peterhinch/micropython_ir))

>Copyright Peter Hinch 2020-2024 Released under the MIT license
