# Temperatur Messung im Kiwibad
misst aller halben Stunde die Temperatur und versendet sie via LoRa, export der historischen Daten via BLE ans Smartphone über [phyphox](https://phyphox.org/)
### Sender 
* Vision Master E213
* [GT-U7 GPS-Modul](https://de.aliexpress.com/item/1005001713388717.html)
* [10 Paar 3 Pin SM JST 15 cm Kabel Buchse/Stecker](https://de.aliexpress.com/item/1005006673736507.html)
* 2x [3400 mAh 18650 Lithium-Akku](https://de.aliexpress.com/item/1005007281673163.html)
* [115x90x55mm wasserdichtes Gehäuse](https://de.aliexpress.com/item/1005005631937759.html)
* [SP1710P SP1712S 5 Pin](https://de.aliexpress.com/item/1005006892125866.html)
* [4 18650 Parallele Batteriebox](https://de.aliexpress.com/item/1005001955162216.html)
* [DS18B20 10m](https://de.aliexpress.com/item/1005006090368473.html)
* [DS18B20 1m](https://de.aliexpress.com/item/1005006090368473.html)
### Empfänger
* Heltec WiFi LoRa 32(V3)
* acos-ht4 3dprints are copys from https://www.printables.com/model/1519914-heltec-lora-32-v4v3-pocket-pager-case [CC BY-SA 4.0.]( https://creativecommons.org/licenses/by-sa/4.0/) [@AlleyCat](https://www.printables.com/@AlleyCat)

## Export der historischen Daten am Empfänger
1. [App installieren](https://phyphox.org/)
2. Oben rechts das Plus-Symbol
3. Neues Experiment für Bluetooth-Gerät
4. Kiwibad auswählen
5. auf den Playbutton drücken

## Build instructions

arduino with
* version 2.1.5 of https://github.com/HelTecAutomation/Heltec_ESP32
* https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series [last version has bug](https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/issues/296)
* Vision Master E213 (sender)
* Heltec WiFi LoRa 32(V3) (receiver)
