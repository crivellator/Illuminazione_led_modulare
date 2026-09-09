# Sistema di illuminazione LED modulare

Sistema modulare per il controllo di una matrice di LED RGB basato su **Raspberry Pi + Arduino + bus seriale**, progettato per gestire più schede LED indipendenti tramite un protocollo proprietario con indirizzamento hardware.

Il progetto nasce per la realizzazione di un'installazione di illuminazione composta da più moduli LED collegati sullo stesso bus, con configurazione della lunghezza delle strisce a runtime e aggiornamento sincronizzato dell'intera installazione.

> **Stato del progetto:** funzionante
> **Firmware:** Arduino / C++
> **Controller:** Raspberry Pi / Python
> **Protocollo:** seriale custom a 9600 baud
> **LED:** NeoPixel RGB

---

## Architettura

Il sistema è composto da un Raspberry Pi che agisce da **controller centrale** e da una serie di schede Arduino che funzionano come **nodi LED indirizzabili**.

```text
                         Raspberry Pi
                              │
                              │ Serial Bus
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
   ┌────▼────┐           ┌────▼────┐           ┌────▼────┐
   │ Board 0 │           │ Board 1 │    ...    │ Board N │
   │ ID = 0  │           │ ID = 1  │           │ ID = N  │
   └────┬────┘           └────┬────┘           └────┬────┘
        │                     │                     │
     3 strips              3 strips              3 strips
        │                     │                     │
      LEDs                  LEDs                  LEDs
```

Ogni scheda possiede:

* un **ID hardware a 6 bit**, impostato tramite DIP switch;
* tre uscite indipendenti per altrettante strisce NeoPixel;
* un'interfaccia seriale per il bus;
* memoria temporanea per il frame ricevuto;
* gestione autonoma del protocollo di comunicazione.

L'architettura permette di aggiungere o rimuovere nodi senza dover modificare il firmware delle singole schede.

---

## Controller Raspberry Pi

Il Raspberry Pi esegue uno script Python che costituisce il livello di controllo superiore.

La pipeline di elaborazione è:

```text
Immagine
   │
   ▼
Pillow
   │
RGB → HSV
   │
   ▼
Matrice logica
   │
   ▼
Suddivisione per scheda
   │
   ▼
Pacchetti seriali
   │
   ▼
Bus
   │
   ├── Board 0
   ├── Board 1
   ├── ...
   └── Board N
```

Lo script può ricevere direttamente un'immagine:

```bash
python led_sender.py immagine.png
```

L'immagine viene:

1. caricata tramite Pillow;
2. convertita in RGB;
3. ridimensionata automaticamente alle dimensioni della matrice;
4. convertita in HSV;
5. trasformata in una sequenza di byte;
6. suddivisa nei payload destinati alle singole schede;
7. trasmessa attraverso il bus seriale.

È inoltre presente una modalità di test senza immagine esterna.

---

## Matrice LED

La configurazione utilizzata durante lo sviluppo prevede:

| Parametro                 |   Valore |
| ------------------------- | -------: |
| Schede                    |        7 |
| Strisce per scheda        |        3 |
| Strisce totali            |       21 |
| LED per striscia          |       20 |
| LED totali                |      420 |
| Dimensione matrice logica |  21 × 20 |
| Dati per LED              |      HSV |
| Payload per scheda        | 180 byte |

La disposizione logica considera le tre strisce di ogni scheda come tre colonne consecutive della matrice.

La funzione `build_board_payloads()` costituisce il punto di separazione tra **layout logico** e **cablaggio fisico**, rendendo possibile adattare la disposizione dei LED senza riscrivere l'intero programma.

---

# Protocollo di comunicazione

La comunicazione utilizza un protocollo seriale custom progettato specificamente per il sistema.

Ogni pacchetto contiene:

```text
[STX] [ID] [CMD] [DATA...] [CHK]
```

dove:

* `STX` identifica l'inizio del pacchetto;
* `ID` identifica il nodo destinatario;
* `CMD` identifica il comando;
* `DATA` contiene gli eventuali dati;
* `CHK` contiene il checksum XOR.

Il checksum viene calcolato effettuando lo XOR dell'ID, del comando e di tutti i byte del payload.

---

## Comandi

### CONFIG — `0x01`

Configura la lunghezza delle strisce a runtime.

```text
[STX][ID][CMD][STRIP_LENGTH][CHK]
```

Il comando viene inviato in **broadcast**, quindi tutte le schede ricevono la configurazione.

La lunghezza può essere modificata senza ricompilare il firmware.

---

### STORE — `0x00`

Invia il frame destinato a una specifica scheda.

```text
[STX][ID][CMD][DATA...][CHK]
```

Il payload contiene:

```text
H S V | H S V | H S V | ...
```

per tutti i LED delle tre strisce della scheda.

Ogni nodo riceve il pacchetto, ma memorizza il payload solamente se l'ID corrisponde al proprio ID hardware.

Questo permette di utilizzare un unico bus condiviso senza collegamenti separati tra Raspberry Pi e ogni singola scheda.

---

### APPLY — `0x02`

Comando broadcast di applicazione del frame.

```text
[STX][ID][CMD][CHK]
```

Una volta caricati i dati nelle singole schede, il Raspberry Pi invia un unico comando `APPLY`.

Ogni nodo applica il proprio frame precedentemente memorizzato alle tre strisce LED.

La separazione tra `STORE` e `APPLY` permette di preparare lo stato dell'intera installazione prima dell'aggiornamento effettivo dei LED.

---

# Firmware Arduino

Il firmware implementa una macchina a stati per la ricezione dei pacchetti:

```text
WAIT_STX
   │
   ▼
WAIT_ID
   │
   ▼
WAIT_CMD
   │
   ├── CONFIG → WAIT_CONFIG_DATA
   │
   ├── STORE  → WAIT_DATA
   │
   └── APPLY  → WAIT_CHK
                    │
                    ▼
                  APPLY
```

La ricezione è gestita tramite `SoftwareSerial` a 9600 baud.

Il parser include:

* riconoscimento dell'inizio pacchetto;
* gestione dei diversi comandi;
* conteggio dei byte ricevuti;
* verifica del checksum;
* timeout del pacchetto;
* controllo della validità della configurazione;
* gestione separata del payload memorizzato e dell'applicazione del frame.

Il timeout di ricezione è impostato a 200 ms.

---

## Identificazione hardware

Ogni scheda utilizza sei ingressi analogici come ingressi digitali per leggere un DIP switch:

```text
A0 ─┐
A1 ─┤
A2 ─┤
A3 ─┤──► 6 bit ID
A4 ─┤
A5 ─┘
```

Gli ingressi utilizzano `INPUT_PULLUP` e la logica del DIP è quindi attiva-bassa.

Sono disponibili:

```text
2^6 = 64
```

ID differenti, da `0` a `63`.

L'ID viene inoltre riletto durante l'esecuzione, permettendo di modificarlo tramite DIP switch senza ricompilare il firmware.

---

# Gestione della memoria LED

Le tre strisce vengono allocate utilizzando la lunghezza massima supportata dal firmware:

```text
MAX_STRIP_LENGTH = 50
```

La lunghezza effettivamente utilizzata viene invece configurata a runtime tramite `CMD_CONFIG`.

Questo evita di dover ricompilare il firmware per installazioni con un numero differente di LED per striscia.

Il firmware invalida inoltre il payload precedentemente memorizzato quando viene ricevuta una nuova configurazione, evitando di applicare dati appartenenti a una configurazione precedente.

---

# Hardware

Ogni nodo LED comprende:

* microcontrollore Arduino;
* tre uscite NeoPixel;
* interfaccia seriale del bus;
* DIP switch a 6 bit;
* linea di controllo dedicata;
* connettori di ingresso/uscita per il bus.

Le schede sono state progettate come **moduli ripetibili**, permettendo di costruire l'installazione aggiungendo lo stesso tipo di nodo più volte.

### Collegamenti principali

```text
Arduino
│
├── D2 ──► NeoPixel strip 1
├── D3 ──► NeoPixel strip 2
├── D4 ──► NeoPixel strip 3
│
├── D7 ──► Serial TX
├── D8 ◄── Serial RX
├── D10 ──► Control
│
└── A0-A5 ◄── DIP switch ID
```

---

# Software

### Raspberry Pi

* Python 3
* Pillow
* pySerial

### Arduino

* C/C++
* SoftwareSerial
* Adafruit NeoPixel

---

# Versioning

### Firmware 3.1.0

* corretto un problema di gestione della dimensione delle strisce durante l'avvio;
* mantenuta l'allocazione alla dimensione massima;
* configurazione della lunghezza demandata esclusivamente a `CMD_CONFIG`;
* migliorata la gestione di installazioni con più LED per striscia e più nodi attivi.

### Sender Python 3.0.0

* configurazione automatica della lunghezza tramite broadcast `CONFIG`;
* supporto alle immagini reali;
* conversione RGB → HSV;
* ridimensionamento automatico;
* suddivisione dell'immagine nei payload delle singole schede;
* gestione degli errori della porta seriale;
* configurazione centralizzata.

---

# Obiettivi progettuali

Il progetto è stato sviluppato ponendo particolare attenzione a:

* **modularità hardware**;
* **indirizzamento dei nodi**;
* **semplicità del protocollo**;
* **riduzione dei collegamenti necessari**;
* **configurazione a runtime**;
* **separazione tra acquisizione, distribuzione e applicazione dei dati**;
* **possibilità di scalare il sistema aggiungendo nuovi moduli**.

L'obiettivo non è semplicemente pilotare una striscia LED, ma realizzare una piccola architettura **controller + nodi distribuiti**, con un protocollo di comunicazione dedicato e una separazione netta tra elaborazione dei contenuti e gestione dell'hardware.

---

# Struttura del progetto

```text
.
├── led_sender.py
├── firmware/
│   └── led_controller/
│       └── led_controller.ino
└── README.md
```

---

# Installazione e utilizzo

Sul Raspberry Pi sono necessarie le librerie Python:

```bash
pip install pillow pyserial
```

Configurare quindi:

* porta seriale;
* numero di schede;
* lunghezza delle strisce.

Per inviare un'immagine:

```bash
python led_sender.py immagine.png
```

Per utilizzare la modalità di test:

```bash
python led_sender.py
```

Il programma:

1. configura la lunghezza delle strisce;
2. genera i payload per le singole schede;
3. invia ogni payload al relativo ID;
4. invia il comando `APPLY` in broadcast;
5. aggiorna l'intera installazione.

---

## Note

Il progetto è stato sviluppato come sistema embedded completo, dalla progettazione della logica di comunicazione alla gestione dei nodi hardware e alla generazione dei dati sul Raspberry Pi.

Il codice e il protocollo sono specifici dell'installazione e sono stati sviluppati con l'obiettivo di mantenere l'architettura semplice, ripetibile e facilmente modificabile.
