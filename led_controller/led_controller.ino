/*
 * ============================================================
 *  LED Controller Arduino - Ricezione seriale multi-ID
 * ============================================================
 *
 * Ogni scheda ha un ID hardware impostato via dipswitch (6 bit).
 * Riceve pacchetti broadcast sul bus seriale (SoftwareSerial) e:
 *   - CMD_CONFIG (0x01): imposta STRIP_LENGTH corrente (una
 *     tantum, tipicamente in broadcast a inizio sessione)
 *   - CMD_STORE  (0x00): memorizza il payload SOLO se l'ID nel
 *     pacchetto corrisponde all'ID della scheda
 *   - CMD_APPLY  (0x02): applica l'ultimo payload memorizzato
 *     alle 3 strip NeoPixel collegate
 *
 * Formato pacchetti:
 *   CONFIG: [STX][ID][CMD][STRIP_LENGTH][CHK]
 *   STORE : [STX][ID][CMD][DATA...][CHK]   (DATA lungo STRIP_LENGTH*STRIPS_PER_BOARD*3,
 *                                            secondo l'ultima CONFIG ricevuta)
 *   APPLY : [STX][ID][CMD][CHK]
 *   CHK = XOR di ID, CMD e tutti i byte extra presenti
 *
 * STRIPS_PER_BOARD resta fisso a 3 (cablaggio fisico: 3 pin LED,
 * L1/L2/L3). STRIP_LENGTH è invece configurabile a runtime via
 * CMD_CONFIG, entro il limite MAX_STRIP_LENGTH: NON richiede più
 * di riflashare il firmware per cambiare lunghezza strip.
 *
 * ------------------------------------------------------------
 *  VERSION: 3.1.0
 * ------------------------------------------------------------
 *  CHANGELOG:
 *  3.1.0 (2025-XX-XX)
 *    - FIX BUG: in setup() veniva chiamato updateLength(currentStripLength)
 *      (=DEFAULT_STRIP_LENGTH, 5) subito dopo aver costruito le strip
 *      NeoPixel a MAX_STRIP_LENGTH. Questo restringeva il buffer interno
 *      a 5 LED, e la successiva riespansione via CMD_CONFIG lasciava i
 *      primi LED (tipicamente 5-10) con stato sporco: causava lo
 *      spegnimento improvviso e stabile osservato su strip >5 LED con
 *      più schede attive. Rimossa la chiamata: le strip restano
 *      allocate a MAX_STRIP_LENGTH dal costruttore, applyConfig() è
 *      l'unico punto che le ridimensiona
 *  3.0.0 (2025-XX-XX)
 *    - Aggiunto CMD_CONFIG: STRIP_LENGTH non è più una costante
 *      di compilazione ma un parametro runtime, inviato una
 *      volta dal Python prima degli STORE. Elimina la necessità
 *      di riflashare le schede per cambiare lunghezza strip.
 *    - Buffer payload e strip NeoPixel dimensionati staticamente
 *      su MAX_STRIP_LENGTH=20 (~860 byte RAM stimati su ATmega328P,
 *      margine ampio su 2KB totali); STRIP_LENGTH runtime non può
 *      superarlo (validato, richiesta CONFIG fuori range scartata)
 *    - strip.updateLength() richiamato ad ogni CONFIG valida per
 *      applicare la nuova geometria alle NeoPixel
 *    - STORE ricevuto prima di una CONFIG valida viene rifiutato
 *      esplicitamente (di default all'avvio è comunque attiva una
 *      STRIP_LENGTH=5 di sicurezza, per compatibilità immediata)
 *    - STRIPS_PER_BOARD resta #define fisso a 3 (cablaggio fisico)
 *  2.0.0
 *    - Costanti dimensionali centralizzate e coerenti
 *      (STRIP_LENGTH / STRIPS_PER_BOARD / PAYLOAD_SIZE)
 *    - Rimosso MAX_PAYLOAD sovradimensionato (era 1000, ora
 *      dimensionato esattamente sul payload atteso)
 *    - Aggiunto timeout FSM: un pacchetto interrotto a metà
 *      non blocca più la ricezione (reset automatico dopo
 *      PACKET_TIMEOUT_MS senza nuovi byte)
 *    - Debug seriale disattivabile via DEBUG_ENABLED (0/1),
 *      per non rallentare la SoftwareSerial in produzione
 *  1.0.0
 *    - Versione originale (FSM base, buffer 1000 byte, debug
 *      sempre attivo)
 * ============================================================
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// === CONFIGURAZIONE HARDWARE ===
#define RX_PIN 8
#define TX_PIN 7
#define CONTROL_PIN 10
#define LED_PIN_1 2
#define LED_PIN_2 3
#define LED_PIN_3 4

#define DIP1 A0
#define DIP2 A1
#define DIP3 A2
#define DIP4 A3
#define DIP5 A4
#define DIP6 A5

// === CONFIGURAZIONE PROTOCOLLO / GEOMETRIA LED ===
#define STRIPS_PER_BOARD 3      // fisso: 3 pin LED cablati (L1/L2/L3)
#define BYTES_PER_LED 3         // H, S, V

// Tetto massimo per STRIP_LENGTH runtime, dimensiona staticamente
// buffer payload e strip NeoPixel. Valore verificato in compilazione:
// con MAX_STRIP_LENGTH=50 le variabili globali usano il 42% dei 2048
// byte SRAM disponibili su ATmega328P, margine ampio per lo stack.
#define MAX_STRIP_LENGTH 50
#define MAX_PAYLOAD_SIZE (MAX_STRIP_LENGTH * STRIPS_PER_BOARD * BYTES_PER_LED)

#define DEFAULT_STRIP_LENGTH 5  // usato finché non arriva una CMD_CONFIG valida

#define STX 0x02
#define CMD_STORE 0x00
#define CMD_CONFIG 0x01
#define CMD_APPLY 0x02

#define PACKET_TIMEOUT_MS 200   // reset FSM se un pacchetto resta a metà troppo a lungo

// Debug: mettere a 0 per disattivare tutti i Serial.print (consigliato in produzione,
// per non rallentare la SoftwareSerial e non intasare l'unica UART hardware)
#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
  #define DBG_PRINT(x) Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
  #define DBG_PRINTLN_HEX(x) Serial.println(x, HEX)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTLN_HEX(x)
#endif

const uint8_t dipPins[] = {DIP1, DIP2, DIP3, DIP4, DIP5, DIP6};

SoftwareSerial busSerial(RX_PIN, TX_PIN);
Adafruit_NeoPixel strip1(MAX_STRIP_LENGTH, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(MAX_STRIP_LENGTH, LED_PIN_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3(MAX_STRIP_LENGTH, LED_PIN_3, NEO_GRB + NEO_KHZ800);

uint8_t boardId = 0;

// Geometria corrente, impostata via CMD_CONFIG (con default di sicurezza all'avvio)
uint8_t currentStripLength = DEFAULT_STRIP_LENGTH;
uint16_t currentPayloadSize = DEFAULT_STRIP_LENGTH * STRIPS_PER_BOARD * BYTES_PER_LED;
bool configReceived = false;

enum RxState
{
    WAIT_STX,
    WAIT_ID,
    WAIT_CMD,
    WAIT_CONFIG_DATA,
    WAIT_DATA,
    WAIT_CHK
};

RxState state = WAIT_STX;
uint8_t rxCmd = 0;
uint8_t rxId = 0;
uint8_t checksum = 0;
uint8_t payload[MAX_PAYLOAD_SIZE];
uint16_t payloadIdx = 0;   // indice di scrittura nel buffer (solo se ID coincide)
uint16_t totalDataIdx = 0; // indice di scorrimento del pacchetto (sempre)
bool hasValidPayload = false;

unsigned long lastByteMillis = 0;

// ------------------------------------------------------------
// Legge l'ID hardware dai dipswitch (attivi bassi, invertiti)
// ------------------------------------------------------------
void readBoardId()
{
    uint8_t raw = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        raw |= (digitalRead(dipPins[i]) << i);
    }
    boardId = raw ^ 0b111111;
}

// ------------------------------------------------------------
// Applica una nuova geometria (STRIP_LENGTH) ricevuta via CONFIG
// ------------------------------------------------------------
void applyConfig(uint8_t newStripLength)
{
    if (newStripLength == 0 || newStripLength > MAX_STRIP_LENGTH)
    {
        DBG_PRINT(F("CONFIG rifiutata, STRIP_LENGTH fuori range: "));
        DBG_PRINTLN(newStripLength);
        return;
    }

    currentStripLength = newStripLength;
    currentPayloadSize = (uint16_t)currentStripLength * STRIPS_PER_BOARD * BYTES_PER_LED;
    configReceived = true;
    hasValidPayload = false; // un payload precedente non è più valido con nuova geometria

    strip1.updateLength(currentStripLength);
    strip2.updateLength(currentStripLength);
    strip3.updateLength(currentStripLength);
    strip1.clear();
    strip2.clear();
    strip3.clear();
    strip1.show();
    strip2.show();
    strip3.show();

    DBG_PRINT(F("OK: CONFIG applicata, STRIP_LENGTH="));
    DBG_PRINTLN(currentStripLength);
}

// ------------------------------------------------------------
// Applica il payload memorizzato alle 3 strip fisiche
// ------------------------------------------------------------
void applyPayloadToLeds()
{
    if (!hasValidPayload)
    {
        DBG_PRINTLN(F("ERRORE: nessun payload valido da applicare"));
        return;
    }

    Adafruit_NeoPixel *strips[STRIPS_PER_BOARD] = {&strip1, &strip2, &strip3};

    uint16_t idx = 0;
    for (uint8_t s = 0; s < STRIPS_PER_BOARD; s++)
    {
        strips[s]->clear();
        for (uint8_t i = 0; i < currentStripLength; i++)
        {
            uint8_t h = payload[idx++];
            uint8_t sVal = payload[idx++];
            uint8_t v = payload[idx++];
            strips[s]->setPixelColor(i, strips[s]->ColorHSV((uint16_t)h * 257, sVal, v));
        }
        strips[s]->show();
    }

    DBG_PRINTLN(F("OK: LED aggiornati"));
}

// ------------------------------------------------------------
// Riporta la FSM in stato di attesa pulito
// ------------------------------------------------------------
void resetFsm()
{
    state = WAIT_STX;
    payloadIdx = 0;
    totalDataIdx = 0;
    checksum = 0;
}

void setup()
{
    for (uint8_t i = 0; i < 6; i++)
    {
        pinMode(dipPins[i], INPUT_PULLUP);
    }

    pinMode(CONTROL_PIN, OUTPUT);
    digitalWrite(CONTROL_PIN, LOW);

#if DEBUG_ENABLED
    Serial.begin(9600);
#endif
    busSerial.begin(9600);

    strip1.begin();
    strip1.clear();
    strip1.show();
    strip2.begin();
    strip2.clear();
    strip2.show();
    strip3.begin();
    strip3.clear();
    strip3.show();
    // NOTA: nessun updateLength() qui. Le strip sono già allocate a
    // MAX_STRIP_LENGTH dal costruttore globale (riga ~118). Chiamare
    // updateLength(currentStripLength=5) qui le restringeva subito
    // dopo l'allocazione, e la successiva riespansione via CMD_CONFIG
    // lasciava i primi LED (in particolare 5-10) con dati/stato
    // sporchi — bug osservato e risolto in 3.1.0, vedi CHANGELOG.

    readBoardId();
    DBG_PRINT(F("ID dispositivo: "));
    DBG_PRINTLN(boardId);
    DBG_PRINT(F("STRIP_LENGTH di default (in attesa di CONFIG): "));
    DBG_PRINTLN(currentStripLength);
    DBG_PRINTLN(F("In attesa di comandi seriali..."));
}

void loop()
{
    readBoardId(); // ID rivalutato ad ogni ciclo (dipswitch può cambiare a runtime)

    // Timeout: se siamo a metà pacchetto da troppo tempo, reset
    if (state != WAIT_STX && (millis() - lastByteMillis > PACKET_TIMEOUT_MS))
    {
        DBG_PRINTLN(F("Timeout pacchetto: reset FSM"));
        resetFsm();
    }

    while (busSerial.available())
    {
        uint8_t b = busSerial.read();
        lastByteMillis = millis();

        switch (state)
        {
        case WAIT_STX:
            if (b == STX)
            {
                checksum = 0;
                payloadIdx = 0;
                totalDataIdx = 0;
                state = WAIT_ID;
            }
            break;

        case WAIT_ID:
            rxId = b;
            checksum ^= b;
            state = WAIT_CMD;
            break;

        case WAIT_CMD:
            rxCmd = b;
            checksum ^= b;
            if (rxCmd == CMD_STORE)
            {
                if (!configReceived)
                {
                    DBG_PRINTLN(F("STORE rifiutato: nessuna CONFIG valida ricevuta ancora"));
                }
                payloadIdx = 0;
                totalDataIdx = 0;
                state = WAIT_DATA;
            }
            else if (rxCmd == CMD_CONFIG)
            {
                state = WAIT_CONFIG_DATA;
            }
            else if (rxCmd == CMD_APPLY)
            {
                state = WAIT_CHK; // APPLY non ha payload
            }
            else
            {
                DBG_PRINTLN(F("CMD sconosciuto, ignorato"));
                resetFsm();
            }
            break;

        case WAIT_CONFIG_DATA:
            checksum ^= b;
            payload[0] = b; // riuso temporaneo del buffer per il byte STRIP_LENGTH ricevuto
            state = WAIT_CHK;
            break;

        case WAIT_DATA:
            checksum ^= b; // il checksum copre sempre tutti i byte dati, per ogni scheda

            if (rxId == boardId && payloadIdx < currentPayloadSize)
            {
                payload[payloadIdx++] = b; // salvo solo se l'ID corrisponde
            }

            totalDataIdx++;
            if (totalDataIdx >= currentPayloadSize)
            {
                state = WAIT_CHK;
            }
            break;

        case WAIT_CHK:
        {
            uint8_t rxChk = b;

            if (checksum == rxChk)
            {
                if (rxCmd == CMD_CONFIG)
                {
                    // CONFIG è sempre broadcast: applicata indipendentemente dall'ID
                    applyConfig(payload[0]);
                }
                else if (rxCmd == CMD_STORE)
                {
                    if (rxId == boardId && configReceived)
                    {
                        hasValidPayload = true;
                        DBG_PRINTLN(F("OK: payload memorizzato"));
                    }
                    else if (rxId == boardId)
                    {
                        DBG_PRINTLN(F("STORE scartato: manca una CONFIG valida"));
                    }
                    else
                    {
                        DBG_PRINTLN(F("Payload per altra scheda, ignorato"));
                    }
                }
                else if (rxCmd == CMD_APPLY)
                {
                    digitalWrite(CONTROL_PIN, HIGH);
                    applyPayloadToLeds();
                    digitalWrite(CONTROL_PIN, LOW);
                }
            }
            else
            {
                DBG_PRINTLN(F("ERRORE: checksum non valido"));
            }

            resetFsm();
            break;
        }
        }
    }
}

