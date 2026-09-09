#!/usr/bin/env python3
"""
============================================================
 LED Sender - invio immagini a matrice di schede LED
============================================================

Costruisce una "matrice logica" di LED a partire da:
  - un'immagine reale (PNG/JPG/...) ridimensionata automaticamente, oppure
  - dati HSV di test hardcoded (fallback se nessun file passato)

e la invia via seriale alle schede Arduino, una alla volta,
seguendo il protocollo:  [STX][ID][CMD][DATA...][CHK]
(CHK = XOR di ID, CMD e tutti i byte di DATA)

Il protocollo NON è stato modificato: compatibile con il
firmware Arduino esistente.

------------------------------------------------------------
IMPORTANTE - come sono disposte le schede:
La matrice logica ha:
  - larghezza  = STRIP_LENGTH   (LED per singola strip)
  - altezza    = STRIPS_PER_BOARD * NUM_BOARDS
Ogni scheda "possiede" un blocco verticale di STRIPS_PER_BOARD
righe consecutive. Se la disposizione fisica reale è diversa,
va adattata SOLO la funzione build_board_payloads().
------------------------------------------------------------

VERSION: 3.0.0

CHANGELOG:
3.0.0 (2025-XX-XX)
  - Aggiunto invio automatico di CMD_CONFIG in broadcast prima
    degli STORE: comunica STRIP_LENGTH alla scheda, che non
    richiede più un firmware ricompilato per lunghezze diverse
    (entro il limite MAX_STRIP_LENGTH=20 lato Arduino)
  - STRIPS_PER_BOARD resta fisso (cablaggio fisico a 3 pin LED),
    non inviato via protocollo
2.0.1
  - Fix: build_board_payloads e MATRIX_WIDTH/MATRIX_HEIGHT
    erano stati riscritti con righe/colonne invertite rispetto
    all'originale (bug introdotto in 2.0.0, catturato con test
    di equivalenza contro la logica 1.0.0 prima del rilascio).
    Ripristinato l'orientamento corretto: riga = posizione LED
    lungo la strip, colonna = indice strip nel bus.
2.0.0 (2025-XX-XX)
  - Aggiunto caricamento immagini reali (Pillow), con resize
    automatico alla dimensione della matrice logica e
    conversione RGB -> HSV (0-255 per canale, come si aspetta
    l'Arduino)
  - CLI: `python led_sender.py immagine.png` oppure nessun
    argomento per usare i dati di test integrati
  - Gestione errori sulla seriale (porta assente, timeout)
  - Parametri di configurazione centralizzati in cima al file
  - Codice morto/commentato rimosso, nomi più espliciti
1.0.0
  - Versione originale, solo dati di test hardcoded,
    nessuna gestione errori
============================================================
"""

import sys
import time
import argparse

import serial
from serial.tools import list_ports
from PIL import Image

# ============================================================
# CONFIGURAZIONE - modificare qui per adattare all'impianto
# ============================================================
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
SERIAL_TIMEOUT_S = 1

STRIP_LENGTH = 20        # LED per singola strip (larghezza matrice)
STRIPS_PER_BOARD = 3    # strip collegate a ciascuna scheda
NUM_BOARDS = 7          # numero di schede fisicamente in bus

MATRIX_WIDTH = STRIPS_PER_BOARD * NUM_BOARDS  # numero totale di strip nel bus (colonne)
MATRIX_HEIGHT = STRIP_LENGTH                   # LED per strip (righe)

STX = 0x02
CMD_STORE = 0x00
CMD_CONFIG = 0x01
CMD_APPLY = 0x02
BROADCAST_ID = 0xFF

MAX_STRIP_LENGTH_HW = 50     # limite hardware lato Arduino (MAX_STRIP_LENGTH nel .ino)

INTER_PACKET_DELAY_S = 0.5  # pausa tra un pacchetto STORE e il successivo
CONFIG_DELAY_S = 0.3        # pausa dopo l'invio della CONFIG, prima degli STORE


# ============================================================
# Dati di test (fallback se non viene passata un'immagine)
# Griglia 6 colonne x 5 righe, formato H,S,V per LED
# ============================================================
TEST_IMAGE_HSV = [
    0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0x30, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0x00,  0xFF, 0xFF, 0x00,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,
    0xC0, 0xFF, 0x80,  0x40, 0xFF, 0x80,  0xA0, 0xFF, 0x80,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0xFF,
]


def load_image_as_hsv(path, width, height):
    """
    Carica un'immagine da file, la ridimensiona alla dimensione
    della matrice logica (width x height) e la converte in una
    lista piatta di byte H,S,V (0-255 ciascuno), riga per riga.

    width  = numero totale di strip nel bus (MATRIX_WIDTH)
    height = LED per strip (MATRIX_HEIGHT)
    Ordine identico a TEST_IMAGE_HSV: per ogni riga (posizione
    lungo la strip), i pixel di tutte le strip in sequenza.
    """
    img = Image.open(path).convert("RGB")
    img = img.resize((width, height), Image.LANCZOS)
    img_hsv = img.convert("HSV")

    flat_hsv = []
    for y in range(height):
        for x in range(width):
            h, s, v = img_hsv.getpixel((x, y))
            flat_hsv.extend([h, s, v])

    return flat_hsv


def build_board_payloads(image_hsv, num_boards, strips_per_board, strip_length):
    """
    Trasforma la lista piatta H,S,V nei payload da inviare a
    ciascuna scheda.

    ATTENZIONE - orientamento della matrice logica (identico
    all'originale, NON invertire senza verificare sul campo):
      - riga    = posizione del LED lungo la strip (0..strip_length-1)
      - colonna = indice della strip complessiva nel bus
                  (0..num_boards*strips_per_board-1)
    image_hsv è quindi lineare per RIGHE: la prima riga contiene,
    in ordine, il primo LED di ogni strip di ogni scheda; la
    seconda riga il secondo LED di ogni strip, e così via.

    Ogni scheda "possiede" un blocco verticale di
    strips_per_board colonne consecutive. Se il cablaggio reale
    è diverso, è questo l'unico punto da modificare.
    """
    total_cols = num_boards * strips_per_board

    # Step 1: ricostruisco la matrice logica [riga=led_idx][colonna=strip] = (h, s, v)
    matrix = []
    idx = 0
    for _led_idx in range(strip_length):
        row = []
        for _col in range(total_cols):
            row.append(image_hsv[idx:idx + 3])
            idx += 3
        matrix.append(row)

    # Step 2: estraggo il payload per ciascuna scheda
    board_payloads = []
    for board_id in range(num_boards):
        payload = []
        for strip_idx in range(strips_per_board):
            col = board_id * strips_per_board + strip_idx
            for led_idx in range(strip_length):
                payload.extend(matrix[led_idx][col])
        board_payloads.append(payload)

    return board_payloads


def send_packet(ser, target_id, cmd, payload=None):
    """Invia un pacchetto [STX][ID][CMD][DATA...][CHK] sulla seriale."""
    if payload is None:
        payload = []

    checksum = target_id ^ cmd
    for b in payload:
        checksum ^= b

    packet = bytearray([STX, target_id, cmd]) + bytearray(payload) + bytearray([checksum])
    ser.write(packet)
    ser.flush()
    print(f"TX -> ID {target_id}, CMD 0x{cmd:02X}, LEN={len(payload)}, CHK=0x{checksum:02X}")


def open_serial(port, baudrate, timeout):
    """Apre la porta seriale con messaggi di errore chiari in caso di problemi."""
    try:
        return serial.Serial(port, baudrate, timeout=timeout)
    except serial.SerialException as exc:
        available = [p.device for p in list_ports.comports()]
        print(f"ERRORE: impossibile aprire la porta seriale '{port}': {exc}")
        if available:
            print(f"Porte disponibili: {', '.join(available)}")
        else:
            print("Nessuna porta seriale rilevata sul sistema.")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Invia un'immagine alla matrice di schede LED")
    parser.add_argument(
        "image", nargs="?", default=None,
        help="Percorso immagine da inviare (PNG/JPG/...). Se omesso, usa i dati di test."
    )
    parser.add_argument("--port", default=SERIAL_PORT, help=f"Porta seriale (default: {SERIAL_PORT})")
    parser.add_argument("--boards", type=int, default=NUM_BOARDS, help=f"Numero di schede (default: {NUM_BOARDS})")
    args = parser.parse_args()

    if args.image:
        try:
            image_hsv = load_image_as_hsv(args.image, MATRIX_WIDTH, MATRIX_HEIGHT)
            print(f"Immagine '{args.image}' caricata e ridimensionata a {MATRIX_WIDTH}x{MATRIX_HEIGHT}")
        except (FileNotFoundError, OSError) as exc:
            print(f"ERRORE: impossibile caricare l'immagine '{args.image}': {exc}")
            sys.exit(1)
    else:
        image_hsv = TEST_IMAGE_HSV
        print("Nessuna immagine specificata: uso i dati di test integrati")

    expected_len = MATRIX_WIDTH * MATRIX_HEIGHT * 3
    if len(image_hsv) != expected_len:
        print(f"ERRORE: dati HSV di lunghezza {len(image_hsv)}, attesi {expected_len}")
        sys.exit(1)

    if STRIP_LENGTH > MAX_STRIP_LENGTH_HW:
        print(f"ERRORE: STRIP_LENGTH={STRIP_LENGTH} supera il limite hardware "
              f"delle schede ({MAX_STRIP_LENGTH_HW}). Verrà scartato dall'Arduino.")
        sys.exit(1)

    board_payloads = build_board_payloads(image_hsv, args.boards, STRIPS_PER_BOARD, STRIP_LENGTH)

    ser = open_serial(args.port, BAUDRATE, SERIAL_TIMEOUT_S)
    try:
        # Configura la geometria (STRIP_LENGTH) su tutte le schede in broadcast,
        # una volta sola, prima di inviare i payload. Le schede applicano subito
        # la nuova lunghezza alle proprie strip NeoPixel.
        send_packet(ser, BROADCAST_ID, CMD_CONFIG, [STRIP_LENGTH])
        time.sleep(CONFIG_DELAY_S)

        for board_id, payload in enumerate(board_payloads):
            send_packet(ser, board_id, CMD_STORE, payload)
            time.sleep(INTER_PACKET_DELAY_S)

        send_packet(ser, BROADCAST_ID, CMD_APPLY)
        print("Broadcast APPLY inviato: sync completata")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
