# ESP32-DIV — Extended Fork (Bulletb8)

Fork of [cifertech/ESP32-DIV](https://github.com/cifertech/ESP32-DIV) with an added **Car Popper** module under the SubGHz menu.

---

## What's added

### Car Popper (SubGHz → Car Popper)

RKE (Remote Keyless Entry) signal capture, replay, and targeted sweep tool.

**Authorization gate:** A valid 17-character VIN must be entered before any radio operation is permitted. The VIN is decoded to determine the vehicle's make, year, and correct operating frequency. All operations are logged to `/logs/carpopper.csv` on the SD card.

**Features:**
| Option | Description |
|--------|-------------|
| Scan & Capture Fob | Listens for a real RKE transmission from an existing key fob |
| Replay Captured | Retransmits the last captured signal |
| Targeted Sweep | Cycles through known code ranges for the decoded make/year |
| Change Frequency | Toggle between primary and alternate frequency for the make |
| View Info | Shows decoded VIN details, frequency, protocol notes |

**Supported vehicle makes & frequencies:**

| Make | Primary | Alternate | Code type |
|------|---------|-----------|-----------|
| Toyota / Lexus | 315 MHz | 433.92 MHz | KeeLoq / rolling |
| Honda / Acura | 315 MHz | 433.92 MHz | Rolling |
| Nissan / Infiniti | 315 MHz | 433.92 MHz | ASK/OOK rolling |
| Mazda / Subaru | 315 MHz | 433.92 MHz | FSK rolling |
| VW / Audi / Skoda / Seat | 433.92 MHz | 868 MHz | KeeLoq rolling |
| BMW | 433.92 MHz | 868 MHz | Proprietary rolling |
| Mercedes-Benz | 433.92 MHz | 433.92 MHz | Proprietary rolling |

**Rolling code note:** Modern RKE systems (post ~2000) use rolling/hopping codes (KeeLoq, AUT64). A replayed signal will only work once. The sweep is most effective on pre-2000 fixed-code vehicles. Combined with the OBD immo tool for full entry + start workflow.

**Audit log:** Every session logs VIN, make, year, action, and timestamp to `/logs/carpopper.csv`.

---

## Original features (unchanged)

All original ESP32-DIV features are intact:

- **WiFi:** Packet Monitor, Beacon Spammer, Deauther, Probe Request Flood, Deauth Detector, WiFi Scanner, Captive Portal
- **Bluetooth:** BLE Jammer, BLE Spoofer, Sour Apple, Sniffer, BLE Scanner, BLE Rubber Ducky
- **SubGHz:** Replay Attack, SubGHz Jammer, Saved Profile, **Car Popper** ← new
- **2.4GHz:** NRF24 Scanner, Proto Kill
- **More:** IR Remote, RFID/NFC, GPS
- **Tools:** Serial Monitor, Firmware Update, Touch Calibrate, SD File Manager

---

## Hardware

Same as original ESP32-DIV. See [cifertech/ESP32-DIV](https://github.com/cifertech/ESP32-DIV) for PCB/schematic.

Key components:
- ESP32-S3
- CC1101 sub-GHz radio (315/433/868 MHz)
- TFT touchscreen (240×320)
- SD card slot
- PN532 RFID/NFC

---

## Build

Same process as original — Arduino IDE with ESP32 board support. See upstream README for library requirements.

---

## License

MIT — same as upstream.
