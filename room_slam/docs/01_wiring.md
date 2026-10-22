# Wiring the RPLIDAR A1 to the ESP32

## The connector

An RPLIDAR A1M8 kit comes with a small USB adapter board and a cable. On the
adapter you will find five pins (names printed on the board, colours vary
between batches — **trust the labels, not the colours**):

| Pin | Meaning | Goes to |
| --- | --- | --- |
| `V5.0` / `VCC` | 4.9 – 5.5 V for the scanner core | +5 V of a **2 A** supply |
| `GND` | ground | common ground |
| `TX` | lidar sends data (3.3 V logic) | ESP32 **GPIO16** (RX2) |
| `RX` | lidar receives commands | ESP32 **GPIO17** (TX2) |
| `MOTOCTL` | motor enable / PWM, active high | ESP32 **GPIO25**, or +5 V |

Some kits expose the motor as two extra wires instead (`5V_MOTO`, `GND_MOTO`,
`CTRL_MOTO`). Same idea: motor 5 V and motor GND to the supply, CTRL to GPIO25.

```
            5 V / 2 A supply
             +--------+---------+
             |        |         |
            [5V]    [GND]       |
             |        |         |
   ┌─────────┴──┐  ┌──┴──────┐  │
   │ RPLIDAR A1 │  │ ESP32   │  │
   │            │  │         │  │
   │ VCC 5V  ───┘  │         │  │
   │ GND     ──────┴─ GND ◀──┘  │     (all grounds joined)
   │ TX      ────────▶ GPIO16   │
   │ RX      ◀──────── GPIO17   │
   │ MOTOCTL ────────▶ GPIO25   │
   └────────────────────────────┘
```

## Why these pins

* **GPIO16 / GPIO17** are UART2 on the ESP32 (`Serial2`). They are plain IO
  pins: not strapping pins, not the flash pins, and they do not disturb the USB
  serial monitor (which owns UART0). Any other free pair works too — just change
  `LIDAR_RX_PIN` / `LIDAR_TX_PIN` in the sketch.
* **GPIO25** is a normal output and also a DAC pin, so it can do
  `analogWrite()` if you later want to slow the motor down for a higher angular
  resolution. Tying MOTOCTL to 5 V also works (full speed).

## Logic levels

The A1M8 UART is specified as 0 – 3.5 V, with the input threshold at 1.6 V, so
it is directly compatible with a 3.3 V ESP32 **in both directions**. You do not
need a level shifter. Do not feed 5 V into the ESP32's RX pin.

## Power — read this before you plug anything in

The lidar is the diva of this project:

* core: 5 V, **minimum 4.9 V**, 2 W typical;
* the motor adds short current spikes.

If the voltage dips, the lidar resets mid-sweep. Symptoms: scans with random
"walls" close to the robot, `badNodes` climbing in the serial monitor, or the
map containing a ghost circle around the start point.

Do this:

1. Give the lidar its **own** 5 V supply rated 2 A or more (a phone charger or
   a power bank is fine). Do not take 5 V from the ESP32's USB pin if you can
   avoid it.
2. Join all grounds: lidar GND, supply GND, ESP32 GND.
3. Solder a **470 µF electrolytic** (16 V) across the lidar's 5 V and GND,
   as close to the lidar as possible. This is cheap insurance.
4. Power the ESP32 from its own USB cable while you are developing. On the robot
   later, use one 5 V battery and two regulators, or a 5 V/3 A UBEC plus a small
   3.3 V LDO for the ESP32.

## Bench test

1. Power the supply, leave the ESP32 unplugged. The lidar should spin up as soon
   as MOTOCTL is high (or as soon as the sketch drives GPIO25).
2. Flash the sketch, open the serial monitor at 115200.
3. You should see `# lidar: scan mode started`, and after ~1 s,
   `beams=352 good=...` in the 5-second status line.

If the motor does not spin: MOTOCTL not connected, or your supply cannot deliver
the current. If `beams=0`: TX/RX swapped, wrong pins, or baud rate
(115200) mismatch.

## Mounting on the robot

* Place the lidar on the **rotation centre** of the robot if you can (midway
  between the two driven wheels). If it is not, measure the offset and set
  `LIDAR_OFFSET_X` / `LIDAR_OFFSET_Y` in `slam2d/config.py`.
* Keep it level. A 3° tilt makes walls converge/diverge in the map.
* Nothing should block the 360° view — cable, antenna, handle… all become
  permanent "obstacles" in the map. If something must be in the way, tell the
  filter: raise `MIN_RANGE_M` above that radius (e.g. `0.25`) to ignore the
  robot's own body.
* A 2D lidar sees a slice at one height. A sofa looks like a wall; a chair looks
  like four legs; a glass door may look like nothing. Expect the map to show
  everything taller than the lidar height.
