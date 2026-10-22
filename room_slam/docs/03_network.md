# Networking: ESP32 → laptop

The ESP32 opens a TCP connection to your laptop and pushes text. That is all.
No brokers, no cloud, no MQTT — just a socket, because a socket is what you can
debug at 2 a.m.

## 1. Find the laptop's LAN address

| OS | Command | Look for |
| --- | --- | --- |
| Windows | `ipconfig` | `IPv4 Address` under the Wi-Fi adapter, e.g. `192.168.1.50` |
| macOS | `ipconfig getifaddr en0` | `192.168.1.50` |
| Linux | `hostname -I` / `ip -4 addr` | `192.168.1.50` |

Put it in the sketch:

```cpp
const char* PC_HOST  = "192.168.1.50";
const uint16_t PC_PORT = 8888;
```

## 2. Same network, and no client isolation

Both devices must be on the same subnet. Two classic traps:

* **Guest WiFi / "AP isolation"** — the router blocks device-to-device traffic.
  Fix: use the main SSID, or a phone hotspot.
* **5 GHz-only network** — the ESP32 is 2.4 GHz only. If your laptop is on
  5 GHz and the ESP32 on 2.4 GHz, many routers still bridge them, but some do
  not. Simplest: put both on the 2.4 GHz SSID, or use a hotspot.

Test from the laptop:

```bash
# Linux / macOS
nc -l 8888            # or: nc -lvn 8888
# Windows PowerShell
Test-NetConnection -ComputerName 192.168.1.50 -Port 8888
```

## 3. Firewall

Windows: Defender Firewall → *Advanced settings* → *Inbound Rules* → *New Rule*
→ Port → TCP → `8888` → Allow. Do it for both Private and Public profiles, or
mark your home network as Private.

macOS: System Settings → Network → Firewall → Options, allow incoming
connections for Python (it asks the first time).

Linux (ufw): `sudo ufw allow 8888/tcp`.

## 4. Give the laptop a stable address

If the router's DHCP hands the laptop a new IP tomorrow, the ESP32 keeps calling
the old one. Two options:

* router admin page → DHCP reservation for the laptop's MAC address
  (best, invisible);
* or a static IP on the laptop in the router's subnet.

## 5. Phone hotspot instead of a router

* Android: Settings → Hotspot → set a simple WPA2 password. The laptop usually
  gets `192.168.43.x`, which is also the gateway.
* iPhone: Personal Hotspot; the laptop gets `172.20.10.x`.

Then `PC_HOST` is whatever the laptop got on that hotspot.

## 6. Diagnosing the connection

| Serial monitor says | Meaning | Do |
| --- | --- | --- |
| `wifi: connecting` forever | wrong SSID/password, or 5 GHz-only network | double-check credentials, use hotspot |
| `wifi: connected` but `tcp: connecting ... failed` | record.py not running, wrong IP, or firewall | start `record.py`, re-check IP, open port 8888 |
| connects, then disconnects every few seconds | WiFi power saving or weak signal | the sketch already calls `WiFi.setSleep(false)`; move closer to the AP |
| connects and streams, then stalls after minutes | laptop sleep, or IP changed | disable sleep; use DHCP reservation |
| `client=1` but `record.py` shows 0 scans | something in between is buffering; check the port matches | |

Handy sanity check — watch the raw stream with netcat:

```bash
nc -l 8888 | head -c 400
```

You should see `SCAN 12345 360` followed by `0:1234 1:1230 ...`.

## 7. The wire protocol (for reference)

```
SCAN <millis> <bins>
0:1234 1:1230 2:0 ... 3:2500 ...        (16 pairs per line, "idx:millimetres")
END
ODOM <millis> <x_m> <y_m> <theta_rad>   (optional, only with wheel encoders)
# free text status / comments
```

* `millis` is the ESP32's `millis()` at the end of the sweep.
* `bins` is the number of angular slots (360 by default = 1° resolution).
* index `i` corresponds to angle `i * 2π / bins`, counter-clockwise in the
  standard ROS convention (x forward, y left).
* `0` mm means "no return in that direction".
* Lines are `\n` terminated; unknown lines are ignored, so you can freely add
  debug lines starting with `#`.

The Python parser (`slam2d/record.py: ScanParser`) is tolerant: it resynchronises
on `SCAN`/`END`, so a dropped WiFi packet costs at most one sweep.
