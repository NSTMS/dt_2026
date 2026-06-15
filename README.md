# Dual-Tech 2026

Workspace ROS 2 (Jazzy) dla czołgu (UGV) i drona (UAV): napęd, chwytak, detekcja YOLO/QR, telemetria MAVROS.

## Wymagania

- Ubuntu 24.04 + [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Raspberry Pi 5 (czołg i/lub komputer pokładowy drona)
- Zależności systemowe (przykład):

```bash
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-msgs \
  python3-lgpio gstreamer1.0-tools gstreamer1.0-plugins-{base,good,bad,ugly}
```

## Budowanie

```bash
cd ~/dual_tech_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Ustaw ten sam `ROS_DOMAIN_ID` na wszystkich maszynach, które mają się widzieć w sieci ROS (domyślnie `0`):

```bash
export ROS_DOMAIN_ID=0
```

---

## Czołg (UGV)

Stack uruchamia: czyszczenie GPIO → sterownik silników → chwytak (stepper + serwo) → opcjonalnie teleop z klawiatury → detekcja z kamery CSI.

### Uruchomienie ręczne

W **interaktywnym** terminalu SSH (wymagane do sterowania klawiaturą):

```bash
source /opt/ros/jazzy/setup.bash
source ~/dual_tech_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch tank_motor tank.launch.py
```

### Argumenty launch

| Argument     | Domyślnie | Opis |
|--------------|-----------|------|
| `teleop`     | `true`    | Sterowanie klawiaturą (`keyboard_controller`) |
| `detection`  | `true`    | Detekcja YOLO z kamery (`ugv_detection_pub`) |

Przykłady:

```bash
# Bez detekcji (mniejsze obciążenie CPU)
ros2 launch tank_motor tank.launch.py detection:=false

# Bez teleopu w launch — uruchom sterowanie w osobnym terminalu
ros2 launch tank_motor tank.launch.py teleop:=false
ros2 launch tank_motor teleop.launch.py
```

### Sterowanie czołgiem

| Klawisz | Akcja |
|---------|-------|
| W / S | Jazda przód / tył |
| A / D | Skręt lewo / prawo |
| Spacja | Stop + pozycja domowa chwytaka |
| ↑ / ↓ | Zmiana prędkości |
| H / K | Szczęki: zamknij / otwórz |
| U / J | Ramię: w dół / w górę |
| Q | Wyjście |

### Usługa systemd (opcjonalnie)

```bash
sudo cp tank_motor/systemd/dualtech-tank.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dualtech-tank.service
```

Usługa startuje stack **bez** teleopu (`teleop:=false`). Sterowanie uruchom osobno w terminalu:

```bash
ros2 launch tank_motor teleop.launch.py
```

---

## Dron (UAV)

Stack uruchamia: MAVROS (połączenie z FC przez USB) → detekcja z strumienia wideo UDP → serwo zrzutu ładunku.

### Przygotowanie

1. Podłącz kontroler lotu do Raspberry Pi (domyślnie `/dev/ttyACM0`).
2. Uruchom strumień wideo H.264 na UDP port **5000** (np. z kamery/kompresora na dronie).

### Uruchomienie ręczne

W **interaktywnym** terminalu SSH:

```bash
source /opt/ros/jazzy/setup.bash
source ~/dual_tech_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch uav_detection drone.launch.py
```

Jeśli FC jest na innym porcie, edytuj `fcu_url` w `uav_detection/launch/drone.launch.py` lub uruchom węzeł mavros osobno z własnymi parametrami.

### Sterowanie zrzutem

W terminalu, w którym działa `servo_controller`:

| Klawisz | Akcja |
|---------|-------|
| Spacja | Zrzut ładunku (ruch serwa) |
| Enter | Ruch serwa w drugą stronę |

Zrzut można też wyzwolić z pilota RC (kanał 8, domyślnie) lub automatycznie po wykryciu kodu QR (gdy `TRIGGER_DROP_ON_QR=true`).

### Zmienne środowiskowe (opcjonalnie)

```bash
export UDP_PORT=5000              # port strumienia wideo
export TARGET_FPS=15              # docelowa liczba klatek detekcji
export TRIGGER_DROP_ON_QR=true    # automatyczny zrzut po QR
```

### Usługa systemd (opcjonalnie)

```bash
sudo cp uav_detection/systemd/uav-detection.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now uav-detection.service
```

---

## Pakiety w workspace

| Pakiet | Rola |
|--------|------|
| `tank_motor` | Napęd L298N, chwytak, teleop |
| `ugv_detection` | Detekcja obiektów na czołgu (kamera CSI) |
| `uav_detection` | MAVROS, detekcja ze strumienia UDP, serwo zrzutu |
| `dualtech_detection` | Wspólna logika YOLO/QR |
| `dualtech_msgs` | Wiadomości ROS między węzłami |
