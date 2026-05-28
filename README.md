ROS2 Vizualizačné Webové Rozhranie

V tejto časti sa zameriava na vytvorenie webového vizualizačného rozhrania pre robotický systém využívajúci ROS2. Cieľom je umožniť zobrazovanie dát zo senzorov a mapovania prostredia priamo vo webovom prehliadači v reálnom čase.

Systém umožňuje:

- vizualizáciu LiDAR dát,
- zobrazenie kamery robota,
- vizualizáciu mapy prostredia,
- komunikáciu medzi ROS2 a webovým rozhraním pomocou WebSocketu,
ovládanie a monitoring systému cez webové rozhranie.

Projekt pozostáva z backendovej a frontendovej časti.

Architektúra systému
ROS2 Nodes
   │
   ├── LiDAR
   ├── Kamera
   ├── IMU
   └── Mapovací node
           │
           ▼
    rosbridge WebSocket
           │
           ▼
     Webové rozhranie 


# 1. Spustenie rosbridge servera

Webová aplikácia komunikuje s ROS2 pomocou `rosbridge_server`.

Spustenie:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

# 2. Spustenie webovej stránky

Prejdeme do priečinka webového rozhrania

```bash
cd src/web_interface
```

Spustíme jednoduchý HTTP server

```bash
python3 -m http.server 8000
```

Webová stránka bude dostupná na:

```text
http://localhost:8000
```

V prípade prístupu z iného zariadenia v sieti (cez JETSON_ORIN):

```text
http://10.42.0.1:8000
```

---
# 4. Spustenie node system_controller

```bash
ros2 run pointcloud_publisher system_controller_node
```
# 5. ROS2 topicy

## Pointcloud publisher

| Topic | Typ |
|---|---|
| `/lidar_points` | subscribe |
| `/lidar` | publish |

---

## Camera publisher

| Topic | Typ |
|---|---|
| `/rgb_img` | subscribe |
| `/camera_image/compressed` | publish |

---

## Map builder

| Topic | Typ |
|---|---|
| `/cloud_registered` | subscribe |
| `/map_points` | publish |

---

## System controller

| Topic / Service | Typ |
|---|---|
| `/ui/status` | publish |
| `/start_system` | service |
| `/stop_system` | service |
| `/list_recordings` | service |
| `/run_all` | service |

---


AGX Orin.
