# WareGV: Warehouse Autonomous Ground Vehicle


![WareGV Thumbnail](assets/thumbnail.png)

An autonomous warehouse ground vehicle that uses SLAM and Nav2 to navigate a building and pick up / deliver goods.

Built with **ROS 2 Jazzy**, **Gazebo Harmonic**, **SLAM**, and **Nav2**.

---
## Start Simulation

```bash
./bin/start_simulation.sh --world-name small_warehouse --max-linear-velocity 0.8 --mapping-enable false
```

---

## Start Hardware
```bash
./bin/start_hardware.sh --max-linear-velocity 0.8 --mapping-enable false
```

---

## Save Map

Save a map created during mapping:
```bash
./bin/save_map.sh <map_name>
```

The map is saved to `waregv_description/map`. 

---

## View URDF

```bash
./bin/view_urdf.sh
```
