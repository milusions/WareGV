# WareGV: Warehouse Autonomous Ground Vehicle


![WareGV Thumbnail](assets/thumbnail.png)

An autonomous warehouse ground vehicle that uses SLAM and Nav2 to navigate a building and pick up / deliver goods.

Built with **ROS 2 Jazzy**, **Gazebo Harmonic**, **SLAM**, and **Nav2**.

---

## Start Simulation

**Mapping mode:**
```bash
./start_sim.sh true false
```

**Autonomous navigation mode:**
```bash
./start_sim.sh <mapping-enabled> <navigation-enabled> <map_name> <world_name>
```

> Once RViz loads in navigation mode, manually set the initial pose to start the map topic stream.

---

## Save Map

Save a map created during mapping:
```bash
./save_map.sh <map_name>
```

The map is saved to `waregv_description/map`.

---

## View URDF

```bash
./view_urdf.sh
```
