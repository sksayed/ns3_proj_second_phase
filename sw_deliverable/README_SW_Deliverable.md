# Robot NW Connection Simulator — SW Deliverable Package

Submission package for **Robot NW Connection Simulator**, structured like the Anomaly Detection SW Unit reference:

For each SW Unit: **(1) SW_Info** · **(2) Code** · **(3) Overview diagram(s)**

## Units

| Unit | Folder | SW Name |
|------|--------|---------|
| 1 | `SWUnit1_HybridSimulator_NS3/` | Robot NW Hybrid Network Simulator (NS-3 / C++) |
| 2 | `SWUnit2_FlowmonParser/` | Robot NW FlowMonitor Data Parser (Python) |
| 3 | `SWUnit3_Visualization/` | Robot NW Connection Visualization Tools (Python) |

## Folder contents (per unit)

```
SWUnitN_*/
  SW_Info_*.txt                 # SW info + how to run
  Overview_*.png                # 1–2 overview diagrams
  code/                         # Source files for this unit
```

## Regenerate diagrams

```bash
cd ns-3.45/sw_deliverable
python3 generate_overview_diagrams.py
```

Requires: `matplotlib`
