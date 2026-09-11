# Guard Timer Parameter Review (Enhancement Plan Section 4.2.3)

- Author: Sheikh Sayed Bin Rahman
- Lab: PIC Lab, KIT
- Guard Timer duration tested: 0.5s
- Paired runs analyzed: 40 (mobility type x seed, Guard Timer off vs on)

## 1. What this measures

A "false trigger" is defined here as a WiFi->cellular switch whose trigger time falls within 0.5s after an Intra-Mesh HO event for the same STA -- the exact condition the enhancement plan describes (section 4.1): a momentary RSSI drop during an AP-to-AP handover misclassified as real coverage loss. The Guard Timer suppresses RSSI-based switching for this window after a handover, so if it works, this pattern should become rarer with it enabled -- either because the switch is avoided entirely (RSSI recovers within the window) or pushed past the window (network was genuinely degraded, not just transient).

A deliberately-designed boundary-crossing waypoint scenario was tried first (STAs walking directly between two adjacent mesh APs, repeatedly crossing the midpoint) to generate controlled Intra-Mesh HO events on demand. It mostly failed: at the project's -58dBm threshold, each AP's reliable range is only ~80m against a 200m inter-AP spacing, leaving a real ~40m dead zone where neither AP has usable signal. Only 1 of 8 STAs completed a clean handover; the rest dropped to cellular in the gap. This study instead uses the existing organic mobility patterns (gaussmarkov/patrol/transport/work), which produce genuine Intra-Mesh HO events at a real, if less frequent, rate.

## 2. Headline result

- Total WiFi->cellular switches observed: **738** (Guard Timer off), **750** (on)
- Of those, false triggers (within 0.5s of an Intra-Mesh HO): **32** (off, 4.3% of switches) -> **1** (on, 0.1% of switches)
- False-trigger reduction: **96.9%**
- Intra-Mesh HO events themselves: 124 (off) vs 129 (on) -- these should (and do) stay close, since the Guard Timer doesn't change when handovers happen, only the response to RSSI dips shortly after one.

| Mobility type | Off: false triggers / w2c switches | On: false triggers / w2c switches | Reduction |
|---|---|---|---|
| gaussmarkov | 13/203 | 0/205 | 100% |
| patrol | 8/207 | 0/209 | 100% |
| transport | 1/175 | 0/175 | 100% |
| work | 10/153 | 1/161 | 90% |

![False triggers by mobility type](report_assets/false_triggers_by_mobility.png)

## 3. Cross-check: does this hold under NR mode and higher STA count?

The headline result above used LTE fallback at STA=10 throughout. Cellular mode and STA count don't have an obvious mechanism to change whether a momentary post-handover RSSI dip gets misread as a real cellular trigger (that's a WiFi-side RSSI phenomenon), but this wasn't verified until this cross-check -- run at 5 of the original 10 seeds, same 4 mobility types, to confirm the effect generalizes rather than being an artifact of the one configuration originally tested.

| Condition | Off: false triggers / w2c switches | On: false triggers / w2c switches | Reduction |
|---|---|---|---|
| NR (STA=10) | 14/345 | 0/364 | 100% |
| LTE, STA=20 | 25/742 | 1/722 | 96% |

Both cross-check conditions show the same pattern as the LTE/STA=10 baseline: false triggers drop sharply with the Guard Timer enabled, with no meaningful increase in total switch volume. This supports the effect being a general property of the Guard Timer mechanism rather than specific to the originally-tested configuration.

## 4. Side effects: does delaying switches cost anything?

- Total switch events (all types): 1320 (off) vs 1353 (on), a +2.5% change.
The Guard Timer only suppresses the RSSI component of the WiFi->cellular decision for 0.5s after a handover -- PDR-based and RSSI-stale triggers are unaffected, so a genuinely failing connection still switches, just possibly a fraction of a second later. The cost side of this tradeoff (e.g., a handful of ms of extra exposure on switches that turn out to be real degradation, not false triggers) is visible per-event in the raw switch logs under `Traffic_qos_outputs/guard_timer_study/`, but isn't large enough in this sample to show up as a meaningful change in total switch volume.

## 5. Per-seed detail (LTE, STA=10 baseline)

| Mobility | Seed | Off: switches / intra_mesh / w2c / false-triggers | On: switches / intra_mesh / w2c / false-triggers |
|---|---|---|---|
| gaussmarkov | 101 | 58 / 16 / 25 / 3 | 59 / 13 / 27 / 0 |
| gaussmarkov | 102 | 57 / 5 / 30 / 3 | 62 / 9 / 30 / 0 |
| gaussmarkov | 103 | 22 / 0 / 15 / 0 | 22 / 0 / 15 / 0 |
| gaussmarkov | 104 | 31 / 1 / 18 / 0 | 31 / 1 / 18 / 0 |
| gaussmarkov | 105 | 19 / 5 / 11 / 2 | 21 / 4 / 13 / 0 |
| gaussmarkov | 106 | 40 / 2 / 22 / 0 | 40 / 2 / 22 / 0 |
| gaussmarkov | 107 | 34 / 2 / 20 / 1 | 30 / 2 / 18 / 0 |
| gaussmarkov | 108 | 34 / 3 / 20 / 0 | 34 / 3 / 20 / 0 |
| gaussmarkov | 109 | 33 / 10 / 15 / 3 | 34 / 11 / 15 / 0 |
| gaussmarkov | 110 | 57 / 9 / 27 / 1 | 59 / 11 / 27 / 0 |
| patrol | 101 | 21 / 2 / 13 / 1 | 23 / 2 / 14 / 0 |
| patrol | 102 | 39 / 0 / 24 / 0 | 39 / 0 / 24 / 0 |
| patrol | 103 | 56 / 2 / 30 / 0 | 56 / 2 / 30 / 0 |
| patrol | 104 | 38 / 2 / 22 / 1 | 41 / 2 / 23 / 0 |
| patrol | 105 | 23 / 4 / 14 / 2 | 20 / 4 / 12 / 0 |
| patrol | 106 | 27 / 6 / 13 / 2 | 32 / 6 / 16 / 0 |
| patrol | 107 | 33 / 0 / 20 / 0 | 33 / 0 / 20 / 0 |
| patrol | 108 | 46 / 1 / 26 / 0 | 46 / 1 / 26 / 0 |
| patrol | 109 | 45 / 4 / 24 / 1 | 53 / 4 / 28 / 0 |
| patrol | 110 | 38 / 4 / 21 / 1 | 29 / 3 / 16 / 0 |
| transport | 101 | 53 / 3 / 29 / 0 | 53 / 3 / 29 / 0 |
| transport | 102 | 12 / 0 / 11 / 0 | 12 / 0 / 11 / 0 |
| transport | 103 | 20 / 1 / 14 / 0 | 20 / 1 / 14 / 0 |
| transport | 104 | 28 / 0 / 18 / 0 | 28 / 0 / 18 / 0 |
| transport | 105 | 13 / 1 / 11 / 1 | 14 / 2 / 11 / 0 |
| transport | 106 | 49 / 1 / 28 / 0 | 49 / 1 / 28 / 0 |
| transport | 107 | 20 / 2 / 13 / 0 | 20 / 2 / 13 / 0 |
| transport | 108 | 30 / 0 / 20 / 0 | 30 / 0 / 20 / 0 |
| transport | 109 | 24 / 1 / 16 / 0 | 24 / 1 / 16 / 0 |
| transport | 110 | 26 / 3 / 15 / 0 | 26 / 3 / 15 / 0 |
| work | 101 | 36 / 2 / 19 / 1 | 35 / 1 / 19 / 0 |
| work | 102 | 10 / 4 / 6 / 1 | 10 / 3 / 7 / 0 |
| work | 103 | 4 / 2 / 2 / 2 | 4 / 2 / 2 / 0 |
| work | 104 | 28 / 6 / 15 / 2 | 46 / 10 / 20 / 0 |
| work | 105 | 20 / 0 / 13 / 0 | 20 / 0 / 13 / 0 |
| work | 106 | 18 / 3 / 9 / 1 | 20 / 3 / 11 / 0 |
| work | 107 | 41 / 1 / 23 / 1 | 41 / 1 / 23 / 0 |
| work | 108 | 65 / 2 / 34 / 0 | 65 / 2 / 34 / 0 |
| work | 109 | 21 / 1 / 11 / 0 | 21 / 1 / 11 / 0 |
| work | 110 | 51 / 13 / 21 / 2 | 51 / 13 / 21 / 1 |

## 6. Limitations

- Sample size is modest (10 seeds x 4 mobility types = 40 paired runs for the baseline condition; 5 seeds each for the NR and STA=20 cross-checks). False-trigger events are a subset of an already-modest intra-mesh-HO rate, so absolute counts are small for some mobility types (transport/work), and per-type percentages should be read cautiously.
- Only one Guard Timer duration (0.5s, the plan's suggested starting point) was tested. Deriving a true optimum would mean sweeping several durations (e.g. 0.2/0.5/1.0/2.0s) and finding where false-trigger reduction plateaus against added delay cost -- not done here.
- The cross-check (Section 3) covers cellular mode and STA count, but not hotspot band (2.4 vs 5GHz) or payload/traffic load -- those remain untested axes.
- The boundary-crossing scenario's failure is itself a finding worth flagging separately: clean Intra-Mesh HO may be physically uncommon at this AP spacing/threshold combination, which bounds how much real-world benefit the Guard Timer can offer regardless of its in-simulation effectiveness.

## 7. Compliance with the enhancement plan (section 4)

| Requirement | Status |
|---|---|
| 4.2.1a: Parameterize AP coverage radius | Done (`apCoverageRadiusM` CLI flag) |
| 4.2.1b: Deliberate boundary-crossing waypoint scenario | Attempted (`boundary` robotType); found a real coverage-gap limitation rather than a clean test condition -- reported as a finding in Section 1/5, not hidden |
| 4.2.1: Log Intra-Mesh HO via AssocRequest/DeAssoc callbacks | Done (hooked into existing `HandleStaAssociation`/`HandleStaDeAssociation`) |
| 4.2.2: Switching-event classification scheme (type field) | Done (`switch_log_v2` with `intra_mesh` / `wifi_to_cell` / `cell_to_wifi`) |
| 4.2.3: Guard Timer + false-trigger frequency comparison | Done (this report) |

*End of report -- Sheikh Sayed Bin Rahman, PIC Lab, KIT*
