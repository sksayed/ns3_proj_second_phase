# August Deliverable: WiFi Mesh Internal Handover Modeling

- Author: Sheikh Sayed Bin Rahman
- ID: 2025210714
- Lab: PIC Lab, KIT
- Re: Enhancement plan Section 4 ("Modeling WiFi Mesh Internal Handover")

This note maps each requirement in Section 4 of the enhancement plan to what was built and verified, so it can be checked directly against `guard_timer_report.pdf` and `traffic_qos.cc`.

---

## 1. The problem being addressed (Section 4.1)

Four mesh APs are fixed in place, and a STA roaming AP-to-AP within the same WiFi mesh (Intra-Mesh Handover) was not explicitly modeled. A momentary RSSI drop during that handover could be misread as real coverage loss, misclassifying a brief (field-measured 0.1-0.5s) Intra-Mesh HO interruption as a full WiFi-to-cellular switch, and mixing two very different kinds of latency together in the analysis.

## 2. What was built (Section 4.2)

**4.2.1 — AP coverage radius and boundary-crossing scenario.** Added `apCoverageRadiusM` as a configurable parameter, and a new `boundary` waypoint robot type that deliberately walks a STA back and forth between two adjacent mesh APs, repeatedly crossing the midpoint. This scenario was built exactly as specified, but it revealed a real limitation rather than providing a clean test condition: at the project's -58dBm threshold, each AP's reliable range is only ~80m against the 200m inter-AP spacing, leaving a genuine ~40m dead zone where neither AP has usable signal. Only 1 of 8 STAs completed a clean handover in testing; the rest dropped to cellular in the gap. This is disclosed as a finding, not hidden, and it directly serves the plan's own Section 4.3 goal of providing "a simulation-based comparison basis for AP placement optimization" — an unplanned but genuine contribution.

Because the boundary scenario couldn't reliably produce controlled handovers, the main study instead used the existing organic mobility patterns (gaussmarkov, patrol, transport, work), which produce genuine Intra-Mesh HO events at a real, if less frequent, rate.

**4.2.1 — Event logging.** Intra-Mesh HO detection is hooked into the existing `HandleStaAssociation`/`HandleStaDeAssociation` callbacks (the project's equivalent of NS-3's AssocRequest/DeAssoc). Two real bugs were found and fixed while implementing this:
- Roaming here is break-before-make, so the STA's previous AP was already erased by the time a new association fired. Fixed by tracking a separate, non-destructive `lastKnownApIndex`.
- The WiFi radio keeps roaming in the background even after a STA's traffic has already switched to cellular. Without a check for this, background reassociation was being logged as a real handover — inflating one early test's count by 6.5x (39 → 6 events after the fix). Fixed by only logging a handover while WiFi is the STA's actual serving path.

**4.2.2 — Classification scheme.** Added a `type` field to the switch log (`intra_mesh` / `wifi_to_cell` / `cell_to_wifi`). Worth noting: the plan's stated cellular-to-WiFi return condition (RSSI recovery ≥ -55dBm) matches this project's existing -58dBm threshold plus 3dB hysteresis exactly (-58 + 3 = -55), confirming the classification only added a label — it didn't change any trigger logic.

**4.2.3 — Guard Timer.** Implemented exactly as specified: suppresses the RSSI-based component of the WiFi-to-cellular decision for 0.5s after an Intra-Mesh HO. PDR-based and stale-RSSI triggers are unaffected, so a genuinely failing connection still switches, just possibly a fraction of a second later.

## 3. Results

**Headline (LTE, STA=10, 10 seeds × 4 mobility types, 80 paired runs):** a "false trigger" (a WiFi→cellular switch within 0.5s after an Intra-Mesh HO for the same STA) dropped from 32 to 1 — a **96.9% reduction** — with total switch volume changing only +2.5%, meaning genuinely-needed switches aren't being meaningfully delayed.

**Cross-check (5 seeds × 4 mobility types each, 80 more runs):** the same pattern holds under NR mode (100% reduction) and a higher STA count of 20 (96% reduction). This supports the effect being a general property of the mechanism rather than an artifact of one configuration.

**Total: 160 runs across 3 conditions, 0 failures.**

## 4. What wasn't done, and why

- **Guard Timer duration sweep.** The plan specifies one concrete value (0.5s) and asks to compare false-switching frequency before/after applying it — which was done thoroughly. It does not specify alternative durations to test, so a broader sweep (e.g., 0.2/1.0/2.0s) to find a "more optimal" value beyond the specified one was not pursued, since it isn't what was actually asked for.
- **Hotspot band and payload/traffic-load cross-checks.** The cross-check covered cellular mode and STA count; band (2.4 vs 5GHz) and payload were not additionally tested.
- **Named deliverable files.** `intra_mesh_ho.cc` was not created as a separate file — the logic was added directly to `traffic_qos.cc`, following the same pattern as Phase 2 Item 1 (where `waypoint_mobility.cc` was similarly implemented inside the existing consolidated scenario file). Likewise, the switch log's **format** matches the `switch_log_v2.csv` specification (the `type` field), but the file itself is still named `wifi-hybrid-switch_log.csv`.

## 5. Attached with this deliverable

1. `traffic_qos.cc` — extended with Intra-Mesh HO detection, classification, Guard Timer, and the boundary-crossing scenario
2. `switch_log_v2_sample.csv` — a representative run's switch log showing the live `type` field
3. `guard_timer_report.pdf` — full parameter review report (160-run analysis)
4. `August_WiFi_Internal_HO_Presentation.pptx` — summary slides of the above

I'm happy to run the additional cross-checks (band, payload) or the duration sweep if useful, but neither is required by the plan as written.

Best regards,
Sheikh Sayed Bin Rahman
PIC Lab, KIT
