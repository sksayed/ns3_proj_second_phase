# WiFi RSSI per-STA report

- Input: `/home/sayed/ns3_phase_2/ns-3.45/Wifi_hybrid_matrix_screening_lte_2g_sta5_50kb_seed6_th-58_h2_t30/wifi-hybrid-rssi_log.csv`
- WiFi column summarized: `inst_rssi_dbm`
- Histogram bin step: 3 dB (bins are `(top-step, top]`)

## STA identity check

All rows match `node_id == sta_index + 6`.

## wifi_rssi

| STA | sta_index | node_id | samples | max (dB) @ time(s) | min (dB) @ time(s) | mean (dB) |
| --- | ---: | ---: | ---: | --- | --- | --- |
| STA1 | 0 | 6 | 347 | -52.7735 @ 0.0670 | -52.8780 @ 29.9066 | -52.8190 |
| STA2 | 1 | 7 | 386 | -47.3167 @ 0.0613 | -47.6209 @ 29.9104 | -47.4397 |
| STA3 | 2 | 8 | 299 | -49.8916 @ 0.0537 | -51.0705 @ 29.9167 | -50.4799 |
| STA4 | 3 | 9 | 297 | -52.2359 @ 29.9167 | -52.8047 @ 0.0577 | -52.4995 |
| STA5 | 4 | 10 | 384 | -51.4546 @ 0.0633 | -52.9150 @ 29.9104 | -52.1375 |

### Per-STA histograms (highest to lowest)

#### STA1 (sta_index=0, node_id=6)

- samples: 347
- max: -52.7735 at 0.0670 s
- min: -52.8780 at 29.9066 s
- mean: -52.8190
- wifi avg_rssi_dbm at max time: -52.7735

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -51 | (-54, -51] | 347 |
| -54 | (-57, -54] | 0 |

#### STA2 (sta_index=1, node_id=7)

- samples: 386
- max: -47.3167 at 0.0613 s
- min: -47.6209 at 29.9104 s
- mean: -47.4397
- wifi avg_rssi_dbm at max time: -47.3167

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -45 | (-48, -45] | 386 |
| -48 | (-51, -48] | 0 |

#### STA3 (sta_index=2, node_id=8)

- samples: 299
- max: -49.8916 at 0.0537 s
- min: -51.0705 at 29.9167 s
- mean: -50.4799
- wifi avg_rssi_dbm at max time: -49.8916

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -48 | (-51, -48] | 281 |
| -51 | (-54, -51] | 18 |
| -54 | (-57, -54] | 0 |

#### STA4 (sta_index=3, node_id=9)

- samples: 297
- max: -52.2359 at 29.9167 s
- min: -52.8047 at 0.0577 s
- mean: -52.4995
- wifi avg_rssi_dbm at max time: -52.2506

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -51 | (-54, -51] | 297 |
| -54 | (-57, -54] | 0 |

#### STA5 (sta_index=4, node_id=10)

- samples: 384
- max: -51.4546 at 0.0633 s
- min: -52.9150 at 29.9104 s
- mean: -52.1375
- wifi avg_rssi_dbm at max time: -51.4546

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -51 | (-54, -51] | 384 |
| -54 | (-57, -54] | 0 |

