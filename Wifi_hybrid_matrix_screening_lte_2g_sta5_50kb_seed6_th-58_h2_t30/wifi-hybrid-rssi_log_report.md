# Link quality per-STA report

- Input: `/home/sayed/ns3_phase_2/ns-3.45/Wifi_hybrid_matrix_screening_lte_2g_sta5_50kb_seed6_th-58_h2_t30/wifi-hybrid-rssi_log.csv`
- WiFi column summarized: `inst_rssi_dbm`
- Histogram bin step: 3 dB (bins are `(top-step, top]`)

## STA identity check

All rows match `node_id == sta_index + 6`.

## lte_rsrp

| STA | sta_index | node_id | samples | max (dB) @ time(s) | min (dB) @ time(s) | mean (dB) |
| --- | ---: | ---: | ---: | --- | --- | --- |
| STA1 | 0 | 6 | 30000 | -65.3952 @ 0.0002 | -66.8153 @ 29.9992 | -66.1125 |
| STA2 | 1 | 7 | 30000 | -48.1615 @ 0.9802 | -49.9420 @ 29.9992 | -48.7375 |
| STA3 | 2 | 8 | 30000 | -70.5360 @ 0.0002 | -71.1901 @ 29.9962 | -70.8559 |
| STA4 | 3 | 9 | 30000 | -61.5024 @ 29.9992 | -63.4797 @ 0.0002 | -62.5135 |
| STA5 | 4 | 10 | 30000 | -43.8094 @ 0.0002 | -46.5182 @ 29.9992 | -45.1375 |

### Per-STA histograms (highest to lowest)

#### STA1 (sta_index=0, node_id=6)

- samples: 30000
- max: -65.3952 at 0.0002 s
- min: -66.8153 at 29.9992 s
- mean: -66.1125

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -63 | (-66, -63] | 12483 |
| -66 | (-69, -66] | 17517 |
| -69 | (-72, -69] | 0 |

#### STA2 (sta_index=1, node_id=7)

- samples: 30000
- max: -48.1615 at 0.9802 s
- min: -49.9420 at 29.9992 s
- mean: -48.7375

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -48 | (-51, -48] | 30000 |
| -51 | (-54, -51] | 0 |

#### STA3 (sta_index=2, node_id=8)

- samples: 30000
- max: -70.5360 at 0.0002 s
- min: -71.1901 at 29.9962 s
- mean: -70.8559

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -69 | (-72, -69] | 30000 |
| -72 | (-75, -72] | 0 |

#### STA4 (sta_index=3, node_id=9)

- samples: 30000
- max: -61.5024 at 29.9992 s
- min: -63.4797 at 0.0002 s
- mean: -62.5135

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -60 | (-63, -60] | 22411 |
| -63 | (-66, -63] | 7589 |
| -66 | (-69, -66] | 0 |

#### STA5 (sta_index=4, node_id=10)

- samples: 30000
- max: -43.8094 at 0.0002 s
- min: -46.5182 at 29.9992 s
- mean: -45.1375

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| -42 | (-45, -42] | 13603 |
| -45 | (-48, -45] | 16397 |
| -48 | (-51, -48] | 0 |

## lte_sinr

| STA | sta_index | node_id | samples | max (dB) @ time(s) | min (dB) @ time(s) | mean (dB) |
| --- | ---: | ---: | ---: | --- | --- | --- |
| STA1 | 0 | 6 | 30000 | 57.8439 @ 0.0002 | 56.4238 @ 29.9992 | 57.1266 |
| STA2 | 1 | 7 | 30000 | 75.0776 @ 0.9852 | 73.2971 @ 29.9992 | 74.5016 |
| STA3 | 2 | 8 | 30000 | 52.7031 @ 0.0002 | 52.0490 @ 29.9952 | 52.3832 |
| STA4 | 3 | 9 | 30000 | 61.7367 @ 29.9992 | 59.7594 @ 0.0002 | 60.7256 |
| STA5 | 4 | 10 | 30000 | 79.4297 @ 0.0002 | 76.7209 @ 29.9992 | 78.1015 |

### Per-STA histograms (highest to lowest)

#### STA1 (sta_index=0, node_id=6)

- samples: 30000
- max: 57.8439 at 0.0002 s
- min: 56.4238 at 29.9992 s
- mean: 57.1266

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| 60 | (57, 60] | 17562 |
| 57 | (54, 57] | 12438 |
| 54 | (51, 54] | 0 |

#### STA2 (sta_index=1, node_id=7)

- samples: 30000
- max: 75.0776 at 0.9852 s
- min: 73.2971 at 29.9992 s
- mean: 74.5016

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| 78 | (75, 78] | 8167 |
| 75 | (72, 75] | 21833 |
| 72 | (69, 72] | 0 |

#### STA3 (sta_index=2, node_id=8)

- samples: 30000
- max: 52.7031 at 0.0002 s
- min: 52.0490 at 29.9952 s
- mean: 52.3832

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| 54 | (51, 54] | 30000 |
| 51 | (48, 51] | 0 |

#### STA4 (sta_index=3, node_id=9)

- samples: 30000
- max: 61.7367 at 29.9992 s
- min: 59.7594 at 0.0002 s
- mean: 60.7256

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| 63 | (60, 63] | 26128 |
| 60 | (57, 60] | 3872 |
| 57 | (54, 57] | 0 |

#### STA5 (sta_index=4, node_id=10)

- samples: 30000
- max: 79.4297 at 0.0002 s
- min: 76.7209 at 29.9992 s
- mean: 78.1015

| bin (dB) | range (dB) | count |
| ---: | --- | ---: |
| 81 | (78, 81] | 16088 |
| 78 | (75, 78] | 13912 |
| 75 | (72, 75] | 0 |

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

