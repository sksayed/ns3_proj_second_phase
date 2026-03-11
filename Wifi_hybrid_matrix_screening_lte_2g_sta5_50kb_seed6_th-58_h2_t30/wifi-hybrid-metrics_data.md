# FlowMonitor STA Metrics

| STA IP | Traffic Types | L4 Protocols | PDR (%) | Avg Delay (ms) | Avg Jitter (ms) | Throughput (Mbps) | TX Packets | RX Packets | Lost Packets |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 192.168.1.1 | TCP Download (TCP), TCP Upload (TCP), VoIP (UDP) | TCP/UDP | 100.00 | 1.54 | 0.30 | 1.6536 | 829 | 829 | 0 |
| 192.168.1.2 | TCP Download (TCP), TCP Upload (TCP), VoIP (UDP) | TCP/UDP | 100.00 | 1.60 | 0.28 | 1.6535 | 829 | 829 | 0 |
| 192.168.1.3 | TCP Download (TCP), TCP Upload (TCP), VoIP (UDP) | TCP/UDP | 100.00 | 0.35 | 0.16 | 0.9203 | 755 | 755 | 0 |
| 192.168.1.4 | TCP Download (TCP), TCP Upload (TCP), VoIP (UDP) | TCP/UDP | 100.00 | 0.35 | 0.15 | 0.9203 | 755 | 755 | 0 |
| 192.168.1.5 | TCP Download (TCP), TCP Upload (TCP), VoIP (UDP) | TCP/UDP | 100.00 | 0.84 | 0.19 | 1.6535 | 829 | 829 | 0 |
| **Average** |  |  | 100.00 | 0.94 | 0.22 | 1.3603 |  |  |  |

## Phase 1 KPI Snapshot

- Active clients: **5**
- Delay target compliance (<= 200.0 ms): **5/5 (100.00%)**
- Aggregate reception rate (sum throughput, active clients): **6.8013 Mbps**
- Data reception rate for >=3 clients (Phase 1 KPI): **6.8013 Mbps**
- Switch events: **0** (to cellular: 0, to WiFi: 0)
- Avg switching latency (service interruption = recovered-lastOK): **N/A**

## Switching Events

No switching events captured.
