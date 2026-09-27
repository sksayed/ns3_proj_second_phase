# September deliverables (statistical reinforcement)

Phase-3 enhancement plan — September Weeks 1–4.

## Included (kept)

| Item | Description |
|---|---|
| `Enhancement_Final_Report.pdf` | Final report integrating all 4 enhancement items + stats |
| `Enhancement_Final_Report.md` | Markdown source of the PDF |
| `stats_analysis.md` | Bootstrap / Mann–Whitney / Kruskal–Wallis write-up |
| `bootstrap_ci.csv` | Bootstrap 95% CIs |
| `mann_whitney.csv` | LTE vs 5G NR tests |
| `kruskal_wallis.csv` | STA-count trend tests |
| `switch_bootstrap_ci.csv` | Resolved switch-interruption CIs |
| `figures/` | Report figures |
| `stats_analysis.py` | Analysis script (also in `tools/`) |
| `run_traffic_qos_matrix.py` | 640-run matrix runner (also in `tools/`) |

## Not included

The 640 per-scenario simulation trees under `Traffic_qos_outputs/Traffic_qos_matrix_sep_seeds7to16/` (~36 GB) stay local / gitignored.

## Regenerate

From `ns-3.45/`:

```bash
python3 tools/stats_analysis.py \
  --campaign-dir Traffic_qos_outputs/Traffic_qos_matrix_sep_seeds7to16
```
