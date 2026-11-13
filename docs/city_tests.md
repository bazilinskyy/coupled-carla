Smoke test, 2026-11-13: 40 s headless autopilot trial per map, CARLA 0.9.16 at Epic, 50 Hz, on lin4080 (RTX 4080 SUPER shared with another job).

| Map | Continent | Size | Traffic | Lane links | Buildings | SUMO vehicles shown | Real-time factor | Left road | Stuck | Put back on lane | Collisions | Result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Eindhoven | Europe | medium | right | 2248 | 3758 | 8 | 0.878 | 0 | 0 | 0 | 14 | ok |
| Cairo (Tahrir) | Africa | large | right | 3837 | 650 | 8 | 0.813 | 2 | 0 | 2 | 6 | ok |
| Kigali | Africa | medium | right | 1655 | 1741 | 1 | 0.866 | 1 | 0 | 1 | 0 | ok |
| Stellenbosch | Africa | small | left | 1748 | 1291 | 3 | 0.822 | 0 | 0 | 0 | 26 | ok |
| Tokyo (Shibuya) | Asia | large | left | 5076 | 3770 | 1 | 0.678 | 17 | 0 | 17 | 152 | ok |
| Taipei (Main Station) | Asia | medium | right | 2902 | 2558 | 3 | 0.735 | 0 | 0 | 0 | 0 | ok |
| Luang Prabang | Asia | small | right | 1418 | 1144 | 1 | 0.989 | 2 | 0 | 2 | 0 | ok |
| London (Westminster) | Europe | large | left | 2756 | 2086 | 0 | 0.846 | 1 | 0 | 1 | 160 | ok |
| Lyon (Bellecour) | Europe | medium | right | 2312 | 2481 | 3 | 0.86 | 2 | 0 | 2 | 171 | ok |
| Leuven | Europe | small | right | 1537 | 5030 | 4 | 0.993 | 2 | 0 | 2 | 65 | ok |
| New York (Midtown) | North America | large | right | 2213 | 1590 | 11 | 0.657 | 1 | 0 | 1 | 7 | ok |
| Portland, Oregon | North America | medium | right | 4356 | 703 | 2 | 0.549 | 0 | 0 | 0 | 7 | ok |
| Burlington, Vermont | North America | small | right | 2034 | 1849 | 4 | 0.882 | 2 | 0 | 2 | 0 | ok |
| São Paulo (Sé) | South America | large | right | 2181 | 2977 | 2 | 0.751 | 1 | 0 | 1 | 0 | ok |
| Montevideo | South America | medium | right | 1917 | 741 | 3 | 0.675 | 0 | 0 | 0 | 0 | ok |
| Ushuaia | South America | small | right | 2403 | 1024 | 1 | 0.941 | 3 | 0 | 3 | 1 | ok |
| Sydney (CBD) | Oceania | large | left | 2598 | 1021 | 0 | 0.663 | 0 | 0 | 0 | 0 | ok |
| Auckland (CBD) | Oceania | medium | left | 3923 | 1140 | 0 | 0.669 | 0 | 0 | 0 | 4 | ok |
| Nouméa | Oceania | small | right | 1784 | 2468 | 3 | 0.97 | 0 | 0 | 0 | 1 | ok |
