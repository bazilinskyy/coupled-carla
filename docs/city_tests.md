Smoke test, 2026-11-14: 40 s headless autopilot trial per map, CARLA 0.9.16 at Epic, 50 Hz, on lin4080 (RTX 4080 SUPER, not shared at the time), with the real-world scenery and terrain (Leuven flat, see the README).

| Map | Continent | Size | Traffic | Lane links | Buildings | Building heights | Relief (m) | Scenery meshes | SUMO vehicles shown | Real-time factor | Left road | Stuck | Put back on lane | Collisions | Result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Eindhoven | Europe | medium | right | 2246 | 2998 | 3dbag | 11 | 13711 | 16 | 0.982 | 0 | 0 | 2 | 0 | ok |
| Cairo (Tahrir) | Africa | large | right | 3835 | 1308 | overture | 66 | 10814 | 4 | 0.993 | 0 | 0 | 0 | 0 | ok |
| Kigali | Africa | medium | right | 1655 | 2942 | overture | 188 | 15270 | 2 | 0.992 | 0 | 0 | 0 | 0 | ok |
| Stellenbosch | Africa | small | left | 1748 | 1683 | overture | 32 | 8665 | 7 | 0.991 | 0 | 9 | 9 | 0 | ok |
| Tokyo (Shibuya) | Asia | large | left | 5057 | 3833 | overture | 87 | 15735 | 11 | 0.986 | 0 | 1 | 1 | 0 | ok |
| Taipei (Main Station) | Asia | medium | right | 2940 | 2679 | overture | 48 | 15461 | 0 | 0.993 | 0 | 2 | 2 | 0 | ok |
| Luang Prabang | Asia | small | right | 1418 | 3611 | overture | 87 | 10402 | 2 | 0.995 | 0 | 0 | 0 | 0 | ok |
| London (Westminster) | Europe | large | left | 2751 | 2069 | overture | 30 | 10053 | 1 | 0.994 | 0 | 1 | 1 | 0 | ok |
| Lyon (Bellecour) | Europe | medium | right | 2276 | 2518 | bdtopo | 135 | 19667 | 3 | 0.991 | 0 | 2 | 2 | 0 | ok |
| Leuven | Europe | small | right | 1537 | 5049 | overture | – | 19197 | 4 | 0.992 | 0 | 3 | 3 | 0 | ok |
| New York (Midtown) | North America | large | right | 2229 | 1404 | nyc | 14 | 7463 | 5 | 0.986 | 0 | 0 | 0 | 0 | ok |
| Portland, Oregon | North America | medium | right | 4368 | 710 | overture | 50 | 14077 | 3 | 0.989 | 0 | 0 | 0 | 0 | ok |
| Burlington, Vermont | North America | small | right | 2033 | 1974 | overture | 83 | 12027 | 4 | 0.992 | 0 | 0 | 0 | 0 | ok |
| São Paulo (Sé) | South America | large | right | 2164 | 3182 | overture | 85 | 16300 | 5 | 0.99 | 0 | 0 | 0 | 0 | ok |
| Montevideo | South America | medium | right | 1917 | 2411 | overture | 64 | 12668 | 8 | 0.992 | 0 | 0 | 0 | 0 | ok |
| Ushuaia | South America | small | right | 2404 | 2309 | overture | 129 | 12561 | 2 | 0.992 | 0 | 0 | 0 | 0 | ok |
| Sydney (CBD) | Oceania | large | left | 2609 | 985 | overture | 111 | 14601 | 1 | 0.991 | 0 | 6 | 6 | 0 | ok |
| Auckland (CBD) | Oceania | medium | left | 3926 | 1142 | overture | 47 | 14633 | 1 | 0.99 | 0 | 0 | 0 | 0 | ok |
| Nouméa | Oceania | small | right | 1786 | 2465 | overture | 67 | 18425 | 3 | 0.994 | 0 | 1 | 1 | 0 | ok |
