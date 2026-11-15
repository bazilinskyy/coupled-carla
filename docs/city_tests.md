Smoke test, 2026-11-15: 40 s headless autopilot trial per map, CARLA 0.9.16 at Epic, 50 Hz, on lin4080, with the real-world scenery, terrain and bus lines.

| Map | Continent | Size | Traffic | Lane links | Buildings | Building heights | Relief (m) | Scenery meshes | Bus lines (with timetable) | SUMO vehicles shown | Real-time factor | Left road | Stuck | Put back on lane | Collisions | Result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Eindhoven | Europe | medium | right | 2318 | 2998 | 3dbag | 11 | 19663 | 51 (45) | 14 | 0.986 | 0 | 1 | 1 | 0 | ok |
| Cairo (Tahrir) | Africa | large | right | 3778 | 1308 | overture | 66 | 10831 | – | 4 | 0.993 | 0 | 1 | 1 | 0 | ok |
| Kigali | Africa | medium | right | 1630 | 2942 | overture | 188 | 15277 | 18 (0) | 13 | 0.989 | 0 | 0 | 0 | 0 | ok |
| Stellenbosch | Africa | small | left | 1748 | 1683 | overture | 32 | 8665 | – | 1 | 0.992 | 0 | 0 | 0 | 0 | ok |
| Tokyo (Shibuya) | Asia | large | left | 5048 | 3982 | overture | 87 | 16084 | 41 (10) | 7 | 0.991 | 0 | 0 | 0 | 0 | ok |
| Taipei (Main Station) | Asia | medium | right | 2914 | 2679 | overture | 48 | 15456 | 150 (0) | 22 | 0.988 | 0 | 1 | 1 | 0 | ok |
| Luang Prabang | Asia | small | right | 1418 | 3611 | overture | 87 | 10402 | – | 2 | 0.994 | 0 | 0 | 0 | 0 | ok |
| London (Westminster) | Europe | large | left | 2751 | 2069 | overture | 30 | 10049 | 47 (1) | 19 | 0.99 | 0 | 1 | 1 | 0 | ok |
| Lyon (Bellecour) | Europe | medium | right | 2276 | 2518 | bdtopo | 135 | 19646 | 21 (20) | 14 | 0.99 | 0 | 2 | 2 | 0 | ok |
| Leuven | Europe | small | right | 1537 | 5049 | overture | 26 | 19189 | 26 (26) | 2 | 0.991 | 0 | 3 | 3 | 0 | ok |
| New York (Midtown) | North America | large | right | 2226 | 1404 | nyc | 14 | 8289 | 67 (54) | 9 | 0.983 | 0 | 0 | 0 | 0 | ok |
| Portland, Oregon | North America | medium | right | 4368 | 710 | overture | 50 | 14077 | 27 (25) | 34 | 0.975 | 0 | 0 | 0 | 0 | ok |
| Burlington, Vermont | North America | small | right | 2033 | 1974 | overture | 83 | 12027 | 10 (2) | 24 | 0.99 | 0 | 1 | 1 | 0 | ok |
| São Paulo (Sé) | South America | large | right | 2128 | 3182 | overture | 85 | 16308 | 64 (56) | 5 | 0.983 | 0 | 0 | 0 | 0 | ok |
| Montevideo | South America | medium | right | 1912 | 2411 | overture | 64 | 12667 | 3 (0) | 2 | 0.987 | 0 | 0 | 0 | 0 | ok |
| Ushuaia | South America | small | right | 2404 | 2309 | overture | 129 | 12561 | 2 (0) | 2 | 0.992 | 0 | 0 | 0 | 0 | ok |
| Sydney (CBD) | Oceania | large | left | 2599 | 985 | overture | 111 | 14582 | 45 (43) | 10 | 0.988 | 0 | 4 | 4 | 0 | ok |
| Auckland (CBD) | Oceania | medium | left | 3931 | 1142 | overture | 47 | 14629 | 54 (52) | 44 | 0.981 | 0 | 0 | 0 | 0 | ok |
| Nouméa | Oceania | small | right | 1782 | 2465 | overture | 67 | 18424 | – | 5 | 0.99 | 0 | 2 | 2 | 0 | ok |
