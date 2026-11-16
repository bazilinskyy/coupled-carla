Smoke test, 2026-11-15: 40 s headless autopilot trial per map, CARLA 0.9.16 at Epic, 50 Hz, on lin4080, with the real-world scenery, terrain and bus lines.

| Map | Continent | Size | Traffic | Lane links | Buildings | Building heights | Relief (m) | Scenery meshes | Bus lines (with timetable) | SUMO vehicles shown | Real-time factor | Left road | Stuck | Put back on lane | Collisions | Result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 🇳🇱&nbsp;Eindhoven | Europe | medium | right | 2318 | 2998 | 3dbag | 11 | 19663 | 51 (45) | 14 | 0.986 | 0 | 1 | 1 | 0 | ok |
| 🇪🇬&nbsp;Cairo&nbsp;(Tahrir) | Africa | large | right | 3778 | 1308 | overture | 66 | 10831 | – | 3 | 0.993 | 0 | 0 | 0 | 0 | ok |
| 🇷🇼&nbsp;Kigali | Africa | medium | right | 1630 | 2942 | overture | 188 | 15277 | 18 (0) | 13 | 0.984 | 0 | 0 | 0 | 0 | ok |
| 🇿🇦&nbsp;Stellenbosch | Africa | small | left | 1748 | 1683 | overture | 32 | 9287 | – | 3 | 0.99 | 0 | 1 | 1 | 0 | ok |
| 🇯🇵&nbsp;Tokyo&nbsp;(Shibuya) | Asia | large | left | 5048 | 3982 | plateau | 87 | 16854 | 41 (10) | 7 | 0.989 | 0 | 6 | 6 | 0 | ok |
| 🇹🇼&nbsp;Taipei&nbsp;(Main&nbsp;Station) | Asia | medium | right | 2914 | 2679 | overture | 48 | 15456 | 150 (0) | 12 | 0.992 | 0 | 3 | 3 | 0 | ok |
| 🇱🇦&nbsp;Luang&nbsp;Prabang | Asia | small | right | 1418 | 3611 | overture | 87 | 10402 | – | 2 | 0.994 | 0 | 0 | 0 | 0 | ok |
| 🇬🇧&nbsp;London&nbsp;(Westminster) | Europe | large | left | 2751 | 2069 | overture | 30 | 10496 | 47 (1) | 18 | 0.989 | 0 | 0 | 0 | 0 | ok |
| 🇫🇷&nbsp;Lyon&nbsp;(Bellecour) | Europe | medium | right | 2276 | 2518 | bdtopo | 135 | 19646 | 21 (20) | 15 | 0.989 | 0 | 2 | 2 | 0 | ok |
| 🇧🇪&nbsp;Leuven | Europe | small | right | 1537 | 5049 | overture | 26 | 19189 | 26 (26) | 3 | 0.99 | 0 | 0 | 0 | 0 | ok |
| 🇺🇸&nbsp;New&nbsp;York&nbsp;(Midtown) | North America | large | right | 2226 | 1404 | nyc | 14 | 8289 | 67 (54) | 9 | 0.983 | 0 | 0 | 0 | 0 | ok |
| 🇺🇸&nbsp;Portland,&nbsp;Oregon | North America | medium | right | 4368 | 710 | overture | 50 | 14077 | 27 (25) | 36 | 0.976 | 0 | 1 | 1 | 0 | ok |
| 🇺🇸&nbsp;Burlington,&nbsp;Vermont | North America | small | right | 2033 | 1974 | overture | 83 | 12027 | 10 (2) | 24 | 0.99 | 0 | 1 | 1 | 0 | ok |
| 🇧🇷&nbsp;São&nbsp;Paulo&nbsp;(Sé) | South America | large | right | 2128 | 3182 | overture | 85 | 16308 | 64 (56) | 5 | 0.982 | 0 | 0 | 0 | 0 | ok |
| 🇺🇾&nbsp;Montevideo | South America | medium | right | 1912 | 2411 | overture | 64 | 12667 | 3 (0) | 2 | 0.989 | 0 | 0 | 0 | 0 | ok |
| 🇦🇷&nbsp;Ushuaia | South America | small | right | 2404 | 2309 | overture | 129 | 12561 | 2 (0) | 2 | 0.992 | 0 | 0 | 0 | 0 | ok |
| 🇦🇺&nbsp;Sydney&nbsp;(CBD) | Oceania | large | left | 2599 | 985 | overture | 111 | 14976 | 45 (43) | 7 | 0.99 | 0 | 1 | 1 | 0 | ok |
| 🇳🇿&nbsp;Auckland&nbsp;(CBD) | Oceania | medium | left | 3931 | 1142 | overture | 47 | 15236 | 54 (52) | 40 | 0.985 | 0 | 0 | 0 | 0 | ok |
| 🇳🇨&nbsp;Nouméa | Oceania | small | right | 1782 | 2465 | overture | 67 | 18424 | – | 2 | 0.993 | 0 | 0 | 0 | 0 | ok |
