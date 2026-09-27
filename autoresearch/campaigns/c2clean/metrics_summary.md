| metric | clean k3 | clean k0 | orig k3 | orig k0 |
|---|---|---|---|---|
| v1 dev success fraction, median | 0.00 | 0.00 | 0.00 | 0.00 |
| time to first dev success (h), median | 0.24 | 0.27 | 0.21 | 0.57 |
| dev episodes to first success, median | 12 | 16 | 12 | 20 |
| version of first success, median | 2 | 4 | 2 | 4 |
| development episodes, median | 46 | 53 | 42 | 53 |
| program versions, median | 4 | 6 | 3 | 6 |
| agent wall-clock (h), median | 0.51 | 0.55 | 0.43 | 0.52 |
| assistant turns, median | 114 | 116 | 89 | 104 |
| output tokens (k), median | 66 | 69 | 60 | 70 |
| cache-read tokens (M), median | 6.5 | 6.5 | 4.2 | 5.0 |
| cost (USD), median | 6.15 | 6.41 | 4.78 | 5.59 |
| agent wall-clock (h), total | 36.55 | 43.77 | 32.46 | 45.66 |
| development episodes, total | 3009 | 3504 | 2855 | 3731 |
| output tokens (k), total | 4756 | 5387 | 4253 | 5469 |
| cost (USD), total | 466.42 | 508.08 | 353.60 | 452.15 |
| cells whose v1 already succeeded on some seed | 29 | 1 | 19 | 6 |
| cells with no dev success | 1 | 1 | 1 | 3 |

paired clean K=3 vs K=0 (per cell, K=0 minus K=3): wins = K=3 cheaper
  v1 dev success fraction              median diff   0.00   K=3 cheaper 0 / K=0 cheaper 29 / tie 30  (n=59)
  time to first dev success (h)        median diff   0.06   K=3 cheaper 39 / K=0 cheaper 19 / tie 0  (n=58)
  dev episodes to first success        median diff      4   K=3 cheaper 35 / K=0 cheaper 21 / tie 2  (n=58)
  version of first success             median diff      2   K=3 cheaper 45 / K=0 cheaper 7 / tie 6  (n=58)
  development episodes                 median diff      4   K=3 cheaper 32 / K=0 cheaper 26 / tie 2  (n=60)
  program versions                     median diff      2   K=3 cheaper 44 / K=0 cheaper 10 / tie 6  (n=60)
  agent wall-clock (h)                 median diff   0.10   K=3 cheaper 40 / K=0 cheaper 20 / tie 0  (n=60)
  assistant turns                      median diff      0   K=3 cheaper 30 / K=0 cheaper 26 / tie 4  (n=60)
  output tokens (k)                    median diff      5   K=3 cheaper 37 / K=0 cheaper 23 / tie 0  (n=60)
  cache-read tokens (M)                median diff    0.0   K=3 cheaper 31 / K=0 cheaper 29 / tie 0  (n=60)
  cost (USD)                           median diff   0.26   K=3 cheaper 33 / K=0 cheaper 27 / tie 0  (n=60)
