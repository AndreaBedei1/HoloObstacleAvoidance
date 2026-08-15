# Confronto sim-reale (pre-registrato)

Soglia di ingaggio 0.05 m/s mantenuta 1.0 s, applicata offline alla stessa traccia di comando in entrambi i domini.

## Previsioni simulate (mediane su 5 ripetizioni)

`>=` indica una misura CENSURATA A SINISTRA: il veicolo gia manovrava al primo campione, quindi l'inizio e precedente alla finestra e il valore e un limite inferiore, non una misura.

### distanza minima (m)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 1.475 | 2.216 | 0.771 | 0.575 |
| K0 | dwa | 2.142 | 2.472 | 0.925 | 0.856 |
| K1 | committed | 1.189 | 2.557 | 0.873 | 0.736 |
| K1 | dwa | 2.461 | 2.330 | 0.870 | 0.652 |

### distanza all'ingaggio (m)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 1.874 | 3.013 | 1.134 | 1.163 |
| K0 | dwa | 3.500 | 3.500 | 3.500 | 3.500 |
| K1 | committed | 1.494 | 3.197 | 1.172 | 1.158 |
| K1 | dwa | 3.517 (1/4 cens.) | 3.517 (1/5 cens.) | 3.517 | 3.517 (2/4 cens.) |

### escursione laterale (m)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 2.477 | 2.485 | 2.473 | 1.108 |
| K0 | dwa | 3.050 | 3.276 | 2.391 | 1.234 |
| K1 | committed | 2.483 | 2.479 | 2.476 | 1.108 |
| K1 | dwa | 2.980 | 2.849 | 2.551 | 1.254 |

### lunghezza percorso (m)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 14.483 | 15.540 | 14.474 | 12.940 |
| K0 | dwa | 15.723 | 16.823 | 14.423 | 13.541 |
| K1 | committed | 15.616 | 16.133 | 14.524 | 13.490 |
| K1 | dwa | 16.500 | 16.815 | 14.836 | 14.271 |

### durata manovra (span) (s)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 64.053 | 54.600 | 54.451 | 49.789 |
| K0 | dwa | 60.057 | 86.265 | 88.609 | 91.709 |
| K1 | committed | 55.755 | 54.401 | 54.293 | 47.637 |
| K1 | dwa | 72.998 (2/5 cens.) | 30.995 (1/5 cens.) | 90.495 | 91.706 (3/5 cens.) |

### tempo di comando laterale (s)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 27.436 | 24.735 | 28.084 | 24.612 |
| K0 | dwa | 24.657 | 35.071 | 47.140 | 56.938 |
| K1 | committed | 25.767 | 24.583 | 28.202 | 24.573 |
| K1 | dwa | 30.810 | 25.905 | 62.760 | 69.698 |

### picco laterale comandato (controllo) (m/s)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 0.300 | 0.300 | 0.300 | 0.300 |
| K0 | dwa | 0.300 | 0.300 | 0.300 | 0.300 |
| K1 | committed | 0.300 | 0.300 | 0.300 | 0.300 |
| K1 | dwa | 0.300 | 0.300 | 0.300 | 0.300 |

## Finestra osservabile comune (<= 1.86 m)

Le STESSE tracce simulate, con tutto cio che sta oltre 1.86 m scartato. La simulazione parte a 3.5 m e il reale a 1.86 m, quindi 1.6 m di avvicinamento simulato non sono osservabili in vasca: confrontare l'ingaggio su finestre diverse fabbricherebbe un errore sim-reale a partire da due numeri che sono entrambi solo "la partenza". Le previsioni congelate restano invariate e sono riportate sopra.

### distanza all'ingaggio, finestra comune (m)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 1.857 (3/5 cens.) | mai in finestra | 1.134 | 1.163 |
| K0 | dwa | mai in finestra | mai in finestra | 1.757 (2/5 cens.) | 1.847 (3/5 cens.) |
| K1 | committed | 1.494 | mai in finestra | 1.172 | 1.158 |
| K1 | dwa | mai in finestra | mai in finestra | 1.850 (2/5 cens.) | >=1.854 |

### durata manovra (span), finestra comune (s)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 64.053 (3/5 cens.) | mai in finestra | 54.451 | 49.789 |
| K0 | dwa | mai in finestra | mai in finestra | 72.348 (2/5 cens.) | 76.511 (3/5 cens.) |
| K1 | committed | 55.755 | mai in finestra | 54.293 | 47.637 |
| K1 | dwa | mai in finestra | mai in finestra | 73.406 (2/5 cens.) | >=77.602 |

### tempo di comando laterale, finestra comune (s)

| geometria | planner | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| K0 | committed | 27.436 | mai in finestra | 28.084 | 24.612 |
| K0 | dwa | mai in finestra | mai in finestra | 37.161 | 43.193 |
| K1 | committed | 25.767 | mai in finestra | 28.202 | 24.573 |
| K1 | dwa | mai in finestra | mai in finestra | 53.170 | 57.008 |

## Confronto con la realta

I 20 run reali non esistono ancora. Questo file contiene solo le previsioni; il confronto viene prodotto dallo stesso script, senza modifiche, quando la campagna reale sara completa.

