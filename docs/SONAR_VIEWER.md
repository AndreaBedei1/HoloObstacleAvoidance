# BlueROV2 Multimodal Recorder

La vecchia applicazione live è stata rifinita per il vero hardware del
veicolo. La finestra mostra tre pannelli:

* **RGB CAMERA LIVE**: flusso H264/RTP della camera BlueROV2;
* **SURVEYOR FAN IMAGE**: immagine fan del Cerulean Surveyor 240-16;
* **PING1D**: distanza, confidenza e profilo `profile_data` completo tramite
  PingProxy BlueOS.

L'app è passiva lato BlueOS: non modifica Camera Manager, WebRTC, stream UDP,
rete o configurazioni del veicolo. Cockpit può rimanere aperto. Per evitare
che due applicazioni consumino lo stesso flusso, è consigliato configurare
manualmente su BlueOS un secondo output dedicato alla Multimodal Recorder:
UDP `5602`. Cockpit resta sul proprio flusso/WebRTC; `5600` rimane il default
compatibile con l'installazione attuale.

## Sicurezza Surveyor

Il Surveyor è attualmente fuori dall'acqua. L'avvio normale mostra:

```text
SURVEYOR TX: LOCKED / DRY MODE
```

In questa modalità il codice non invia il comando di acquisizione e non abilita
la trasmissione acustica. `--wet-authorized` è solo una protezione per una
futura prova in acqua: non usarlo fuori dall'acqua e non è usato dai test.

## Installazione e modalità offline

Dal prompt nella radice del progetto:

```bat
py -m pip install -r requirements_sonar.txt
py scripts\real\test_sonar_viewer_offline.py
```

Modalità da usare dopo aver verificato il veicolo in acqua:

```bat
:: DRY / normale: il Surveyor resta TX LOCKED
py scripts\real\sonar_viewer.py

:: SOLO CAMERA + PING1D; non istanzia Surveyor240
py scripts\real\sonar_viewer.py --skip-surveyor

:: replay locale; non apre il TCP Surveyor
py scripts\real\sonar_viewer.py --replay-surveyor "C:\percorso\file.svlog"

:: GUI senza hardware, utile per smoke test
py scripts\real\sonar_viewer.py --offline
```

Quando `--replay-surveyor` e `--skip-surveyor` sono entrambi presenti,
prevale il replay. `--skip-surveyor` prevale sulla connessione hardware.
Durante questa fase, con il BlueROV2 scollegato, non eseguire le modalità live.

## Camera: PyAV single-ingest

Il backend preferito è PyAV. Un singolo demux legge il flusso H264/RTP e lo
usa contemporaneamente per:

* remux dei packet encoded in `camera_rgb.mkv`, senza ricompressione;
* decode dei frame per la preview GUI;
* timestamp e PTS/DTS.

Sono supportati UDP 5600, UDP 5602, file SDP locale e URL SDP quando il backend
FFmpeg di PyAV lo consente. L'app non crea automaticamente file SDP né cambia
BlueOS. Se PyAV non riesce ad aprire la sorgente, viene mostrato l'errore e si
prova OpenCV solo come fallback preview/registrazione decodificata. In quel
caso la GUI e i metadata mostrano chiaramente:

```text
Camera backend: OpenCV fallback
Camera record: DECODED/REENCODED FALLBACK
Video PTS: UNAVAILABLE
```

La registrazione fallback non viene chiamata raw H264. Installare PyAV con
`av` in `requirements_sonar.txt` per il percorso REMUX H264.

## Timestamp camera

`camera_timestamps.csv` contiene una riga per frame e questi campi:

```text
frame_index, packet_index, pts, dts, time_base_num, time_base_den,
pts_seconds, dts_seconds, host_monotonic_ns, host_utc_ns,
session_time_s, key_frame, packet_size
```

`pts_seconds` e `dts_seconds` sono valorizzati solo quando PyAV fornisce un
timestamp nativo e il relativo time base. Non vengono inventati. Il campo
`session_time_s` è invece sempre il clock host relativo:

```text
(host_monotonic_ns - session_start_monotonic_ns) / 1e9
```

Questi due concetti restano separati: il PTS appartiene al video/container,
mentre `session_time_s` permette di sincronizzare camera, Surveyor e Ping1D.

## Dati Surveyor e fan image

Il decoder usa solo il formato confermato di SonarView per il packet `3009`
(`ChPairGoertzelData`): header da 20 byte, otto pacchetti per ping, 16 canali
in coppie e campioni IQ Float32 little-endian. I campioni sono ordinati
`[I0,Q0,I1,Q1,...]` per canale; i byte successivi all'area definita dal
protocollo vengono ignorati.

I packet `3010` forniscono ping, intervallo, range step e timestamp; `3012`
fornisce le detection ATOF; `504` è conservato come attitude. La fan image usa
l'apertura reale del Surveyor (`-40°..+40°`) e il beamforming dei 16 canali.
`CHANNEL DATA: AVAILABLE` compare solo quando l'intero set dei canali è valido.
Il cursore threshold è un filtro di visualizzazione: a `0%` il background
acustico resta visibile e il `.svlog` non viene modificato.

## Registrazione di sessione

Con **START SESSION** viene creata una directory nuova, mai sovrascritta:

```text
records/real_sessions/<session_id>/
  session.json
  camera_rgb.mkv
  camera_timestamps.csv
  surveyor_raw.svlog
  surveyor_pings.jsonl
  ping1d.jsonl
  events.jsonl
```

`session.json` salva `session_id`, `session_start_utc_ns`,
`session_start_monotonic_ns` e:

```text
camera.backend
camera.recording_mode
camera.source
camera.codec
camera.width / camera.height
camera.nominal_fps
camera.time_base
surveyor.mode = live | replay | skipped
surveyor.replay_source, se presente
```

Con `--skip-surveyor` non viene creato un falso `surveyor_raw.svlog`.
Con `--replay-surveyor` il raw è una copia locale del file replayato e il file
originale resta invariato.

## Matching offline

`match_surveyor_ping()` trova il frame RGB e il campione Ping1D più vicini al
ping Surveyor usando `host_monotonic_ns`. Restituisce inoltre:

* `camera_pts` e `camera_pts_seconds` del frame associato, se disponibili;
* `time_delta_camera_ms`;
* `time_delta_ping1d_ms`.

I delta sono firmati: un valore positivo significa che il campione associato
è arrivato dopo il ping Surveyor.

## Diagnostica

Il controllo read-only opzionale controlla BlueOS, il TCP Surveyor
`192.168.2.86:62312` e Ping1D `192.168.2.2:9090`, senza inviare comandi:

```bat
py scripts\test_real_sonar_connections.py
```

## Limiti e Git

I test disponibili sono offline/sintetici; non sono stati eseguiti test reali
perché il veicolo è scollegato. Un piccolo fixture video può essere generato
solo durante i test se PyAV/FFmpeg è installato, senza aggiungere binari al
repository.

I file raw e i video pesanti sono esclusi da Git. Il controllo staged
`scripts/check_large_files.py --staged` rifiuta file oltre 10 MiB. Per attivare
il pre-commit locale una volta per clone:

```bat
git config core.hooksPath .githooks
```

Riferimenti: [Surveyor 240-16](https://ceruleansonar.com/product/surveyor-240-16/),
[API set ping parameters](https://docs.ceruleansonar.com/c/surveyor-240-16/application-programming-interface/set_ping_parameters),
[release SonarView](https://github.com/CeruleanSonar/SonarView/releases).
