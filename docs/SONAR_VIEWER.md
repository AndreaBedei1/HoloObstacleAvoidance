# BlueROV2 Multimodal Recorder

La vecchia applicazione live è stata aggiornata per il vero hardware del
veicolo. La finestra mostra tre pannelli:

* **RGB CAMERA LIVE**: flusso H264 della camera BlueROV2, normalmente UDP
  `192.168.2.1:5600`; `5602` è disponibile per un secondo output configurato
  manualmente su BlueOS;
* **SURVEYOR FAN IMAGE**: immagine fan del Cerulean Surveyor 240-16,
  collegato a `192.168.2.86:62312`;
* **PING1D**: distanza, confidenza e profilo `profile_data` completo tramite
  PingProxy BlueOS `192.168.2.2:9090`.

L'app non modifica configurazioni BlueOS, WebRTC, camera, rete o file originali.

## Sicurezza Surveyor

Il Surveyor è attualmente **fuori dall'acqua**. L'avvio normale è quindi
bloccato:

```text
SURVEYOR TX: LOCKED / DRY MODE
```

In questa modalità l'app può fare un controllo TCP passivo, leggere dati già
presenti e riprodurre un `.svlog`, ma non invia il comando di acquisizione e
non abilita la trasmissione acustica. Il pulsante di avvio Surveyor è disabilitato.

Il codice per una futura prova bagnata è protetto da `--wet-authorized`. Non
usare questa opzione mentre il sensore è fuori dall'acqua; non è usata dal
launcher né dai test offline.

## Installazione e avvio Windows

Dal prompt nella radice del progetto:

```bat
py -m pip install -r requirements_sonar.txt
py scripts\real\test_sonar_viewer_offline.py
scripts\real\start_sonar_viewer.bat
```

Per verificare solo la costruzione della GUI senza rete o dispositivi:

```bat
py scripts\real\sonar_viewer.py --offline
```

Per riprodurre un log locale, senza creare una connessione Surveyor e senza
inviare comandi:

```bat
py scripts\real\sonar_viewer.py --replay-surveyor "C:\percorso\file.svlog"
```

## Dati Surveyor e fan image

Il decoder usa solo il formato confermato di SonarView per il packet `3009`
(`ChPairGoertzelData`): header di 20 byte, otto pacchetti per ping, 16 canali
in coppie e campioni IQ Float32 little-endian. I campioni sono ordinati
`[I0,Q0,I1,Q1,...]` per canale; i byte successivi all'area definita dal
protocollo vengono ignorati.

I packet `3010` forniscono ping, intervallo, numero di range step e timestamp;
`3012` fornisce le detection ATOF; `504` è conservato come attitude. La fan
image usa l'apertura reale del Surveyor (`-40°..+40°`), l'apertura di 16 canali
e beamforming coerente con la pipeline ufficiale. `CHANNEL DATA: AVAILABLE`
compare solo quando la serie completa dei canali è valida.

Il cursore **threshold %** è un filtro di visualizzazione. A `0%` il background
acustico resta visibile; non cambia il file `.svlog`.

## Camera

Il campo SDP accetta un file locale o un URL SDP. Se resta vuoto, l'app prova
il ricevitore UDP sulla porta selezionata. Il pannello usa un solo ingest:
gli stessi frame decodificati alimentano preview, timestamp CSV e registrazione.
L'app non scrive automaticamente file SDP o configurazioni su BlueOS. Su
Windows è richiesto OpenCV per il preview camera; Surveyor replay e test
offline restano disponibili anche senza OpenCV.

## Registrazione di sessione

Con **START SESSION** viene creata una nuova cartella:

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

Le directory hanno sempre un identificativo nuovo: non vengono sovrascritte.
`session.json` contiene `session_id`, `session_start_utc_ns` e
`session_start_monotonic_ns`. I JSONL e il CSV mantengono timestamp host
monotonic/UTC e, quando disponibili, timestamp del dispositivo. Il raw
Surveyor è una copia locale del flusso di packet e l'originale resta invariato.

`match_surveyor_ping()` e `nearest_sample()` forniscono il criterio offline
per associare un ping Surveyor al frame RGB e al campione Ping1D più vicini
nel tempo monotonic.

## Diagnostica

Il controllo read-only opzionale è:

```bat
py scripts\test_real_sonar_connections.py
```

Controlla BlueOS, il TCP Surveyor `192.168.2.86:62312` e Ping1D. Non abilita
la trasmissione Surveyor.

## Limiti attuali

La scrittura `camera_rgb.mkv` usa i frame decodificati dal singolo ingest e un
writer OpenCV; i timestamp originali sono conservati in
`camera_timestamps.csv`. Quando servirà una cattura H264 bitstream con PTS
nativi, va aggiunto un backend FFmpeg/PyAV dedicato senza aprire un secondo
consumer UDP.

I file raw e i video pesanti sono esclusi da Git. Il controllo staged
`scripts/check_large_files.py --staged` rifiuta file oltre 10 MiB. Per
installare il pre-commit locale una volta per clone:

```bat
git config core.hooksPath .githooks
```

Il controllo può anche essere eseguito manualmente con:

```bat
py scripts\check_large_files.py --staged
```

Riferimenti: [Surveyor 240-16](https://ceruleansonar.com/product/surveyor-240-16/),
[API set ping parameters](https://docs.ceruleansonar.com/c/surveyor-240-16/application-programming-interface/set_ping_parameters),
[release SonarView](https://github.com/CeruleanSonar/SonarView/releases).
