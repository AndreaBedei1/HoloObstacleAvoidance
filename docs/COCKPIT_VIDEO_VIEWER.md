# COCKPIT VIDEO + TELEMETRY EXPLORER

Viewer Windows indipendente per le registrazioni Cockpit del BlueROV2. Il
viewer abbina un video `MKV` al relativo file Advanced SubStation Alpha
`ASS`, mostra l'overlay originale tramite VLC e interpreta gli eventi ASS in
un pannello di telemetria sincronizzato con il video.

## File e sicurezza

La cartella iniziale è `records/video/`. È possibile scegliere qualunque altra
cartella con **Open folder**; per esempio una cartella locale che contiene le
copie scaricate dal rover. Il programma non sposta, riscrive o aggiorna i file
`.mkv` e `.ass`: li apre in lettura e crea eventuali CSV o screenshot solo nel
percorso scelto dall'utente.

Il pairing usa prima il basename identico, senza l'estensione, e mantiene la
sottocartella relativa quando la scansione è ricorsiva. Sono mostrati anche i
file non accoppiati:

- `VIDEO OK · ASS OK`: entrambe le parti sono presenti;
- `VIDEO WITHOUT ASS`: video senza telemetria ASS;
- `ASS WITHOUT VIDEO`: ASS senza video.

I video grezzi pesanti (`*.mkv`, `*.mp4`, `*.mov`, `*.avi`, `*.wmv`) e
`records/video/**/*.mkv` ecc. sono esclusi da Git. I file `.ass` non sono
ignorati automaticamente, perché possono essere piccoli e utili; prima di
condividerli va comunque verificato se contengono dati sensibili.

## Installazione e avvio

Installare le dipendenze Python:

```powershell
python -m pip install -r requirements_cockpit_video.txt
```

Su Windows installare anche VLC Desktop (versione 64 bit se si usa Python
64 bit). `python-vlc` è il binding Python, ma non contiene `libvlc.dll`:
senza VLC installato l'applicazione resta utilizzabile per pairing, parsing,
telemetria e test, e mostra un messaggio esplicito invece di terminare con un
errore.

Avvio consigliato:

```powershell
scripts\real\start_cockpit_video_viewer.bat
```

Oppure:

```powershell
python scripts\real\cockpit_video_viewer.py --folder records\video
```

Per controllare una cartella senza aprire la GUI:

```powershell
python scripts\real\cockpit_video_viewer.py --folder records\video --summary
```

## Riproduzione e sincronizzazione

Il player incorporato supporta MKV, play, pausa, stop, seek sulla timeline,
salti di -5/+5 secondi, volume e velocità `0.5x`, `1x`, `1.5x`, `2x`. Il tempo
corrente e la durata sono visualizzati sopra la timeline. **Previous recording**
e **Next recording** cambiano la selezione nella lista.

Quando è presente un ASS, il file viene passato a VLC come subtitle track.
**Show original ASS overlay** abilita o disabilita l'overlay originale Cockpit
nel video; questa opzione non altera il file ASS.

Gli eventi ASS vengono indicizzati una sola volta al caricamento. Il pannello
destro viene aggiornato circa 15 volte al secondo usando il timestamp reale di
VLC, quindi resta corretto durante riproduzione, pausa, seek e salti di ±5 s.
Non viene riletto il file dal disco a ogni aggiornamento.

## Interpretazione della telemetria ASS

`CURRENT TELEMETRY` contiene tutti gli eventi attivi al timestamp corrente.
Il testo leggibile rimuove solo il markup/styling ASS (`{...}`) e converte i
comandi di ritorno a capo in righe normali. Le coppie riconosciute come
`key: value` o `key=value` vengono mostrate nella tabella `FIELD | VALUE`.
Separatori `|` e tabulazioni permettono di riconoscere più campi nella stessa
riga. Le righe che non seguono questo formato restano in `Other / Raw` e non
vengono scartate.

`RAW ASS EVENTS AT CURRENT TIME` conserva il testo originale dell'evento,
oltre alla versione leggibile e agli intervalli temporali. Se non c'è alcun
evento attivo, il pannello lo dichiara esplicitamente: non vengono inventati
campi come depth, heading, roll o pitch.

La vista CSV esporta una riga per ogni evento temporizzato con start, end,
layer, style, testo raw, testo leggibile e campi riconosciuti. L'ASS originale
resta invariato.

## Informazioni file

Per ogni registrazione sono mostrati filename, data ricavata dal pattern
`Cockpit (Mon DD, YYYY - HH꞉MM꞉SS GMT±N)`, durata, ASS associato, numero eventi
e intervallo temporale ASS. Risoluzione, FPS e codec vengono aggiunti quando
`ffprobe` è disponibile; la sua assenza non impedisce la riproduzione con VLC.
È disponibile anche lo screenshot del frame corrente quando VLC lo supporta.

## Cosa rappresentano i dati

Il testo `angolo/distanza` dell'ASS è una rappresentazione della misura o della
detection nel singolo istante/ping, con angolo riferito all'asse del sonar e
distanza riferita alla stima temporale del ritorno. Non è automaticamente una
mappa 3D né una misura georeferenziata.

La registrazione Cockpit è una sequenza temporale: il video mostra il frame
corrente, mentre gli eventi ASS mostrano i valori associati allo stesso tempo
del video. Lo stesso campo può quindi cambiare tra eventi successivi.

Un ritorno ricorrente e coerente nella parte bassa del campo, con distanza
compatibile con il fondale e continuità tra ping vicini, è probabilmente il
fondale. Questa è un'interpretazione geometrica, non una classificazione certa:
vegetazione, schiuma, multipath, assetto del veicolo e rumore possono produrre
ritorni simili. Gli ostacoli sono evidenze separate solo quando il contenuto
ASS o la geometria del sonar li distingue; il viewer non inventa una
classificazione assente nei dati.

Questa app visualizza telemetria e overlay Cockpit. Non va confusa con un
viewer di imaging sonar: un multibeam Surveyor fornisce tipicamente misure o
detection associate a più beam/angoli, mentre un imaging sonar produce una
matrice di intensità acustica che può essere ricostruita come immagine. Un ASS
contiene overlay e telemetria temporizzata, non necessariamente i campioni raw
di intensità necessari per un'immagine sonar.

## Test offline

I test non richiedono video né VLC funzionante e usano un ASS sintetico:

```powershell
python scripts\real\test_cockpit_video_viewer_offline.py
```

Verificano pairing, parsing ASS e markup, eventi sovrapposti, coppie generiche
chiave/valore, righe sconosciute, esportazione CSV senza modifica dell'ASS,
seek con telemetria aggiornata e costruzione della GUI con il backend VLC
disabilitato.

Il controllo pre-commit già configurato rifiuta nuovi file più grandi di 10 MB
(`scripts/check_large_files.py`).
