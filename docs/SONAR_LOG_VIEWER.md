# Surveyor 240-16 Sonar Log Explorer

The desktop viewer opens the local copies of SonarView `.svlog` recordings in
`records\sonar\rover_20260910_SonarView`. It is read-only: the rover files and
the downloaded copies are never rewritten.

Start it from the repository root with:

```bat
python scripts\real\sonar_log_viewer.py
```

or double-click [start_sonar_log_viewer.bat](../scripts/real/start_sonar_log_viewer.bat).

## Views

### ANGLE / DISTANCE

The original single-ping view. Every point is one ATOF echo reported by the
Surveyor 240-16. The horizontal coordinate is the detected bearing from -40°
to +40° and the vertical coordinate is slant range in metres. The protocol
stores the angle in radians; the viewer converts it to degrees only for the
labels. A point is a detected return, not a camera pixel and not a complete
acoustic waveform.

### POLAR FAN

The classical forward-looking sonar display. The transducer is at the vertex,
the sector is -40°…+40°, radial distance is range, and the selected ping is
drawn as coloured detections. The slider and play/pause button move through
the selected log.

### CROSS-TRACK / DEPTH

The detections are projected into metric coordinates using the Surveyor speed
of sound and time of flight:

```text
distance = 0.5 * sos * tof
y = distance * sin(angle)
z = -distance * cos(angle)
```

Here `y` is cross-track and `z` is the forward/depth axis under this viewer's
sign convention. The yellow line is a *probable bottom envelope*: for each
cross-track bin it selects the farthest return. It is a visual hypothesis, not
a bathymetric survey; a far obstacle, multipath, missing returns, or the pool
wall can change that envelope.

### PANORAMICA TEMPORALE

The lower plot is retained under every view. Each column is one ping, the
vertical axis is range, and the colour encodes bearing (blue = left/negative
angle, yellow = right/positive angle). The white vertical cursor is the
selected ping and the yellow horizontal cursor marks its farthest displayed
return. Click the panorama to jump to a ping.

### SONAR IMAGE / INTENSITY FAN

The first four recordings contain message-3009 raw profile tiles in addition
to ATOF detections. In these files the viewer lazily decodes the raw complex
half-float samples, uses their magnitude as intensity, and lays the tiles out
as a polar fan. Only the selected ping is decoded, so the 500 MB recording is
not loaded wholly into memory.

The short `2026-09-10-13-14.svlog` recording contains ATOF detections but no
message-3009 raw profile tiles. For that file the intensity view deliberately
reports that no raw intensity is available and the POLAR FAN remains the
correct representation.

## Surveyor multibeam vs imaging sonar

The Surveyor 240-16 is a multibeam/angle-of-time-of-flight sonar: its ATOF
message is a sparse set of echoes, each with an angle and a time of flight.
That is why ANGLE / DISTANCE and POLAR FAN show individual returns.

An imaging sonar display is an intensity field: many range/beam samples are
shown as pixels or cells, with brightness representing echo strength. When
message-3009 raw beam/profile data is present, SONAR IMAGE / INTENSITY FAN
provides that richer view. It should still be treated as a reconstruction of
the recorded beam tiles, not as a calibrated photographic image or a
guaranteed bathymetric map.

## Raw files and Git safety

The raw recordings are intentionally ignored by Git. The repository also has
a staged-file pre-commit check that rejects any file larger than 10 MiB. Enable
the tracked hook once in a clone with:

```bat
git config core.hooksPath .githooks
```

Run a text-only inventory without opening the GUI:

```bat
python scripts\real\sonar_log_viewer.py --summary
```
