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

### SURVEYOR FAN IMAGE

When a ping contains a complete set of validated message-3009 channel data,
the viewer reconstructs a Surveyor fan by beamforming the 16 receive channels.
The image uses the real Surveyor sector (-40°…+40°), 1° beam spacing, and the
range start/end stored in message 3010. ATOF detections can be overlaid as
white rings. Brightness is a robust *relative* normalization of beam power;
it is not presented as calibrated dB.

The viewer does not guess the sample format. The decoder follows the
`ChPairGoertzelData` definition used by SonarView:

```text
offset 0   uint32  ping_number
offset 4   float32 analog_gain
offset 8   uint8   device_number
offset 10  uint16  device_index_unused
offset 12  int32   adc_pp_signal
offset 16  uint8   ch1
offset 17  uint8   ch2
offset 18  uint16  results_per_channel
offset 20  float32 IQ pairs for ch1, then float32 IQ pairs for ch2
```

Each channel contains `[I,Q]` for each range step. Eight packets with channel
pairs 0/1 through 14/15 therefore make one 16-channel ping. The public
`bluerobotics-ping` package installed in this environment does not expose the
3009 definition; the format was cross-checked against the official SonarView
bundle and its Surveyor beamformer. The decoder reads only the
protocol-defined Float32 region. If a packet has extra trailing bytes, they
are reported in the diagnostic and are not reinterpreted as pixels.

For the large 2026-09-10 logs, `CHANNEL DATA: AVAILABLE` is shown only after
the ping passes all checks: eight packets, one consistent ping number, all 16
channels exactly once, a consistent range-step count, and a complete Float32
IQ region. Logs with only ATOF detections remain usable in POLAR FAN but do
not claim to contain a fan image.

An ATOF-only recording contains detections but no validated message-3009
channel set. For that file the fan-image view reports that channel data is not
available and POLAR FAN remains the correct representation.

## Surveyor multibeam vs imaging sonar

The Surveyor 240-16 is a multibeam/angle-of-time-of-flight sonar: its ATOF
message is a sparse set of echoes, each with an angle and a time of flight.
That is why ANGLE / DISTANCE and POLAR FAN show individual returns.

An imaging sonar display is an intensity field: many range/beam samples are
shown as pixels or cells, with brightness representing echo strength. The
Surveyor fan above is reconstructed from coherent channel IQ and a known
receive aperture; it is therefore richer than sparse ATOF points, but it is
still a relative visualization, not a calibrated photographic image or a
guaranteed bathymetric map.

This is different from an imaging sonar such as a mechanically scanned or
forward-looking camera-like sonar: those systems usually provide a directly
sampled angle/range intensity image. The Surveyor 240-16 is a 16-channel
multibeam echosounder. SonarView combines the receive-channel IQ data with
the array geometry to form beams, while ATOF is a separate compact detection
report. The two views should not be interpreted as interchangeable sensor
products.

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
