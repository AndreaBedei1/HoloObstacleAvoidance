# Live Omniscan viewer

The small Windows app in `scripts/real/sonar_viewer.py` displays the Cerulean
Omniscan 450 profile stream as a live scrolling waterfall image.

## Start

Close SonarView if it is already connected to the sonar, then run:

```bat
scripts\real\start_sonar_viewer.bat
```

The defaults are the device recorded in this project:

- IP: `192.168.2.86`
- TCP port: `51200`

Click **Connetti**, set the range if needed, then click **Avvia ping**. The
viewer sends no vehicle/MAVLink commands. Closing the window sends the sonar
stop-ping command and closes the TCP connection.

The first lake test should use a short range and auto gain. The app currently
uses an explicit 8 ping/s cap, saves the displayed waterfall as PNG, and does
not write a sonar log file.

The app uses the Cerulean/Ping Protocol `os_ping_params` command (`2197`) and
`os_mono_profile` response (`2198`) over TCP. The protocol details are in the
vendor documentation:

- <https://docs.ceruleansonar.com/c/omniscan-450/application-programming-interface>
- <https://docs.ceruleansonar.com/c/sonarview/mission-configurations/omniscan-450>
