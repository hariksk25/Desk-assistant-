# v6 screen design (built in 6.3.0 to 6.3.3, 12 Oct 2026)

As built differs from the draft below in: STATS and SYSTEM right dial = the next page (their left pages already had the hour bars / graphs); NEWS left dial = one big headline; POMODORO right dial = this week (home already has today and the 7 days); NETWORK left OLED = gateway / DNS / host; F1 circuit and date moved to the left OLED; RADAR right ring = distance of the subject.

Rules (owner, 12 Oct):

| Rule | Detail |
|---|---|
| Right OLED = clock | Time (12 h, AM/PM, seconds) in the blue area, day and date in the yellow band, in every app except CLOCK |
| No clock anywhere else | Left OLED headers lose their time (v5 `oledTimeHead`); the right dial companion clock goes |
| Nothing repeated | A value appears on one display only. Left dial = the app's main view (v5, unchanged unless noted); right dial = the companion; left OLED = a third, different piece |
| Same subject | When a dial and an OLED talk about one item (an aircraft, a symbol) they show different facts about it |

Data available today (no new sources needed unless marked NEW).

| App | Left dial (main) | Right dial (companion) | Left OLED | Rings L / R |
|---|---|---|---|---|
| RADAR (done, 6.2.0) | radar, ring on the subject | ARRIVALS / DEPARTURES mode: logo, flight, route, type + side profile | the other side as a list | sweep + aircraft dots / distance of the subject to touchdown (was seconds) |
| ARRIVALS board | board list (v5) | highlighted row's flight: logo, route, type, big ETA countdown | runway in use: runway drawn in its direction, wind arrow, head / crosswind, QNH | next arrival countdown / board position |
| APPROACH | glideslope side view of all traffic (v5) | highlighted aircraft: logo, flight, slope deviation needle (high / low), speed trend | decoded METAR / TAF / SIGMET: wind, visibility, cloud, runway change | distance to touchdown of the highlighted one / slope deviation (green centred, amber off) |
| CLOCK | chosen face (17 faces: Bold, Analog, Dashboard, World, 24 Hour, Minimal, Flip, Seconds, Night Sky, Year Dots, Huge, Rings, Segments, Nixie, Words, Weather, Orbit; Analog and Rings sweep at 20 fps) | configurable (SETTINGS > CLOCK > right dial): weather now, agenda (now / next), world (two zones, big), sun and moon, next arrivals | sunrise, sunset, daylight (left); week / day, next event, alarm (right, no clock) | follow the face: hands, fills or glow (6.18.0) |
| CLOCK, right OLED | (free in this app) | | | alarm time and status, week number, day of year |
| WEATHER | now / 3-day / 24 h pages (v5) | next 12 h: temperature line + rain probability bars, rain in the next 2 h | AQI breakdown (PM2.5, PM10, O3, NO2); indoor temp + pressure once the BMP280 is fixed | AQI colour / rain probability for the next 24 h around the ring (blue) |
| POMODORO | session countdown (v5) | today: goal ring, sessions as dots, streak, last 7 days bars | task label, what comes next (ends at, break length), alerts held | session progress / daily goal progress |
| MARKETS | selected symbol price + intraday chart (v5) | watchlist: every other symbol with price and % as coloured tiles | market status (open / closes in / opens in), day range and previous close of the selected symbol, fear and greed | green or red with the selected move / trading session progress 9:15 to 15:30 |
| NEWS | current headline (v5) | the next 3 headlines | source, age, position n / N | quiet |
| F1 | next race countdown (v5) | last podium (3 drivers) | circuit name, race date and local start time | days to race / quiet |
| STATS | today (v5 pages) | aircraft per hour today as bars (arr / dep / over stacked) | records: highest, fastest, busiest hour; yesterday's total | quiet |
| SYSTEM | heap, fps, uptime pages (v5) | 24 h graphs: heap, fps | IP, signal, source, version | quiet |
| NETWORK | v5 page (probes are off) | WiFi signal history graph (NEW: sample RSSI every 30 s) | node, channel, outages | quiet |
| DISPLAY / LOCATION | settings list (v5) | live preview: theme colours / map dot of the chosen airport | the value being changed and the hint | preview colour / quiet |
| MENU (hold left 2 s) | app list (v5) | big icon + one-line description of the highlighted app | position n / N and the hint | app position / quiet |
| Alerts | banner on both dials (v5 toast) | same | alert text (v5) | flash in the alert colour |

Fixes folded in:

| Item | Fix |
|---|---|
| Departure profile | own height scale (departures climb far steeper than 3 degrees, the icon hit the top and covered the text) |
| On-screen hints | v5 hints say "rotate / click"; rewrite as "right knob: ..." |
| Left OLED headers | drop the time, use the slot for the app's own value |

Build order proposal: CLOCK (right OLED rule + clock app), WEATHER, POMODORO, MARKETS, then ARRIVALS / APPROACH, then the rest.
