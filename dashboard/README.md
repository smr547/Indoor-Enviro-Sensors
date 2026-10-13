# Weather Station Dashboard

First-cut Signal K webapp for the unified Reid/Barking Owl weather UI.

The dashboard is deliberately hardware-independent. It consumes the compatibility contract documented in ../docs/weather-data-dictionary.md.

## Deploy on shorepi

Clone or pull the weather-dashboard-v0.1 branch, then:

    cd dashboard
    make deploy

The webapp derives its Signal K WebSocket endpoint from the browser location, so no Reid IP address is compiled into the application.

v0.1 scope:
- live Signal K connection
- temperature, humidity and pressure
- wind direction, speed, 10-minute average and gust when available
- five-minute rain and absolute LoRa bucket count when available
- Reid Victron station-power telemetry when available
- alias handling for known Reid/Barking Owl path differences

## Rainfall aggregates (v0.2)
The same package also provides a Signal K server plugin (`index.js`) that reads InfluxDB 1.x on ShorePi via localhost:8086 and serves `/plugins/weather-station-dashboard/rain`. The browser never accesses InfluxDB directly.

It reports four totals from `mydata` / `environment.rain.5min` with source `IndoorSensors.XX`:
- Today, starting at local midnight
- Since the most recent local 09:00
- Rolling last 24 hours
- Current calendar month

Calendar boundaries use `Australia/Sydney` and handle daylight saving. Values are rounded for display only; missing data is flagged rather than interpreted as a reliable zero. Refresh is every 60 seconds.

Defaults: `WEATHER_INFLUX_HOST=127.0.0.1`, `WEATHER_INFLUX_PORT=8086`, `WEATHER_INFLUX_DB=mydata`. Optional `WEATHER_INFLUX_USER` and `WEATHER_INFLUX_PASSWORD` can be supplied in the Signal K service environment.

**Deployment check:** Ensure the Signal K server loads/enables the `weather-station-dashboard` plugin as well as serving the webapp. Check the JSON endpoint after deployment. Existing firmware is unchanged.

**Data caveats:** October 2026 includes artificial calibration tips and historical double-counted readings. Five-minute bucket intervals crossing 09:00/midnight are attributed by report timestamp. Daily sample completeness is estimated from 5-minute cadence; a partial interval near the boundary can affect the total.

Historical charts remain deferred.
