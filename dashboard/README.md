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

Historical InfluxDB plots are intentionally deferred to the next increment.
