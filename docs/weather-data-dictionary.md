# Weather data dictionary v0.1

This document defines the application-facing weather data contract shared by Reid and Barking Owl. It records observed production paths first; canonicalisation of firmware paths is deliberately deferred until the dashboard has proved the model.

## Principles

1. The dashboard consumes weather concepts, not station hardware.
2. Existing production stations are not changed merely to satisfy the UI.
3. A compatibility layer maps observed station dialects to canonical concepts.
4. Signal K units and semantics are part of the contract.
5. Raw observations are retained where useful; interval and historical values may be derived downstream.
6. Power/health capabilities are optional because stations may use different power systems.

## Core concepts

| Concept | Canonical application key | Reid observed path | Barking Owl observed path | Units | Notes |
|---|---|---|---|---|---|
| Air temperature | temperature | environment.outside.temperature | environment.outside.temperature | K | Common path |
| Relative humidity | humidity | environment.outside.relativeHumidity | environment.outside.humidity | ratio | Path dialect differs |
| Ambient pressure | pressure | environment.outside.pressure | environment.outside.pressure | Pa preferred | Reid legacy producer reports hPa; Barking Owl reports Pa |
| MSL pressure | pressureMSL | environment.outside.pressureMSL | derived when available | Pa/hPa by metadata | Derived quantity; altitude-dependent |
| Station position | position | navigation.position | when available | WGS84 degrees | GPS-derived latitude/longitude; dashboard may link to map |
| Station elevation | altitude | navigation.gnss.antennaAltitude | when available | m | GPS GGA antenna altitude; input to MSL pressure derivation at Reid |
| Wind speed | windSpeed | environment.wind.speedTrue | environment.wind.speedApparent | m/s | Fixed-station semantics need final naming decision |
| Wind direction | windDirection | environment.wind.directionTrue | environment.wind.angleApparent | rad | Fixed-station semantics need final naming decision |
| 10 min mean speed | windAverage | environment.wind.speedAverage | not observed | m/s | Reid legacy firmware |
| 10 min mean direction | windDirectionAverage | environment.wind.directionAverage | not observed | rad | Vector average |
| Gust | windGust | environment.wind.gust | not observed | m/s | Reid implementation: max 12 s pulse window over 10 min |
| Rain interval | rain5min | environment.rain.5min | environment.rain.volume5min | mm | Accumulation over five-minute reporting interval |
| Absolute bucket count | rainBucketCount | environment.outside.rainGauge.bucketCount | not observed | count | LoRa rain gauge; monotonic counter is preferred raw transport |

## Optional station-health concepts observed at Reid

The Victron MPPT is independently published to Signal K over BLE:

- electrical.solar.Weather_Solar.voltage
- electrical.solar.Weather_Solar.current
- electrical.solar.Weather_Solar.panelPower
- electrical.solar.Weather_Solar.loadCurrent
- electrical.solar.Weather_Solar.chargingMode
- electrical.solar.Weather_Solar.yieldToday

These are capabilities, not requirements. A future common station design may expose equivalent health information from different hardware.

## Compatibility rules for dashboard v0.1

The webapp tries aliases in order. It must not require firmware changes.

- humidity: environment.outside.relativeHumidity, then environment.outside.humidity
- windSpeed: environment.wind.speedTrue, then environment.wind.speedApparent
- windDirection: environment.wind.directionTrue, then environment.wind.angleApparent
- rain5min: environment.rain.5min, then environment.rain.volume5min

Pressure conversion is metadata-sensitive in the long-term contract. For the first Reid deployment, values below 2000 are treated as hPa and values above 2000 as Pa.

## Open questions

- Final Signal K names for fixed-station wind speed and meteorological direction.
- Canonical raw rainfall namespace and reset semantics for cumulative counters.
- Historical API between the webapp and InfluxDB for today/24 h rainfall and trend plots.
- Common station-health namespace independent of Victron hardware.
- Freshness thresholds for live/late/offline status.
- For fixed stations, whether MSL pressure should use a configured elevation rather than instantaneous GPS altitude.

## Provenance

The Reid legacy firmware in this repository publishes the Reid paths documented above. Barking Owl observations were captured from its live Signal K vessels/self API on 2026-10-07. This document intentionally distinguishes observed production behaviour from proposed future standardisation.
