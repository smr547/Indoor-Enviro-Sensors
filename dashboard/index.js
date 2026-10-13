'use strict';

// Signal K server plugin: historical rainfall summaries from local InfluxDB 1.x.
// No credentials or InfluxDB access are exposed to the browser.
const http = require('http');
const ZONE = 'Australia/Sydney';
const DB = process.env.WEATHER_INFLUX_DB || 'mydata';
const HOST = process.env.WEATHER_INFLUX_HOST || '127.0.0.1';
const PORT = Number(process.env.WEATHER_INFLUX_PORT || 8086);
const SOURCE = 'IndoorSensors.XX';
const fmt = new Intl.DateTimeFormat('en-AU', {
  timeZone: ZONE, year: 'numeric', month: '2-digit', day: '2-digit',
  hour: '2-digit', hourCycle: 'h23', timeZoneName: 'shortOffset'
});

function localParts(date) {
  const p = Object.fromEntries(fmt.formatToParts(date).map(x => [x.type, x.value]));
  return { y: +p.year, m: +p.month, d: +p.day, h: +p.hour };
}
function utcBoundary(y, m, d, hour) {
  // Iterate the zone offset at the boundary (not at the current instant);
  // handles AEDT/AEST and dates crossing DST transitions.
  const naive = Date.UTC(y, m - 1, d, hour);
  let timestamp = naive;
  for (let i = 0; i < 3; i++) {
    const tz = fmt.formatToParts(new Date(timestamp)).find(p => p.type === 'timeZoneName').value;
    const match = tz.match(/GMT([+-])(\d{1,2})(?::(\d{2}))?/);
    if (!match) throw new Error('Cannot determine Australia/Sydney offset: ' + tz);
    const offset = (match[1] === '+' ? 1 : -1) * (+match[2] * 60 + +(match[3] || 0)) * 60000;
    timestamp = naive - offset;
  }
  return new Date(timestamp);
}
function queryInflux(sql) {
  return new Promise((resolve, reject) => {
    const qs = new URLSearchParams({ db: DB, q: sql });
    const auth = process.env.WEATHER_INFLUX_USER;
    if (auth) { qs.set('u', auth); qs.set('p', process.env.WEATHER_INFLUX_PASSWORD || ''); }
    const req = http.get({
      hostname: HOST, port: PORT, path: '/query?' + qs.toString(),
      timeout: 8000
    }, res => {
      let body = '';
      res.on('data', chunk => { body += chunk; if (body.length > 100000) req.destroy(new Error('Response too large')); });
      res.on('end', () => {
        if (res.statusCode !== 200) return reject(new Error('InfluxDB HTTP ' + res.statusCode));
        try {
          const data = JSON.parse(body);
          if (data.error || data.results?.[0]?.error) throw new Error(data.error || data.results[0].error);
          const series = data.results?.[0]?.series?.[0];
          resolve(series ? Object.fromEntries(series.columns.map((k, i) => [k, series.values[0][i]])) : {});
        } catch (e) { reject(e); }
      });
    });
    req.on('timeout', () => req.destroy(new Error('InfluxDB timeout')));
    req.on('error', reject);
  });
}
async function rainfall() {
  const now = new Date(), p = localParts(now);
  const midnight = utcBoundary(p.y, p.m, p.d, 0);
  const month = utcBoundary(p.y, p.m, 1, 0);
  const nine = utcBoundary(p.y, p.m, p.d, 9);
  const nineStart = now >= nine ? nine : (() => {
    const yesterday = new Date(Date.UTC(p.y, p.m - 1, p.d - 1));
    return utcBoundary(yesterday.getUTCFullYear(), yesterday.getUTCMonth() + 1, yesterday.getUTCDate(), 9);
  })();
  const windows = {
    today: midnight, since9am: nineStart,
    last24h: new Date(now.getTime() - 86400000), month
  };
  const end = now.toISOString();
  const entries = await Promise.all(Object.entries(windows).map(async ([name, start]) => {
    const sql = 'SELECT SUM("value") AS total, COUNT("value") AS samples ' +
      'FROM "environment.rain.5min" WHERE time >= \''
      + start.toISOString() + '\' AND time <= \'' + end +
      '\' AND "source" = \'' + SOURCE + '\'';
    const result = await queryInflux(sql);
    const elapsed = Math.max(0, now - start);
    const expected = Math.floor(elapsed / 300000);
    return [name, {
      mm: typeof result.total === 'number' ? Math.round(result.total * 100) / 100 : null,
      samples: result.samples || 0,
      expectedSamples: expected,
      complete: expected === 0 ? true : (result.samples || 0) >= Math.max(0, expected - 1),
      start: start.toISOString()
    }];
  }));
  return { timezone: ZONE, asOf: end, source: SOURCE, windows: Object.fromEntries(entries) };
}
module.exports = function (app) {
  return {
    id: 'weather-station-dashboard',
    name: 'Weather Station Dashboard Rainfall',
    description: 'Historical rainfall aggregates from local InfluxDB',
    schema: { type: 'object', properties: {} },
    start() { app.debug('Rainfall history API ready'); },
    stop() {},
    registerWithRouter(router) {
      router.get('/rain', async (req, res) => {
        try { res.set('Cache-Control', 'no-store'); res.json(await rainfall()); }
        catch (e) { app.error('Rainfall history query failed: ' + e.message); res.status(503).json({ error: 'Rainfall history unavailable' }); }
      });
    }
  };
};
