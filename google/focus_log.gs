// Desk Plane Radar: Focus log
// Receives each Pomodoro focus session from the ESP32 and writes it to
//   1. a Google Sheet "Desk Radar Focus Log" (tab Log = one row per session, tab Stats = summaries)
//   2. a Google Calendar named "Focus" (one event per session, at the real time)
//
// Setup (once, on your PERSONAL Google account):
//   1. script.google.com > New project > paste this file > Save
//   2. Run setup() once. Approve the permissions (Sheets, Calendar). Open View > Logs (Execution log):
//      it prints the sheet link and the TOKEN.
//   3. Deploy > New deployment > type Web app > Execute as: Me > Who has access: Anyone > Deploy.
//      Copy the Web app URL (ends in /exec).
//   4. In desk_plane_radar_v5.ino set FOCUS_LOG_URL to that URL and FOCUS_LOG_TOKEN to the token. Flash.
//   Run testLog() any time to add a fake 1-minute session and check both outputs.
//   After editing this script, use Deploy > Manage deployments > edit > New version (the URL stays the same).
//   v2 (firmware 5.5.0): the device also asks for its stats ({action:'stats'}): today, week, streaks, last 7 days.
//   Run testStats() to see what the device will get.

var SHEET_NAME = 'Desk Radar Focus Log';
var CAL_NAME = 'Focus';
var HEAD = ['id', 'date', 'start', 'end', 'planned min', 'focused min', 'status', 'task', 'week of', 'hour', 'logged at'];

function setup() {
  var p = PropertiesService.getScriptProperties();
  if (!p.getProperty('TOKEN')) p.setProperty('TOKEN', Utilities.getUuid().replace(/-/g, ''));
  var ss = getSheet_();
  getCal_();
  Logger.log('Sheet: ' + ss.getUrl());
  Logger.log('TOKEN: ' + p.getProperty('TOKEN'));
}

function doPost(e) {
  try {
    var d = JSON.parse(e.postData.contents);
    var token = PropertiesService.getScriptProperties().getProperty('TOKEN');
    if (!token || d.token !== token) return out_({ ok: false, err: 'token' });
    if (d.action === 'stats') return out_(stats_(d));
    return out_(record_(d));
  } catch (err) {
    return out_({ ok: false, err: String(err) });
  }
}

function doGet() { return out_({ ok: false, err: 'POST only' }); }

function record_(d) {
  var lock = LockService.getScriptLock();
  lock.waitLock(20000);
  try {
    var ss = getSheet_(), log = ss.getSheetByName('Log');
    var id = String(d.id);
    // retries from the device are safe: an id already in the sheet is acknowledged and skipped
    if (log.getLastRow() > 1 && log.getRange(2, 1, log.getLastRow() - 1, 1).createTextFinder(id).matchEntireCell(true).findNext())
      return { ok: true, dup: true };
    var start = new Date(Number(d.start) * 1000), end = new Date(Number(d.end) * 1000);
    var mins = Number(d.min) || 0, plan = Number(d.plan) || 0, done = d.done === true;
    var task = String(d.task || '').substring(0, 40);
    var tz = ss.getSpreadsheetTimeZone();
    var dayStr = Utilities.formatDate(start, tz, 'yyyy-MM-dd');
    var day = Utilities.parseDate(dayStr, tz, 'yyyy-MM-dd');               // midnight, as a real date
    var dow = Number(Utilities.formatDate(start, tz, 'u'));                 // 1 = Monday
    var monday = new Date(day.getTime() - (dow - 1) * 86400000);
    var hour = Number(Utilities.formatDate(start, tz, 'H'));
    log.appendRow(["'" + id, day, start, end, plan, mins, done ? 'done' : 'partial', "'" + task,
                   monday, hour, new Date()]);
    var title = (done ? 'Focus' : 'Focus (partial)') + (task ? ': ' + task : '') + ' - ' + mins + ' min';
    var ev = getCal_().createEvent(title, start, end, { description: 'Pomodoro from Desk Plane Radar. Planned ' + plan + ' min, focused ' + mins + ' min.' });
    ev.setColor(done ? CalendarApp.EventColor.GREEN : CalendarApp.EventColor.YELLOW);
    return { ok: true };
  } finally {
    lock.releaseLock();
  }
}

function stats_(d) {
  var ss = getSheet_(), log = ss.getSheetByName('Log'), tz = ss.getSpreadsheetTimeZone();
  var goal = Number(d.goal) || 180, wk = Number(d.wk) === 1, DAY = 86400000;
  var n = log.getLastRow() - 1, rows = n > 0 ? log.getRange(2, 1, n, 10).getValues() : [];
  var now = new Date(), cutoff = now.getTime() - 30 * DAY;
  function key(dt) { return Utilities.formatDate(dt, tz, 'yyyy-MM-dd'); }
  function dow(dt) { return Number(Utilities.formatDate(dt, tz, 'u')); }      // 1 = Monday .. 7 = Sunday
  function ago(i) { return new Date(now.getTime() - i * DAY); }
  var todayKey = key(now), byDay = {}, hours = {}, sess30 = 0, done30 = 0, min30 = 0, total = 0;
  var todaySess = 0, todayDone = 0, longest = 0;
  rows.forEach(function (r) {
    if (r[0] === '' || !r[1]) return;
    var day = key(new Date(r[1])), m = Number(r[5]) || 0, st = String(r[6]);
    byDay[day] = (byDay[day] || 0) + m; total += m;
    if (new Date(r[2]).getTime() >= cutoff) { sess30++; min30 += m; if (st === 'done') done30++; var h = Number(r[9]); hours[h] = (hours[h] || 0) + m; }
    if (day === todayKey) { todaySess++; if (st === 'done') todayDone++; if (m > longest) longest = m; }
  });
  function mins(dt) { return byDay[key(dt)] || 0; }
  function met(dt) { return mins(dt) >= goal; }
  function skip(dt) { return wk && dow(dt) >= 6; }                             // Mon-Fri mode: weekends neither count nor break
  var last7 = [], bestDayMin = 0, bestDay = '';
  for (var i = 6; i >= 0; i--) { var v = mins(ago(i)); last7.push(v); if (v > bestDayMin) { bestDayMin = v; bestDay = Utilities.formatDate(ago(i), tz, 'EEE'); } }
  var week = 0, prev = 0, dw = dow(now);
  for (var j = 0; j < dw; j++) { week += mins(ago(j)); prev += mins(ago(j + 7)); }   // Monday..today vs the same days last week
  var streak = 0;
  for (var k = met(now) ? 0 : 1; k < 3650; k++) { var dt = ago(k); if (skip(dt)) continue; if (met(dt)) streak++; else break; }
  var best = streak, days = Object.keys(byDay).sort();
  if (days.length) {
    var run = 0, first = Utilities.parseDate(days[0] + ' 12:00', tz, 'yyyy-MM-dd HH:mm').getTime();
    for (var t = first; t <= now.getTime(); t += DAY) { var dd = new Date(t); if (skip(dd)) continue; if (met(dd)) { run++; if (run > best) best = run; } else run = 0; }
  }
  var bestHour = -1, bh = 0;
  Object.keys(hours).forEach(function (h) { if (hours[h] > bh) { bh = hours[h]; bestHour = Number(h); } });
  return { ok: true, today: mins(now), sess: todaySess, done: todayDone, longest: longest, week: week, prev: prev,
           best: bestDayMin, bestDay: bestDay, avg: sess30 ? Math.round(min30 / sess30) : 0, donePct: sess30 ? Math.round(done30 * 100 / sess30) : 0,
           hour: bestHour, streak: streak, bestStreak: best, last7: last7, totalH: Math.round(total / 60) };
}

function testStats() { Logger.log(JSON.stringify(stats_({ goal: 180, wk: 0 }))); }

function getSheet_() {
  var p = PropertiesService.getScriptProperties(), id = p.getProperty('SHEET_ID'), ss = null;
  if (id) { try { ss = SpreadsheetApp.openById(id); } catch (e) { ss = null; } }
  if (!ss) {
    ss = SpreadsheetApp.create(SHEET_NAME);
    ss.setSpreadsheetTimeZone(Session.getScriptTimeZone());
    var log = ss.getSheets()[0].setName('Log');
    log.getRange(1, 1, 1, HEAD.length).setValues([HEAD]).setFontWeight('bold');
    log.setFrozenRows(1);
    log.getRange('C:D').setNumberFormat('yyyy-mm-dd hh:mm');
    log.getRange('K:K').setNumberFormat('yyyy-mm-dd hh:mm');
    log.getRange('B:B').setNumberFormat('yyyy-mm-dd');
    log.getRange('I:I').setNumberFormat('yyyy-mm-dd');
    buildStats_(ss);
    p.setProperty('SHEET_ID', ss.getId());
  }
  return ss;
}

function buildStats_(ss) {
  var st = ss.insertSheet('Stats');
  var f = [
    ['Focused min today', '=SUMIFS(Log!F:F,Log!B:B,TODAY())'],
    ['Focused min this week', '=SUMIFS(Log!F:F,Log!I:I,TODAY()-WEEKDAY(TODAY(),3))'],
    ['Focused hours, all time', '=ROUND(SUM(Log!F2:F)/60,1)'],
    ['Sessions, all time', '=COUNTA(Log!A2:A)'],
    ['Completed %', '=IFERROR(COUNTIF(Log!G2:G,"done")/COUNTA(Log!G2:G),0)'],
    ['Avg focused min per session', '=IFERROR(ROUND(AVERAGE(Log!F2:F),1),0)']
  ];
  st.getRange(1, 1, f.length, 2).setValues(f);
  st.getRange('A1:A' + f.length).setFontWeight('bold');
  st.getRange('B5').setNumberFormat('0%');
  st.getRange('D1').setValue('By day').setFontWeight('bold');
  st.getRange('D2').setFormula('=QUERY(Log!A:K,"select B, sum(F), count(A) where A is not null group by B order by B desc label B \'Day\', sum(F) \'Minutes\', count(A) \'Sessions\'",1)');
  st.getRange('H1').setValue('By week').setFontWeight('bold');
  st.getRange('H2').setFormula('=QUERY(Log!A:K,"select I, sum(F), count(A) where A is not null group by I order by I desc label I \'Week of\', sum(F) \'Minutes\', count(A) \'Sessions\'",1)');
  st.getRange('L1').setValue('By start hour').setFontWeight('bold');
  st.getRange('L2').setFormula('=QUERY(Log!A:K,"select J, sum(F), count(A) where A is not null group by J order by J label J \'Hour\', sum(F) \'Minutes\', count(A) \'Sessions\'",1)');
  st.getRange('P1').setValue('By task').setFontWeight('bold');
  st.getRange('P2').setFormula('=QUERY(Log!A:K,"select H, sum(F), count(A) where A is not null group by H order by sum(F) desc label H \'Task\', sum(F) \'Minutes\', count(A) \'Sessions\'",1)');
  SpreadsheetApp.flush();
  var chart = st.newChart().asColumnChart()
    .addRange(st.getRange('H2:I30'))
    .setOption('title', 'Focused minutes per week')
    .setOption('legend', { position: 'none' })
    .setPosition(10, 1, 0, 0).build();
  st.insertChart(chart);
}

function getCal_() {
  var cals = CalendarApp.getCalendarsByName(CAL_NAME);
  return cals.length ? cals[0] : CalendarApp.createCalendar(CAL_NAME, { color: CalendarApp.Color.GREEN });
}

function out_(o) {
  return ContentService.createTextOutput(JSON.stringify(o)).setMimeType(ContentService.MimeType.JSON);
}

function testLog() {
  var now = Math.floor(Date.now() / 1000);
  Logger.log(JSON.stringify(record_({ id: 'test-' + now, start: now - 60, end: now, plan: 25, min: 1, done: false, task: 'test entry' })));
}
