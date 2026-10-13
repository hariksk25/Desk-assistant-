// DeskRadar feed: next calendar events + Gmail inbox summary.
// Deploy as Web app: Execute as "Me", Who has access "Anyone".
// Returns JSON. Called by the Make scenario, which merges Slack and relays to the ESP32.

function doGet(e) {
  var now = new Date();
  var later = new Date(now.getTime() + 24 * 3600 * 1000);

  // Calendar: next 3 events in 24 h, skipping all-day and declined
  var events = CalendarApp.getDefaultCalendar().getEvents(now, later);
  var cal = [];
  for (var i = 0; i < events.length && cal.length < 3; i++) {
    var ev = events[i];
    if (ev.isAllDayEvent()) continue;
    if (ev.getMyStatus && ev.getMyStatus() == CalendarApp.GuestStatus.NO) continue;
    var guests = ev.getGuestList ? ev.getGuestList().length : 0;
    cal.push({
      t: ev.getTitle().substring(0, 40),
      s: Math.floor(ev.getStartTime().getTime() / 1000),
      e: Math.floor(ev.getEndTime().getTime() / 1000),
      n: guests,
      v: (ev.getLocation() || '').indexOf('zoom') >= 0 || (ev.getDescription() || '').indexOf('zoom') >= 0 ? 'zoom'
       : (ev.getLocation() || '').indexOf('meet.google') >= 0 ? 'meet' : ''
    });
  }

  // Gmail: unread in Primary, plus the newest unread's sender and subject
  var unread = GmailApp.getInboxUnreadCount();
  var from = '', subj = '', age = 0;
  var threads = GmailApp.search('is:unread category:primary in:inbox', 0, 1);
  if (threads.length) {
    var msgs = threads[0].getMessages();
    var m = msgs[msgs.length - 1];
    from = m.getFrom().replace(/<.*>/, '').replace(/"/g, '').trim().substring(0, 24);
    subj = m.getSubject().substring(0, 40);
    age = Math.floor((now.getTime() - m.getDate().getTime()) / 60000);
  }

  var out = {
    ts: Math.floor(now.getTime() / 1000),
    cal: cal,
    mail: { unread: unread, from: from, subject: subj, age_min: age }
  };
  return ContentService.createTextOutput(JSON.stringify(out)).setMimeType(ContentService.MimeType.JSON);
}
