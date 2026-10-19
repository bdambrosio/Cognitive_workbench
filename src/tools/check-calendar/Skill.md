---
name: check-calendar
description: Read events from the user's calendars (their own Google calendar and any others configured, such as a school calendar) for a date range. Returns events in time order, grouped by day, with location, organizer and the user's RSVP response where they are invited. Read-only — it cannot create, change or answer events.
args:
  calendar: optional string — one calendar by name; default is all calendars. An unknown name returns the list of configured names.
  start: optional string — first day, YYYY-MM-DD; default today
  days: optional int (default 7, max 92) — number of days from `start`
  end: optional string — last day, YYYY-MM-DD, included; used instead of `days` when given
---

# check-calendar

Reads iCal (.ics) feeds and lists the events in a date range, in local
time (Pacific), including events a feed gives in another time zone.
Recurring events appear once per occurrence. Cancelled events are left
out. A range is at most 92 days.

## Calendars

Each calendar is one environment variable, `CALENDAR_ICS_<NAME>`, holding
its feed address. The name used in `calendar` and shown on each event is
`<NAME>` lowercased with `_` turned into `-`: `CALENDAR_ICS_GRANDKIDS_SCHOOL`
is `grandkids-school`. Which calendars exist is set by the user; a call
that finds no events, or names an unknown calendar, lists them.

The feed addresses are secrets and are never shown.

## Each event

- Time range, or "all day" (with the last day if it spans several).
- Calendar name in brackets, then the title.
- Location and organizer when the feed has them.
- `Your response:` accepted / declined / tentative / not answered — only
  on events where the user is an attendee.
- The description, cut at 500 characters with a line giving its full length.

If one calendar fails to load, the result names it and still shows the
others.

## Examples

```json
{"thought": "what's on this week", "tool": "check-calendar"}
```

```json
{"thought": "is there a school holiday in November", "tool": "check-calendar", "calendar": "grandkids-school", "start": "2026-11-01", "days": 30}
```
