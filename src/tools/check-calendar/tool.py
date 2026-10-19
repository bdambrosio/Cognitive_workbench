"""
check-calendar — read-only events from one or more iCal (.ics) feeds.

Each calendar is one environment variable, CALENDAR_ICS_<NAME>, holding
the feed's address. For a Google calendar that is the "Secret address in
iCal format"; any other .ics feed (a school calendar, say) works the same
way. The calendar's name as the agent sees it is <NAME> lowercased, with
"_" turned into "-".

The addresses are bearer secrets: anyone holding one can read the
calendar. They never appear in the observation, in a log line or in an
error message; a calendar is only ever referred to by its name.

Recurring events are expanded with recurring-ical-events, which handles
repeat rules, skipped dates, edited single occurrences and time zones.
"""

import logging
import os
import urllib.request
from datetime import date, datetime, timedelta
from typing import Any, Dict, List, Optional, Tuple

import icalendar
import recurring_ical_events

from utils.doc_extract import html_to_markdown

_logger = logging.getLogger(__name__)

_ENV_PREFIX = "CALENDAR_ICS_"
_DEFAULT_DAYS = 7
_MAX_DAYS = 92
_DESCRIPTION_CHARS = 500
_HTTP_TIMEOUT_S = 30

_RESPONSES = {
    "ACCEPTED": "accepted",
    "DECLINED": "declined",
    "TENTATIVE": "tentative",
    "NEEDS-ACTION": "not answered",
    "DELEGATED": "delegated",
}


def _configured_calendars() -> Dict[str, str]:
    """Calendar name -> feed address, from the environment."""
    calendars = {}
    for key, value in os.environ.items():
        if key.startswith(_ENV_PREFIX) and value.strip():
            name = key[len(_ENV_PREFIX):].lower().replace("_", "-")
            calendars[name] = value.strip()
    return dict(sorted(calendars.items()))


def _load(url: str) -> icalendar.Calendar:
    if url.startswith("webcal://"):
        url = "https://" + url[len("webcal://"):]
    # urllib rather than requests: at DEBUG level urllib3 logs each request
    # path, and for a secret address the path is the secret.
    with urllib.request.urlopen(url, timeout=_HTTP_TIMEOUT_S) as response:
        return icalendar.Calendar.from_ical(response.read())


def _owner_addresses(cal: icalendar.Calendar) -> set:
    """Addresses whose RSVP counts as the user's own. Google names a
    primary calendar's feed after the account's address; GMAIL_ADDRESS
    covers a feed named otherwise."""
    addresses = set()
    for candidate in (str(cal.get("X-WR-CALNAME", "")), os.getenv("GMAIL_ADDRESS", "")):
        if "@" in candidate:
            addresses.add(candidate.strip().lower())
    return addresses


def _as_list(value) -> list:
    if value is None:
        return []
    return value if isinstance(value, list) else [value]


def _address(prop) -> str:
    return str(prop).lower().removeprefix("mailto:")


def _user_response(event, owners: set) -> Optional[str]:
    for attendee in _as_list(event.get("ATTENDEE")):
        if _address(attendee) in owners:
            partstat = str(attendee.params.get("PARTSTAT", "NEEDS-ACTION")).upper()
            return _RESPONSES.get(partstat, partstat.lower())
    return None


def _organizer(event) -> Optional[str]:
    organizer = event.get("ORGANIZER")
    if organizer is None:
        return None
    name = organizer.params.get("CN")
    return str(name) if name else _address(organizer)


def _local(value) -> Tuple[datetime, bool]:
    """(start as a local datetime, is_all_day)."""
    if isinstance(value, datetime):
        if value.tzinfo is None:
            return value, False
        return value.astimezone().replace(tzinfo=None), False
    return datetime.combine(value, datetime.min.time()), True


def _description_text(raw: str) -> str:
    """Google writes a description edited in its web page as HTML; other
    descriptions are plain text and keep their line breaks."""
    raw = raw.strip()
    if "<" in raw and ">" in raw:
        return html_to_markdown(raw)
    return raw


def _event_row(event, calendar_name: str, owners: set) -> Dict[str, Any]:
    start, all_day = _local(event.get("DTSTART").dt)
    end_prop = event.get("DTEND")
    end = _local(end_prop.dt)[0] if end_prop is not None else None
    return {
        "start": start,
        "end": end,
        "all_day": all_day,
        "calendar": calendar_name,
        "title": str(event.get("SUMMARY", "(no title)")),
        "location": str(event.get("LOCATION", "")).strip(),
        "organizer": _organizer(event),
        "response": _user_response(event, owners),
        "description": _description_text(str(event.get("DESCRIPTION", ""))),
    }


def _format_row(row: Dict[str, Any]) -> str:
    if row["all_day"]:
        when = "all day"
        if row["end"] and row["end"] - row["start"] > timedelta(days=1):
            last_day = (row["end"] - timedelta(days=1)).strftime("%a %b %d")
            when = f"all day, through {last_day}"
    else:
        when = row["start"].strftime("%H:%M")
        if row["end"]:
            when += "-" + row["end"].strftime("%H:%M")
    lines = [f"- {when}  [{row['calendar']}]  {row['title']}"]
    if row["location"]:
        lines.append("  Location: " + " ".join(row["location"].split()))
    if row["organizer"]:
        lines.append(f"  Organizer: {row['organizer']}")
    if row["response"]:
        lines.append(f"  Your response: {row['response']}")
    description = row["description"]
    if description:
        if len(description) > _DESCRIPTION_CHARS:
            description = (description[:_DESCRIPTION_CHARS]
                           + f" [cut; {len(row['description'])} characters in full]")
        lines.append("  " + description.replace("\n", "\n  "))
    return "\n".join(lines)


def _parse_start(raw) -> date:
    if not raw:
        return date.today()
    return datetime.strptime(str(raw).strip(), "%Y-%m-%d").date()


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    calendars = _configured_calendars()
    if not calendars:
        return {"status": "error",
                "text": f"check-calendar: no calendars configured "
                        f"(no {_ENV_PREFIX}<NAME> environment variables)"}

    wanted = (args.get("calendar") or "").strip().lower()
    if wanted and wanted not in calendars:
        return {"status": "error",
                "text": f"check-calendar: no calendar named '{wanted}'. "
                        f"Configured: {', '.join(calendars)}"}

    try:
        start = _parse_start(args.get("start"))
    except ValueError:
        return {"status": "error",
                "text": "check-calendar: `start` must be YYYY-MM-DD"}
    if args.get("end"):
        try:
            last = datetime.strptime(str(args["end"]).strip(), "%Y-%m-%d").date()
        except ValueError:
            return {"status": "error",
                    "text": "check-calendar: `end` must be YYYY-MM-DD"}
        days = (last - start).days + 1
        if days < 1:
            return {"status": "error",
                    "text": "check-calendar: `end` is before `start`"}
    else:
        try:
            days = int(args.get("days") or _DEFAULT_DAYS)
        except (TypeError, ValueError):
            return {"status": "error",
                    "text": "check-calendar: `days` must be a whole number"}
    days = max(1, min(days, _MAX_DAYS))
    end = start + timedelta(days=days)

    selected = {wanted: calendars[wanted]} if wanted else calendars
    rows: List[Dict[str, Any]] = []
    failures: List[str] = []
    for name, url in selected.items():
        try:
            cal = _load(url)
            owners = _owner_addresses(cal)
            for event in recurring_ical_events.of(cal).between(start, end):
                if str(event.get("STATUS", "")).upper() != "CANCELLED":
                    rows.append(_event_row(event, name, owners))
        except Exception as e:
            # The exception text can contain the address, so only its type
            # is logged.
            _logger.warning(f"check-calendar: calendar '{name}' failed: {type(e).__name__}")
            failures.append(f"{name} ({type(e).__name__})")

    rows.sort(key=lambda r: (r["start"].date(), not r["all_day"], r["start"]))
    last_day = (end - timedelta(days=1)).strftime("%a %b %d %Y")
    header = (f"Events from {start.strftime('%a %b %d %Y')} through {last_day}, "
              f"local time. Calendars: {', '.join(selected)}.")
    parts = [header]
    if failures:
        parts.append(f"Could not load: {', '.join(failures)}.")
    if not rows:
        parts.append("No events in this range.")
        if not wanted:
            parts.append(f"Configured calendars: {', '.join(calendars)}.")
    current_day = None
    for row in rows:
        if row["start"].date() != current_day:
            current_day = row["start"].date()
            parts.append(f"\n{current_day.strftime('%A %b %d')}")
        parts.append(_format_row(row))

    status = "error" if failures and len(failures) == len(selected) else "ok"
    return {"status": status, "text": "\n".join(parts)}
