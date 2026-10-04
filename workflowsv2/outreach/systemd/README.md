# Timer for the outreach daily run

The two unit files here start `daily.sh` every day at 08:30 as a user
systemd timer. To install or update them:

    cp workflowsv2/outreach/systemd/outreach-daily.* ~/.config/systemd/user/
    systemctl --user daemon-reload
    systemctl --user enable --now outreach-daily.timer

- Next run: `systemctl --user list-timers outreach-daily.timer`
- Output of the last run: `journalctl --user -u outreach-daily.service -e`
- Run it now: `systemctl --user start outreach-daily.service`
- Stop the daily runs: `systemctl --user disable --now outreach-daily.timer`

The run leaves a report in `scenarios/jill_chat/Jill/inbox/`, which Jill's
`job-reports` sensor delivers to her. Her scenario file expects that report
by 10:00 and has her told once if it is not there.
