# Reaching the practice and client pages away from the desk

## The client site (since 2026-09-05)

`https://client.tuuyi.com` is the client site, `src/client_ui/site.py`,
running on the desk machine as the systemd user unit `tuuyi-site` on
127.0.0.1:8803. The internet reaches it through a Cloudflare Tunnel: the
system service `cloudflared` on the desk machine holds an outbound
connection, and Cloudflare routes `client.tuuyi.com` down it. No port is
open. Cloudflare Access sits in front: a visitor signs in with a one-time
code sent to their email, and the site trusts only the token Cloudflare
signs for that login (`src/client_ui/access.py`).

Who gets in: the practice's address opens every engagement and `/p/`; a
client's address opens `/e/<engagement>/` for the engagements whose
`client_emails` list it. Adding a client is two steps: their email in the
engagement (the practice page's "new engagement" form, or engagement.yaml)
and their email in the Access application's policy (Zero Trust, Access,
Applications, "Tuuyi client site", or the API with the token below).

Where things are on the desk machine:

- `~/.config/tuuyi-site.env` — PRACTICE_EMAILS, CF_ACCESS_TEAM, CF_ACCESS_AUD,
  SITE_URL, SMTP_USER, SMTP_PASS, MAIL_FROM (info@tuuyi.com; Gmail sends
  from the account address until that alias is verified under "Send mail as").
- `~/.config/systemd/user/tuuyi-site.service` — `systemctl --user status
  tuuyi-site`; `journalctl --user -u tuuyi-site`. `sudo loginctl
  enable-linger bruce` keeps it running with nobody logged in.
- `~/.config/cloudflare-tuuyi.token` — an API token with Tunnel, Access and
  DNS edit rights; `cloudflare-tuuyi.app.json` and `.idp.json` hold the
  Access application and login-method ids.
- `systemctl status cloudflared` — the tunnel. Account 1a01a511…, tunnel
  "client" df4fc357…, zone tuuyi.com 8948b346….

## The older route: SSH tunnels through the droplet

The single-session pages (`src/client_ui/practice.py`, port 8802;
`src/client_ui/app.py`, ports 8800 and 8801) still run only on the desk
machine and are not on tuuyi.com. To use them from elsewhere, go through
the droplet with two SSH tunnels. Nothing is opened to the internet.

## Once, on the desk machine, leave it running

    ssh -i ~/.ssh/droplet_deploy -N -R 8802:127.0.0.1:8802 root@165.227.60.233

This lends the desk machine's port 8802 to the droplet's own loopback. Add
`-R 8800:127.0.0.1:8800 -R 8801:127.0.0.1:8801` to lend the client pages
too. The droplet's firewall keeps these ports off the public interface;
only someone logged in to the droplet can reach them.

Start the page itself as usual:

    python3 src/client_ui/practice.py --port 8802

## From the laptop, wherever it is

    ssh -L 8802:127.0.0.1:8802 root@165.227.60.233

Then open, in the laptop's browser, the URL the practice page printed when
it started, which is `http://127.0.0.1:8802/?token=...`. The token is the
only login, so treat that URL as a password.

## When it stops working

- The reverse tunnel drops when the desk machine sleeps or the network
  changes. Rerun the first command. `autossh` keeps it up unattended if that
  becomes a habit.
- `ssh` refuses the reverse forward if port 8802 is already taken on the
  droplet by an earlier tunnel that never closed. Log in to the droplet and
  `pkill -f "8802:127.0.0.1:8802"`.
