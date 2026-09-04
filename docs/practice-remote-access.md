# Reaching the practice page away from the desk

The practice page (`src/client_ui/practice.py`, port 8802) and the client
pages (`src/client_ui/app.py`, ports 8800 and 8801) run only on the desk
machine, because they hold every engagement's record and change its state.
They are not on tuuyi.com and should not be. To use them from elsewhere, go
through the droplet with two SSH tunnels. Nothing is opened to the internet.

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
