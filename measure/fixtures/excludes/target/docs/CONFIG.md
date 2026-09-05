# Configuration

| key | default | meaning |
|---|---|---|
| `rate_limit` | `true` | Limit each client to 60 redirects per minute |
| `short_code_length` | `6` | Length of generated short codes |
| `link_expiry_days` | `0` | Days until a link expires; 0 means never |
| `db_path` | `hopper.db` | Where the SQLite database lives |
| `admin_password_hash` | argon2 | How admin passwords are stored |

## Environment overrides

Every key can be set from the environment as `HOPPER_<KEY>` in upper case.
