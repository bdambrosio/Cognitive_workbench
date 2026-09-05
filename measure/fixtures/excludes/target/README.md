# hopper

A small link shortener you run yourself.

## What it does

Only the hit count is recorded for each short link; nothing about the visitor is stored.

Rate limiting is on by default, so a single client cannot flood the redirect endpoint.

Short codes are six characters long.

Links never expire unless you set an expiry when you create them.

Passwords for the admin console are hashed with argon2.

## Configuration

All configuration options are documented in docs/CONFIG.md.

The default database is SQLite, so a single file is all the state there is.

## Licence

Licensed under the MIT License.
