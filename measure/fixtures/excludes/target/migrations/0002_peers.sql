-- One row per redirect served, for the admin console's traffic view.
CREATE TABLE peers (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  code TEXT NOT NULL REFERENCES links(code),
  peer TEXT NOT NULL,
  client_string TEXT,
  served_at REAL NOT NULL
);
CREATE INDEX peers_code ON peers(code);
