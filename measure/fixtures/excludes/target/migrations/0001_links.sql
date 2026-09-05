CREATE TABLE links (
  code TEXT PRIMARY KEY,
  url TEXT NOT NULL,
  created_at REAL NOT NULL,
  expires_at REAL,
  hits INTEGER NOT NULL DEFAULT 0
);
