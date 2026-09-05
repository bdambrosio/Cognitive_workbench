CREATE TABLE admins (
  name TEXT PRIMARY KEY,
  digest TEXT NOT NULL,
  salt TEXT NOT NULL
);
