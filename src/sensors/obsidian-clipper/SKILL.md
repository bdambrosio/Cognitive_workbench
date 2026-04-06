---
name: obsidian-clipper
description: Detects new files in the Obsidian vault Clippings folder (from Web Clipper)
type: code
schedule: "30s"
parameters:
  vault_path: "/home/bruce/Documents/Obsidian Vault"
  clippings_folder: "Clippings"
---

Watches the Obsidian vault's Clippings folder for new markdown files created by the Obsidian Web Clipper extension. Reports the title, source URL, tags, and a content preview for each new clipping.
