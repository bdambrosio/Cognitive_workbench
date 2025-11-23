# Static Assets Directory

This directory contains static files served by the FastAPI Action Display.

## Favicon

Place a favicon file here to remove the 404 error:
- `/static/favicon.ico` (traditional format)
- `/static/favicon.png` (modern format, easier to create)

### Creating a Favicon in GIMP

1. **Create new image**: File → New → 32x32 pixels (or 16x16)
2. **Design your icon**: Use simple, high-contrast graphics
3. **Export as PNG**: File → Export As → `favicon.png`
   - For `.ico` format, use an online converter or GIMP plugin

The FastAPI server will automatically serve either `.ico` or `.png` format.

### Recommended Specifications

- **Size**: 32x32 pixels (most common) or 16x16 pixels
- **Format**: PNG (easiest) or ICO (traditional)
- **Design**: Simple, recognizable at small sizes
- **Colors**: High contrast for visibility

