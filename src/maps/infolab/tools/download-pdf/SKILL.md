---
name: download-pdf
description: Download PDF from URL and return as transient Note with binary content
type: python
trusted: true
parameters:
  - name: url
    type: string
    description: URL of PDF to download
---

# Download PDF

Downloads a PDF file from a URL and returns it as a transient Note.

## Input
- `url`: URL string pointing to a PDF file

## Output
Returns a Note containing the PDF binary content (base64 encoded).

## Usage
Use this to fetch papers before text extraction:
1. Download PDF with this tool
2. Pass result to `extract-paper-text` for processing

