---
name: extract-paper-text
description: Extract text content from PDF using pymupdf
type: python
trusted: true
parameters:
  - name: pdf_content
    type: string
    description: Base64 encoded PDF content
examples:
  - '{"type":"extract-paper-text","target":"$pdf_note","out":"$paper_text"}'
---

# Extract Paper Text

Extracts text content from a PDF using pymupdf library.

## Input
- `pdf_content`: Base64 encoded PDF binary data (from download-pdf)

## Output
Returns structured text containing:
- Full text extracted from all pages
- Page count
- Basic metadata if available

## Usage
Use after downloading PDF:
1. Download PDF with `download-pdf`
2. Extract text with this tool
3. Parse metadata with `extract-metadata`

