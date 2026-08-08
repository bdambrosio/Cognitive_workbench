import requests
from lxml import etree
import re, time
import numpy as np
import os
import tempfile
import logging
from urllib.request import Request, urlopen
GROBID_URL = "http://localhost:8070/api/processFulltextDocument"

# Full-text parsing of a long paper takes tens of seconds; without a
# bound a wedged GROBID would hang the calling ReAct loop indefinitely.
_GROBID_TIMEOUT = 120

logger = logging.getLogger(__name__)

def _unixify_title(title: str) -> str:
    """Convert title to filesystem-safe filename."""
    safe = re.sub(r'[<>:"/\\|?*]', '', title)
    safe = re.sub(r'\s+', '_', safe)
    safe = re.sub(r'_+', '_', safe)
    safe = safe.strip('_')
    if len(safe) > 200:
        safe = safe[:200]
    return safe if safe else "paper"

def parse_pdf_grobid(pdf_filepath=None, pdf_url=None, title=None, chunk_size=1000, grobid_url=None):
    """
    Parses a PDF via GROBID and returns a list of semantically bounded chunks.
    Accepts either a local filepath or a URL (which will be downloaded).
    
    Args:
        pdf_filepath: Path to local PDF file (mutually exclusive with pdf_url)
        pdf_url: URL to PDF file (mutually exclusive with pdf_filepath)
        title: Title for PDF (used for temp filename when downloading from URL)
        chunk_size: Size of text chunks (default 1000)
        grobid_url: Optional GROBID server URL (defaults to module constant GROBID_URL)
    
    Returns:
        dict: {
            "title": str,
            "authors": str,
            "abstract": str,
            "chunks": List[Tuple[str, str]]  # [(Section Title, Text Content), ...]
        }
        or None if parsing fails
    """
    # Handle URL case: download to temp file
    if pdf_url:
        if pdf_filepath:
            raise ValueError("Cannot specify both pdf_filepath and pdf_url")
        
        # Generate title from URL if not provided
        if not title:
            title = pdf_url.split('/')[-1].replace('.pdf', '') or "document"
        
        # Create temp file path with unixified title
        safe_title = _unixify_title(title)
        temp_dir = tempfile.gettempdir()
        temp_filepath = os.path.join(temp_dir, f"{safe_title}.pdf")
        
        # Check if PDF already exists in temp folder
        if os.path.exists(temp_filepath):
            logger.info(f"PDF already exists at {temp_filepath}, skipping download")
            pdf_filepath = temp_filepath
        else:
            # Download PDF
            try:
                headers = {'User-Agent': 'Mozilla/5.0 (compatible; CognitiveWorkbench/1.0)'}
                request = Request(pdf_url, headers=headers)
                response = urlopen(request, timeout=30)
                pdf_content = response.read()
                
                with open(temp_filepath, 'wb') as f:
                    f.write(pdf_content)
                
                logger.info(f"Downloaded PDF to {temp_filepath}")
                pdf_filepath = temp_filepath
            except Exception as e:
                logger.error(f"Failed to download PDF from {pdf_url}: {e}")
                return None
    
    if not pdf_filepath:
        raise ValueError("Must specify either pdf_filepath or pdf_url")
    
    if not os.path.exists(pdf_filepath):
        logger.error(f"PDF file not found: {pdf_filepath}")
        return None
    
    # Use provided grobid_url or fall back to module constant
    # If grobid_url is provided but doesn't include the endpoint path, append it
    if grobid_url:
        if grobid_url.endswith('/'):
            url = grobid_url.rstrip('/') + '/api/processFulltextDocument'
        elif '/api/' not in grobid_url:
            url = grobid_url.rstrip('/') + '/api/processFulltextDocument'
        else:
            url = grobid_url
    else:
        url = GROBID_URL
    
    logger.info(f'Processing: {pdf_filepath}')
    
    # 1. Send to GROBID
    try:
        with open(pdf_filepath, 'rb') as f:
            files = {'input': f}
            response = requests.post(url, files=files,
                                     timeout=_GROBID_TIMEOUT)
        if response.status_code != 200:
            logger.error(f'GROBID Error {response.status_code}')
            return None
    except Exception as e:
        logger.error(f"Connection Error: {e}")
        return None

    # 2. Parse XML
    xml_content = response.text
    ns = {'tei': 'http://www.tei-c.org/ns/1.0'}
    try:
        tree = etree.fromstring(xml_content.encode('utf-8'))
    except etree.XMLSyntaxError:
        logger.error("Failed to parse GROBID XML output")
        return None

    extract = {"title": "Unknown", "authors": "", "abstract": "", "chunks": []}

    # --- Metadata Extraction ---
    
    # Title
    title_node = tree.xpath('.//tei:titleStmt/tei:title[@type="main"]', namespaces=ns)
    if title_node:
        extract["title"] = title_node[0].text or "Unknown Title"

    # Authors
    authors = tree.xpath('.//tei:fileDesc//tei:author/tei:persName', namespaces=ns)
    author_list = []
    for author in authors:
        names = author.xpath('.//tei:forename | .//tei:surname', namespaces=ns)
        full_name = ' '.join([n.text for n in names if n.text])
        if full_name: author_list.append(full_name)
    extract["authors"] = ', '.join(author_list)

    # Abstract (Treat as the first chunk)
    abstract_nodes = tree.xpath('.//tei:profileDesc/tei:abstract//text()', namespaces=ns)
    if abstract_nodes:
        abstract_text = ' '.join(abstract_nodes).strip()
        extract["abstract"] = abstract_text
        extract["chunks"].append(("Abstract", abstract_text))

    # --- Section Processing ---
    
    # Get all top-level divs in the body
    body_divs = tree.xpath('./tei:text/tei:body/tei:div', namespaces=ns)
    
    # Regex to clean citation markers like [12] or (Author, 2020) to reduce noise
    # This is aggressive; comment out if you want to keep citations.
    citation_pattern = re.compile(r'\[\d+\]|\(\w+ et al\., \d{4}\)')

    for div in body_divs:
        # 1. Get Section Header
        head_node = div.xpath('./tei:head', namespaces=ns)
        section_title = head_node[0].text if head_node and head_node[0].text else "Untitled Section"
        
        # 2. Filter Irrelevant Sections
        # We skip sections that are purely bibliographic or administrative
        skip_keywords = ['reference', 'bibliography', 'acknowledgement', 'declaration', 'conflict of interest']
        if any(keyword in section_title.lower() for keyword in skip_keywords):
            continue

        # 3. Get Paragraphs
        # We iterate paragraphs <p> to respect local structure
        paragraphs = div.xpath('./tei:p', namespaces=ns)
        
        current_chunk = ""
        
        for p in paragraphs:
            # Get text, strip whitespace
            p_text = ' '.join(p.xpath('.//text()')).strip()
            
            # Optional: Clean citations
            p_text = citation_pattern.sub('', p_text)
            
            if not p_text: continue

            # 4. Chunking Logic
            # If adding this paragraph exceeds chunk_size, save current and start new
            if len(current_chunk) + len(p_text) > chunk_size:
                if current_chunk:
                    extract["chunks"].append((section_title, current_chunk))
                current_chunk = p_text
            else:
                # Add to current chunk (with space)
                current_chunk = (current_chunk + "\n" + p_text) if current_chunk else p_text
        
        # Append any remaining text in this section
        if current_chunk:
            extract["chunks"].append((section_title, current_chunk))

    return extract


def extract_references_grobid(pdf_filepath=None, pdf_url=None, grobid_url=None):
    """
    Extract bibliography/references from PDF via GROBID.
    
    Args:
        pdf_filepath: Path to local PDF file (mutually exclusive with pdf_url)
        pdf_url: URL to PDF file (mutually exclusive with pdf_filepath)
        grobid_url: Optional GROBID server URL (defaults to module constant GROBID_URL)
    
    Returns:
        List[dict]: List of reference dicts with fields: authors, title, year, venue, doi, url, raw_citation
        or None if parsing fails
    """
    # Handle URL case: download to temp file (reuse logic from parse_pdf_grobid)
    if pdf_url:
        if pdf_filepath:
            raise ValueError("Cannot specify both pdf_filepath and pdf_url")
        
        title = pdf_url.split('/')[-1].replace('.pdf', '') or "document"
        safe_title = _unixify_title(title)
        temp_dir = tempfile.gettempdir()
        temp_filepath = os.path.join(temp_dir, f"{safe_title}.pdf")
        
        if os.path.exists(temp_filepath):
            logger.info(f"PDF already exists at {temp_filepath}, skipping download")
            pdf_filepath = temp_filepath
        else:
            try:
                headers = {'User-Agent': 'Mozilla/5.0 (compatible; CognitiveWorkbench/1.0)'}
                request = Request(pdf_url, headers=headers)
                response = urlopen(request, timeout=30)
                pdf_content = response.read()
                
                with open(temp_filepath, 'wb') as f:
                    f.write(pdf_content)
                
                logger.info(f"Downloaded PDF to {temp_filepath}")
                pdf_filepath = temp_filepath
            except Exception as e:
                logger.error(f"Failed to download PDF from {pdf_url}: {e}")
                return None
    
    if not pdf_filepath:
        raise ValueError("Must specify either pdf_filepath or pdf_url")
    
    if not os.path.exists(pdf_filepath):
        logger.error(f"PDF file not found: {pdf_filepath}")
        return None
    
    # Use provided grobid_url or fall back to module constant
    if grobid_url:
        if grobid_url.endswith('/'):
            url = grobid_url.rstrip('/') + '/api/processFulltextDocument'
        elif '/api/' not in grobid_url:
            url = grobid_url.rstrip('/') + '/api/processFulltextDocument'
        else:
            url = grobid_url
    else:
        url = GROBID_URL
    
    logger.info(f'Extracting references from: {pdf_filepath}')
    
    # Send to GROBID
    try:
        with open(pdf_filepath, 'rb') as f:
            files = {'input': f}
            response = requests.post(url, files=files,
                                     timeout=_GROBID_TIMEOUT)
        if response.status_code != 200:
            logger.error(f'GROBID Error {response.status_code}')
            return None
    except Exception as e:
        logger.error(f"Connection Error: {e}")
        return None
    
    # Parse XML
    xml_content = response.text
    ns = {'tei': 'http://www.tei-c.org/ns/1.0'}
    try:
        tree = etree.fromstring(xml_content.encode('utf-8'))
    except etree.XMLSyntaxError:
        logger.error("Failed to parse GROBID XML output")
        return None
    
    # Extract references from <bibl> elements
    references = []
    bibl_elements = tree.xpath('.//tei:listBibl/tei:biblStruct | .//tei:div[@type="references"]//tei:biblStruct | .//tei:biblStruct', namespaces=ns)
    
    for bibl in bibl_elements:
        ref = {}
        
        # Authors
        authors = []
        author_nodes = bibl.xpath('.//tei:author/tei:persName', namespaces=ns)
        for author in author_nodes:
            names = author.xpath('.//tei:forename | .//tei:surname', namespaces=ns)
            full_name = ' '.join([n.text for n in names if n.text])
            if full_name:
                authors.append(full_name)
        ref['authors'] = authors
        
        # Title (can be article, journal, or monograph title)
        title_nodes = bibl.xpath('.//tei:title[@level="a"] | .//tei:title[@level="m"]', namespaces=ns)
        if title_nodes:
            ref['title'] = ' '.join([t.text for t in title_nodes if t.text]).strip()
        else:
            ref['title'] = ""
        
        # Journal/venue
        journal_nodes = bibl.xpath('.//tei:title[@level="j"]', namespaces=ns)
        if journal_nodes:
            ref['venue'] = ' '.join([j.text for j in journal_nodes if j.text]).strip()
        else:
            ref['venue'] = ""
        
        # Year
        date_nodes = bibl.xpath('.//tei:date[@when]', namespaces=ns)
        if date_nodes:
            year_str = date_nodes[0].get('when', '')[:4]
            try:
                ref['year'] = int(year_str) if year_str.isdigit() else 0
            except:
                ref['year'] = 0
        else:
            ref['year'] = 0
        
        # DOI
        doi_nodes = bibl.xpath('.//tei:idno[@type="DOI"]', namespaces=ns)
        if doi_nodes:
            ref['doi'] = doi_nodes[0].text.strip() if doi_nodes[0].text else ""
        else:
            ref['doi'] = ""
        
        # URL
        url_nodes = bibl.xpath('.//tei:ptr[@type="web"]/@target | .//tei:idno[@type="URL"]', namespaces=ns)
        if url_nodes:
            if isinstance(url_nodes[0], str):
                ref['url'] = url_nodes[0]
            else:
                ref['url'] = url_nodes[0].text.strip() if url_nodes[0].text else ""
        else:
            ref['url'] = ""
        
        # Raw citation text (all text in bibl element)
        raw_text = ' '.join(bibl.xpath('.//text()'))
        ref['raw_citation'] = ' '.join(raw_text.split()).strip()
        
        # Only add if we have at least title or authors
        if ref.get('title') or ref.get('authors'):
            references.append(ref)
    
    logger.info(f"Extracted {len(references)} references from PDF")
    return references

# --- Usage Example ---
# data = parse_pdf_grobid("./my_paper.pdf")
# for section, text in data['chunks']:
#     print(f"--- {section} ---\n{text[:100]}...\n")

if __name__ == "__main__":
    import argparse
    import sys

    # 1. Setup simple CLI arguments
    parser = argparse.ArgumentParser(description="Test GROBID PDF Parsing")
    parser.add_argument("filepath", nargs="?", help="Path to the PDF file", 
                        default="/home/bruce/Downloads/pdfs/evo-memory.pdf") # Replace with a valid default path on your system
    args = parser.parse_args()

    # 2. Check if file exists
    import os
    if not os.path.exists(args.filepath):
        print(f"Error: File not found at {args.filepath}")
        sys.exit(1)

    # 3. Run the parser
    print(f"-" * 60)
    print(f"TESTING: {os.path.basename(args.filepath)}")
    print(f"-" * 60)
    
    start_time = time.time()
    result = parse_pdf_grobid(args.filepath)
    end_time = time.time()

    if result:
        # 4. Print Metadata
        print(f"\n[METADATA]")
        print(f"Title:    {result['title']}")
        print(f"Authors:  {result['authors']}")
        print(f"Abstract: {len(result['abstract'])} chars")
        print(f"Time:     {end_time - start_time:.2f}s")

        # 5. Print Chunk Preview (First 3 chunks + random middle chunk)
        print(f"\n[CHUNKS GENERATED: {len(result['chunks'])}]")
        
        # Helper to print a chunk nicely
        def print_chunk(index, section, text):
            preview = text.replace('\n', ' ')[:150] + "..."
            print(f"[{index}] {section.upper()}:")
            print(f"    {preview} ({len(text)} chars)")

        # Show first few
        for i, (section, text) in enumerate(result['chunks'][:3]):
            print_chunk(i, section, text)
        
        # Show one from the middle if enough chunks exist
        if len(result['chunks']) > 5:
            mid = len(result['chunks']) // 2
            print(f"    ... skipping ...")
            print_chunk(mid, result['chunks'][mid][0], result['chunks'][mid][1])

        print(f"-" * 60)
    else:
        print("Parsing failed.")
