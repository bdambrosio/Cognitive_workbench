"""
Format tool - formats text content for readable display.
"""
import json
import re

def tool(value, **kwargs):
    """
    Format text content with headers, structure, and spacing.
    
    Args:
        value: Input text (string) - can be JSON string or plain text
        **kwargs: Optional parameters
            - title: Custom title for formatted output
            - style: Formatting style ('document', 'list', 'outline')
    
    Returns:
        Formatted text string
    """
    if not isinstance(value, str):
        return {
            'status': 'failed',
            'reason': 'Input must be a string'
        }
    
    if not value.strip():
        return {
            'status': 'failed',
            'reason': 'Input is empty'
        }
    
    # Step 1: Try parsing as JSON, extract text field if present
    text = value
    try:
        parsed = json.loads(value)
        if isinstance(parsed, dict) and 'text' in parsed:
            text = parsed['text']
        elif isinstance(parsed, str):
            text = parsed
    except (json.JSONDecodeError, TypeError):
        # Not JSON, use as-is
        pass
    
    # Step 2: Unescape common sequences
    # Handle \\n, \\t, \\u#### sequences
    text = text.replace('\\n', '\n')
    text = text.replace('\\t', '\t')
    text = text.replace('\\r', '\r')
    
    # Handle Unicode escapes like \\u2019
    def unescape_unicode(match):
        try:
            return chr(int(match.group(1), 16))
        except (ValueError, OverflowError):
            return match.group(0)
    text = re.sub(r'\\u([0-9a-fA-F]{4})', unescape_unicode, text)
    
    # Step 3: Detect if it looks like a research paper
    is_paper = False
    if any(pattern in text[:500] for pattern in [
        'arXiv:', 'doi:', 'Abstract', 'Introduction', 'References',
        'Corresponding author', 'Figure ', 'Table '
    ]):
        is_paper = True
    
    # Get optional parameters
    title = kwargs.get('title', 'Formatted Content')
    style = kwargs.get('style', 'document')
    
    # If paper-like and no explicit style, use document style with better line preservation
    if is_paper and style == 'document':
        # Preserve line breaks more aggressively for papers
        return _format_paper(text, title)
    
    # Build formatted output
    lines = []
    
    # Add title header
    lines.append('=' * 60)
    lines.append(title.center(60))
    lines.append('=' * 60)
    lines.append('')
    
    # Format content based on style
    if style == 'list':
        # Format as bulleted list (split by sentences or newlines)
        items = [s.strip() for s in text.replace('\n', '. ').split('.') if s.strip()]
        lines.append('Content:')
        lines.append('')
        for item in items:
            if item:
                lines.append(f'  • {item}')
        lines.append('')
    
    elif style == 'outline':
        # Format as numbered outline
        items = [s.strip() for s in text.split('\n') if s.strip()]
        lines.append('Content:')
        lines.append('')
        for i, item in enumerate(items, 1):
            lines.append(f'{i}. {item}')
        lines.append('')
    
    else:  # 'document' style (default)
        # Format as document - preserve line breaks, only wrap very long lines
        lines.append('Content:')
        lines.append('')
        
        for line in text.split('\n'):
            line = line.rstrip()
            if not line:
                lines.append('')
            elif len(line) > 120:
                # Only wrap if line is extremely long
                words = line.split()
                current_line = []
                current_length = 0
                for word in words:
                    if current_length + len(word) + 1 > 80:
                        if current_line:
                            lines.append('  ' + ' '.join(current_line))
                            current_line = [word]
                            current_length = len(word)
                    else:
                        current_line.append(word)
                        current_length += len(word) + 1
                if current_line:
                    lines.append('  ' + ' '.join(current_line))
            else:
                # Preserve original line if reasonable length
                lines.append('  ' + line)
    
    # Add footer
    lines.append('-' * 60)
    
    return '\n'.join(lines)


def _format_paper(text, title):
    """Format research paper content with better structure preservation."""
    lines = []
    
    # Header
    lines.append('=' * 60)
    lines.append(title.center(60))
    lines.append('=' * 60)
    lines.append('')
    
    # Preserve structure - split by double newlines for paragraphs
    paragraphs = text.split('\n\n')
    
    for para in paragraphs:
        para = para.strip()
        if not para:
            continue
        
        # Check if paragraph looks like a heading (all caps, short, or starts with number)
        if (len(para) < 80 and para.isupper()) or re.match(r'^\d+\.\s+[A-Z]', para):
            lines.append('')
            lines.append(para)
            lines.append('')
        else:
            # Regular paragraph - preserve line breaks within paragraph
            for line in para.split('\n'):
                line = line.rstrip()
                if line:
                    # Only wrap if extremely long
                    if len(line) > 85:
                        words = line.split()
                        current_line = []
                        current_length = 0
                        for word in words:
                            if current_length + len(word) + 1 > 85:
                                if current_line:
                                    lines.append('  ' + ' '.join(current_line))
                                    current_line = [word]
                                    current_length = len(word)
                            else:
                                current_line.append(word)
                                current_length += len(word) + 1
                        if current_line:
                            lines.append('  ' + ' '.join(current_line))
                    else:
                        lines.append('  ' + line)
            lines.append('')
    
    lines.append('-' * 60)
    return '\n'.join(lines)
