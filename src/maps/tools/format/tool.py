"""
Format tool - formats text content for readable display.
"""

def tool(value, **kwargs):
    """
    Format text content with headers, structure, and spacing.
    
    Args:
        value: Input text (string)
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
    
    # Get optional parameters
    title = kwargs.get('title', 'Formatted Content')
    style = kwargs.get('style', 'document')
    
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
        items = [s.strip() for s in value.replace('\n', '. ').split('.') if s.strip()]
        lines.append('Content:')
        lines.append('')
        for item in items:
            if item:
                lines.append(f'  • {item}')
        lines.append('')
    
    elif style == 'outline':
        # Format as numbered outline
        items = [s.strip() for s in value.split('\n') if s.strip()]
        lines.append('Content:')
        lines.append('')
        for i, item in enumerate(items, 1):
            lines.append(f'{i}. {item}')
        lines.append('')
    
    else:  # 'document' style (default)
        # Format as document with paragraphs
        paragraphs = [p.strip() for p in value.split('\n\n') if p.strip()]
        lines.append('Content:')
        lines.append('')
        for para in paragraphs:
            # Wrap long paragraphs
            words = para.split()
            current_line = []
            current_length = 0
            for word in words:
                if current_length + len(word) + 1 > 70:
                    if current_line:
                        lines.append('  ' + ' '.join(current_line))
                        current_line = [word]
                        current_length = len(word)
                else:
                    current_line.append(word)
                    current_length += len(word) + 1
            if current_line:
                lines.append('  ' + ' '.join(current_line))
            lines.append('')
    
    # Add footer
    lines.append('-' * 60)
    
    return '\n'.join(lines)

