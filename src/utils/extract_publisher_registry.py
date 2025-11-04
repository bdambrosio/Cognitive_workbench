#!/usr/bin/env python3
"""
Extract Zenoh publication metadata from annotated source files.
Generates a JSON registry for activity pattern designers.
"""

import re
import json
import sys
from pathlib import Path


def extract_publication_metadata(source_file):
    """
    Parse structured publication comments into registry.
    
    Looks for:
    # === ZENOH PUBLICATION ===
    # NAME: <name>
    # TOPIC: <topic>
    # DESCRIPTION: <description>
    # PAYLOAD: <payload_spec>
    # TRIGGERS: <activities>
    # ========================
    """
    with open(source_file) as f:
        content = f.read()
    
    pattern = r'# === ZENOH PUBLICATION(?:\s+\(ad-hoc\))? ===\n(.*?)# ========================'
    matches = re.findall(pattern, content, re.DOTALL)
    
    registry = []
    for match in matches:
        lines = [l.strip('# ').strip() for l in match.strip().split('\n')]
        metadata = {}
        for line in lines:
            if ': ' in line:
                key, value = line.split(': ', 1)
                metadata[key.lower()] = value
        
        if metadata.get('name'):
            registry.append({
                'name': metadata.get('name'),
                'topic': metadata.get('topic'),
                'description': metadata.get('description'),
                'payload': metadata.get('payload'),
                'triggers': metadata.get('triggers'),
                'source_file': str(source_file)
            })
    
    return registry


def main():
    """Extract from all source files and generate registry."""
    src_dir = Path(__file__).parent.parent
    
    # Files with annotations
    annotated_files = [
        'executive_node.py',
        'map_node.py',
        'infospace_executor.py',
        'situation_node.py'
    ]
    
    all_publications = []
    
    for filename in annotated_files:
        file_path = src_dir / filename
        if file_path.exists():
            print(f"Extracting from {filename}...", file=sys.stderr)
            publications = extract_publication_metadata(file_path)
            all_publications.extend(publications)
            print(f"  Found {len(publications)} publications", file=sys.stderr)
    
    # Sort by name
    all_publications.sort(key=lambda x: x['name'])
    
    # Write registry
    output_path = src_dir / 'publisher_registry.json'
    with open(output_path, 'w') as f:
        json.dump(all_publications, f, indent=2)
    
    print(f"\n✅ Registry written to {output_path}", file=sys.stderr)
    print(f"   Total publications: {len(all_publications)}", file=sys.stderr)
    
    # Also print to stdout for inspection
    print(json.dumps(all_publications, indent=2))


if __name__ == '__main__':
    main()

