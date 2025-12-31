"""
Comprehensive diagnostic script to find digging/attacking methods in Minescript.
Run in Minecraft with: \diagnostic
"""
import minescript as m  # pyright: ignore[reportMissingImports]

def print_chat(msg):
    print(msg)

print_chat("=== Targeted Method Search ===")
# Search for anything related to manipulation or destruction
keywords = ['dig', 'break', 'attack', 'punch', 'hit', 'swing', 'use', 'block']
found = []
for attr in dir(m):
    if any(k in attr.lower() for k in keywords) and not attr.startswith('_'):
        found.append(attr)

for method in sorted(found):
    try:
        func = getattr(m, method)
        if callable(func):
            import inspect
            try:
                sig = inspect.signature(func)
                print_chat(f" ✓ {method}{sig}")
            except:
                print_chat(f" ✓ {method} (callable)")
        else:
            print_chat(f" • {method} (attribute: {type(func)})")
    except Exception as e:
        print_chat(f" ✗ {method} failed: {e}")

print_chat("\n=== Full Method List (alphabetical) ===")
all_methods = [x for x in dir(m) if not x.startswith('_')]
# List more items this time, but in chunks
for i in range(0, len(all_methods), 50):
    chunk = all_methods[i:i+50]
    print_chat(f"Batch {i}-{i+len(chunk)}: {', '.join(chunk)}")

print_chat("\n=== Testing Targeted Block Info ===")
try:
    targeted = m.player_get_targeted_block(10.0)
    if targeted:
        print_chat(f"Targeted: {targeted.position} block={getattr(targeted, 'block', 'N/A')}")
except Exception as e:
    print_chat(f"Targeted block check failed: {e}")
