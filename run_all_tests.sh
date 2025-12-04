#!/bin/bash
# Run all Cognitive Workbench tests with various options

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CHARACTER="${CHARACTER:-Jill}"
VERBOSE=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --character)
            CHARACTER="$2"
            shift 2
            ;;
        --verbose|-v)
            VERBOSE="--verbose"
            shift
            ;;
        --all)
            RUN_ALL=true
            shift
            ;;
        --primitives)
            RUN_PRIMITIVES=true
            shift
            ;;
        --tools)
            RUN_TOOLS=true
            shift
            ;;
        --non-llm)
            RUN_NON_LLM=true
            shift
            ;;
        --quick)
            RUN_QUICK=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --all              Run all tests (primitives + tools + map tools)"
            echo "  --primitives       Run only primitive tests"
            echo "  --tools            Run all tool tests (non-LLM + LLM + map tools)"
            echo "  --non-llm          Run primitives + non-LLM tools (fast subset)"
            echo "  --quick            Same as --non-llm (primitives + non-LLM tools)"
            echo "  --character NAME   Character name (default: Jill)"
            echo "  --verbose, -v      Verbose output"
            echo "  --help, -h         Show this help"
            echo ""
            echo "Examples:"
            echo "  $0 --quick                    # Fast tests only"
            echo "  $0 --all                      # Run everything"
            echo "  $0 --primitives --verbose      # Primitive tests with verbose output"
            echo "  $0 --tools --character Alice   # Tool tests for Alice character"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Default: run quick tests if no specific option provided
if [[ -z "$RUN_ALL" && -z "$RUN_PRIMITIVES" && -z "$RUN_TOOLS" && -z "$RUN_NON_LLM" && -z "$RUN_QUICK" ]]; then
    RUN_QUICK=true
fi

# Track failures
FAILED=0

# Check if character is running
echo "Checking if character '$CHARACTER' is running..."
python3 << EOF
import zenoh
import json
import sys

try:
    config = zenoh.Config()
    session = zenoh.open(config)
    query_key = f'cognitive/$CHARACTER/execute_plan_sync'
    test_plan = [{"type": "create-note", "value": "test", "out": "\$check"}]
    payload = json.dumps({'plan': test_plan}).encode('utf-8')
    replies = session.get(
        query_key,
        payload=payload,
        target=zenoh.QueryTarget.BEST_MATCHING,
        consolidation=zenoh.ConsolidationMode.NONE,
        timeout=2.0
    )
    found = False
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            found = True
            break
    session.close()
    sys.exit(0 if found else 1)
except Exception:
    sys.exit(1)
EOF

if [ $? -ne 0 ]; then
    echo ""
    echo "⚠️  WARNING: Character '$CHARACTER' does not appear to be running!"
    echo "   The test suite requires an active Cognitive Workbench session."
    echo "   Please start the character with: python launcher.py scenarios/jill.yaml"
    echo ""
    echo "   Tests will likely fail if the character is not running."
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
    echo ""
fi

echo "=========================================="
echo "Cognitive Workbench Test Suite"
echo "=========================================="
echo "Character: $CHARACTER"
echo "Timestamp: $(date -Iseconds)"
echo ""

# Run primitive tests
if [[ "$RUN_ALL" == true || "$RUN_PRIMITIVES" == true || "$RUN_NON_LLM" == true || "$RUN_QUICK" == true ]]; then
    echo "Running primitive tests..."
    echo "----------------------------------------"
    
    python tests/test_primitives_basic.py --character "$CHARACTER" $VERBOSE || FAILED=$((FAILED + 1))
    echo ""
    
    python tests/test_primitives_mutators.py --character "$CHARACTER" $VERBOSE || FAILED=$((FAILED + 1))
    echo ""
    
    python tests/test_primitives_remaining.py --character "$CHARACTER" $VERBOSE || FAILED=$((FAILED + 1))
    echo ""
    
    python tests/test_primitives_sql.py --character "$CHARACTER" $VERBOSE || FAILED=$((FAILED + 1))
    echo ""
    
    python tests/test_map.py --character "$CHARACTER" $VERBOSE || FAILED=$((FAILED + 1))
    echo ""
    
    python tests/test_map_tools.py --character "$CHARACTER" $VERBOSE --skip-api || FAILED=$((FAILED + 1))
    echo ""
fi

# Run non-LLM tool tests
if [[ "$RUN_ALL" == true || "$RUN_TOOLS" == true || "$RUN_NON_LLM" == true || "$RUN_QUICK" == true ]]; then
    echo "Running non-LLM tool tests..."
    echo "----------------------------------------"
    
    python tests/test_tools_non_llm.py --character "$CHARACTER" $VERBOSE || FAILED=$((FAILED + 1))
    echo ""
fi

# Run LLM tool tests
if [[ "$RUN_ALL" == true || "$RUN_TOOLS" == true ]]; then
    echo "Running LLM tool tests..."
    echo "----------------------------------------"
    echo "⚠️  Note: LLM tests require model inference and may take longer."
    echo ""
    
    python tests/test_tools_llm.py --character "$CHARACTER" $VERBOSE || FAILED=$((FAILED + 1))
    echo ""
    
    echo "Running map tools tests (with API tests)..."
    echo "----------------------------------------"
    echo "⚠️  Note: Map tools tests include API calls and may take longer."
    echo ""
    
    python tests/test_map_tools.py --character "$CHARACTER" $VERBOSE || FAILED=$((FAILED + 1))
    echo ""
fi

# Summary
echo "=========================================="
echo "Test Suite Complete"
echo "=========================================="
if [[ $FAILED -eq 0 ]]; then
    echo "✅ All tests passed!"
    exit 0
else
    echo "❌ $FAILED test suite(s) failed"
    exit 1
fi

