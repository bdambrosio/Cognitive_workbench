import os
import sys
import json
import argparse
import logging
import unittest

# Ensure src/ is importable when running standalone
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SRC_DIR = os.path.join(ROOT_DIR, 'src')
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)

# Ensure a 'logs' directory exists for imported modules that configure FileHandlers
try:
    os.makedirs('logs', exist_ok=True)
except Exception:
    pass

from middle_ontology import build_allowed_verbs
from templates import PLAN_VERBS


MIDDLE_ONTOLOGY = None


class TestBuildAllowedVerbs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if MIDDLE_ONTOLOGY is None:
            raise unittest.SkipTest("No middle ontology loaded; run this file with -f/--file to execute standalone.")
        # Compute once and print for visibility when running standalone
        logger = logging.getLogger('test_build_allowed_verbs')
        cls.ALLOWED = build_allowed_verbs(MIDDLE_ONTOLOGY, logger)
        try:
            print("allowed_verbs:", json.dumps(cls.ALLOWED))
        except Exception:
            print("allowed_verbs:", cls.ALLOWED)

    def test_returns_sorted_unique_list(self):
        logger = logging.getLogger('test_build_allowed_verbs')
        result = build_allowed_verbs(MIDDLE_ONTOLOGY, logger)
        self.assertIsInstance(result, list)
        self.assertEqual(result, sorted(result), "Result should be lexicographically sorted")
        self.assertEqual(len(result), len(set(result)), "Result should not contain duplicates")

    def test_includes_primitives_from_plan_verbs(self):
        primitives = {line.strip().lower() for line in (PLAN_VERBS or '').splitlines() if line.strip()}
        logger = logging.getLogger('test_build_allowed_verbs')
        result = set(build_allowed_verbs(MIDDLE_ONTOLOGY, logger))
        missing = primitives - result
        self.assertFalse(missing, f"Missing primitives from allowed verbs: {sorted(missing)}")

    def test_includes_verbs_with_primitive_hint_anchor_if_any(self):
        verbs = (MIDDLE_ONTOLOGY or {}).get('verbs', {})
        nodes = verbs.get('nodes', []) or []
        primitives = {line.strip().lower() for line in (PLAN_VERBS or '').splitlines() if line.strip()}
        anchored = [
            v.get('id')
            for v in nodes
            if isinstance(v, dict) and v.get('id') and str(v.get('hint_anchor', '')).lower() in primitives
        ]
        if not anchored:
            self.skipTest("No verbs with primitive hint_anchor present in provided middle ontology")
        logger = logging.getLogger('test_build_allowed_verbs')
        result = set(build_allowed_verbs(MIDDLE_ONTOLOGY, logger))
        # For verbs that differ only by case from primitives, ensure primitive is present and cased variant excluded
        for vid in anchored:
            lc = str(vid).lower()
            if lc in primitives and vid != lc:
                self.assertIn(lc, result, f"Primitive '{lc}' should be included for anchored verb '{vid}'")
                self.assertNotIn(vid, result, f"Cased variant '{vid}' should be excluded in favor of primitive '{lc}'")
            else:
                self.assertIn(vid, result, f"Anchored verb '{vid}' should be included")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Standalone unit test for build_allowed_verbs')
    parser.add_argument('-f', '--file', required=True, help='Path to middle ontology JSON file')
    args, remaining = parser.parse_known_args()

    with open(args.file, 'r') as f:
        MIDDLE_ONTOLOGY = json.load(f)

    # Clean up argv so unittest does not see our custom args
    sys.argv = [sys.argv[0]] + remaining

    # Configure a basic logger for the test run
    logging.basicConfig(level=logging.INFO, format='%(levelname)s:%(name)s:%(message)s')

    unittest.main()


