import re
import unittest
from pathlib import Path

_IMPORT_FANUC = re.compile(
    r"^\s*(?:from|import)\s+fanuc_er4ia_sim", re.MULTILINE
)
_IMPORT_ISAAC = re.compile(r"^\s*(?:from|import)\s+isaacsim", re.MULTILINE)


class TestIsolation(unittest.TestCase):
    def _scan(self, pattern: re.Pattern[str]) -> list[Path]:
        root = Path(__file__).resolve().parent.parent
        hits = []
        for path in root.rglob("*.py"):
            if "tests" in path.parts and path.name.startswith("test_"):
                continue
            if pattern.search(path.read_text(encoding="utf-8")):
                hits.append(path)
        return hits

    def test_no_fanuc_imports(self):
        self.assertEqual(self._scan(_IMPORT_FANUC), [])

    def test_no_isaac_imports(self):
        self.assertEqual(self._scan(_IMPORT_ISAAC), [])


if __name__ == "__main__":
    unittest.main()
