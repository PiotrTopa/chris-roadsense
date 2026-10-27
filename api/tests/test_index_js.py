"""The status page's inline script must not contain a raw newline inside a quoted JS string: a "\\n" written without
the double backslash in the non-raw Python template becomes a line break and the whole script fails to parse (every
box on the page then stays at "..."; seen in L7 and L11)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from roadsense_api.server import INDEX_HTML  # noqa: E402


def newline_in_js_string(html):
    """Lines of the page's <script> that end inside a '...' or "..." literal (template literals may span lines)."""
    js = html.split("<script>")[1].split("</script>")[0]
    bad, state, esc = [], None, False
    for n, line in enumerate(js.split("\n"), 1):
        for ch in line:
            if esc:
                esc = False
                continue
            if ch == "\\" and state:
                esc = True
                continue
            if state is None and ch in "\"'`":
                state = ch
            elif state == ch:
                state = None
        if state in ("\"", "'"):
            bad.append((n, line.strip()))
            state = None
    return bad


class IndexJsTest(unittest.TestCase):
    def test_no_raw_newline_inside_js_strings(self):
        self.assertEqual(newline_in_js_string(INDEX_HTML), [])

    def test_detects_the_bug(self):
        self.assertEqual(newline_in_js_string('<script>\nx = [1].join("\n");\n</script>')[0], (2, 'x = [1].join("'))

    def test_balanced_brackets(self):
        js = INDEX_HTML.split("<script>")[1].split("</script>")[0]
        for a, b in ("()", "[]", "{}"):
            self.assertEqual(js.count(a), js.count(b), a + b)
        self.assertEqual(js.count("`") % 2, 0)


if __name__ == "__main__":
    unittest.main()
