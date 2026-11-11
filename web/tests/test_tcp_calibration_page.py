from html.parser import HTMLParser
from pathlib import Path
import unittest

PUBLIC = Path(__file__).resolve().parents[1] / "public"


class Page(HTMLParser):
    def __init__(self, source):
        super().__init__()
        self.ids = {}
        self.scripts = []
        self.feed(source)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if "id" in attrs:
            self.ids[attrs["id"]] = attrs
        if tag == "script":
            self.scripts.append(attrs.get("src"))


class TcpCalibrationPageTests(unittest.TestCase):
    def test_console_links_to_dedicated_tcp_recording_page(self):
        source = (PUBLIC / "index.html").read_text(encoding="utf-8")
        self.assertIn('href="tcp-calibration.html"', source)
        self.assertEqual(source.count('href="tcp-calibration.html"'), 1)

    def test_page_exposes_record_solve_and_candidate_controls(self):
        page = Page((PUBLIC / "tcp-calibration.html").read_text(encoding="utf-8"))
        for element in (
            "tcp-arm-left", "tcp-arm-right", "tcp-live-status", "tcp-live-pose",
            "tcp-contact-confirm", "tcp-capture", "tcp-samples", "tcp-solve",
            "tcp-result", "tcp-save", "tcp-reset", "tcp-error",
        ):
            self.assertIn(element, page.ids)
        self.assertIn("disabled", page.ids["tcp-capture"])
        self.assertIn("disabled", page.ids["tcp-solve"])
        self.assertIn("disabled", page.ids["tcp-save"])
        self.assertEqual(page.scripts, ["js/tcp-calibration.js"])
        self.assertTrue((PUBLIC / "js/tcp-calibration.js").is_file())
        self.assertTrue((PUBLIC / "css/tcp-calibration.css").is_file())


if __name__ == "__main__":
    unittest.main()
