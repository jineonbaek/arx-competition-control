"""Static entry contract; browser rendering is an explicit development check."""
from html.parser import HTMLParser
from pathlib import Path
import unittest

PUBLIC = Path(__file__).resolve().parents[1] / "public"


class Page(HTMLParser):
    def __init__(self, source):
        super().__init__()
        self.ids = {}
        self.scripts = []
        self.options = []
        self.forms = 0
        self.feed(source)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if "id" in attrs:
            self.ids[attrs["id"]] = attrs
        if tag == "script":
            self.scripts.append(attrs.get("src"))
        if tag == "option":
            self.options.append(attrs.get("value"))
        if tag == "form":
            self.forms += 1


class HandeyePageTests(unittest.TestCase):
    def test_entry_is_inside_d405_card_only(self):
        source = (PUBLIC / "index.html").read_text()
        start = source.index('data-camera="d405"')
        end = source.index("</article>", start)
        self.assertIn('href="handeye.html"', source[start:end])
        self.assertEqual(source.count('href="handeye.html"'), 1)
        self.assertTrue((PUBLIC / "handeye.html").is_file())

    def test_initial_state_cannot_submit_and_does_not_guess_a_side(self):
        page = Page((PUBLIC / "handeye.html").read_text())
        self.assertIn("disabled", page.ids["handeye-start"])
        self.assertEqual(page.ids["handeye-start"]["type"], "button")
        self.assertEqual(page.options, ["", "left", "right"])
        self.assertEqual(page.forms, 0)

    def test_entry_loads_only_its_read_only_script_and_local_styles(self):
        page = Page((PUBLIC / "handeye.html").read_text())
        self.assertEqual(page.scripts, ["js/handeye.js"])
        self.assertTrue((PUBLIC / page.scripts[0]).is_file())
        self.assertTrue((PUBLIC / "css/handeye.css").is_file())


if __name__ == "__main__":
    unittest.main()
