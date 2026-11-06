"""Do not follow redirects carrying API authorization or robot requests."""
from urllib.request import HTTPRedirectHandler, build_opener


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def open_request(request, timeout):
    return build_opener(NoRedirect()).open(request, timeout=timeout)
