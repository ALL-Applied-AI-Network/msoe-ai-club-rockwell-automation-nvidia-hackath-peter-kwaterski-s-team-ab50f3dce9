"""TLS configuration shared by the camera/status sender and desktop viewer."""

import ipaddress
import ssl
from urllib.error import HTTPError
from urllib.request import HTTPRedirectHandler, HTTPSHandler, ProxyHandler, build_opener


def is_loopback(host):
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def tls_context(*, server, ca, cert, key):
    """Trust only the supplied CA; require peer authentication and TLS 1.3."""
    if not all((ca, cert, key)):
        raise ValueError("Mutual TLS requires --tls-ca, --tls-cert and --tls-key together")
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER if server else ssl.PROTOCOL_TLS_CLIENT)
    context.minimum_version = ssl.TLSVersion.TLSv1_3
    context.verify_mode = ssl.CERT_REQUIRED
    context.load_verify_locations(cafile=str(ca))
    context.load_cert_chain(certfile=str(cert), keyfile=str(key))
    context.set_alpn_protocols(["http/1.1"])
    return context


class NoRedirects(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        # Never downgrade HTTPS or present a client certificate to another host.
        raise HTTPError(req.full_url, code, "Secure stream redirects are disabled", headers, fp)


def secure_opener(context):
    # Connect directly to the configured sender, independent of environment proxies.
    return build_opener(ProxyHandler({}), HTTPSHandler(context=context), NoRedirects())
