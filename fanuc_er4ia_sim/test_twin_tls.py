"""Real-socket mutual TLS checks; no Isaac, Cosmos or Tk display required."""

import base64
from contextlib import contextmanager
import http.client
import json
from pathlib import Path
import select
import shutil
import socket
import socketserver
import ssl
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from twin_stream_server import create_server
from TkinterView import RemoteStatusSource, SortingView
from scripts.generate_twin_certificates import generate
from twin_tls import secure_opener, tls_context


PNG = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aXioAAAAASUVORK5CYII="
)


@contextmanager
def running(server):
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


class RecordingRelay(socketserver.BaseRequestHandler):
    """Observe the bytes crossing the network without access to TLS keys."""

    def handle(self):
        with socket.create_connection(self.server.target, timeout=3) as upstream:
            sockets = (self.request, upstream)
            while True:
                readable, _, _ = select.select(sockets, [], [], 3)
                if not readable:
                    return
                for source in readable:
                    data = source.recv(65536)
                    if not data:
                        return
                    self.server.records.append(data)
                    destination = upstream if source is self.request else self.request
                    destination.sendall(data)


@unittest.skipUnless(shutil.which("openssl"), "OpenSSL CLI is required for test certificates")
class MutualTLSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        cls.root = Path(cls.temp.name)
        cls.trusted = generate(cls.root / "trusted")
        cls.rogue = generate(cls.root / "rogue")
        # Issue a client certificate with an already expired validity period.
        subprocess.run([
            shutil.which("openssl"), "req", "-new", "-key", str(cls.trusted / "client.key"),
            "-subj", "/CN=Expired demo viewer", "-out", str(cls.trusted / "expired.csr"),
        ], check=True, capture_output=True)
        (cls.trusted / "expired.ext").write_text("extendedKeyUsage=clientAuth\n")
        subprocess.run([
            shutil.which("openssl"), "x509", "-req", "-in", str(cls.trusted / "expired.csr"),
            "-CA", str(cls.trusted / "ca.crt"), "-CAkey", str(cls.trusted / "ca.key"),
            "-extfile", str(cls.trusted / "expired.ext"),
            "-set_serial", "123", "-days", "-1", "-out", str(cls.trusted / "expired.crt"),
        ], check=True, capture_output=True)

    def setUp(self):
        self.run = self.root / self._testMethodName
        self.run.mkdir()
        (self.run / "detections.json").write_text(json.dumps({"objects": [{"category": "cube"}]}))
        (self.run / "step_state.json").write_text(json.dumps({"phase": "executing"}))
        (self.run / "live_frame.png").write_bytes(PNG)
        self.server = self.enterContext(running(create_server(self.run, "127.0.0.1", 0, **self.server_keys())))
        self.address = self.server.server_address
        self.url = f"https://127.0.0.1:{self.address[1]}"

    def server_keys(self):
        return dict(ca=self.trusted / "ca.crt", cert=self.trusted / "server.crt", key=self.trusted / "server.key")

    def client_keys(self):
        return dict(ca=self.trusted / "ca.crt", cert=self.trusted / "client.crt", key=self.trusted / "client.key")

    def test_authenticated_status_image_and_all_endpoints(self):
        source = RemoteStatusSource(self.url, **self.client_keys())
        status = source.read_status()
        self.assertTrue(source.secure)
        self.assertTrue(status.simulated)
        self.assertEqual(status.counts["Cubes"]["detected"], 1)
        self.assertEqual(status.frame_image, PNG)
        with source._open("/step_state") as response:
            self.assertEqual(json.load(response)["phase"], "executing")
        with source._open("/health") as response:
            self.assertTrue(json.load(response)["ok"])

    def test_negotiates_tls_13_with_authenticated_encryption(self):
        context = tls_context(server=False, **self.client_keys())
        with socket.create_connection(self.address, timeout=2) as connection:
            with context.wrap_socket(connection, server_hostname="localhost") as encrypted:
                self.assertEqual(encrypted.version(), "TLSv1.3")
                self.assertIn(encrypted.cipher()[0], (
                    "TLS_AES_256_GCM_SHA384", "TLS_AES_128_GCM_SHA256", "TLS_CHACHA20_POLY1305_SHA256"))
                self.assertTrue(encrypted.getpeercert())

    def test_missing_client_certificate_cannot_read_any_endpoint(self):
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.load_verify_locations(str(self.trusted / "ca.crt"))
        opener = secure_opener(context)
        for endpoint in ("/health", "/status", "/step_state", "/frame"):
            with self.subTest(endpoint=endpoint):
                with self.assertRaises((OSError, http.client.HTTPException)):
                    opener.open(self.url + endpoint, timeout=2)

    def test_untrusted_client_is_rejected(self):
        keys = self.client_keys() | dict(cert=self.rogue / "client.crt", key=self.rogue / "client.key")
        with self.assertRaises((OSError, http.client.HTTPException)):
            RemoteStatusSource(self.url, **keys).read_status()

    def test_expired_client_is_rejected(self):
        keys = self.client_keys() | dict(cert=self.trusted / "expired.crt")
        with self.assertRaises((OSError, http.client.HTTPException)):
            RemoteStatusSource(self.url, **keys).read_status()

    def test_untrusted_server_is_rejected(self):
        keys = self.client_keys() | dict(ca=self.rogue / "ca.crt")
        with self.assertRaisesRegex(ConnectionError, "CERTIFICATE_VERIFY_FAILED"):
            RemoteStatusSource(self.url, **keys).read_status()

    def test_server_hostname_must_match_certificate(self):
        getaddrinfo = socket.getaddrinfo

        def resolve(host, *args, **kwargs):
            return getaddrinfo("127.0.0.1" if host == "wrong.invalid" else host, *args, **kwargs)

        with patch("socket.getaddrinfo", side_effect=resolve):
            source = RemoteStatusSource(self.url.replace("127.0.0.1", "wrong.invalid"), **self.client_keys())
            with self.assertRaisesRegex(ConnectionError, "CERTIFICATE_VERIFY_FAILED"):
                source.read_status()

    def test_server_only_certificate_is_rejected_as_client_identity(self):
        with self.assertRaises((OSError, http.client.HTTPException)):
            RemoteStatusSource(self.url, **self.server_keys()).read_status()

    def test_plaintext_is_rejected_on_secure_port(self):
        with socket.create_connection(self.address, timeout=2) as connection:
            connection.sendall(b"GET /frame HTTP/1.0\r\nHost: localhost\r\n\r\n")
            try:
                response = connection.recv(4096)
            except ConnectionResetError:
                response = b""
        self.assertNotIn(b"HTTP/", response)
        self.assertNotIn(PNG, response)

    def test_tls_12_is_rejected(self):
        context = tls_context(server=False, **self.client_keys())
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.maximum_version = ssl.TLSVersion.TLSv1_2
        with socket.create_connection(self.address, timeout=2) as connection:
            with self.assertRaises(ssl.SSLError):
                context.wrap_socket(connection, server_hostname="localhost")

    def test_invalid_server_configuration_fails_before_binding(self):
        for keys in (dict(ca=self.trusted / "ca.crt"),
                     self.server_keys() | dict(key=self.trusted / "client.key"),
                     self.server_keys() | dict(ca=self.trusted / "missing.crt")):
            with self.subTest(keys=keys):
                with self.assertRaises((ValueError, OSError)):
                    create_server(self.run, "127.0.0.1", 0, **keys)

    def test_remote_plaintext_binding_is_refused(self):
        with self.assertRaisesRegex(ValueError, "Non-loopback access requires mutual TLS"):
            create_server(self.run, "0.0.0.0", 0)

    def test_image_is_ciphertext_on_network_and_decodes_identically(self):
        relay = socketserver.ThreadingTCPServer(("127.0.0.1", 0), RecordingRelay)
        relay.daemon_threads = True
        relay.target = self.address
        relay.records = []
        with running(relay):
            source = RemoteStatusSource(f"https://127.0.0.1:{relay.server_address[1]}", **self.client_keys())
            status = source.read_status()
            self.assertEqual(status.frame_image, PNG)
        captured = b"".join(relay.records)
        self.assertGreater(len(captured), len(PNG))
        for plaintext in (PNG, b"GET /frame", b'"controller_phase"', b"Content-Type: image/png"):
            self.assertNotIn(plaintext, captured)

    def test_redirects_including_plaintext_downgrade_are_rejected(self):
        handler = self.server.RequestHandlerClass
        original = handler.do_GET
        for location in ("http://127.0.0.1:1/frame", "https://other.invalid/frame", self.url + "/frame"):
            def redirect(request):
                request.send_response(302)
                request.send_header("Location", location)
                request.end_headers()

            with self.subTest(location=location), patch.object(handler, "do_GET", redirect):
                with self.assertRaisesRegex(ConnectionError, "redirects are disabled"):
                    RemoteStatusSource(self.url, **self.client_keys()).read_status()
        self.assertIs(handler.do_GET, original)

    def test_frame_certificate_failure_is_visible_instead_of_silently_ignored(self):
        source = RemoteStatusSource(self.url, **self.client_keys())
        open_original = source._open

        def fail_frame(path):
            if path == "/frame":
                from urllib.error import URLError
                raise URLError(ssl.SSLCertVerificationError("frame certificate rejected"))
            return open_original(path)

        with patch.object(source, "_open", side_effect=fail_frame):
            with self.assertRaisesRegex(ConnectionError, "Secure frame connection failed"):
                source.read_status()

    def test_missing_frame_during_initialization_still_allows_authenticated_status(self):
        (self.run / "live_frame.png").unlink()
        status = RemoteStatusSource(self.url, **self.client_keys()).read_status()
        self.assertIsNone(status.frame_image)
        self.assertEqual(status.counts["Cubes"]["detected"], 1)

    def test_unauthenticated_stalled_handshake_does_not_block_viewer(self):
        with socket.create_connection(self.address, timeout=2):
            status = RemoteStatusSource(self.url, **self.client_keys()).read_status()
            self.assertEqual(status.frame_image, PNG)

    def test_certificate_helper_does_not_overwrite_and_protects_keys(self):
        with self.assertRaises(FileExistsError):
            generate(self.trusted)
        for name in ("ca", "server", "client"):
            self.assertEqual((self.trusted / f"{name}.key").stat().st_mode & 0o777, 0o600)
        self.assertEqual(self.trusted.stat().st_mode & 0o777, 0o700)

    def test_secure_connection_indicator_tracks_errors_and_stale_reads(self):
        view = SortingView.__new__(SortingView)
        view.source = RemoteStatusSource(self.url, **self.client_keys())
        view.connection_text = Mock()
        view.error = None
        view.last_received = None
        view.update_connection_status()
        self.assertIn("waiting", view.connection_text.set.call_args.args[0])
        view.last_received = time.monotonic()
        view.update_connection_status()
        self.assertIn("client certificate accepted", view.connection_text.set.call_args.args[0])
        view.last_received -= 10
        view.update_connection_status()
        self.assertIn("not currently confirmed", view.connection_text.set.call_args.args[0])
        view.error = "certificate rejected"
        view.update_connection_status()
        self.assertIn("unavailable or rejected", view.connection_text.set.call_args.args[0])


class TLSConfigurationTests(unittest.TestCase):
    def test_viewer_rejects_incomplete_credentials_and_unsafe_urls(self):
        cases = (
            ("https://localhost:8765", {}),
            ("https://localhost:8765", {"ca": "ca.crt"}),
            ("http://localhost:8765", {"ca": "ca.crt"}),
            ("http://robot.local:8765", {}),
            ("http://192.168.1.10:8765", {}),
            ("file:///tmp/frame.png", {}),
            ("http://user:password@localhost:8765", {}),
            ("http://localhost:8765/path", {}),
        )
        for url, options in cases:
            with self.subTest(url=url, options=options), self.assertRaises(ValueError):
                RemoteStatusSource(url, **options)

    def test_existing_loopback_ssh_workflow_is_supported(self):
        source = RemoteStatusSource("http://127.0.0.1:8765")
        self.assertFalse(source.secure)

    def test_certificate_helper_rejects_extension_injection_before_creating_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "certificates"
            with self.assertRaises(ValueError):
                generate(output, hosts=["robot\nextendedKeyUsage=clientAuth"])
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
