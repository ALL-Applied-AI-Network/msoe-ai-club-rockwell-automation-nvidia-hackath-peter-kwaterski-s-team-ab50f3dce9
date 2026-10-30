"""Read-only camera/status adapter with optional mutual TLS (required off loopback)."""

from __future__ import annotations

import argparse
import json
import socket
import ssl
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dashboard_status import read_dashboard_status, status_to_dict  # noqa: E402
from run_artifacts import read_json  # noqa: E402
from twin_tls import is_loopback, tls_context  # noqa: E402


class TwinStreamHandler(BaseHTTPRequestHandler):
    run_dir: Path = Path(".")

    def log_message(self, format, *args):  # noqa: A003
        return

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, payload: bytes, content_type: str):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):  # noqa: N802
        path = urlparse(self.path).path
        if path == "/health":
            self._send_json({"ok": True, "run": str(self.run_dir)})
            return
        if path == "/status":
            try:
                status = read_dashboard_status(self.run_dir)
                payload = status_to_dict(status)
                payload["frame_url"] = "/frame"
                self._send_json(payload)
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=500)
            return
        if path == "/step_state":
            state = read_json(self.run_dir / "step_state.json")
            if state is None:
                self._send_json({"error": "step_state.json not found"}, status=404)
                return
            self._send_json(state)
            return
        if path == "/frame":
            frame_path = self.run_dir / "live_frame.png"
            if not frame_path.is_file():
                self.send_response(404)
                self.end_headers()
                return
            self._send_bytes(frame_path.read_bytes(), "image/png")
            return
        self.send_response(404)
        self.end_headers()


class TwinStreamServer(ThreadingHTTPServer):
    """Perform bounded TLS handshakes in workers, without blocking new viewers."""

    def __init__(self, address, handler, *, context=None):
        self.context = context
        if ":" in address[0]:
            self.address_family = socket.AF_INET6
        super().__init__(address, handler)

    def finish_request(self, request, client_address):
        if self.context is None:
            return super().finish_request(request, client_address)
        request.settimeout(3.0)
        try:
            connection = self.context.wrap_socket(request, server_side=True)
        except (ssl.SSLError, OSError):
            # Missing/untrusted client certificates never reach the HTTP handler.
            return
        with connection:
            super().finish_request(connection, client_address)


def create_server(run_dir: Path, host: str, port: int, *, ca=None, cert=None, key=None):
    context = None
    if any((ca, cert, key)):
        context = tls_context(server=True, ca=ca, cert=cert, key=key)
    elif not is_loopback(host):
        raise ValueError("Non-loopback access requires mutual TLS; configure --tls-ca, --tls-cert and --tls-key")
    run_dir = Path(run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(f"Run directory not found: {run_dir}")
    handler = type("RunHandler", (TwinStreamHandler,), {"run_dir": run_dir})
    return TwinStreamServer((host, port), handler, context=context)


def serve(run_dir: Path, host: str, port: int, *, ca=None, cert=None, key=None) -> None:
    server = create_server(run_dir, host, port, ca=ca, cert=cert, key=key)
    scheme = "https" if server.context else "http"
    security = "mutual TLS 1.3" if server.context else "loopback only; use SSH for remote viewing"
    print(f"[twin-stream] serving {run_dir} at {scheme}://{host}:{port}/ ({security})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True, help="pipeline run directory")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--tls-ca", type=Path, help="CA certificate trusted to issue viewer certificates")
    parser.add_argument("--tls-cert", type=Path, help="Sender certificate with a matching DNS/IP subjectAltName")
    parser.add_argument("--tls-key", type=Path, help="Sender private key")
    parser.add_argument("--check-config", action="store_true", help="Validate certificates and binding, then exit")
    args = parser.parse_args(argv)
    options = dict(ca=args.tls_ca, cert=args.tls_cert, key=args.tls_key)
    try:
        if args.check_config:
            create_server(args.run, args.host, args.port, **options).server_close()
        else:
            serve(args.run, args.host, args.port, **options)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
