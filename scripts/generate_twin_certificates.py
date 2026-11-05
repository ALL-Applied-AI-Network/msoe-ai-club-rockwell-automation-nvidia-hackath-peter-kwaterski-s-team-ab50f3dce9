"""Generate short-lived development certificates for the twin's mutual TLS demo.

Uses the local OpenSSL executable; creates a new private directory and never
overwrites an existing certificate set. No Python packages are required.
"""

import argparse
import ipaddress
from pathlib import Path
import re
import secrets
import shutil
import subprocess
import tempfile


def subject_alt_name(host):
    try:
        return f"IP:{ipaddress.ip_address(host)}"
    except ValueError:
        labels = host.split(".")
        if len(host) > 253 or any(
            not re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?", label)
            for label in labels
        ):
            raise ValueError(f"Invalid server DNS name or IP address: {host!r}")
        return f"DNS:{host}"


def generate(output, hosts=(), days=7):
    if not 1 <= days <= 30:
        raise ValueError("Development certificate lifetime must be between 1 and 30 days")
    names = list(dict.fromkeys(subject_alt_name(host) for host in ("localhost", "127.0.0.1", *hosts)))
    openssl = shutil.which("openssl")
    if not openssl:
        raise ValueError("OpenSSL is required; install it before generating development certificates")
    output = Path(output).expanduser().absolute()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(mode=0o700)  # Refuse to replace any existing directory or symlink.

    def run(*args):
        subprocess.run([openssl, *map(str, args)], check=True, capture_output=True, text=True)

    def request(name, destination, *extra):
        key = output / f"{name}.key"
        # Pre-create with private permissions, independent of the caller's umask.
        key.touch(mode=0o600, exist_ok=False)
        run("req", "-new", "-newkey", "ec", "-pkeyopt", "ec_paramgen_curve:P-256",
            "-nodes", "-sha256", "-subj", f"/CN=Twin demo {name}",
            "-keyout", key, "-out", destination, *extra)
        key.chmod(0o600)

    request("ca", output / "ca.crt", "-x509", "-days", days,
            "-addext", "basicConstraints=critical,CA:TRUE,pathlen:0",
            "-addext", "keyUsage=critical,keyCertSign,cRLSign",
            "-addext", "subjectKeyIdentifier=hash")
    with tempfile.TemporaryDirectory(dir=output) as scratch:
        scratch = Path(scratch)
        for name, purpose in (("server", "serverAuth"), ("client", "clientAuth")):
            csr = scratch / f"{name}.csr"
            request(name, csr)
            extensions = scratch / f"{name}.ext"
            content = ("basicConstraints=critical,CA:FALSE\n"
                       "keyUsage=critical,digitalSignature\n"
                       f"extendedKeyUsage={purpose}\n"
                       "subjectKeyIdentifier=hash\n"
                       "authorityKeyIdentifier=keyid,issuer\n")
            if name == "server":
                content += f"subjectAltName={','.join(names)}\n"
            extensions.write_text(content, encoding="ascii")
            run("x509", "-req", "-in", csr, "-CA", output / "ca.crt",
                "-CAkey", output / "ca.key", "-set_serial", secrets.randbits(128) or 1,
                "-days", days, "-sha256", "-extfile", extensions, "-out", output / f"{name}.crt")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("certs/twin-demo"))
    parser.add_argument("--server-host", action="append", default=[],
                        help="Additional sender DNS name or IP address; repeat as needed")
    parser.add_argument("--days", type=int, default=7)
    args = parser.parse_args(argv)
    try:
        output = generate(args.out, args.server_host, args.days)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.error(str(exc))
    print(f"Development certificates created in {output} (valid {args.days} days).")
    print("Sender needs ca.crt, server.crt, server.key; viewer needs ca.crt, client.crt, client.key.")
    print("Keep ca.key offline. Never distribute it with either application.")


if __name__ == "__main__":
    main()
