# Encrypted camera and telemetry transmission

The existing twin stream can now use HTTPS with **mutual TLS 1.3**. The sender
requires a viewer certificate issued by its configured CA; the viewer verifies
the sender's certificate chain, expiry and DNS/IP identity. Both PNG images and
JSON telemetry travel over the authenticated, encrypted connection. Public-key
cryptography authenticates the endpoints and establishes session keys; TLS uses
authenticated symmetric encryption for the image and status bytes.

```text
Isaac camera → local live_frame.png → twin_stream_server → TLS → Tkinter computer
                                     simulated sender          verifies/decrypts
```

The image polling rate, camera, perception, planning and motion behavior are
unchanged. The source is still labelled DIGITAL TWIN. This demonstrates real
encryption of simulated data; it does not establish physical-camera security.

## 1. Generate development certificates

From the repository root, on a trusted setup machine with OpenSSL installed:

```bash
fanuc_er4ia_sim/.venv/bin/python scripts/generate_twin_certificates.py \
  --out certs/twin-demo --server-host your-gpu-host
```

Replace `your-gpu-host` with the DNS name or IP that the viewing computer will
actually use. Repeat `--server-host` to include additional names or addresses.
`localhost` and `127.0.0.1` are always included, allowing a same-machine test or
HTTPS through an SSH tunnel. An IP URL must match an IP subjectAltName, even if
it resolves to the same computer as a DNS name in the certificate.

The helper creates an entirely new directory, refuses to overwrite an existing
one, and creates private keys with owner-only permissions on POSIX systems.
Certificates expire after seven days (`--days` accepts 1–30). Regenerate into a
new directory and redeploy the necessary files when they expire. No package
installation is needed beyond Python's SSL support and the OpenSSL executable.

Distribute only these files, using a trusted channel such as SSH/SCP or a secure
USB transfer:

| Location | Files |
| --- | --- |
| Sender (GPU host; future robot-side computer) | `ca.crt`, `server.crt`, `server.key` |
| Viewing computer | `ca.crt`, `client.crt`, `client.key` |
| Offline setup machine | `ca.key` |

The CA private key can authorize new devices. Keep it offline after provisioning;
neither runtime application needs it. Do not copy the entire generated directory
to the viewer. On Windows, restrict private-key access with file ACLs. The demo
keys are unencrypted on disk for unattended startup; Git ignores `certs/`,
`*.key`, `*.p12` and `*.pfx`. Runtime TLS uses only the supplied CA trust store.
Any valid client certificate from that CA with client-authentication usage is
authorized; this is not a separate per-user permission system. Use a dedicated
demo CA. The application does not implement certificate revocation checking;
short lifetimes and replacing the configured trust/certificates are the demo's
credential replacement mechanism.

## 2. Start the sender

From the repository root, for an existing run folder:

```bash
fanuc_er4ia_sim/.venv/bin/python fanuc_er4ia_sim/twin_stream_server.py \
  --run fanuc_er4ia_sim/runs/stack_step_01 \
  --host 0.0.0.0 --port 8765 \
  --tls-ca certs/twin-demo/ca.crt \
  --tls-cert certs/twin-demo/server.crt \
  --tls-key certs/twin-demo/server.key
```

Choose a run directory that exists. Live images appear when the simulator writes
`live_frame.png`; a run with no frame yet can still provide status. No fake
camera data is created by enabling TLS. `--host 0.0.0.0` enables IPv4 network
listening; use the sender's actual certified hostname/IP in the viewer URL.
Network routing/firewall access to the selected port must already exist.
The application does not modify firewall rules or cloud hosting.

To enable TLS in the existing stepwise pipeline, set these variables from the
repository root, then launch as usual:

```bash
export TWIN_TLS_CA="$PWD/certs/twin-demo/ca.crt"
export TWIN_TLS_CERT="$PWD/certs/twin-demo/server.crt"
export TWIN_TLS_KEY="$PWD/certs/twin-demo/server.key"
export TWIN_STREAM_HOST=0.0.0.0
export TWIN_STREAM_PORT=8765
cd fanuc_er4ia_sim
bash run_step_pipeline.sh runs/stack_step_secure_01
```

Keep the existing Isaac/Cosmos environment configuration. The launcher validates
certificates and the port binding before starting capture or motion. The stream
server's lifetime is the same as before: it stops when the launcher exits. Start
the standalone server above to review a saved run afterward.

## 3. Connect the desktop dashboard

On a desktop with Tkinter and a checkout of this repository, place the viewer's
three provisioned files in `certs/twin-viewer/`, then run:

```bash
python3 TkinterView.py --url https://your-gpu-host:8765 \
  --tls-ca certs/twin-viewer/ca.crt \
  --tls-cert certs/twin-viewer/client.crt \
  --tls-key certs/twin-viewer/client.key
```

Use the same certified DNS/IP identity as in step 1. The dashboard reports
authentication after successful reads and reports unavailable/rejected or delayed
connections. On an error, a retained image is an earlier snapshot. It never
switches to plaintext, disables certificate verification or follows redirects.
HTTPS connects directly to the configured sender, ignoring environment proxy
settings. Missing or untrusted client certificates are rejected before any HTTP
endpoint, including `/health`, is available.

For HTTPS through SSH, forward the port as in the existing README and use
`https://127.0.0.1:8765` with the same three viewer TLS options. The sender can
then remain bound to its default `127.0.0.1`.

## Compatibility and failure behavior

- Without TLS options, the sender and viewer permit HTTP only on loopback for
  the existing SSH workflow. The dashboard cannot verify that a tunnel exists.
- Direct non-loopback HTTP is refused. All three TLS options are required
  together; supplying them to an `http://` URL is an error.
- TLS 1.2 and older, expired certificates, incorrect certificate purposes,
  untrusted issuers and hostname mismatches are rejected. Fix the certificates
  or URL; do not bypass verification.
- Local-file and fictional-demo dashboard modes need no certificates.
- When a run finishes, the sender exposes ``GET /bundle.zip`` (review artifacts as
  a zip, excluding raw ``frames/`` and ``side_frames/``) only after
  ``sort_table.mp4``, ``sort_side.mp4``, and ``run_finalized.json`` are present
  (written at the end of ``run_step_pipeline.sh`` / ``run_sort_pipeline.sh``).
  A remote dashboard connected with ``--url`` downloads it automatically into
  the viewer's Downloads folder once that gate passes. After
  ``run_step_pipeline.sh`` encodes review videos it pre-builds the zip and keeps
  the twin stream alive for ``TWIN_STREAM_POST_RUN_SEC`` seconds (default 900)
  so the laptop can finish downloading before the sender stops.
- Disk images, JSON artifacts and the dashboard's existing temporary display
  image are outside transport encryption. A camera-to-sender cable is also
  outside this protected network segment.

## Validation and presentation

Host tests, using real TLS sockets and temporary certificates:

```bash
cd fanuc_er4ia_sim
.venv/bin/python -m unittest test_twin_tls test_twin_stream_server
cd ..
fanuc_er4ia_sim/.venv/bin/python -m unittest test_tkinter_view
```

The TLS tests check image bytes after decryption, encrypted bytes observed by a
TCP relay without keys, all protected endpoints, certificate/hostname failures,
plaintext refusal, redirect refusal and a stalled unauthenticated connection.
They also test the dashboard's connection indicator without opening Tk.

A local smoke measurement using a copied, existing 563,651-byte simulator PNG
and 30 reads per mode averaged **1.56 ms over loopback HTTP** and **5.75 ms over
mutual TLS 1.3**. Each read included both status and the full image, with the
current separate HTTP connections. These measurements exclude rendering,
inference, Tk display time and a real network; they do not establish hardware
or remote-display performance.

For a live presentation, first show an authorized viewer receiving an image,
then try a certificate from a separately generated CA to show rejection. Keep
the authorized window visible so a rejected viewer is not mistaken for a stopped
simulation. A network capture on the sender-to-viewer segment can show TLS
records; image contents should not be readable. Do not enable TLS session-key
logging or provide decryption keys to the capture tool. Authentication occurs
per connection, while this dashboard currently makes separate status and image
requests. Measure actual latency on the target hardware before claiming a frame
rate or real-time performance.

The Sim2Real Gap includes actual camera drivers, selecting the physical sender,
device key provisioning/storage, credential replacement and measured bandwidth,
CPU load and latency. If a camera sends plaintext to a gateway, encryption begins
at that gateway; it does not retroactively protect the camera-to-gateway link.
