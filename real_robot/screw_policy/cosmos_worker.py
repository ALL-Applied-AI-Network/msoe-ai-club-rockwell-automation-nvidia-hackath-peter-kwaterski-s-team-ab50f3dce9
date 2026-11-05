"""Persistent Cosmos inference worker (separate Python interpreter).

Run: REAL_ROBOT_COSMOS_PYTHON=... python -m screw_policy.cosmos_worker
Protocol: one JSON line per request {"id": int, "path": "/tmp/img.png"} or
{"id": int, "rgb_b64": "..."}; response {"id", "ok", "parsed"|"error"}.
"""

from __future__ import annotations

import base64
import json
import os
import sys
import tempfile
from pathlib import Path


def _find_nano():
    selected = os.environ.get("COSMOS3_NANO_PATH")
    if selected:
        checkpoint = Path(selected).expanduser()
        if not (checkpoint / "config.json").is_file():
            raise SystemExit(f"Model snapshot missing config.json: {checkpoint}")
        return checkpoint
    cache_home = Path(
        os.environ.get(
            "HF_HOME",
            str(
                Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache")))
                / "huggingface"
            ),
        )
    ).expanduser()
    hub = Path(os.environ.get("HF_HUB_CACHE", str(cache_home / "hub"))).expanduser()
    base = hub / "models--nvidia--Cosmos3-Nano"
    snapshots = sorted((base / "snapshots").glob("*/config.json"))
    if not snapshots:
        raise SystemExit(
            f"Cosmos3-Nano not found under {base}. Set COSMOS3_NANO_PATH."
        )
    return snapshots[-1].parent


def _configure_cosmos_imports():
    checkout = os.environ.get("COSMOS3_ROOT")
    if checkout:
        root = Path(checkout).expanduser().resolve()
        package = root / "packages" / "transformers-cosmos3"
        if (package / "transformers_cosmos3").is_dir():
            sys.path.insert(0, str(package))
            sys.path.insert(0, str(root))


def load_model(gpu: int = 0):
    _configure_cosmos_imports()
    import torch
    from transformers import AutoConfig
    from transformers_cosmos3 import Cosmos3ForConditionalGeneration

    path = _find_nano()
    config = AutoConfig.from_pretrained(str(path), trust_remote_code=True)
    model = Cosmos3ForConditionalGeneration.from_pretrained(
        str(path), config=config, torch_dtype=torch.bfloat16, trust_remote_code=True
    )
    device = f"cuda:{gpu}" if torch.cuda.is_available() else "cpu"
    model = model.to(device).eval()
    from transformers import AutoProcessor

    processor = AutoProcessor.from_pretrained(str(path), trust_remote_code=True)
    return model, processor, device


def ask(model, processor, device, image_path: Path, prompt: str, max_new_tokens: int = 2500):
    import torch
    from PIL import Image

    img = Image.open(image_path).convert("RGB")
    messages = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": prompt}]}]
    text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    inputs = processor(text=[text], images=[img], return_tensors="pt").to(device)
    with torch.inference_mode():
        out = model.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)
    reply = processor.decode(out[0, inputs["input_ids"].shape[1] :], skip_special_tokens=True)
    return reply


def main():
    from screw_policy.json_utils import canonicalise_detection, parse_json
    from screw_policy.perception_prompt import SCREW_BIN_PROMPT

    gpu = int(os.environ.get("REAL_ROBOT_COSMOS_GPU", "0"))
    model, processor, device = load_model(gpu)
    sys.stderr.write("[cosmos_worker] model ready\n")
    sys.stderr.flush()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        req = json.loads(line)
        rid = req.get("id", 0)
        try:
            if "path" in req:
                path = Path(req["path"])
            elif "rgb_b64" in req:
                data = base64.b64decode(req["rgb_b64"])
                fd, name = tempfile.mkstemp(suffix=".png")
                os.close(fd)
                path = Path(name)
                path.write_bytes(data)
            else:
                raise ValueError("need path or rgb_b64")
            raw = ask(model, processor, device, path, SCREW_BIN_PROMPT)
            parsed = canonicalise_detection(parse_json(raw) or {})
            print(json.dumps({"id": rid, "ok": True, "parsed": parsed, "raw": raw[:2000]}), flush=True)
        except Exception as e:  # noqa: BLE001
            print(json.dumps({"id": rid, "ok": False, "error": str(e)}), flush=True)


if __name__ == "__main__":
    main()
