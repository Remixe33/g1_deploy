import argparse
import hashlib
from http.client import HTTPException
import json
import re
import shutil
import tarfile
import tempfile
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, build_opener


COMPONENTS = ("runtime", "unitree-sdk2", "cyclonedds", "onnxruntime", "realsense")
REQUIRED_FILES = {"BUNDLE_VERSION", "SOURCES.txt", *(f"{name}.tar.gz" for name in COMPONENTS)}


class HTTPSRedirectHandler(HTTPRedirectHandler):
    def redirect_request(self, request, response, code, message, headers, new_url):
        if urlsplit(new_url).scheme != "https":
            raise ValueError(f"Refusing non-HTTPS download redirect: {new_url}")
        return super().redirect_request(request, response, code, message, headers, new_url)


def sha256(path):
    with path.open("rb") as stream:
        checksum = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def verify_cache(cache, version):
    if (cache / "BUNDLE_VERSION").read_text().strip() != version:
        raise ValueError(f"Bundle version does not match {version}: {cache}")
    files = {path.relative_to(cache).as_posix(): path
             for path in cache.rglob("*") if path.is_file() and path != cache / "SHA256SUMS"}
    missing = REQUIRED_FILES - files.keys()
    if missing:
        raise ValueError(f"Missing bundle files: {', '.join(sorted(missing))}")
    if not (cache / "deb").is_dir() or not any((cache / "wheels").glob("*.whl")):
        raise ValueError("The bundle must have deb/ and a nonempty wheels/ directory.")
    checksums = {}
    for line in (cache / "SHA256SUMS").read_text().splitlines():
        digest, separator, filename = line.partition("  ")
        if not separator or filename in checksums or not re.fullmatch(r"[a-f0-9]{64}", digest):
            raise ValueError(f"Invalid SHA256SUMS entry: {line}")
        checksums[filename] = digest
    if files.keys() != checksums.keys():
        differences = sorted(files.keys() ^ checksums.keys())
        raise ValueError(f"Bundle contents do not match SHA256SUMS: {', '.join(differences)}")
    for filename, path in files.items():
        if path.is_symlink() or sha256(path) != checksums[filename]:
            raise ValueError(f"Checksum failed: {filename}")


def prepare(args, lock):
    digest = lock["sha256"]
    url = lock["url"]
    if not isinstance(digest, str) or not re.fullmatch(r"[a-f0-9]{64}", digest):
        raise ValueError("This source checkout has no published dependency checksum. "
                         "Use the matching source release, or see docs/ROBOT_SETUP.md to build a bundle.")
    if not args.archive and (not url or urlsplit(url).scheme != "https" or not urlsplit(url).hostname):
        raise ValueError("No HTTPS dependency URL is recorded for this source release.")
    args.destination.mkdir(parents=True, exist_ok=True)
    destination = args.destination / digest
    with tempfile.TemporaryDirectory(prefix=".bundle-", dir=args.destination) as temp:
        archive = args.archive or Path(temp) / "robot-deps.tar.gz"
        if not args.archive and not destination.is_dir():
            opener = build_opener(HTTPSRedirectHandler())
            for attempt in range(3):
                try:
                    with opener.open(url, timeout=30) as response, archive.open("wb") as output:
                        shutil.copyfileobj(response, output)
                    if sha256(archive) != digest:
                        raise ValueError("Downloaded dependency archive SHA-256 mismatch.")
                    break
                except (HTTPError, URLError, HTTPException, ConnectionError, TimeoutError, ValueError) as error:
                    if attempt == 2 or (isinstance(error, HTTPError) and 400 <= error.code < 500 and error.code != 429):
                        raise ValueError(f"Download failed: {error}. Use --offline FILE if the robot cannot access {url}.") from error
        if args.archive and sha256(archive) != digest:
            raise ValueError(f"Dependency archive SHA-256 mismatch: {archive}")
        if not destination.is_dir():
            stage = Path(temp) / "payload"
            stage.mkdir()
            with tarfile.open(archive, "r:gz") as payload:
                entries = payload.getmembers()
                seen = set()
                for entry in entries:
                    names = entry.name.split("/")
                    regular = entry.isfile() and (
                        (len(names) == 1 and names[0] in REQUIRED_FILES | {"SHA256SUMS"}) or
                        (len(names) == 2 and ((names[0] == "deb" and names[1].endswith(".deb")) or
                                              (names[0] == "wheels" and names[1].endswith(".whl")))))
                    directory = entry.isdir() and entry.name in {"deb", "wheels"}
                    if entry.name in seen or not (regular or directory):
                        raise ValueError(f"Unexpected archive entry: {entry.name}")
                    seen.add(entry.name)
                payload.extractall(stage, members=entries)
            verify_cache(stage, lock["version"])
            stage.rename(destination)
        else:
            verify_cache(destination, lock["version"])
    print(destination)


def pack(args, lock):
    url = urlsplit(args.url)
    if url.scheme != "https" or not url.hostname:
        raise ValueError("Specify the final HTTPS URL of the bundle.")
    (args.stage / "BUNDLE_VERSION").write_text(lock["version"] + "\n")
    manifest = args.stage / "SHA256SUMS"
    files = sorted(path for path in args.stage.rglob("*") if path.is_file() and path != manifest)
    manifest.write_text("".join(f"{sha256(path)}  {path.relative_to(args.stage).as_posix()}\n" for path in files))
    verify_cache(args.stage, lock["version"])
    args.destination.mkdir(parents=True, exist_ok=True)
    archive = args.destination / f"robot-deps-{lock['version']}.tar.gz"
    with tarfile.open(archive, "w:gz") as payload:
        for path in sorted(args.stage.rglob("*")):
            payload.add(path, arcname=path.relative_to(args.stage).as_posix(), recursive=False)
    lock["url"] = args.url
    lock["sha256"] = sha256(archive)
    (args.destination / args.lock.name).write_text(json.dumps(lock, indent=2) + "\n")
    print(archive)


def main():
    parser = argparse.ArgumentParser(description="Prepare or package a versioned G1 robot dependency bundle.")
    parser.add_argument("--lock", type=Path, required=True)
    modes = parser.add_subparsers(dest="mode", required=True)
    prepare_args = modes.add_parser("prepare")
    prepare_args.add_argument("destination", type=Path)
    prepare_args.add_argument("--archive", type=Path)
    cache_args = modes.add_parser("check-cache")
    cache_args.add_argument("cache", type=Path)
    pack_args = modes.add_parser("pack")
    pack_args.add_argument("stage", type=Path)
    pack_args.add_argument("destination", type=Path)
    pack_args.add_argument("url")
    args = parser.parse_args()
    try:
        lock = json.loads(args.lock.read_text())
        if args.mode == "prepare":
            prepare(args, lock)
        elif args.mode == "check-cache":
            verify_cache(args.cache, lock["version"])
        else:
            pack(args, lock)
    except (OSError, ValueError, KeyError, tarfile.TarError) as error:
        parser.exit(1, f"Robot dependencies: {error}\n")


if __name__ == "__main__":
    main()
