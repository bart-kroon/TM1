#!/usr/bin/env python3

import datetime
from pathlib import Path
import shutil
import subprocess


def main():
    workingDir = Path(__file__).parent
    shutil.copy(workingDir.parent.parent.parent / "requirements.txt", workingDir)
    run(["docker", "build", "-t", tag(), "."], workingDir)


def tag():
    now = datetime.datetime.now()
    date = now.strftime("%Y%m%d")
    return f"tmiv-checks:{date}"


def run(args: list, workingDir: Path):
    print(f"> {' '.join(map(str, args))}")
    subprocess.run(args, cwd=workingDir, check=True)


if __name__ == "__main__":
    main()
