# hvsd-3ch-software
Firmware for the 3ch STM32G473 based high voltage servo drive

## Python Environment Setup

Use this when working with scripts in `python/` (for example `merge_hex.py`).

### In VS Code

Run the task:

- `Create Python venv and install dependencies`

This task creates `python/.venv` if it does not already exist, then installs modules from `python/requirements.txt`.

### Manual Command (without VS Code)

From the repository root:

Windows (cmd):

```bat
if not exist python\.venv python -m venv python\.venv
python\.venv\Scripts\python -m pip install -r python\requirements.txt
```

Linux/macOS (bash):

```bash
python3 -m venv python/.venv
python/.venv/bin/python -m pip install -r python/requirements.txt
```

### Update AURA
'''bash
python/.venv/bin/python -m pip install --force-reinstall git+https://github.com/ExcessiveOverkill/AURA.git
'''