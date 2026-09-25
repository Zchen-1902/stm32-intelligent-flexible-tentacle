# -*- mode: python ; coding: utf-8 -*-

source_root = r"F:\act_vla\flexible_tentacle_act\pc_collector"

analysis = Analysis(
    [source_root + r"\main.py"],
    pathex=[source_root],
    binaries=[],
    datas=[],
    hiddenimports=["PySide6.QtSerialPort"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)

python_archive = PYZ(analysis.pure)

executable = EXE(
    python_archive,
    analysis.scripts,
    [],
    exclude_binaries=True,
    name="FlexibleTentacleACT",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

bundle = COLLECT(
    executable,
    analysis.binaries,
    analysis.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name="FlexibleTentacleACT",
)

