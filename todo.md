# TODO

## Extract Dynamixel IO layer into `py/dynamixel/` package

### Motivation
`py/read_dynamixel.py` currently mixes two concerns:
1. A reusable Dynamixel IO layer (`DynamixelPortReader`, constants, helpers).
2. A standalone TUI viewer (CLI + screen rendering).

`py/calibrate/calibrate_dynamixel.py` already reaches into `read_dynamixel` to import
`BAUDRATE`, `DXL_IDS`, `UNAVAILABLE_VALUE`, `DynamixelPortReader`, `read_with_fallback`.
That makes a viewer script load-bearing as a library, which is backwards.

Goal: extract the IO layer into its own package so both scripts depend on a clean
shared module, and `read_dynamixel.py` becomes just a thin TUI on top of it.

### Target layout
```
py/
├── dynamixel/
│   ├── __init__.py        # re-exports the public API
│   └── port_reader.py     # the actual code
├── read_dynamixel.py      # TUI viewer only — imports from dynamixel
└── calibrate/
    └── calibrate_dynamixel.py  # imports from dynamixel instead of read_dynamixel
```

### What moves into `py/dynamixel/port_reader.py`
From `py/read_dynamixel.py`:
- Constants: `PROTOCOL_VERSION`, `BAUDRATE`, `DXL_IDS`, `DXL_POSITION_MODULUS`,
  `UNAVAILABLE_VALUE`, `ADDR_REALTIME_TICK`, `LEN_REALTIME_TICK`,
  `ADDR_PRESENT_POSITION`, `LEN_PRESENT_POSITION`, `SYNC_READ_START`,
  `SYNC_READ_LEN`, `TICK_MAX`.
- The `dynamixel_sdk` import block (lines 11–16). Note: the fallback path
  `Path(__file__).resolve().parent / "third_party" / "DynamixelSDK" / ...`
  must be updated since `__file__` moves down one directory — use
  `parents[1] / "third_party" / "DynamixelSDK" / "python" / "src"` so it still
  resolves to `py/third_party/...` (currently dead but keep behavior identical).
- Classes: `TimestampUnwrapper`, `DynamixelPortReader`.
- Helpers: `unavailable_values()`, `read_with_fallback()`.

### What stays in `py/read_dynamixel.py`
- `clear_screen()`, `render_output()`, `parse_args()`, `main()`, the
  `if __name__ == "__main__"` block.
- Import the moved symbols from the new package:
  `from dynamixel import BAUDRATE, DXL_IDS, DynamixelPortReader, read_with_fallback, unavailable_values`

### `py/dynamixel/__init__.py`
Re-export the public surface so callers can `from dynamixel import ...`:
- `DynamixelPortReader`
- `read_with_fallback`
- `unavailable_values`
- `BAUDRATE`
- `DXL_IDS`
- `UNAVAILABLE_VALUE`

(Keep internal constants like the ADDR_* values in `port_reader.py`; don't re-export.)

### Consumer updates
- `py/read_dynamixel.py`: replace local definitions with the import above.
- `py/calibrate/calibrate_dynamixel.py:19`: change
  `from read_dynamixel import BAUDRATE, DXL_IDS, UNAVAILABLE_VALUE, DynamixelPortReader, read_with_fallback`
  to
  `from dynamixel import BAUDRATE, DXL_IDS, UNAVAILABLE_VALUE, DynamixelPortReader, read_with_fallback`.

### Verification
- `python py/read_dynamixel.py --help` runs without import errors.
- `python py/calibrate/calibrate_dynamixel.py --help` runs without import errors.
- `python -c "from dynamixel import DynamixelPortReader"` succeeds from `py/`.
- Optional: run the live reader against the arms and confirm timestamps + positions
  still display correctly.

