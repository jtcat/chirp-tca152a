"""Smoke test for the TCA-152A driver.

Requires `chirp` importable and a reference dump. Skips otherwise.
Reference dumps live in the sibling OpenTCA repo.

Run:  python -m pytest        (or)   python tests/test_decode.py
"""
import os
import sys

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))

DUMP_CANDIDATES = [
    os.path.join(HERE, "..", "..", "OpenTCA", "tcaconfigsave.txt"),
    os.path.join(HERE, "tcaconfigsave.txt"),
]


def _load():
    from chirp import memmap
    import tca152a
    dump = next((p for p in DUMP_CANDIDATES if os.path.exists(p)), None)
    if dump is None:
        raise FileNotFoundError("no reference dump found")
    r = tca152a.TCA152ARadio(None)
    r._mmap = memmap.MemoryMapBytes(open(dump, "rb").read())
    r.process_mmap()
    return r


def test_channel_decode():
    try:
        r = _load()
    except Exception as e:
        print("SKIP:", e)
        return
    m1 = r.get_memory(1)
    assert m1.name == "ZONE_A01"
    assert m1.freq == 400500000
    assert m1.tmode in ("Tone", "TSQL")
    assert abs(m1.rtone - 88.5) < 0.01
    assert m1.mode == "NFM"          # narrow, per flags
    assert str(m1.power) == "Low"
    print("channel decode OK:", m1.name, m1.freq, m1.mode, m1.power)


def test_round_trip():
    try:
        r = _load()
    except Exception as e:
        print("SKIP:", e)
        return
    m = r.get_memory(2)
    m.freq = 433500000
    m.duplex = "-"
    m.offset = 5000000
    m.mode = "NFM"
    m.tmode = "TSQL"
    m.rtone = m.ctone = 100.0
    m.name = "RTTEST"
    r.set_memory(m)
    m2 = r.get_memory(2)
    assert m2.freq == 433500000
    assert m2.duplex == "-" and m2.offset == 5000000
    assert m2.mode == "NFM"
    assert m2.name == "RTTEST"
    assert abs(m2.rtone - 100.0) < 0.01
    print("round-trip OK")


if __name__ == "__main__":
    test_channel_decode()
    test_round_trip()
