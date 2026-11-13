# These are standalone scripts (`python3 test_*.py`), not pytest tests: they
# run a procedural check/PASS/FAIL sequence at module scope and end in
# sys.exit(), which crashes pytest's own import machinery if it ever tries to
# collect them. Keep pytest out of this directory.
collect_ignore_glob = ["test_*.py"]
