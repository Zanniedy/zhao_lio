import json

from liobench.dev import append_history, check_gates, pick_reference, read_history, version_summary


def test_gates():
    gates = {"success_rate": {"min": 1.0}, "ate_rmse_m": {"max_increase_pct": 5},
             "internal_mean_ms": {"max": 2.0}}
    ref = {"pw": {"success_rate": 1.0, "ate_rmse_m": 1.0, "internal_mean_ms": 1.0}}
    good = {"pw": {"success_rate": 1.0, "ate_rmse_m": 1.04, "internal_mean_ms": 1.5}}
    assert all(ok for _, _, ok, _ in check_gates(gates, good, ref))
    bad = {"pw": {"success_rate": 0.5, "ate_rmse_m": 1.10, "internal_mean_ms": 2.5}}
    res = {m: ok for _, m, ok, _ in check_gates(gates, bad, ref)}
    assert res == {"success_rate": False, "ate_rmse_m": False, "internal_mean_ms": False}
    no_gt = {"pw": {"success_rate": 1.0, "ate_rmse_m": None, "internal_mean_ms": 1.0}}
    res = {m: ok for _, m, ok, _ in check_gates(gates, no_gt, ref)}
    assert res["ate_rmse_m"] is None            # skipped, does not fail the version
    first = {m: ok for _, m, ok, _ in check_gates(gates, good, None)}
    assert first["ate_rmse_m"] is True          # relative gate without reference passes


def test_reference_choice():
    hist = [{"version": "a", "passed": "True", "ate_rmse_m": "0.5"},
            {"version": "b", "passed": "False", "ate_rmse_m": "0.1"},
            {"version": "c", "passed": "True", "ate_rmse_m": "0.7"}]
    cur = hist + [{"version": "d"}]
    assert pick_reference({"reference": "previous", "primary": "ate_rmse_m"}, cur, "d") == "c"
    assert pick_reference({"reference": "best", "primary": "ate_rmse_m"}, cur, "d") == "a"
    assert pick_reference({"reference": "a", "primary": "ate_rmse_m"}, cur, "d") == "a"
    assert pick_reference({"reference": "previous", "primary": "x"}, [{"version": "d"}], "d") is None
    # quick checks (--duration) are only compared with quick checks of the same length
    mixed = [{"version": "full", "duration": ""}, {"version": "q1", "duration": "60"}, {"version": "d", "duration": "60"}]
    assert pick_reference({"reference": "previous", "primary": "x"}, mixed, "d", "60") == "q1"
    assert pick_reference({"reference": "previous", "primary": "x"}, mixed[:1] + [{"version": "e"}], "e", "") == "full"


def test_history_and_summary(tmp_path):
    runs = tmp_path / "runs"
    for i, (variant, seq, lat) in enumerate([("pw", "ds/a", 1.0), ("pw", "ds/b", 3.0), ("pm", "ds/a", 2.0)]):
        d = runs / str(i)
        d.mkdir(parents=True)
        (d / "run.json").write_text(json.dumps({"algorithm": "x", "variant": variant, "sequence": seq, "repeat": 0,
                                                "status": "ok", "reasons": [], "has_gt": False,
                                                "latency_mean_ms": lat}))
    s = version_summary(runs)
    assert s["pw"]["latency_mean_ms"] == 2.0 and s["pw"]["sequences"] == 2
    assert s["pw"]["ate_rmse_m"] is None
    append_history(tmp_path, [{"version": "v1", **s["pw"]}])
    append_history(tmp_path, [{"version": "v2", **s["pm"]}])
    h = read_history(tmp_path)
    assert [r["version"] for r in h] == ["v1", "v2"]
