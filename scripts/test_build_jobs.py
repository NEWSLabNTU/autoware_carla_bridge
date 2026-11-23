"""acb's just build job count (scensim issue 121): pure, no build."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_jobs as bj  # noqa: E402


def test_little_memory_caps_below_the_cores():
    out, note = bj.plan(cpus=24, mem_gb=16.0, env={})
    # half of 16 GB at 2.5 GB a job: 3 jobs, one package at a time under 4
    assert out == {"MAKEFLAGS": "-j3", "CARGO_BUILD_JOBS": "3", "COLCON_WORKERS": "1"}
    assert "16.0 GB available" in note


def test_plenty_of_memory_stops_at_the_cores():
    out, _ = bj.plan(cpus=24, mem_gb=200.0, env={})
    assert out == {"MAKEFLAGS": "-j12", "CARGO_BUILD_JOBS": "12", "COLCON_WORKERS": "2"}


def test_no_memory_still_builds():
    out, _ = bj.plan(cpus=24, mem_gb=1.0, env={})
    assert out["MAKEFLAGS"] == "-j1" and out["COLCON_WORKERS"] == "1"


def test_the_host_setting_wins():
    out, note = bj.plan(cpus=24, mem_gb=200.0, env={bj.ENV: "6"})
    assert out == {"MAKEFLAGS": "-j3", "CARGO_BUILD_JOBS": "3", "COLCON_WORKERS": "2"}
    assert f"{bj.ENV}=6" in note


@pytest.mark.parametrize("bad", ["0", "-2", "many", "2.5"])
def test_a_bad_host_setting_is_refused(bad):
    with pytest.raises(ValueError, match=bj.ENV):
        bj.plan(cpus=24, mem_gb=200.0, env={bj.ENV: bad})


def test_settings_in_the_environment_are_kept():
    env = {"MAKEFLAGS": "-j6", "CARGO_BUILD_JOBS": "6", "COLCON_DEFAULTS_FILE": "/x.yaml"}
    out, note = bj.plan(cpus=24, mem_gb=200.0, env=env)
    assert out == {"COLCON_WORKERS": ""}           # nothing assigned, no --parallel-workers
    assert "kept from the environment" in note


def test_other_make_flags_stay():
    out, _ = bj.plan(cpus=24, mem_gb=40.0, env={"MAKEFLAGS": "-s"})
    assert out["MAKEFLAGS"] == "-s -j4"


def test_meminfo_is_read(tmp_path):
    f = tmp_path / "meminfo"
    f.write_text("MemTotal: 131257868 kB\nMemAvailable:   10485760 kB\n")
    assert bj.mem_available_gb(str(f)) == 10.0
