import json

import pytest

from core import trajectory_store as store

PIECES = [[(0.123456789, 1.0), (2.0, 3.0)], [(5.0, 5.0), (6.0, 4.0), (7.0, 5.0)]]


def test_save_load_round_trip(tmp_path):
    meta = store.save(str(tmp_path), name="T", source="box", mode="manual", pieces=PIECES, bounds=[0, 1, 7, 5])
    assert meta["piece_count"] == 2 and meta["point_count"] == 5
    loaded_meta, pieces = store.load(str(tmp_path), meta["id"])
    assert loaded_meta == meta
    assert pieces[0][0] == [0.123457, 1.0] and pieces[1] == [[5, 5], [6, 4], [7, 5]]


def test_list_newest_first_and_delete(tmp_path):
    a = store.save(str(tmp_path), name="a", source="s", mode="auto", pieces=PIECES, bounds=[0, 0, 1, 1])
    b = store.save(str(tmp_path), name="b", source="s", mode="auto", pieces=PIECES, bounds=[0, 0, 1, 1])
    assert [m["name"] for m in store.list_meta(str(tmp_path))] == ["b", "a"]
    store.delete(str(tmp_path), a["id"])
    assert [m["id"] for m in store.list_meta(str(tmp_path))] == [b["id"]]


def test_list_picks_up_rewritten_files(tmp_path):
    meta = store.save(str(tmp_path), name="old", source="s", mode="auto", pieces=PIECES, bounds=[0, 0, 1, 1])
    store.list_meta(str(tmp_path))
    path = tmp_path / f"{meta['id']}.json"
    data = json.loads(path.read_text())
    data["meta"]["name"] = "new"
    path.write_text(json.dumps(data))
    import os
    os.utime(path, (1, 1))   # force a different mtime
    assert store.list_meta(str(tmp_path))[0]["name"] == "new"


@pytest.mark.parametrize("bad", ["", "../x", "0123456789AB", "0123456789abc", "nope"])
def test_bad_ids_are_rejected(tmp_path, bad):
    with pytest.raises(store.TrajectoryNotFound):
        store.load(str(tmp_path), bad)


def test_missing_folder_lists_nothing(tmp_path):
    assert store.list_meta(str(tmp_path / "missing")) == []
