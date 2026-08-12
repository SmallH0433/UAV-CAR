import json

from air_ground_sim.audit_journal import AuditJournal


def test_journal_redacts_credentials_and_is_valid_jsonl(tmp_path):
    path = tmp_path / "audit.jsonl"
    journal = AuditJournal(str(path), required=True)
    assert journal.write(
        {"event": "command", "payload": {"token": "secret", "x": 1}}
    )
    record = json.loads(path.read_text(encoding="utf-8"))
    assert record["payload"]["token"] == "[REDACTED]"
    assert record["payload"]["x"] == 1
    assert record["schema_version"] == "1.0"
    assert record["recorded_at"].endswith("+00:00")


def test_journal_rotates_without_losing_latest_event(tmp_path):
    path = tmp_path / "audit.jsonl"
    journal = AuditJournal(str(path), max_bytes=4096, backup_count=2, required=True)
    for index in range(30):
        journal.write({"event": "test", "index": index, "padding": "x" * 300})
    assert path.exists()
    assert (tmp_path / "audit.jsonl.1").exists()
    latest = json.loads(path.read_text(encoding="utf-8").splitlines()[-1])
    assert latest["index"] == 29

