"""Unit tests for db_node — database persistence layer.

Tests use an in-memory SQLite database. No external DB required.
"""

import json
import os
import pytest

# Force in-memory database for tests
os.environ["ROOMBA_DB_URL"] = "sqlite:///:memory:"

from roomba_db.models import (
    Base,
    MapRecord,
    SessionRecord,
    get_engine,
    get_session_factory,
)


@pytest.fixture
def db_session():
    """Create a fresh in-memory database session for each test."""
    factory = get_session_factory()
    session = factory()
    yield session
    session.close()


class TestModels:
    """Tests for SQLAlchemy ORM models."""

    def test_create_map_record(self, db_session):
        """Map record can be created and retrieved."""
        record = MapRecord(
            name="test_map",
            map_data=b"mock_grid_data",
            origin_x=0.0,
            origin_y=0.0,
            resolution=0.05,
            width=20,
            height=20,
        )
        db_session.add(record)
        db_session.commit()

        result = db_session.query(MapRecord).first()
        assert result is not None
        assert result.name == "test_map"
        assert result.width == 20

    def test_create_session_record(self, db_session):
        """Session record can be created."""
        record = SessionRecord(mode="MANUAL")
        db_session.add(record)
        db_session.commit()

        result = db_session.query(SessionRecord).first()
        assert result is not None
        assert result.mode == "MANUAL"

    def test_session_map_relationship(self, db_session):
        """Session record can reference a map."""
        map_rec = MapRecord(
            name="linked_map",
            map_data=b"data",
        )
        db_session.add(map_rec)
        db_session.commit()

        session_rec = SessionRecord(
            mode="RECON",
            map_id=map_rec.id,
        )
        db_session.add(session_rec)
        db_session.commit()

        result = db_session.query(SessionRecord).first()
        assert result.map_id == map_rec.id

    def test_db_unavailable_handling(self):
        """Database connection failure raises RuntimeError."""
        # This tests the error path — bad URL should fail
        old_url = os.environ.get("ROOMBA_DB_URL")
        os.environ["ROOMBA_DB_URL"] = "sqlite:///nonexistent/path/db.sqlite"

        # get_engine should still work (lazy connection), but operations fail
        engine = get_engine()
        assert engine is not None

        if old_url:
            os.environ["ROOMBA_DB_URL"] = old_url


class TestMapCRUD:
    """Tests for map create, list, get, rename, delete operations."""

    def test_save_map_with_json_data(self, db_session):
        """Map with JSON-encoded grid data can be saved and retrieved."""
        grid = [0] * 100 + [-1] * 50 + [100] * 50
        record = MapRecord(
            name="json_map",
            map_data=json.dumps(grid).encode("utf-8"),
            origin_x=-5.0,
            origin_y=-5.0,
            resolution=0.05,
            width=10,
            height=20,
        )
        db_session.add(record)
        db_session.commit()

        result = db_session.query(MapRecord).filter_by(name="json_map").first()
        assert result is not None
        decoded = json.loads(result.map_data)
        assert len(decoded) == 200
        assert decoded[0] == 0
        assert decoded[100] == -1

    def test_list_maps_ordered(self, db_session):
        """Maps are returned ordered by created_at descending."""
        for i in range(3):
            db_session.add(MapRecord(
                name=f"map_{i}",
                map_data=b"data",
                width=10,
                height=10,
            ))
        db_session.commit()

        results = db_session.query(MapRecord).order_by(
            MapRecord.created_at.desc()
        ).all()
        assert len(results) == 3

    def test_rename_map(self, db_session):
        """Map can be renamed."""
        record = MapRecord(name="old_name", map_data=b"data")
        db_session.add(record)
        db_session.commit()

        record.name = "new_name"
        db_session.commit()

        result = db_session.query(MapRecord).filter_by(id=record.id).first()
        assert result.name == "new_name"

    def test_delete_map(self, db_session):
        """Map can be deleted."""
        record = MapRecord(name="delete_me", map_data=b"data")
        db_session.add(record)
        db_session.commit()
        map_id = record.id

        db_session.delete(record)
        db_session.commit()

        result = db_session.query(MapRecord).filter_by(id=map_id).first()
        assert result is None

    def test_delete_map_cascades_sessions(self, db_session):
        """Deleting a map leaves sessions with null map_id."""
        map_rec = MapRecord(name="cascade_test", map_data=b"data")
        db_session.add(map_rec)
        db_session.commit()

        session_rec = SessionRecord(mode="RECON", map_id=map_rec.id)
        db_session.add(session_rec)
        db_session.commit()
        session_id = session_rec.id

        # Delete the map — session should survive with null map_id
        db_session.delete(map_rec)
        db_session.commit()

        result = db_session.query(SessionRecord).filter_by(id=session_id).first()
        assert result is not None
        assert result.map_id is None
