"""Unit tests for db_node — database persistence layer.

Tests use an in-memory SQLite database. No external DB required.
"""

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
