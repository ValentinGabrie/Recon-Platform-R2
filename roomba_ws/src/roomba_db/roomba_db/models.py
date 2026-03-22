"""Database models for roomba — SQLAlchemy ORM definitions.

All database tables are defined here using SQLAlchemy ORM.
No raw SQL strings in application code.
"""

import logging
import os
from datetime import datetime

from sqlalchemy import (
    Column,
    DateTime,
    Float,
    Integer,
    LargeBinary,
    String,
    ForeignKey,
    create_engine,
)
from sqlalchemy.orm import (
    DeclarativeBase,
    Session,
    sessionmaker,
    relationship,
)
from sqlalchemy.pool import QueuePool, StaticPool

logger = logging.getLogger(__name__)


class Base(DeclarativeBase):
    """SQLAlchemy declarative base for all roomba models."""
    pass


class MapRecord(Base):
    """Stored occupancy grid map.

    Attributes:
        id: Auto-incrementing primary key.
        name: Human-readable map name.
        created_at: Timestamp of creation.
        updated_at: Timestamp of last update.
        map_data: Serialised OccupancyGrid (JSON or msgpack).
        origin_x: Map origin X coordinate.
        origin_y: Map origin Y coordinate.
        resolution: Map resolution in metres/cell.
        width: Map width in cells.
        height: Map height in cells.
        thumbnail: Optional PNG preview for web UI.
    """
    __tablename__ = "maps"

    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, onupdate=datetime.utcnow)
    map_data = Column(LargeBinary, nullable=False)
    origin_x = Column(Float)
    origin_y = Column(Float)
    resolution = Column(Float)
    width = Column(Integer)
    height = Column(Integer)
    thumbnail = Column(LargeBinary, nullable=True)  # [OPTIONAL]

    sessions = relationship("SessionRecord", back_populates="map")


class SessionRecord(Base):
    """Robot operating session record.

    Attributes:
        id: Auto-incrementing primary key.
        started_at: Session start timestamp.
        ended_at: Session end timestamp.
        mode: Operating mode (MANUAL, RECON, IDLE).
        map_id: Foreign key to associated map.
    """
    __tablename__ = "sessions"

    id = Column(Integer, primary_key=True, autoincrement=True)
    started_at = Column(DateTime, default=datetime.utcnow)
    ended_at = Column(DateTime, nullable=True)
    mode = Column(String)
    map_id = Column(Integer, ForeignKey("maps.id"), nullable=True)

    map = relationship("MapRecord", back_populates="sessions")


class MapEvent(Base):
    """Tracks map save/delete events for UI polling.

    The web UI polls this table to detect maps saved headlessly by
    db_node (controller X button) without requiring the UI to be open.

    Attributes:
        id: Auto-incrementing primary key.
        event_type: SAVED or DELETED.
        map_id: Associated map id (nullable — map may be deleted).
        map_name: Snapshot of the map name at event time.
        created_at: Timestamp of the event.
    """
    __tablename__ = "map_events"

    id = Column(Integer, primary_key=True, autoincrement=True)
    event_type = Column(String, nullable=False)
    map_id = Column(Integer, nullable=True)
    map_name = Column(String, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)


def get_engine(db_url: str | None = None):
    """Create SQLAlchemy engine from ROOMBA_DB_URL environment variable.

    Args:
        db_url: Optional override for the database URL. If None, reads
                from ROOMBA_DB_URL env var (default: sqlite:///roomba.db).

    Returns:
        SQLAlchemy Engine instance.

    Raises:
        Exception: If engine creation fails (invalid URL, driver missing, etc.).
    """
    url = db_url or os.environ.get(
        "ROOMBA_DB_URL", "sqlite:///roomba.db"
    )
    logger.info("Creating DB engine — url=%s", _sanitise_url(url))

    # SQLite needs StaticPool for in-memory DBs and no pool for file DBs.
    # PostgreSQL benefits from a connection pool.
    if url.startswith("sqlite"):
        if ":memory:" in url:
            engine = create_engine(
                url, echo=False,
                connect_args={"check_same_thread": False},
                poolclass=StaticPool,
            )
        else:
            engine = create_engine(url, echo=False)
    else:
        engine = create_engine(
            url, echo=False,
            poolclass=QueuePool,
            pool_size=5,
            max_overflow=5,
            pool_pre_ping=True,
        )
    return engine


def _sanitise_url(url: str) -> str:
    """Mask password in DB URL for safe logging."""
    if "@" in url:
        # postgresql://user:pass@host → postgresql://user:***@host
        pre_at = url.split("@")[0]
        post_at = url.split("@", 1)[1]
        if ":" in pre_at.split("//", 1)[-1]:
            scheme_user = pre_at.rsplit(":", 1)[0]
            return f"{scheme_user}:***@{post_at}"
    return url


def get_session_factory(db_url: str | None = None):
    """Create a session factory bound to the default engine.

    Args:
        db_url: Optional override for the database URL.

    Returns:
        SQLAlchemy sessionmaker instance.
    """
    engine = get_engine(db_url)
    try:
        Base.metadata.create_all(engine)
    except Exception as exc:
        logger.error("Failed to create DB tables: %s", exc)
        raise
    logger.info("DB tables ensured — session factory ready")
    return sessionmaker(bind=engine)
