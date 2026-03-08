"""Database models for roomba — SQLAlchemy ORM definitions.

All database tables are defined here using SQLAlchemy ORM.
No raw SQL strings in application code.
"""

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


def get_engine():
    """Create SQLAlchemy engine from ROOMBA_DB_URL environment variable.

    Returns:
        SQLAlchemy Engine instance.

    Raises:
        RuntimeError: If ROOMBA_DB_URL is not set and default path fails.
    """
    db_url = os.environ.get("ROOMBA_DB_URL", "sqlite:///roomba.db")
    return create_engine(db_url, echo=False)


def get_session_factory():
    """Create a session factory bound to the default engine.

    Returns:
        SQLAlchemy sessionmaker instance.
    """
    engine = get_engine()
    Base.metadata.create_all(engine)
    return sessionmaker(bind=engine)
