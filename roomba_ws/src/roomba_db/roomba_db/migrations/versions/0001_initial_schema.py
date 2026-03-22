"""initial schema — maps and sessions tables

Revision ID: 0001
Revises:
Create Date: 2026-03-21

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa

# revision identifiers, used by Alembic.
revision: str = "0001"
down_revision: Union[str, None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    op.create_table(
        "maps",
        sa.Column("id", sa.Integer, primary_key=True, autoincrement=True),
        sa.Column("name", sa.String, nullable=False),
        sa.Column("created_at", sa.DateTime),
        sa.Column("updated_at", sa.DateTime),
        sa.Column("map_data", sa.LargeBinary, nullable=False),
        sa.Column("origin_x", sa.Float),
        sa.Column("origin_y", sa.Float),
        sa.Column("resolution", sa.Float),
        sa.Column("width", sa.Integer),
        sa.Column("height", sa.Integer),
        sa.Column("thumbnail", sa.LargeBinary, nullable=True),
    )

    op.create_table(
        "sessions",
        sa.Column("id", sa.Integer, primary_key=True, autoincrement=True),
        sa.Column("started_at", sa.DateTime),
        sa.Column("ended_at", sa.DateTime, nullable=True),
        sa.Column("mode", sa.String),
        sa.Column("map_id", sa.Integer, sa.ForeignKey("maps.id"), nullable=True),
    )


def downgrade() -> None:
    op.drop_table("sessions")
    op.drop_table("maps")
