"""Check copyright headers (skipped: sources carry no license header)."""

from ament_copyright.main import main
import pytest


@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Assert ament_copyright reports no errors."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'
