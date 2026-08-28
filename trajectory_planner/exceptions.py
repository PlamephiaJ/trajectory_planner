"""Package-specific exceptions."""


class PlanningError(RuntimeError):
    """Raised when a map does not contain a usable closed track."""
