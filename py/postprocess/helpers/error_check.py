def ensure(condition: object, message: str) -> None:
    if not condition:
        raise ValueError(message)
