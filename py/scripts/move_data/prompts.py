from InquirerPy import inquirer

DATA_KINDS = ["data", "training_data"]


def select_data_kind() -> str:
    return inquirer.select(
        message="What kind of data are you moving?",
        choices=DATA_KINDS,
    ).execute()


def select_folder(kind: str, folders: list[str]) -> str:
    return inquirer.select(
        message=f"Select a folder from {kind}:",
        choices=folders,
    ).execute()


def select_destination(destinations: list[str]) -> str:
    return inquirer.select(
        message="Select the destination:",
        choices=destinations,
    ).execute()
