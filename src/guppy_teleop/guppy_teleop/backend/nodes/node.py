from abc import abstractmethod

class Node:
    @property
    @abstractmethod
    def name(self) -> str: ...
    
    @abstractmethod
    def get_payload(self, payload: dict) -> dict:  ...

    def handle_command(self, payload: dict):
        pass