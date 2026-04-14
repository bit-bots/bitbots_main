import os

from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from rclpy.logging import get_logger

load_dotenv()


class ProblemGenerator:
    def __init__(self):
        # Open-AI Key
        if not os.getenv("OPENAI_API_KEY"):
            raise ValueError("OPENAI_API_KEY isn't set!")

        self.logger = get_logger()

        self.llm = ChatOpenAI(model="gpt-4o", temperature="0.0", max_tokens=700)

        self.domain = self._load_domain()

    def _load_domain(self):
        path = os.path.join(os.path.dirname(__file__), "../pddl/domain.pddl")
        if not os.path.exists(path):
            raise FileNotFoundError(f"domain.pddl not found: {path}")
        with open(path, encoding="utf-8") as f:
            return f.read()
