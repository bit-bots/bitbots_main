import re
import subprocess
import tempfile
import time
from pathlib import Path

from rclpy.logging import get_logger


class PlanningResult:
    def __init__(self, plan: list[dict], success: bool, raw_output: str = "", time_taken: float = 0.0):
        self.plan = plan
        self.success = success
        self.raw_output = raw_output
        self.time_taken = time_taken


class PlannerInterface:
    def __init__(self, fd_path: str = "~/downward/fast-downward.py"):
        self.fd_path = Path(fd_path).expanduser().resolve()
        self.logger = get_logger("planner_interface")

        if not self.fd_path.exists():
            self.logger.error(f"Fast-Downward wasn't found {self.fd_path}")
            self.logger.error("Please install Fast-Downward and adjust path in __init__.")

    def solve(self, domain: str, problem: str, timeout: int = 8) -> PlanningResult:
        # Solve a pddl problem with fast-forward and returns a structured plan
        start_time = time.time()

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            domain_file = tmp_path / "domain.pddl"
            problem_file = tmp_path / "problem.pddl"

            domain_file.write_text(domain)
            problem_file.write_text(problem)

            # Calling fast-downward
            cmd = [
                str(self.fd_path),
                "--alias",
                "seq-opt-lmcut",
                str(domain_file),
                str(problem_file),
                "--search-time-limit",
                f"{timeout}s",
            ]

            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 5)

                exec_time = time.time() - start_time

                # Detect success
                if result.returncode == 0 and "Solution found" in result.stdout:
                    plan = self._parse_plan(result.stdout)
                    self.logger.info(f"Plan found ({len(plan)} steps, {exec_time:.2f}s)")
                    return PlanningResult(plan, True, result.stdout, exec_time)
                else:
                    self.logger.warn("No plan found")
                    return PlanningResult([], False, result.stdout + result.stderr, exec_time)

            except subprocess.TimeoutExpired:
                self.logger.error("Planner Timeout")
                return PlanningResult([], False, "Timeout", timeout)
            except Exception as e:
                self.logger.error(f"Failure at planner call: {e}")
                return PlanningResult([], False, str(e), 0.0)

    # Parses the fast-downward output-text in a list of actions
    def _parse_plan(self, output: str) -> list[dict]:
        plan = []
        for line in output.splitlines():
            line = line.strip()
            if not line or line.startswith(";") or "cost" in line.lower():
                continue

            # Typical Fast-Downward format: 0.0000: (walk_to robot1 pos_ball)
            if ":" in line and "(" in line and ")" in line:
                time_str, action_part = line.split(":", 1)
                time_val = float(time_str.strip())

                match = re.search(r"\((.+?)\)", action_part)
                if match:
                    action_str = match.group(1).strip()
                    parts = action_str.split()
                    action_name = parts[0]
                    params = parts[1:] if len(parts) > 1 else []

                    plan.append({"time": time_val, "action": action_name, "params": params})
        return plan
