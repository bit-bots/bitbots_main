"""Execute the plain Python tutorial cells and embed their outputs without Jupyter."""

import argparse
import base64
import contextlib
import io
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

DIRECTORY = Path(__file__).resolve().parent


def render(path):
    notebook = json.loads(path.read_text())
    namespace = {"__name__": "__main__"}
    counter = 0
    original_show = plt.show
    try:
        for cell in notebook["cells"]:
            if cell["cell_type"] != "code":
                continue
            counter += 1
            outputs = []
            stream = io.StringIO()

            def flush_text(stream=stream, outputs=outputs):
                text = stream.getvalue()
                if text:
                    outputs.append({"output_type": "stream", "name": "stdout", "text": text.splitlines(True)})
                    stream.seek(0)
                    stream.truncate()

            def show(*args, outputs=outputs, flush=flush_text, **kwargs):
                flush()
                for number in plt.get_fignums():
                    image = io.BytesIO()
                    plt.figure(number).savefig(image, format="png", bbox_inches="tight")
                    outputs.append(
                        {
                            "output_type": "display_data",
                            "metadata": {},
                            "data": {"image/png": base64.b64encode(image.getvalue()).decode("ascii")},
                        }
                    )
                plt.close("all")

            plt.show = show
            with contextlib.redirect_stdout(stream):
                exec(compile("".join(cell["source"]), f"{path.name}:cell-{counter}", "exec"), namespace)
                if plt.get_fignums():
                    show()
            flush_text()
            cell["outputs"] = outputs
            cell["execution_count"] = counter
    finally:
        plt.show = original_show
        plt.close("all")
    path.write_text(json.dumps(notebook, ensure_ascii=False, indent=1) + "\n")
    print(f"{path.name}: {counter} Python-Zellen erfolgreich ausgeführt.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "notebooks", type=Path, nargs="*", default=[DIRECTORY / "aufgabe.ipynb", DIRECTORY / "loesung.ipynb"]
    )
    arguments = parser.parse_args()
    for notebook_path in arguments.notebooks:
        render(notebook_path)
