# How to document

Our documentation is published under [docs.bit-bots.de](https://docs.bit-bots.de) and is generated with [MkDocs](https://www.mkdocs.org/) using the [rtd-dropdown](https://github.com/bread_and_noise/mkdocs-rtd-dropdown) theme.
All documentation for this repository lives in the top-level `docs` directory and is written in Markdown.

## Dependencies

The Pixi `docs` environment provides MkDocs and the theme used for the build, so no additional installation is necessary.

## How to build the documentation

Build the documentation from the workspace root:

```bash
pixi run -e docs docs
```

The result is written to `_build/index.html`, which can be opened in a browser.

## How to write documentation

Create your `.md` documents somewhere under `docs/manual` and add them to the `nav` section of `mkdocs.yml` under the matching caption.
Links to other pages use relative paths to the `.md` file, for example `[Testing](../getting_started/simulation_testing.md)`.

Notes are written as admonitions:

```markdown
!!! note

    Some hint for the reader.
```

Build the documentation after adding a page to make sure it is referenced and renders correctly.
