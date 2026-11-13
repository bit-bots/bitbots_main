// Checks issue and PR titles and PR branch names against the conventions in
// CONTRIBUTING.md and keeps one comment up to date that lists the problems.
// The allowed types and scopes are read from the tables in CONTRIBUTING.md, so
// that document stays the single source of truth.

const fs = require("node:fs");
const path = require("node:path");
const { execFileSync } = require("node:child_process");

const MARKER = "<!-- conventions-check -->";
const TITLE_PATTERN = /^(?<type>[a-z]+)(?:\((?<scope>[^()]*)\))?(?<breaking>!)?: (?<description>\S.*)$/;
const BRANCH_PATTERN = /^(?<type>[a-z]+)\/[a-z0-9]+(?:-[a-z0-9]+)*$/;

// Returns the backticked first-column values of the table below `### <heading>`.
function tableKeys(doc, heading) {
  const start = doc.search(new RegExp(`^### ${heading}\\s*$`, "m"));
  if (start < 0) {
    throw new Error(`CONTRIBUTING.md has no "### ${heading}" section`);
  }
  const rest = doc.slice(start).split("\n").slice(1).join("\n");
  const end = rest.search(/^#{1,3} /m);
  const section = end < 0 ? rest : rest.slice(0, end);
  const keys = [...section.matchAll(/^\|\s*`([^`]+)`/gm)].map((match) => match[1]);
  if (keys.length === 0) {
    throw new Error(`The "### ${heading}" section in CONTRIBUTING.md has no table entries`);
  }
  return keys;
}

function loadConventions(repoRoot) {
  const doc = fs.readFileSync(path.join(repoRoot, "CONTRIBUTING.md"), "utf8");
  const packages = execFileSync("git", ["ls-tree", "-r", "--name-only", "HEAD"], {
    cwd: repoRoot,
    encoding: "utf8",
  })
    .split("\n")
    .filter((file) => path.basename(file) === "package.xml")
    .map((file) => path.basename(path.dirname(file)));
  return {
    types: tableKeys(doc, "Types"),
    scopes: [...tableKeys(doc, "Scopes"), ...packages],
  };
}

// Returns a list of human-readable problems, empty if the title is fine.
function checkTitle(title, { types, scopes }) {
  const match = TITLE_PATTERN.exec(title.trim());
  if (!match) {
    return [
      `The title \`${title}\` does not match \`<type>(<scope>): <description>\`.`,
    ];
  }
  const problems = [];
  const { type, scope } = match.groups;
  if (!types.includes(type)) {
    problems.push(`\`${type}\` is not a known type. Use one of ${list(types)}.`);
  }
  if (scope !== undefined && !scopes.includes(scope)) {
    problems.push(
      `\`${scope}\` is not a known scope. Use a scope from CONTRIBUTING.md or a package name, or omit the scope for repository-wide changes.`,
    );
  }
  return problems;
}

function checkBranch(branch, { types }) {
  const match = BRANCH_PATTERN.exec(branch);
  if (!match) {
    return [
      `The branch name \`${branch}\` does not match \`<type>/<short-description>\` in lowercase kebab-case.`,
    ];
  }
  if (!types.includes(match.groups.type)) {
    return [`The branch prefix \`${match.groups.type}/\` is not a known type. Use one of ${list(types)}.`];
  }
  return [];
}

function list(values) {
  return values.map((value) => `\`${value}\``).join(", ");
}

async function run({ github, context, core }) {
  const conventions = loadConventions(process.env.GITHUB_WORKSPACE);
  const pr = context.payload.pull_request;
  const item = pr ?? context.payload.issue;
  const kind = pr ? "pull request" : "issue";

  const problems = checkTitle(item.title, conventions);
  // Bot branches, such as Dependabot's, follow their own naming scheme.
  if (pr && pr.user.type !== "Bot") {
    problems.push(...checkBranch(pr.head.ref, conventions));
  }

  const repo = context.payload.repository;
  const guide = `${repo.html_url}/blob/${repo.default_branch}/CONTRIBUTING.md`;
  const body = [
    MARKER,
    `This ${kind} does not follow our [contribution conventions](${guide}#titles-and-commit-messages):`,
    "",
    ...problems.map((problem) => `- ${problem}`),
    "",
    "This comment updates automatically and is removed once everything is fixed.",
  ].join("\n");

  const { owner, repo: name } = context.repo;
  const comments = await github.paginate(github.rest.issues.listComments, {
    owner,
    repo: name,
    issue_number: item.number,
    per_page: 100,
  });
  const existing = comments.find(
    (comment) => comment.user?.type === "Bot" && comment.body?.startsWith(MARKER),
  );

  if (problems.length === 0) {
    core.info(`#${item.number} follows the conventions.`);
    if (existing) {
      await github.rest.issues.deleteComment({ owner, repo: name, comment_id: existing.id });
    }
    return;
  }

  for (const problem of problems) {
    core.warning(problem);
  }
  if (!existing) {
    await github.rest.issues.createComment({ owner, repo: name, issue_number: item.number, body });
  } else if (existing.body !== body) {
    await github.rest.issues.updateComment({ owner, repo: name, comment_id: existing.id, body });
  }
}

module.exports = { run, checkTitle, checkBranch, tableKeys, loadConventions };
