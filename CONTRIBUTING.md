# GitHub Rules and Guidelines

These rules apply to all members working within the club’s GitHub organization. The goal is to keep our repositories organized, traceable, and easy for both current and future members to maintain.

## 1. Repository Access

* Members should only be given the level of access required for their work.
* Organization-wide administrator or owner access should be limited to management or designated maintainers.
* Do not add external collaborators without approval from a Team Lead or relevant management member.
* Access should be reviewed when members change teams or leave the club.

## 2. Repository Creation

Do not create a new repository without first checking whether the project can be included in an existing repository.

When creating a repository:

* Use a clear and descriptive name.
* Avoid names such as `test`, `new-project`, `final`, or `final-v2`.
* Add a `README.md` explaining the purpose of the repository.
* Add an appropriate `.gitignore`.
* Add documentation explaining how to build, run, or use the project where applicable.
* Assign a responsible team or maintainer.

## 3. Repository Structure

Projects should use a clear and consistent folder structure.

Example:

```text
Project-Name/
├── README.md
├── docs/
├── hardware/
├── software/
├── mechanical/
├── tests/
└── resources/
```

The exact structure may vary depending on the project, but files should always be placed somewhere that another member can reasonably find them.

Do not upload large numbers of unrelated files directly into the repository root.

## 4. Branches

The `main` branch should always represent a stable and usable version of the project.

For normal development, create a separate branch.

Recommended naming:

```text
feature/flight-logger
feature/can-communication
fix/esc-telemetry
hardware/power-board
docs/github-rules
```

Avoid working directly on `main` for significant changes.

Small documentation corrections or other low-risk changes may be committed directly if permitted by the repository maintainer.

## 5. Commits

Commit messages should clearly describe what changed.

Good examples:

```text
Add ESC telemetry support
Fix incorrect CAN message ID
Update power distribution schematic
Add thrust stand calibration procedure
```

Avoid:

```text
update
stuff
test
123
final
final final
```

Try to keep each commit focused on one logical change.

## 6. Pull Requests

Significant changes should normally be submitted through a Pull Request (PR).

A PR should explain:

* What was changed
* Why the change was made
* Any important design decisions
* How the change was tested
* Any known issues or unfinished work

Where practical, another member should review the PR before it is merged.

Major changes involving system architecture, safety-critical hardware/software, or interfaces between teams should be reviewed by the relevant Team Lead or maintainer.

## 7. Do Not Break `main`

Before merging into `main`:

* Make sure the project builds or works as expected.
* Check that your changes do not break existing functionality.
* Resolve merge conflicts properly.
* Remove unnecessary debugging code.
* Do not merge incomplete experimental work unless it is clearly intended to be part of the main repository.

If you are unsure whether something is ready, ask the repository maintainer or Team Lead.

## 8. Hardware Files

For PCB, electrical, CAD, and other hardware projects:

* Keep editable source files in the repository, not only exported PDFs or images.
* Clearly identify the latest design.
* Do not use filenames such as `final`, `final2`, or `final_really_final`.
* Use Git history, releases, tags, or documented version numbers instead.
* Include relevant datasheets or links where appropriate.
* Document important design decisions and interfaces.

For PCB projects, manufacturing files such as Gerbers should be clearly separated from editable design files.

## 9. Large and Generated Files

Avoid committing unnecessary:

* Build outputs
* Temporary files
* Cache files
* IDE-generated files
* Large binaries
* Duplicate exports

Use `.gitignore` wherever appropriate.

Large files should only be stored in Git when there is a clear reason for doing so.

## 10. Documentation

Someone who joins the team next semester should be able to understand what the project does without having to ask the original developer.

At minimum, important projects should document:

* What the project is
* Current project status
* How the system works
* How to build/use/test it
* Hardware and software dependencies
* Important interfaces
* Known problems
* Who or which team is responsible for it

Documentation is part of the project, not an optional extra.

## 11. Issues

Use GitHub Issues to track bugs, tasks, improvements, and technical problems where appropriate.

Issues should have a clear title and enough information for another member to understand the problem.

When possible, include:

* Description
* Expected behaviour
* Actual behaviour
* Relevant hardware/software version
* Screenshots, logs, or test results
* Person/team responsible

Close issues when the work is completed.

## 12. Releases and Versions

Important working versions should be tagged or released.

Examples:

```text
v0.1
v0.2
v1.0
NFC-2026
Competition-Release
```

Do not rely on filenames to maintain version history.

Git already provides version control — use it.

## 13. Secrets and Sensitive Information

Never commit:

* Passwords
* API keys
* Access tokens
* Private SSH keys
* Personal credentials
* Other confidential information

If a secret is accidentally committed, notify a Team Lead or organization administrator immediately. Deleting the file in a later commit is not sufficient because the information may remain in Git history.

## 14. Deleting or Rewriting History

Do not:

* Delete repositories
* Force-push shared branches
* Rewrite important Git history
* Change repository visibility
* Remove branch protection
* Delete releases or major branches

without approval from the relevant maintainer or management.

Commands such as:

```text
git push --force
```

should be used with extreme caution on shared repositories.

## 15. Ownership and Handover

Projects belong to the club rather than an individual member.

Before leaving a project or the club, members should:

* Push all relevant work to GitHub.
* Update documentation.
* Record unfinished tasks and known problems.
* Make sure another member can access and understand the project.
* Remove dependencies on personal accounts where possible.

A project should not become unusable simply because one member leaves.

## 16. General Principle

Before making a change, ask yourself:

> **Will another member be able to understand what I did six months from now?**

If the answer is no, add documentation, improve the commit message, or explain the change in a Pull Request.

GitHub is not only a place to store files. It is the club’s engineering history and should allow future members to understand, reproduce, and continue our work.
