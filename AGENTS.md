# Working instructions for Autonomous Explorer

## Read the current project state
Read README.md, PLAN.md, recent commits, and the relevant implementation before continuing. Preserve completed milestones, existing results, and unrelated local work. The book and its official exercises remain the primary technical references. Explain deviations from those sources rather than silently changing the scientific objective.

## GitHub delivery is part of the task
The user's standing expectation is to implement, verify, commit, and push coherent progress to GitHub. This is already authorized for the requested project work; do not ask the user to repeat permission to commit or push.

- Publish meaningful, verified checkpoints during long tasks, including completed implementation stages, documented experiment results, and finished galleries. Do not wait until an entire multi-milestone project is complete.
- At the end of every implementation task, commit and push all completed task work to the intended branch. A local commit is not publication.
- Inspect the working tree and stage only the intended changes. Preserve pre-existing and unrelated work.
- Fetch and inspect the remote before publishing. Integrate concurrent compatible changes using the current branch/PR workflow, preserving both local and remote work. Never force-push or discard work to make publication succeed.
- Run checks appropriate to the change. Documentation-only updates do not require rerunning simulation campaigns. Fix relevant failures before claiming a verified checkpoint.
- After pushing, verify that the intended commit is on GitHub. Compare local and remote commit IDs; if the remote advanced concurrently, verify that it contains the delivered commit.
- If publication is blocked, report the exact command/error, the branch and local commit, and what remains unpublished. Continue independent authorized work where possible. Never silently omit the push or claim a push succeeded without verification.

## Maintain the GitHub About panel
A useful GitHub About description and accurate topics are part of repository delivery. Inspect them at project setup and when implemented capabilities change. A README or pyproject description does not update GitHub's About panel.

Use connected metadata tools or authenticated GitHub CLI access, preserve unrelated settings, and verify the remote result. If the current connection cannot edit repository metadata, explicitly record that blocker rather than treating the metadata as complete.

Suggested description, accurate for the published project as of 2026-10-25:
> A visual robotics lab following Principles of Robot Autonomy: rover control, navigation, localization, SLAM, and exploration with reproducible experiments.

Suitable topics include robotics, mobile-robotics, simulation, control-theory, path-planning, localization, slam, visualization, and python. Add a capability topic only after it is implemented. Do not claim hardware validation or reinforcement learning prematurely.

## Visual evidence and explanations
Generate informative animations and static figures from saved experiment data as part of each milestone, and include them in the published gallery. Keep the dark navy background, readable pale-blue labels, consistent blue/cyan emphasis, and accessible curve distinctions. Preserve the same run, time coordinates, and spatial scale in temporal comparisons. Show failures and assumptions honestly.

Connect the physical problem, equations, code, observed behavior, and limitations. Keep curated media compact and retain numerical evidence needed for regeneration.

## Continuity
Update PLAN.md and the README with actual progress, evidence links, unresolved issues, and the exact next step. Do not restart completed milestones. Inspect current evidence before marking a milestone complete; distinguish reported results from independently rerun checks.

For the user's local workflow, use the actual Linux checkout under ~/actir in a new VS Code WSL window, preserve other windows, and clone only when absent. When a task prompt is requested, give one complete inline prompt rather than a bundle of manual file-creation steps.

## End-of-task report
State what changed, relevant verification, the published branch and commit link, the visualization/result links, and any remaining blocker. Distinguish published, committed-but-local, and uncommitted work. For metadata changes, state whether GitHub's About panel was actually verified.
