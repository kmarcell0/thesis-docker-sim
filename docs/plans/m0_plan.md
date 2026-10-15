# M0 plan: Housekeeping

M0 needs no approved milestone plan (`CLAUDE.md`); this file only logs the
briefs and reports of its steps. Notes are short bullets.

Status: log (no approval needed)

---

## M0.1 Add the docs and correct the stale ones

**Goal:** new docs in the repository, stale statements corrected, `ws/data/`
ignored, `CONTEXT.md` settled, everything pushed.
**Done when:** `git fetch` + `git status` up to date with GitHub;
`git check-ignore -v ws/data/test.bag` names the new `.gitignore` line;
`grep -rn "already exists" ws/src/asv_sim/docs/milestone1_plan.md` finds nothing.
**Decisions needed:** none (D-numbers); two "ask first" items (below).

### Brief (agreed on 2026-10-15, short form)
- State at start: eeb6c81 already added `CLAUDE.md`, `docs/`, the corrected
  `milestone1_plan.md`; not pushed. Missing: `.gitignore` line, `CONTEXT.md`,
  this log, the `[x]` mark.
- Marcell's answers: remove `CONTEXT.md`; a second `M0.1:` commit (no amend);
  add `ws/data/` to `.gitignore` and push; nothing else unpushed or untracked
  on his machine.
- Files: ⚠ `.gitignore`, ⚠ `CONTEXT.md` (delete), `docs/plans/m0_plan.md` (new),
  `docs/master_plan.md` (mark), `CLAUDE.md` (CONTEXT.md paragraph, Known state,
  layout note).
- Check: the three "done when" commands; Python tests still 28 OK.

### Report (2026-10-15)
Commits: `git log --grep "^M0.1:"` (eeb6c81 docs, plus the second M0.1 commit).

1. What changed
   - `.gitignore`: new line `ws/data/`.
   - `CONTEXT.md`: removed (`git rm`).
   - `CLAUDE.md`: CONTEXT.md paragraph now says it was removed; Known-state
     bullet about `ws/data/` dropped; layout note says "git-ignored".
   - `docs/master_plan.md`: M0.1 marked `[x]`.
   - `docs/plans/m0_plan.md`: this log.
2. How it was checked
   - `git check-ignore -v ws/data/test.bag` → `.gitignore:8:ws/data/	ws/data/test.bag`
   - `grep -rn "already exists" ws/src/asv_sim/docs/milestone1_plan.md` → no output (rc 1)
   - `python3 -m unittest discover -s tests` (in container) → `Ran 28 tests ... OK`
   - `colcon test`: not run, `asv_sim` has no C++ tests yet (baseline unchanged).
   - Push and `git status` after `git fetch`: see the chat summary of this step
     (done after this file is committed).
3. What could not be checked: that nothing is left on Marcell's machine
   (he confirmed it, 2026-10-15).
4. How to read the code: no code in this step.
5. Numbers for the thesis: none.
6. Check your understanding
   - Why does `ws/data/` have to be ignored before the first bag is recorded?
   - Which files are now the source of truth instead of `CONTEXT.md`?

### Marcell's notes
Written by Marcell only; Claude never edits this part.
