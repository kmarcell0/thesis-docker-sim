# MN plan: <milestone name>

Copy this file to `mN_plan.md` at the start of a milestone. One section per step
from `docs/master_plan.md`. Marcell approves this plan before the first step
starts. Each step's brief and report are saved here, so this file becomes the
logbook of the milestone. Notes are short bullets and equations, not finished
paragraphs.

Status: draft / approved on <date>

---

## MN.1 <step name>

**Goal:** one sentence.
**Done when:** copied from the master plan, made concrete.
**Decisions needed:** D-numbers, or none.

### Brief (agreed on <date>)
Leave out items that would only say "none". Short form (no new idea, no
equation): goal, open decisions, files (with ⚠ marks), how we check it, in about
five lines.

1. Goal
2. Decisions needed (options, recommendation; Marcell's choice once made)
3. Why (thesis section by title)
4. New ideas (at most three, plain words, glossary links)
5. What will change (files; ⚠ for anything on the "Ask first" list)
6. How it works (words first, then equations with the glossary's symbols)
7. How we will check it (tests, commands, expected numbers)
8. What I can't check

### Report (<date>)
Commit: the one whose message starts with `MN.1:` (`git log --grep "^MN.1:"`).

1. What changed
2. How it was checked (commands and the relevant lines of actual output)
3. What could not be checked
4. How to read the code
5. Numbers for the thesis (with source tags)
6. Check your understanding (2-3 questions)

### Marcell's notes
Written by Marcell only; Claude never edits this part.

---

## MN.2 <step name>
...
