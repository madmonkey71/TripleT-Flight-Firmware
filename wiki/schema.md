---
title: Wiki Schema & Conventions
type: overview
tags: [schema, conventions, wiki]
created: 2026-04-15
updated: 2026-04-15
---

Conventions for maintaining this wiki.

## Frontmatter

Every page requires:
```yaml
---
title: Human-readable title
type: concept | entity | query | overview
tags: [tag1, tag2]
created: YYYY-MM-DD
updated: YYYY-MM-DD
related_files: [src/path/file.cpp, src/path/file.h]
---
```

## Cross-References

- Wiki pages: `[[page-name]]` or `[[folder/page-name]]`
- Source code: `src/path/file.cpp:42` (file:line)
- Config parameters: backtick code style, e.g., `BOOST_ACCEL_THRESHOLD`

## Page Types

| Type | Location | Purpose |
|------|----------|---------|
| `overview` | `wiki/` root | Architecture overview, schema |
| `concept` | `wiki/concepts/` | Architectural patterns, algorithms, flows |
| `entity` | `wiki/entities/` | Specific modules, files, classes |
| `query` | `wiki/queries/` | Filed answers to interesting questions |

## Source of Truth

**Source code always wins over wiki.** When they disagree, update the wiki — never rationalize the wiki as correct.

## Drift Detection

Before editing a wiki page, verify that referenced file paths still exist and referenced functions/symbols are still present. Use `grep` or `Glob` to confirm.

## Update Protocol

1. Identify changed files via `git diff`
2. Classify: structural (new module, renamed API, changed flow) vs trivial (formatting, comments)
3. Update affected entity/concept pages for structural changes only
4. Update `overview.md` if architecture shifted
5. Append to `log.md`
6. Update `index.md` if pages added/removed
