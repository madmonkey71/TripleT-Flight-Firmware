# Documentation Maintenance Protocol

To ensure documentation remains accurate and useful:

1.  **Source of Truth:** `UPDATED_GAP_ANALYSIS_2025.md` is the primary tracking document for high-level features.
2.  **When to Update:**
    *   **Feature Completion:** When a feature listed in "Remaining Gaps" is implemented, move it to "Completed" and mark it with a ✅.
    *   **New Findings:** If a bug or missing requirement is discovered, add it to "Remaining Gaps".
    *   **Architecture Changes:** If the design changes (e.g., switching from Euler to Quaternion), update the relevant section.
3.  **Code & Docs Sync:**
    *   Any PR that implements a feature *must* update `UPDATED_GAP_ANALYSIS_2025.md`.
    *   Function headers and comments in `.cpp` files should reference the design docs if complex logic is involved.

**Verification Checklist:**
- [ ] Does `README.md` reflect the current "Beta" status and major limitations?
- [ ] Are implemented features correctly marked as completed in the Gap Analysis?
- [ ] Are placeholder files (like `esp32_*`) clearly identified as such in the documentation?
