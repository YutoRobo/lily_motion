# v3.0.30 valid-case-only constrained repeated sweep

v3.0.29 introduced constrained parameterization, but it still allowed a
candidate that passed the periodicity gate to remain in the ranking even when
constraint evaluation or repeated-roll generation failed.  v3.0.30 changes the
selection rule:

1. A candidate must pass the periodicity gate.
2. Constraint evaluation must complete without an `error` field.
3. Repeated roll generation/evaluation must complete without an `error` field.
4. `repeated_roll.candidate_completed` must be true.
5. Sentinel penalty values such as `999.0` and `999999` are treated as invalid,
   not as merely poor scores.

The `best-command-output` now writes the valid repeated-roll command sequence,
not the single quarter-roll sequence.  If no valid candidate exists,
`best_case` is `null`, `valid_case_count` is zero, and no best command is
written.
