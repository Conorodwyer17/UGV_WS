## Summary
- Add structured, fail-hard logging and watchdogs to trace detection → goal → motion chain.
- Add optional photo-trigger distance guard + tests.
- Document 2026-02-25 timeline, diagnosis, and log gaps.

## Test plan
- [x] `PYTHONPATH=... python3 -m pytest src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/test/test_photo_trigger.py -q`
- [ ] Run live mission with Aurora connected and capture `/cmd_vel`, `/tf`, and detection topics.
