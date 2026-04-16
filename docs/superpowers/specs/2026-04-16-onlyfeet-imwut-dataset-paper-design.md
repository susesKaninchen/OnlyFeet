# OnlyFeet: IMWUT Dataset Paper Design

Date: 2026-04-16

## Overview

Transform the existing OnlyFeet paper (currently formatted for VRST 2025, ACM SIGCONF) into a dataset paper for submission to ACM IMWUT. The primary contribution is the combination of the hardware platform and the annotated multimodal foot-worn sensor dataset.

---

## 1. Repo Cleanup

Remove from git repository:

- `main.tex` (paper is maintained in Overleaf via MCP, not in the repo)

Keep as-is:

- `data0/`, `data5/`, `data8/` (used for analysis and viewer development)
- `BOOT/` (keep for now)
- `FeetHubTest.mp3` (keep for now)

---

## 2. Overleaf Template Changes

### Document Class

Change from:

```latex
\documentclass[manuscript,anonymous]{acmart}
```

To:

```latex
\documentclass[acmsmall,manuscript,anonymous]{acmart}
```

### Venue Metadata

Remove the VRST conference block entirely. Replace with:

```latex
\acmJournal{IMWUT}
\acmVolume{XX}
\acmNumber{XX}
\acmArticle{XX}
\acmYear{2026}
\acmMonth{XX}
```

### CCS Concepts

Remove current (incorrect, belong to a haptics paper):

```latex
\ccsdesc[300]{Human-centered computing~Haptic devices}
```

Replace with:

```latex
\ccsdesc[500]{Human-centered computing~Ubiquitous and mobile computing}
\ccsdesc[300]{Human-centered computing~Activity recognition}
\ccsdesc[300]{Computing methodologies~Sensor fusion}
```

### Keywords

Remove current (incorrect): Foot-based haptics, Thermal feedback, Vibrotactile feedback, Virtual reality, Wearable computing

Replace with: Foot-worn sensing, Multimodal dataset, Activity recognition, Wearable computing, IMU, ESP32

### Title

Change to:

```
OnlyFeet: A Multimodal Foot-Worn Sensor Dataset for Activity Recognition and Surface Classification
```

---

## 3. Paper Structure

### New Section Order

| Section | Title | Status |
|---------|-------|--------|
| 1 | Introduction | Rewrite: dataset as primary contribution |
| 2 | Related Work | Keep, move Dataset Creation subsection earlier |
| 3 | System Overview (Platform) | Keep almost as-is |
| 4 | Dataset Collection Protocol | NEW |
| 5 | Dataset Description | NEW (placeholders until data collected) |
| 6 | Discussion | Adapt to dataset utility and limitations |
| 7 | Conclusion | Rewrite |

The current "Sensor analyses" / pilot section is removed entirely and replaced by sections 4 and 5.

---

## 4. New Section: Dataset Collection Protocol

### Participants

- 10 participants total
- Shoe type: unconstrained (participants wear their own shoes)
- 1 session per participant
- [PLACEHOLDER: participant demographics, age range, gender distribution]

### Session Structure

Each session is one continuous run. The parcours starts outdoors and transitions to indoors at a defined point. Both outdoor and indoor segments are covered in a single uninterrupted session.

### Parcours and Surfaces

Seven surface types are covered in the parcours:

1. Asphalt
2. Sand
3. Gras
4. Steinplatten (paving slabs)
5. Grobe Steine (coarse stones / rubble)
6. Verdichteter Sandweg (compacted sand path)
7. Fliesen (tiles, indoor)

### Activities

Seven activity types are recorded:

1. Normales Gehen (normal walking pace)
2. Schnelles Gehen (fast walking)
3. Laufen (running)
4. Treppensteigen rauf (ascending stairs)
5. Treppensteigen runter (descending stairs)
6. Stehen (standing)
7. Sitzen (sitting)

### Annotation Method

Annotation is performed manually during recording using timestamps. The annotator logs the start time of each surface/activity segment using a separate timing device or app. After recording, each 1-second packet receives two labels based on the timestamp mapping:

- `surface_label`: one of the 7 surface classes
- `activity_label`: one of the 7 activity classes

Labels are added directly to the dataset (JSON packets or a separate annotation CSV indexed by session and packet index).

---

## 5. New Section: Dataset Description (Placeholders)

The following content is filled in after data collection:

- [PLACEHOLDER: total number of packets across all participants]
- [PLACEHOLDER: total recording duration in hours]
- [PLACEHOLDER: class distribution per surface label]
- [PLACEHOLDER: class distribution per activity label]
- [PLACEHOLDER: data collection period]
- [PLACEHOLDER: dataset archive DOI / download link]

Data format remains the existing OnlyFeet format: `pkt_N.json`, `img_N_ts.jpg`, `rec_N_ts.wav` per 1-second window, organized in per-session directories.

---

## 6. Open Placeholders Summary

Items to fill in before submission:

- Participant demographics (age, gender)
- Total dataset size (hours, packets, GB)
- Class label distribution
- Recording period
- Ethics approval / statement
- Dataset DOI and archive link
- Acknowledgements (funding, participants)
- `\acmVolume`, `\acmNumber`, `\acmArticle`, `\acmMonth` (assigned by ACM after acceptance)
