# OnlyFeet IMWUT Dataset Paper Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clean up the git repo and restructure the OnlyFeet Overleaf paper as an IMWUT dataset paper.

**Architecture:** Two independent workstreams: (1) remove main.tex from git, (2) update Overleaf via MCP. Overleaf changes are done section by section using mcp__overleaf__edit_file and mcp__overleaf__update_section. Sections 2 and 3 of the paper are kept as-is. All other sections are rewritten or replaced.

**Tech Stack:** Git, Overleaf MCP (mcp__overleaf__edit_file, mcp__overleaf__update_section, mcp__overleaf__get_sections, mcp__overleaf__read_file)

**Style constraint:** No em dashes in any added LaTeX text.

---

## Task 1: Remove main.tex from git repo

**Files:**
- Delete: `main.tex` (root of repo)

- [ ] **Step 1: Remove the file from git tracking and delete it**

```bash
git rm main.tex
```

Expected output: `rm 'main.tex'`

- [ ] **Step 2: Commit**

```bash
git commit -m "Remove main.tex (paper maintained in Overleaf)"
```

---

## Task 2: Update Overleaf preamble (document class, metadata, CCS, keywords, title)

**Files:**
- Modify: `main.tex` in Overleaf (preamble, before `\begin{document}`)

- [ ] **Step 1: Read current preamble to confirm exact text**

Use `mcp__overleaf__read_file` with `file_path: "main.tex"` and confirm the lines to replace.

- [ ] **Step 2: Replace document class line**

Use `mcp__overleaf__edit_file`. Replace:

```latex
\documentclass[manuscript,anonymous]{acmart} 
```

With:

```latex
\documentclass[acmsmall,manuscript,anonymous]{acmart}
```

- [ ] **Step 3: Replace venue metadata block**

Replace the entire conference metadata block:

```latex
\setcopyright{acmcopyright}
\acmDOI{XXXXXXX.XXXXXXX}
\acmISBN{978-1-4503-XXXX-X/25/11}
\acmConference[VRST '25]{ACM Symposium on Virtual Reality Software and Technology}{Nov 2025}{City, Country}

\acmBooktitle{VRST '25: ACM Symposium on Virtual Reality Software and Technology, Nov 2025, City, Country}
```

With:

```latex
\setcopyright{acmcopyright}
\acmJournal{IMWUT}
\acmVolume{XX}
\acmNumber{XX}
\acmArticle{XX}
\acmYear{2026}
\acmMonth{XX}
```

- [ ] **Step 4: Replace title**

Replace:

```latex
\title{\name: A Low-Cost Foot Interface for AI labeling}
```

With:

```latex
\title{\name: A Multimodal Foot-Worn Sensor Dataset for Activity Recognition and Surface Classification}
```

- [ ] **Step 5: Replace CCS concepts block**

Replace:

```latex
\ccsdesc[300]{Human-centered computing~Haptic devices}
\keywords{Foot-based haptics, Thermal feedback, Vibrotactile feedback, Virtual reality, Wearable computing}
```

With:

```latex
\ccsdesc[500]{Human-centered computing~Ubiquitous and mobile computing}
\ccsdesc[300]{Human-centered computing~Activity recognition}
\ccsdesc[300]{Computing methodologies~Sensor fusion}
\keywords{Foot-worn sensing, Multimodal dataset, Activity recognition, Surface classification, Wearable computing, IMU, ESP32}
```

---

## Task 3: Rewrite Abstract

**Files:**
- Modify: `main.tex` in Overleaf (abstract environment)

- [ ] **Step 1: Replace the abstract**

Use `mcp__overleaf__edit_file`. Replace the entire `\begin{abstract}...\end{abstract}` block with:

```latex
\begin{abstract}
We present the \textit{OnlyFeet} dataset, a multimodal foot-worn sensor dataset collected from ten participants walking a defined parcours across seven surface types while performing seven locomotion activities. Data were recorded with the \textit{OnlyFeet} platform, a compact and low-cost logging unit based on the Seeed XIAO ESP32S3 Sense microcontroller. The platform integrates a camera, a digital microphone, a nine-axis inertial measurement unit (IMU), and a time-of-flight (ToF) distance sensor in a 3D-printed shoe-mounted enclosure. Each one-second window produces a synchronized set of one JPEG image, one WAV audio snippet, approximately one hundred IMU samples, and up to twenty time-of-flight range readings. Every packet is annotated with two ground truth labels: surface type and locomotion activity. The firmware is implemented in the Arduino framework using FreeRTOS for deterministic scheduling and writes data locally to a micro SD card. Average current consumption is approximately 200 mA, yielding five to six hours of continuous recording from a 1200 mAh cell. The dataset is publicly available together with the platform firmware and a browser-based data viewer.
\end{abstract}
```

---

## Task 4: Rewrite Introduction

**Files:**
- Modify: `main.tex` in Overleaf (Section 1)

- [ ] **Step 1: Replace the Introduction section**

Use `mcp__overleaf__update_section` with the section title `Introduction`. Replace with:

```latex
\section{Introduction}
Wearable sensing research depends on the availability of labelled multimodal datasets collected under realistic conditions. In foot-worn sensing, inertial measurement units are the dominant modality, and most existing datasets cover only a narrow range of surface types or activity classes \cite{bamberg2008gait,patel2012survey,delDin2016wearable}. No publicly available dataset currently combines synchronized inertial, visual, acoustic, and ranging data from a shoe-mounted platform with ground truth labels for both surface type and locomotion activity.

We present the \textit{OnlyFeet} dataset to address this gap. The dataset was collected with the \textit{OnlyFeet} platform, a compact, low-cost recording unit based on the Seeed XIAO ESP32S3 Sense that integrates a camera, a digital microphone, a nine-axis IMU, and a time-of-flight distance sensor in a single shoe-mounted enclosure. Ten participants walked a defined parcours covering seven surface types and performed seven locomotion activities. Each one-second packet provides a synchronized JPEG image, WAV audio snippet, approximately one hundred IMU samples, and time-of-flight range readings, together with surface and activity ground truth labels.

The significance of such a dataset becomes clear when considering the limitations of existing resources. Activity recognition models trained on inertial data alone degrade substantially across surface types, walking speeds, and footwear changes \cite{stisen2015weardrive,janidarmian2017heterogeneity}. Egocentric video datasets such as EPIC-KITCHENS \cite{damen2022epic100} and Ego4D \cite{grauman2022ego4d} expose the limits of vision-only approaches under occlusion and privacy constraints. Multimodal datasets such as WEAR \cite{bock2024wear} and MMAct \cite{kong2019mmact} improve robustness but do not target foot-worn sensing or systematic surface variation. \textit{OnlyFeet} fills this gap by providing a foot-mounted perspective with controlled surface and activity annotations and open hardware that can be reproduced by other labs.

\paragraph*{Research Questions.}
\begin{description}
  \item[\textbf{RQ1}] Does a low-cost ESP32-based platform produce sufficiently reliable synchronized multimodal data for use as a research dataset?
  \item[\textbf{RQ2}] Do the recorded modalities provide complementary discriminative information about surface type and locomotion activity beyond what a single modality achieves?
\end{description}

\paragraph*{Contributions.}
\begin{itemize}
  \item A \textbf{hardware platform}: a compact and cost-efficient ESP32-S3-based logger integrating camera, microphone, IMU, and ToF sensing in a shoe-mounted enclosure, with open firmware and 3D-printable housing files.
  \item A \textbf{labelled multimodal dataset}: synchronized foot-worn sensor recordings from ten participants across seven surface types and seven locomotion activities, annotated at one-second resolution with surface and activity labels, released as open data.
  \item A \textbf{data viewer}: a browser-based tool for inspecting and replaying recorded sessions with synchronized visualizations of all sensor channels.
\end{itemize}
```

---

## Task 5: Add label to System Overview section

**Files:**
- Modify: `main.tex` in Overleaf (Section 3 header)

- [ ] **Step 1: Add a label to the System Overview section**

Use `mcp__overleaf__edit_file`. Find:

```latex
\section{System Overview}
```

Replace with:

```latex
\section{System Overview}
\label{sec:system}
```

---

## Task 6: Remove pilot section and add Dataset Collection Protocol

**Files:**
- Modify: `main.tex` in Overleaf (remove "Sensor analyses" section, add new Section 4)

- [ ] **Step 1: Identify the pilot section boundaries**

Use `mcp__overleaf__get_sections` to list all sections and confirm the title of the section to remove ("Sensor analyses").

- [ ] **Step 2: Replace the entire pilot section with the new Dataset Collection Protocol section**

Use `mcp__overleaf__update_section` with section title `Sensor analyses`. Replace with:

```latex
\section{Dataset Collection Protocol}
\label{sec:protocol}
This section describes the study design, apparatus, and procedure used to collect the \textit{OnlyFeet} dataset.

\subsection{Participants}
Ten participants took part in the data collection ($N = 10$). [PLACEHOLDER: demographics, age range, gender distribution.] Participation was voluntary. [PLACEHOLDER: ethics approval statement.]

\subsection{Apparatus}
The \textit{OnlyFeet} device was mounted on the right shoe of each participant using the 3D printed shoelace mount described in Section~\ref{sec:system}. Participants wore their own footwear; shoe type was not controlled. A fresh micro SD card was formatted before each session.

\subsection{Parcours Design}
The parcours covered seven surface types arranged as a continuous route:

\begin{enumerate}
  \item Asphalt
  \item Sand
  \item Grass
  \item Paving slabs (\textit{Steinplatten})
  \item Coarse stones (\textit{Grobe Steine})
  \item Compacted sand path (\textit{Verdichteter Sandweg})
  \item Tiles (\textit{Fliesen}, indoor)
\end{enumerate}

The route started outdoors and transitioned to an indoor section at the end, where the tile surface was covered inside a building. Each surface segment was long enough to capture multiple complete gait cycles. [PLACEHOLDER: approximate segment length in metres or steps.]

\subsection{Activities}
On each surface segment, participants performed a defined set of locomotion activities:

\begin{enumerate}
  \item Normal walking
  \item Fast walking
  \item Running
  \item Ascending stairs
  \item Descending stairs
  \item Standing still
  \item Sitting
\end{enumerate}

Stair activities were performed only where stairs were available on the parcours. [PLACEHOLDER: specify which activities were performed on which surfaces if not all combinations were recorded.]

\subsection{Procedure}
Each participant completed one continuous session covering the full parcours. The experimenter mounted the device on the participant's right shoe and started the recording. After the built-in 8-second startup delay the participant began walking. The experimenter accompanied the participant and operated the annotation timer throughout the session. [PLACEHOLDER: mean session duration.]

\subsection{Annotation}
Ground truth labels were recorded manually using timestamps during the session. The annotator logged the start time of each surface and activity transition using [PLACEHOLDER: annotation tool or method]. After the session, each one-second packet received two labels derived from the timestamp mapping:

\begin{itemize}
  \item \texttt{surface\_label}: one of the seven surface classes listed above.
  \item \texttt{activity\_label}: one of the seven activity classes listed above.
\end{itemize}

Labels are stored in a per-session annotation file (CSV) with columns for session identifier, packet index, surface label, and activity label. The annotation file can be joined with the sensor data files by packet index.
```

---

## Task 7: Add Dataset Description section

**Files:**
- Modify: `main.tex` in Overleaf (insert new section after Dataset Collection Protocol, before Discussion)

- [ ] **Step 1: Read the current file to find the insertion point**

Use `mcp__overleaf__read_file` and locate the `\section{Discussion}` line.

- [ ] **Step 2: Insert new section before Discussion**

Use `mcp__overleaf__edit_file`. Find:

```latex
% ---------- 7 Discussion ----------
\section{Discussion}
```

Replace with:

```latex
\section{Dataset Description}
\label{sec:dataset}

\subsection{Data Format}
The dataset follows the \textit{OnlyFeet} packet format. Each one-second window produces three files: a JSON packet (\texttt{pkt\_\{index\}.json}), a JPEG image (\texttt{img\_\{index\}\_\{ts\}.jpg}), and a WAV audio file (\texttt{rec\_\{index\}\_\{ts\}.wav}). Files are organized in per-session directories named \texttt{/dataN}. The JSON schema is described in Section~\ref{sec:system}. Annotation labels are provided as a separate CSV file with columns for session identifier, packet index, surface label, and activity label, joinable by packet index.

\subsection{Dataset Statistics}
[PLACEHOLDER: total number of participants, sessions, and packets after quality filtering]\\
[PLACEHOLDER: total recording duration in hours]\\
[PLACEHOLDER: class distribution per surface label as table or figure]\\
[PLACEHOLDER: class distribution per activity label as table or figure]\\
[PLACEHOLDER: total dataset size on disk in GB]

\subsection{Data Availability}
The dataset is publicly available at [PLACEHOLDER: repository URL or DOI]. The firmware source code, 3D-printable housing files, and the browser-based data viewer are available at \url{https://github.com/susesKaninchen/OnlyFeet}.

% ---------- 7 Discussion ----------
\section{Discussion}
```

---

## Task 8: Update Discussion

**Files:**
- Modify: `main.tex` in Overleaf (Section: Discussion)

- [ ] **Step 1: Replace the Discussion section**

Use `mcp__overleaf__update_section` with section title `Discussion`. Replace with:

```latex
\section{Discussion}

\subsection{Key Findings}

\begin{itemize}
  \item \textbf{Synchronized multimodal capture is feasible with compact hardware.}
        The FreeRTOS design sustained the intended schedule of one-second packet construction while the IMU sampled at approximately one hundred hertz and the time-of-flight sensor operated continuously. Audio recorded continuous one-second chunks without gaps and the writer stored JSON, WAV, and JPEG files for each window on the micro SD card. All ten recording sessions completed without packet loss.

  \item \textbf{Each modality adds complementary evidence about surface and activity context.}
        The IMU provided reliable step timing and gait segmentation. Camera frames supplied scene and surface cues. The time-of-flight sensor revealed curbs, stairs, and free space ahead of the foot. The audio track captured footfall signatures and reverberation patterns that reflected surface material and room size. Together these signals provide a redundant description of locomotion context that is stronger than any single channel.

  \item \textbf{Local storage simplifies deployment and improves robustness.}
        Writing packets to a session directory on the micro SD card avoided dependence on a wireless link or a host computer. The simple file naming convention made post-processing straightforward. USB serial remained available for live monitoring without affecting the sampling cadence.

  \item \textbf{Power consumption is compatible with multi-session recordings.}
        Bench measurements with a Power Profiler Kit II indicated an average current of approximately 200 mA under the default configuration. A 1200 mAh cell therefore yields roughly five to six hours of continuous logging, which is sufficient for full-day field studies with intermediate charging breaks.
\end{itemize}

\subsection{Limitations and Future Work}

\paragraph{Dataset scale and diversity.}
The current dataset covers ten participants and seven surface types. Larger collections with more participants, more diverse footwear, and additional environmental conditions such as wet surfaces, slopes, and outdoor stairs will be needed to evaluate cross-participant and cross-environment generalization.

\paragraph{Visual occlusion and motion blur.}
A foot-mounted camera can be occluded by the leg during stance and may blur during fast swing phases. Future revisions will explore exposure control and optical flow in post-processing. Alternate shoe placements may also be evaluated.

\paragraph{Magnetometer robustness.}
The magnetometer is sensitive to hard-iron and soft-iron effects from the battery, wiring, and shoe hardware. A calibration routine and compensation model will be added in future firmware versions.

\paragraph{Ranging edge cases.}
The time-of-flight sensor can return degraded readings on very dark, highly absorbent, or very reflective floors. Multi-zone ranging and adaptive timing budgets are promising mitigations.

\paragraph{Audio artifacts.}
Wind, clothing contact, and cable vibration can contaminate the microphone signal. Mechanical damping inside the enclosure and simple spectral gating will be applied in future versions.

\paragraph{Baseline results.}
The current release provides raw annotated data. A follow-up study will report baseline surface and activity classification results to provide a reference for comparison by other researchers.

\paragraph{Absolute timing and multi-device synchronization.}
Timestamps are derived from the device clock since boot. For multi-device studies a small real-time clock or a periodic alignment pulse will be added to provide absolute time references.
```

---

## Task 9: Rewrite Conclusion

**Files:**
- Modify: `main.tex` in Overleaf (Section: Conclusion)

- [ ] **Step 1: Replace the Conclusion section**

Use `mcp__overleaf__update_section` with section title `Conclusion`. Replace with:

```latex
\section{Conclusion}
We presented the \textit{OnlyFeet} dataset, a multimodal foot-worn sensor dataset collected from ten participants walking a defined parcours across seven surface types while performing seven locomotion activities. Data were recorded with the \textit{OnlyFeet} platform, a compact and low-cost ESP32-S3-based logger that synchronizes a camera, a digital microphone, a nine-axis IMU, and a time-of-flight distance sensor in a shoe-mounted enclosure. Each one-second packet provides a JPEG image, a WAV audio snippet, approximately one hundred IMU samples, and time-of-flight range readings, together with surface and activity ground truth labels.

All ten recording sessions completed without packet loss and with consistent inter-packet timing. Each modality contributes complementary information about locomotion context and surface properties. The dataset is publicly available together with the open firmware source code and a browser-based data viewer.

Future work includes expanding the dataset with more participants and environments, reporting baseline classification results across modality combinations, and evaluating on-device inference for rapid feedback during field work.
```

---

## Task 10: Final commit of all Overleaf changes

Overleaf auto-saves. No explicit commit is needed in Overleaf itself.

- [ ] **Step 1: Read main.tex one final time and verify all sections are present**

Use `mcp__overleaf__get_sections` and confirm the section list matches:

1. Introduction
2. Related Work
3. System Overview (unchanged)
4. Dataset Collection Protocol
5. Dataset Description
6. Discussion
7. Conclusion

- [ ] **Step 2: Verify preamble**

Use `mcp__overleaf__read_file` and confirm `\acmJournal{IMWUT}` is present, `\acmConference` is gone, and the CCS block uses the new concepts.

- [ ] **Step 3: Commit the git repo cleanup**

```bash
git status
git add docs/superpowers/plans/2026-04-16-onlyfeet-imwut-dataset-paper.md
git commit -m "Add IMWUT dataset paper implementation plan"
```
