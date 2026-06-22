# Real-Time Closed-Loop Brain–Body Simulation site

This directory is a static GitHub Pages site for the NEURO2026 poster page.

## Included

- `index.html` and `styles.css`
- bilingual English/Japanese language toggle with `?lang=en` and `?lang=ja`
- poster PDF: `assets/poster.pdf`
- implementation note: `assets/SITE_IMPLEMENTATION_TEXT.md`
- run data archive: `assets/data/run0001_20260623_0325_connected.tar.gz`
- extracted CSV/log files under `assets/data/run0001_20260623_0325_connected/`
- generated SVG charts under `assets/figures/`

## Add videos

Place the two MP4 files here:

```text
assets/nrp-lever-pull.mp4
assets/mujoco-virtual-rodent-gait.mp4
```

Recommended browser-compatible encoding: MP4 H.264 video + AAC audio.

## Run 0001 summary

- simulation time: 10.05 s
- wall-clock time: 9.38 s
- sim/wall: 1.071677
- mean 50-ms block latency: 46.625816 ms
- max 50-ms block latency: 58.290863 ms
- blocks within 50 ms: 174/201 (86.6%)
- deadline misses: 27

## Before publishing

Replace the placeholder footer with:

- contact email or lab page
- GitHub repository URL
- commit hash / release tag for the code used in run0001
- license information
- DOI, preprint, or conference abstract link when available
- video and model attribution for NRP/Gazebo and MuJoCo Virtual Rodent assets
