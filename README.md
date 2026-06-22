# NEURO2026 poster project page

Static GitHub Pages site for the poster PDF and simulation videos.

## Files to add

Place the following MP4 files in `assets/`:

- `assets/nrp-lever-pull.mp4` — NRP/Gazebo mouse forearm musculoskeletal lever-pull model used in the poster-related demo
- `assets/mujoco-virtual-rodent-gait.mp4` — MuJoCo Virtual Rodent walking prototype shown as a future direction

The poster PDF is already placed at:

- `assets/poster.pdf`

## Scope note

The lever-pull video is the demo directly connected to the poster. The MuJoCo Virtual Rodent video is not part of the current poster result. It is included to show the planned direction toward a freely moving virtual rodent validation platform.

The MuJoCo Virtual Rodent prototype described here is not an explicit musculoskeletal model. Virtual muscle-length differences are computed internally and used to estimate joint angles.

## Local preview

Open `index.html` in a browser, or run a local server:

```bash
python3 -m http.server 8000
```

Then open `http://localhost:8000`.

## GitHub Pages

Upload all files to the repository root, then enable GitHub Pages from `Settings` → `Pages` → `Deploy from a branch` → `main` / `root`.

## Video notes

Prefer MP4 encoded with H.264 video. Keep each video short and compressed for GitHub Pages delivery. If either video is too large for the repository, host it externally and replace the video source URL in `index.html`.
