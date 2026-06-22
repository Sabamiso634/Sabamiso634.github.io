# Data manifest: run0001_20260623_0325_connected

This directory contains the latest connected NRP/Gazebo `RUN_ID=1` result from 2026-06-23 03:25 JST.

## Main benchmark files

- `run0001_realtime_summary.csv`  
  One-row realtime summary.

- `run0001_loop_metrics.csv`  
  Per-50 ms loop timing log.

- `m1_rates_50ms.csv`  
  M1 output values sampled/emitted every 50 ms.

- `foot1_path_points.csv`  
  Representative body-position feedback log.

- `foot1_path_points_summary.csv`  
  Summary of row count, non-zero count, minimum, and maximum.

## Additional logs

- `cpg_spike_CSV.csv`
- `cpg_spikes_raster.csv`
- `cpg_spike_count_summary.csv`
- `m1_l5b_pt_spike_count_summary.csv`
- `s1_l4_pyr_spike_count_summary.csv`
- `run0001.csv`  
  Legacy header-only GPU timing placeholder.

## Environment and connection evidence

- `benchmark_conditions.md`
- `gpu_info.csv`
- `cuda_nvcc_version.txt`
- `docker_ps.tsv`
- `docker_images.tsv`
- `ros_topics.txt`
- `ros_nodes.txt`
- `ros_clock_info.txt`
- `ros_muscle_states_info.txt`
- `ros_foot1_cmd_activation_info.txt`
- `ros_cle_status_sample.txt`
- `key_checksums.sha256`
- `file_checksums.sha256`

## Key values

- `wall_ms`: `9377.824530`
- `sim/wall`: `1.071677`
- `loop_latency_mean_ms`: `46.625816`
- `deadline_miss_count`: `27`

