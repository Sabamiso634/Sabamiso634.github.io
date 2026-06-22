# Benchmark conditions: run0001_20260623_0325_connected

## Run identity

- Run ID: `1`
- Execution timestamp: `2026-06-23 03:23-03:25 JST`
- Working directory: `/home/yamazakilab/NRP_backup/CBCT-NRP`
- Executed binary: `./main-measured-nvidia_cuda_11_5_2-devel-ubuntu20_04`
- Executed binary SHA256: `ce70b01f770cf66dbff124bea666ed5f52d876b854322b12634416e8406e6dfc`

## Command

```bash
cd /home/yamazakilab/NRP_backup/CBCT-NRP
export LD_LIBRARY_PATH=/home/yamazakilab/NRP_backup/CppRosBridge_demo/build/src:$LD_LIBRARY_PATH
RUN_ID=1 CUDA_VISIBLE_DEVICES=0 ./main-measured-nvidia_cuda_11_5_2-devel-ubuntu20_04
```

## GPU and CUDA

- GPU used for CBCT: Tesla V100-PCIE-16GB
- NVIDIA driver: `535.309.01`
- CUDA compiler: CUDA 11.5, `nvcc V11.5.119`

See `gpu_info.csv` and `cuda_nvcc_version.txt`.

## NRP/Gazebo state

This run was connected to an active NRP/Gazebo muscle-interface experiment.

- `/gazebo_muscle_interface/robot/muscle_states` was present.
- `/gazebo_muscle_interface/robot/*/cmd_activation` topics were present.
- `/clock` was published by `/gazebo`.
- NRP/CLE status sample reported `"state": "started"`.
- `foot1_path_points.csv` contains non-zero body-position values for all 201 samples.

## Realtime result

From `run0001_realtime_summary.csv`:

- Simulated time: `10050.000000 ms`
- Wall-clock time: `9377.824530 ms`
- Simulation / wall-clock ratio: `1.071677`
- 50 ms loop blocks: `201`
- Mean loop latency: `46.625816 ms`
- Max loop latency: `58.290863 ms`
- Deadline misses: `27`

## Body-position feedback

From `foot1_path_points_summary.csv`:

- Rows: `201`
- Non-zero rows: `201`
- Minimum `foot1_path_points`: `-0.085478`
- Maximum `foot1_path_points`: `-0.069505`

