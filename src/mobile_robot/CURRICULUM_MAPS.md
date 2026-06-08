# Retraining Curriculum Maps

This is the clean retraining set. The SAC algorithm, observation size, and action size stay unchanged. The worlds only change the Gazebo layout and the environment presets for safe robot/goal/obstacle sampling.

Use these defaults for retraining unless you are intentionally testing a different policy speed:

- `--max-lidar-range 4.0`
- `--max-linear-velocity 0.25`
- `--max-angular-velocity 1.20`
- `--collision-distance 0.18`
- `--near-obstacle-distance 0.30` to `0.35`
- `--goal-threshold 0.22`
- `--min-goal-distance 1.0`

For training speed, launch Gazebo headless with `gui:=false`. Open the GUI only for short visual checks, because rendering Gazebo and the LiDAR fan can make SAC training much slower.

The training script does not show the green Gazebo goal marker by default. Add `--show-goal-marker` only for debugging, because moving the visual marker every reset adds Gazebo service calls.

Real static boxes use `0.25 x 0.15 x 0.50 m`. Circular stages use `arena_high_resolution`, radius about `2.0 m`, with collision geometry visible to LiDAR.

## World List

| Stage | World | Curriculum map | Purpose |
| --- | --- | --- | --- |
| 0 | `retrain_stage0_empty.world` | `retrain_stage0_empty` | Re-learn stable goal reaching without obstacles. |
| 1 | `retrain_stage1_one_box_easy.world` | `retrain_stage1_one_box_easy` | One small real-size box, goal usually visible. |
| 2 | `retrain_stage2_two_boxes_easy.world` | `retrain_stage2_two_boxes_easy` | Two boxes, wide routes, low hidden-goal probability. |
| 3 | `retrain_stage3_detour_wall.world` | `retrain_stage3_detour_wall` | One wide detour wall plus boxes, no maze traps. |
| 4 | `retrain_stage4_circular_one_box.world` | `retrain_stage4_circular_one_box` | Adapt to the circular arena with one box. |
| 5 | `retrain_stage5_circular_random_boxes.world` | `retrain_stage5_circular_random_boxes` | Circular arena with randomized real boxes and yaw. |
| 6 | `retrain_stage6_circular_final.world` | `retrain_stage6_circular_final` | Final circular static-box layout, optional randomization. |

## Stage 0

Stage 0 is only for teaching basic approach behavior. It intentionally starts
near the arena center, uses short robot-goal distances, points the robot roughly
toward the goal at reset, and uses a forgiving success radius. After success is
reliable, run a short continuation with `--goal-threshold 0.35`, then later use
`--goal-threshold 0.22` for stricter stages.

Launch:

```bash
ros2 launch mobile_robot gazebo_model.launch.py world:=retrain_stage0_empty.world gui:=false verbose:=false
```

Train:

```bash
python3 -m mobile_robot.rl.train_sac \
    --total-steps 50000 \
    --checkpoint-freq 10000 \
    --learning-starts 3000 \
    --batch-size 128 \
    --train-freq 2 \
    --gradient-steps 1 \
    --max-episode-steps 1000 \
    --stuck-check-after-steps 700 \
    --stuck-window-steps 300 \
    --stuck-min-progress 0.0 \
    --min-goal-distance 0.8 \
    --goal-threshold 0.55 \
    --near-goal-distance 0.70 \
    --curriculum-map retrain_stage0_empty \
    --world-name retrain_stage0_empty.world \
    --hidden-goal-probability 0.0 \
    --max-lidar-range 4.0 \
    --max-linear-velocity 0.25 \
    --max-angular-velocity 1.20
```

Move on when success is high and STUCK/COLLISION are rare.

## Stage 1

Launch:

```bash
ros2 launch mobile_robot gazebo_model.launch.py world:=retrain_stage1_one_box_easy.world
```

Train from the best Stage 0 checkpoint:

```bash
python3 -m mobile_robot.rl.train_sac \
    --load-model /home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/best_model.zip \
    --total-steps 150000 \
    --checkpoint-freq 10000 \
    --max-episode-steps 850 \
    --stuck-check-after-steps 250 \
    --stuck-window-steps 180 \
    --stuck-min-progress 0.03 \
    --min-goal-distance 1.0 \
    --curriculum-map retrain_stage1_one_box_easy \
    --world-name retrain_stage1_one_box_easy.world \
    --hidden-goal-probability 0.0 \
    --collision-distance 0.18 \
    --near-obstacle-distance 0.30 \
    --max-lidar-range 4.0 \
    --max-linear-velocity 0.25 \
    --max-angular-velocity 1.20
```

## Stage 2

Launch:

```bash
ros2 launch mobile_robot gazebo_model.launch.py world:=retrain_stage2_two_boxes_easy.world
```

Train:

```bash
python3 -m mobile_robot.rl.train_sac \
    --load-model /home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/best_model.zip \
    --total-steps 200000 \
    --checkpoint-freq 10000 \
    --max-episode-steps 950 \
    --stuck-check-after-steps 270 \
    --stuck-window-steps 200 \
    --stuck-min-progress 0.03 \
    --min-goal-distance 1.0 \
    --curriculum-map retrain_stage2_two_boxes_easy \
    --world-name retrain_stage2_two_boxes_easy.world \
    --hidden-goal-probability 0.05 \
    --collision-distance 0.18 \
    --near-obstacle-distance 0.32 \
    --max-lidar-range 4.0 \
    --max-linear-velocity 0.25 \
    --max-angular-velocity 1.20
```

## Stage 3

Launch:

```bash
ros2 launch mobile_robot gazebo_model.launch.py world:=retrain_stage3_detour_wall.world
```

Train:

```bash
python3 -m mobile_robot.rl.train_sac \
    --load-model /home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/best_model.zip \
    --total-steps 250000 \
    --checkpoint-freq 10000 \
    --max-episode-steps 1100 \
    --stuck-check-after-steps 300 \
    --stuck-window-steps 220 \
    --stuck-min-progress 0.03 \
    --min-goal-distance 1.0 \
    --curriculum-map retrain_stage3_detour_wall \
    --world-name retrain_stage3_detour_wall.world \
    --hidden-goal-probability 0.15 \
    --collision-distance 0.18 \
    --near-obstacle-distance 0.32 \
    --max-lidar-range 4.0 \
    --max-linear-velocity 0.25 \
    --max-angular-velocity 1.20
```

## Stage 4

Launch:

```bash
ros2 launch mobile_robot gazebo_model.launch.py world:=retrain_stage4_circular_one_box.world
```

Train checkpoint-only first when changing from rectangular to circular:

```bash
python3 -m mobile_robot.rl.train_sac \
    --load-model /home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/best_model.zip \
    --total-steps 250000 \
    --checkpoint-freq 10000 \
    --max-episode-steps 1100 \
    --stuck-check-after-steps 300 \
    --stuck-window-steps 220 \
    --stuck-min-progress 0.03 \
    --min-goal-distance 1.0 \
    --curriculum-map retrain_stage4_circular_one_box \
    --world-name retrain_stage4_circular_one_box.world \
    --hidden-goal-probability 0.20 \
    --collision-distance 0.18 \
    --near-obstacle-distance 0.32 \
    --max-lidar-range 4.0 \
    --max-linear-velocity 0.25 \
    --max-angular-velocity 1.20
```

## Stage 5

Launch:

```bash
ros2 launch mobile_robot gazebo_model.launch.py world:=retrain_stage5_circular_random_boxes.world
```

Train with one randomized box first:

```bash
python3 -m mobile_robot.rl.train_sac \
    --load-model /home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/best_model.zip \
    --total-steps 350000 \
    --checkpoint-freq 10000 \
    --max-episode-steps 1200 \
    --stuck-check-after-steps 330 \
    --stuck-window-steps 240 \
    --stuck-min-progress 0.025 \
    --min-goal-distance 1.0 \
    --curriculum-map retrain_stage5_circular_random_boxes \
    --world-name retrain_stage5_circular_random_boxes.world \
    --hidden-goal-probability 0.30 \
    --randomize-obstacles \
    --obstacle-randomization-frequency every_10_episodes \
    --num-random-obstacles 1 \
    --randomize-obstacle-yaw \
    --collision-distance 0.18 \
    --near-obstacle-distance 0.35 \
    --max-lidar-range 4.0 \
    --max-linear-velocity 0.25 \
    --max-angular-velocity 1.20
```

After success is reliable, increase `--num-random-obstacles` to `2`, then `3`. Do not jump to five random boxes immediately.

## Stage 6

Launch:

```bash
ros2 launch mobile_robot gazebo_model.launch.py world:=retrain_stage6_circular_final.world
```

Train:

```bash
python3 -m mobile_robot.rl.train_sac \
    --load-model /home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/best_model.zip \
    --total-steps 400000 \
    --checkpoint-freq 10000 \
    --max-episode-steps 1300 \
    --stuck-check-after-steps 350 \
    --stuck-window-steps 260 \
    --stuck-min-progress 0.025 \
    --min-goal-distance 1.0 \
    --curriculum-map retrain_stage6_circular_final \
    --world-name retrain_stage6_circular_final.world \
    --hidden-goal-probability 0.40 \
    --collision-distance 0.18 \
    --near-obstacle-distance 0.35 \
    --max-lidar-range 4.0 \
    --max-linear-velocity 0.25 \
    --max-angular-velocity 1.20
```

Optional harder Stage 6 after the fixed layout works:

```bash
python3 -m mobile_robot.rl.train_sac \
    --load-model /home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/best_model.zip \
    --total-steps 300000 \
    --checkpoint-freq 10000 \
    --max-episode-steps 1300 \
    --stuck-check-after-steps 350 \
    --stuck-window-steps 260 \
    --stuck-min-progress 0.025 \
    --min-goal-distance 1.0 \
    --curriculum-map retrain_stage6_circular_final \
    --world-name retrain_stage6_circular_final.world \
    --hidden-goal-probability 0.45 \
    --randomize-obstacles \
    --obstacle-randomization-frequency every_10_episodes \
    --num-random-obstacles 2 \
    --randomize-obstacle-yaw \
    --collision-distance 0.18 \
    --near-obstacle-distance 0.35 \
    --max-lidar-range 4.0 \
    --max-linear-velocity 0.25 \
    --max-angular-velocity 1.20
```

## Replay Buffer Policy

- Same world and same obstacle behavior: checkpoint plus replay buffer is OK.
- New world shape or major obstacle change: checkpoint yes, replay buffer optional.
- Rectangular to circular: checkpoint only at first.
- Randomized obstacles: checkpoint yes, replay buffer only after the randomization settings are stable.

## Move-Up Rule

Move to the next stage when success is high, early collisions are rare, STUCK counts are falling, near-goal spinning is rare, and reset warnings are not repeating. If a stage becomes unstable, lower hidden-goal probability first before changing SAC hyperparameters.
