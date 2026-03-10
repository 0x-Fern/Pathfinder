# Pathfinder Demo Grids

These grids are intended for terminal demos with `./bin/pathfinder`.

Recommended runs:

```bash
./bin/pathfinder pathfinder/examples/grid_basic.txt --animate --delay-ms 120
./bin/pathfinder pathfinder/examples/grid_city_blocks.txt --animate --delay-ms 80
./bin/pathfinder pathfinder/examples/grid_open_field.txt --animate --delay-ms 60
./bin/pathfinder pathfinder/examples/grid_snake_large.txt --animate --delay-ms 35
./bin/pathfinder pathfinder/examples/grid_no_path.txt --animate --delay-ms 120
```

What each grid is good for:

- `grid_basic.txt`: fastest explanation of the path overlay.
- `grid_city_blocks.txt`: balanced demo with turns, dead ends, and a readable final path.
- `grid_open_field.txt`: shows wider frontier expansion because there is more open space.
- `grid_snake_large.txt`: long path that looks dramatic in a recording even though the rules stay simple.
- `grid_no_path.txt`: useful for showing how the visualizer handles failure cases cleanly.
