#!/usr/bin/env python3
"""
Generate an SDF world for an edge-based maze with embedded ground and sun.

- Avoids model:// includes by defining a simple ground box and a directional light.
- Walls rest on the ground (bottom at z=0).
"""

from textwrap import dedent

# Your inputs (edge-based)
maze_left2right = [
    "1010101",
    "1101011",
    "1110101",
    "1101011",
    "1000011",
    "1000001"
]

maze_top2bottom = [
    "1000011",
    "1000101",
    "1010111",
    "1010111",
    "1011011",
    "1001001"
]

# Parameters
CELL_SIZE = 0.5         # meters
WALL_HEIGHT = 0.5       # meters
WALL_THICKNESS = 0.05   # meters
EPS = 1e-4

# World header: embedded ground (large thin box) and a directional light ("sun")
WORLD_HEADER = dedent("""\
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="maze_world">
                      
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- core Gazebo plugins (keep as in official world) -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu" />
                      
    <!-- Embedded ground: large thin static box whose top surface is at z=0 -->
    <model name="ground_plane_custom">
      <static>true</static>
      <link name="ground_link">
        <pose>0 0 0 0 0 0</pose>
        <collision name="ground_collision">
          <geometry>
            <!-- big plane approximated by a thin box (100m x 100m x 0.01m) -->
            <box>
              <size>100.0 100.0 0.01</size>
            </box>
          </geometry>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <box>
              <size>100.0 100.0 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
      <!-- put the thin box so its top is at z=0 (box thickness 0.01 -> center at -0.005) -->
      <pose>0 0 -0.005 0 0 0</pose>
    </model>

    <!-- Embedded directional light (sun) -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.5 -1</direction>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.0</constant>
        <linear>0.0</linear>
        <quadratic>0.0</quadratic>
      </attenuation>
    </light>

""")

WORLD_FOOTER = "</world>\n</sdf>\n"

MODEL_TEMPLATE = dedent("""\
<model name="{name}">
  <static>true</static>
  <link name="link">
    <pose>0 0 0 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>{size_x:.6f} {size_y:.6f} {size_z:.6f}</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>{size_x:.6f} {size_y:.6f} {size_z:.6f}</size>
        </box>
      </geometry>
      <material>
        <ambient>0.6 0.6 0.6 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
  </link>
  <!-- model pose sets box center; half_z ensures bottom rests on z=0 -->
  <pose>{x:.6f} {y:.6f} {half_z:.6f} 0 0 0</pose>
</model>
""")

def as_grid(arr):
    grid = [list(r.strip()) for r in arr]
    if len(set(len(r) for r in grid)) != 1:
        raise ValueError("All rows must have same length")
    return grid

def transpose(grid):
    return [list(col) for col in zip(*grid)]

def generate_sdf(vertical_edges, horizontal_edges,
                 cell_size=CELL_SIZE, wall_thickness=WALL_THICKNESS,
                 wall_height=WALL_HEIGHT):
    V = as_grid(vertical_edges)   # R x (C+1)
    H = as_grid(horizontal_edges) # (R+1) x C or common transposed shapes

    R = len(V)
    C_plus1 = len(V[0])
    C = C_plus1 - 1

    # Detect/fix horizontal orientation
    if len(H) == R + 1 and len(H[0]) == C:
        H_grid = H
    elif len(H) == C and len(H[0]) == R + 1:
        H_grid = transpose(H)
    elif len(H) == R and len(H[0]) == C_plus1:
        H_grid = transpose(H)
    else:
        raise ValueError(
            f"Unexpected horizontal grid shape. Vertical: {R}x{C_plus1}. Horizontal provided: {len(H)}x{len(H[0])}."
        )

    lines = [WORLD_HEADER]
    half_z = wall_height / 2.0

    # Vertical edges
    for r in range(R):
        for ce in range(C + 1):
            if V[r][ce] == '1':
                x = ce * cell_size
                y = (r + 0.5) * cell_size
                size_x = wall_thickness + EPS
                size_y = cell_size + EPS
                size_z = wall_height
                name = f"wall_v_r{r}_c{ce}"
                lines.append(MODEL_TEMPLATE.format(
                    name=name, size_x=size_x, size_y=size_y,
                    size_z=size_z, x=x, y=y, half_z=half_z
                ))

    # Horizontal edges
    for er in range(R + 1):
        for c in range(C):
            if H_grid[er][c] == '1':
                x = (c + 0.5) * cell_size
                y = er * cell_size
                size_x = cell_size + EPS
                size_y = wall_thickness + EPS
                size_z = wall_height
                name = f"wall_h_r{er}_c{c}"
                lines.append(MODEL_TEMPLATE.format(
                    name=name, size_x=size_x, size_y=size_y,
                    size_z=size_z, x=x, y=y, half_z=half_z
                ))

    lines.append(WORLD_FOOTER)
    return "\n".join(lines)

if __name__ == "__main__":
    sdf = generate_sdf(maze_left2right, maze_top2bottom)
    with open("maze_world.sdf", "w") as f:
        f.write(sdf)
    print("Generated maze_world.sdf with embedded ground and sun.")
