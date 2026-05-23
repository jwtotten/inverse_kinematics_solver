"""
Interactive hexapod simulation.
Controls: F/B/S=direction  T/W/R=gait  +/-=speed
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import RadioButtons, Button, Slider
from Scripts.iksolver import IkSolver
from Scripts.gait_controller import GaitPattern
from Scripts.leg_collision_checker import get_crossing_leg_indices
from Scripts.animation_controller import (
    advance_frame, compute_body_offset, clamp_speed,
    translate_joints_y, build_ground_grid,
    compute_phase_offsets,
)

NUM_SAMPLES = 20
STEP_LENGTH_ABS = 0.6
INITIAL_SPEED = 1.0
VIEW_HALF = 12.0


def main():
    # --- Setup IK solvers ---
    ik_solvers = [IkSolver(femur_length=3.5, tibia_length=3.5) for _ in range(6)]
    base_trajectories = [ik.get_motion() for ik in ik_solvers]

    # --- Mutable state (use dict to allow closure mutation) ---
    state = {
        'body_y_offset': 0.0,
        'direction': 0,         # start stopped
        'speed': INITIAL_SPEED,
        'frame_counter': 0.0,
        'gait': GaitPattern.TRIPOD,
        'phase_offsets': compute_phase_offsets(GaitPattern.TRIPOD),
    }

    # --- Figure layout ---
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_axes([0.05, 0.1, 0.62, 0.82], projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-VIEW_HALF, VIEW_HALF])
    ax.set_zlim([-2, 8])
    ax.set_title('F/B/S: direction   T/W/R: gait   +/-: speed', fontsize=9)

    # --- Body box (12 edges, green, translated by body_offset each frame) ---
    # Build body box edges using x_length, y_length, z_length from first IkSolver
    xl = ik_solvers[0].x_length / 2
    yl = ik_solvers[0].y_length / 2
    zl = ik_solvers[0].z_length / 2
    # Each edge: (x_coords, y_coords, z_coords) — x coords will shift by body_offset
    body_edges_base = [
        ([-xl, 0],  [0, 0],   [zl, zl]),
        ([0, 0],    [-yl, 0], [zl, zl]),
        ([0, 0],    [0, 0],   [-zl, zl]),
        ([-xl, 0],  [0, 0],   [-zl, -zl]),
        ([0, 0],    [-yl, 0], [-zl, -zl]),
        ([-xl, -xl],[0, 0],   [-zl, zl]),
        ([0, 0],    [-yl, -yl],[-zl, zl]),
        ([-xl, -xl],[-yl, -yl],[-zl, zl]),
        ([-xl, -xl],[-yl, 0], [zl, zl]),
        ([-xl, 0],  [-yl, -yl],[zl, zl]),
        ([-xl, -xl],[-yl, 0], [-zl, -zl]),
        ([-xl, 0],  [-yl, -yl],[-zl, -zl]),
    ]
    body_lines = []
    for (bx, by, bz) in body_edges_base:
        line, = ax.plot(bx, by, bz, c='green', linewidth=1.5)
        body_lines.append((line, bx, by, bz))

    # --- Ground grid (None → NaN for matplotlib 3D line-break support) ---
    def _flatten_grid(segs):
        return [float('nan') if v is None else v for seg in segs for v in seg]

    xs0, ys0, zs0 = build_ground_grid(0.0)
    ground_line, = ax.plot(_flatten_grid(xs0), _flatten_grid(ys0), _flatten_grid(zs0),
                           'k--', alpha=0.25, linewidth=0.5)

    # --- Leg line artists (placeholder data; update() fills real positions on frame 0) ---
    joint_plots = []
    leg_plots = []
    for _ in ik_solvers:
        jp, = ax.plot([0, 0, 0], [0, 0, 0], [0, 0, 0], 'ro', markersize=4)
        lp, = ax.plot([0, 0, 0], [0, 0, 0], [0, 0, 0], 'b-', linewidth=1.5)
        joint_plots.append(jp)
        leg_plots.append(lp)

    # --- Widget panel (right side) ---
    # Gait radio buttons
    ax_radio = fig.add_axes([0.70, 0.60, 0.25, 0.22])
    radio = RadioButtons(ax_radio, ('Tripod', 'Wave', 'Ripple'), active=0)

    # Direction buttons
    ax_fwd  = fig.add_axes([0.70, 0.48, 0.25, 0.07])
    ax_stop = fig.add_axes([0.70, 0.39, 0.25, 0.07])
    ax_bwd  = fig.add_axes([0.70, 0.30, 0.25, 0.07])
    btn_fwd  = Button(ax_fwd,  'Forward')
    btn_stop = Button(ax_stop, 'Stop')
    btn_bwd  = Button(ax_bwd,  'Backward')

    # Speed slider
    ax_slider = fig.add_axes([0.70, 0.15, 0.25, 0.04])
    slider = Slider(ax_slider, 'Speed', 0.2, 3.0, valinit=INITIAL_SPEED, valstep=0.1)

    # --- Gait label ---
    ax_label = fig.add_axes([0.70, 0.85, 0.25, 0.08])
    ax_label.axis('off')
    ax_label.text(0.5, 0.5, 'GAIT', ha='center', va='center', fontsize=12, fontweight='bold')

    # --- Helpers ---
    def set_direction(d):
        state['direction'] = d
        # no GaitController to update

    def set_gait(pattern):
        state['gait'] = pattern
        state['phase_offsets'] = compute_phase_offsets(pattern)

    # --- Widget callbacks ---
    def on_radio(label):
        mapping = {'Tripod': GaitPattern.TRIPOD, 'Wave': GaitPattern.WAVE, 'Ripple': GaitPattern.RIPPLE}
        set_gait(mapping[label])

    def on_fwd(event):
        set_direction(1)

    def on_stop(event):
        set_direction(0)

    def on_bwd(event):
        set_direction(-1)

    def on_slider(val):
        state['speed'] = clamp_speed(val)

    radio.on_clicked(on_radio)
    btn_fwd.on_clicked(on_fwd)
    btn_stop.on_clicked(on_stop)
    btn_bwd.on_clicked(on_bwd)
    slider.on_changed(on_slider)

    # --- Keyboard callback ---
    def on_key(event):
        k = event.key
        if k == 'f':
            set_direction(1)
        elif k == 'b':
            set_direction(-1)
        elif k == 's':
            set_direction(0)
        elif k == 't':
            set_gait(GaitPattern.TRIPOD)
            radio.set_active(0)
        elif k == 'w':
            set_gait(GaitPattern.WAVE)
            radio.set_active(1)
        elif k == 'r':
            set_gait(GaitPattern.RIPPLE)
            radio.set_active(2)
        elif k in ('+', '='):
            new_speed = clamp_speed(state['speed'] + 0.2)
            state['speed'] = new_speed
            slider.set_val(new_speed)
        elif k == '-':
            new_speed = clamp_speed(state['speed'] - 0.2)
            state['speed'] = new_speed
            slider.set_val(new_speed)

    fig.canvas.mpl_connect('key_press_event', on_key)

    # --- Animation update ---
    def update(frame):
        state['frame_counter'] = advance_frame(state['frame_counter'], state['speed'], NUM_SAMPLES)
        i = int(state['frame_counter'])
        state['body_y_offset'] = compute_body_offset(
            state['body_y_offset'], -state['direction'], STEP_LENGTH_ABS, NUM_SAMPLES
        )
        bo = state['body_y_offset']

        # Update leg artists
        all_coords = []
        for idx, ik in enumerate(ik_solvers):
            x_arr, y_arr, z_arr = base_trajectories[idx]
            x_arr = np.array(x_arr)
            y_arr = np.array(y_arr)
            z_arr = np.array(z_arr)

            # Apply direction to x trajectory
            x_home = x_arr[0]  # stance position (first element = home x)
            if state['direction'] == -1:
                x_arr_eff = 2 * x_home - x_arr  # mirror x around home
            elif state['direction'] == 0:
                x_arr_eff = np.full_like(x_arr, x_home)  # hold at home
            else:
                x_arr_eff = x_arr

            # Apply phase offset for gait pattern
            phase_shift = int(NUM_SAMPLES * state['phase_offsets'][idx])
            x_rolled = np.roll(x_arr_eff, phase_shift)
            y_rolled = np.roll(y_arr, phase_shift)
            z_rolled = np.roll(z_arr, phase_shift)

            xi, yi, zi = float(x_rolled[i]), float(y_rolled[i]), float(z_rolled[i])
            coords = ik.solve_leg_position_from_target_coordinates(xi, yi, zi, verbose=False)

            translated = translate_joints_y(coords, bo)
            jx = [c[0] for c in translated]
            jy = [c[1] for c in translated]
            jz = [c[2] for c in translated]
            joint_plots[idx].set_data_3d(jx, jy, jz)
            leg_plots[idx].set_data_3d(jx, jy, jz)
            all_coords.append(coords)

        # Collision colours
        crossing = get_crossing_leg_indices(all_coords, threshold=0.05)
        for idx in range(6):
            leg_plots[idx].set_color('red' if idx in crossing else 'blue')

        # Update body box
        for (line, bx, by, bz) in body_lines:
            line.set_data_3d(bx, [y + bo for y in by], bz)

        # Update ground grid (Y-scrolling: swap roles so grid centers on Y offset)
        ys_segs, xs_segs, zs = build_ground_grid(bo)
        ground_line.set_data_3d(_flatten_grid(xs_segs), _flatten_grid(ys_segs), _flatten_grid(zs))

        # Camera follows body along Y
        ax.set_ylim([bo - VIEW_HALF, bo + VIEW_HALF])

        return leg_plots + joint_plots + [ground_line]

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
