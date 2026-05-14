"""
Interactive hexapod simulation.
Controls: F/B/S=direction  T/W/R=gait  +/-=speed
"""
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import RadioButtons, Button, Slider
from Scripts.iksolver import IkSolver
from Scripts.gait_controller import GaitController, GaitPattern
from Scripts.leg_collision_checker import get_crossing_leg_indices
from Scripts.animation_controller import (
    advance_frame, compute_body_offset, clamp_speed,
    translate_joints, build_ground_grid,
    apply_direction_to_controllers, apply_gait_to_controllers,
)

NUM_SAMPLES = 50
STEP_LENGTH_ABS = 4.0
INITIAL_SPEED = 1.0
XLIM_HALF = 12.0


def main():
    # --- Setup IK solvers and gait controllers ---
    ik_solvers = [IkSolver(femur_length=3.5, tibia_length=3.5) for _ in range(6)]
    gait_controllers = [GaitController(iksolver=ik, number_samples=NUM_SAMPLES) for ik in ik_solvers]
    for gc in gait_controllers:
        gc.set_gait_pattern(GaitPattern.TRIPOD)

    # --- Mutable state (use dict to allow closure mutation) ---
    state = {
        'body_offset': 0.0,
        'direction': 0,         # start stopped
        'speed': INITIAL_SPEED,
        'frame_counter': 0.0,
        'gait': GaitPattern.TRIPOD,
    }

    # --- Figure layout ---
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_axes([0.05, 0.1, 0.62, 0.82], projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_ylim([-10, 10])
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

    # --- Ground grid (single line artist, data updated each frame) ---
    xs0, ys0, zs0 = build_ground_grid(0.0)
    flat_x0 = [v for seg in xs0 for v in seg]
    flat_y0 = [v for seg in ys0 for v in seg]
    flat_z0 = [v for seg in zs0 for v in seg]
    ground_line, = ax.plot(flat_x0, flat_y0, flat_z0, 'k--', alpha=0.25, linewidth=0.5)

    # --- Leg line artists (placeholder data; update() fills real positions on frame 0) ---
    joint_plots = []
    leg_plots = []
    for _ in gait_controllers:
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
        apply_direction_to_controllers(gait_controllers, d, STEP_LENGTH_ABS)

    def set_gait(pattern):
        state['gait'] = pattern
        apply_gait_to_controllers(gait_controllers, pattern)

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
        state['body_offset'] = compute_body_offset(
            state['body_offset'], state['direction'], STEP_LENGTH_ABS, NUM_SAMPLES
        )
        bo = state['body_offset']

        # Update leg artists
        all_coords = []
        for idx, gc in enumerate(gait_controllers):
            xp, yp, zp = gc.get_motion()
            try:
                coords = gc.solve_leg_position_from_target_coordinates(xp[i], yp[i], zp[i], verbose=False)
            except ValueError:
                all_coords.append(None)
                continue
            translated = translate_joints(coords, bo)
            jx = [c[0] for c in translated]
            jy = [c[1] for c in translated]
            jz = [c[2] for c in translated]
            joint_plots[idx].set_data_3d(jx, jy, jz)
            leg_plots[idx].set_data_3d(jx, jy, jz)
            all_coords.append(coords)

        # Collision colours (only check legs that solved successfully)
        valid_coords = [c for c in all_coords if c is not None]
        crossing = get_crossing_leg_indices(valid_coords, threshold=0.05) if valid_coords else set()
        for idx in range(6):
            leg_plots[idx].set_color('red' if idx in crossing else 'blue')

        # Update body box
        for (line, bx, by, bz) in body_lines:
            line.set_data_3d([x + bo for x in bx], by, bz)

        # Update ground grid
        xs, ys, zs = build_ground_grid(bo)
        flat_x = [v for seg in xs for v in seg]
        flat_y = [v for seg in ys for v in seg]
        flat_z = [v for seg in zs for v in seg]
        ground_line.set_data_3d(flat_x, flat_y, flat_z)

        # Camera follows body
        ax.set_xlim([bo - XLIM_HALF, bo + XLIM_HALF])

        return leg_plots + joint_plots + [ground_line]

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
