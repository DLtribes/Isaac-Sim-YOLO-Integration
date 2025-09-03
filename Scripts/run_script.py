#!/usr/bin/env python3

"""
Isaac Sim standalone: Franka pick-and-place with predefined cube positions from file
No manual cube setup required — all objects are loaded from the USD scene
"""

import numpy as np
import os
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.world import World
from omni.isaac.core.tasks import BaseTask
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from isaacsim.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import Camera
import PIL.Image

# Create demo_targets.txt with 4 pick positions
# Projected Y and X values (from image centroids)
X_vals = [0.3717, -0.1894, 0.0722, -0.4144, -0.4144]
Y_vals = [0.5650, 0.6083, 0.3737, 0.4000, 0.4000]
deg = [135.00, 56.03, 50.90, -56.31, 56.31]

with open("demo_targets.txt", "w") as f:
    for y, x, d in zip(Y_vals, X_vals, deg):
        f.write(f"{y:.4f} {x:.4f} 0.026 {d:.4f}\n")

print("📄 Created demo_targets.txt with projected positions.")



class FrankaPickTask(BaseTask):
    def __init__(self, usd_path=None):
        super().__init__(name="franka_pick_task")
        self._usd_path = usd_path
        self._franka = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        print("🏗️ Setting up scene...")

        if self._usd_path:
            success = self._load_usd_scene()
            if not success:
                raise RuntimeError("❌ USD scene failed to load!")
        else:
            raise RuntimeError("❌ No USD file path provided!")

        self._franka = scene.add(Franka(prim_path="/World/Franka", name="franka"))
        print("✅ Franka robot added")

    def _load_usd_scene(self):
        try:
            print(f"🔄 Loading USD scene from: {self._usd_path}")
            if not os.path.exists(self._usd_path):
                print(f"❌ USD file not found: {self._usd_path}")
                return False

            stage = omni.usd.get_context().get_stage()
            from pxr import Sdf
            layer = stage.GetRootLayer()
            layer.subLayerPaths.append(self._usd_path)
            print("✅ USD loaded via sublayer")
            return True

        except Exception as e:
            print(f"❌ Error loading USD: {e}")
            return False

    def get_observations(self):
        return {
            self._franka.name: {
                "joint_positions": self._franka.get_joint_positions()
            }
        }

    def pre_step(self, control_index, simulation_time):
        pass

    def post_reset(self):
        if self._franka and self._franka.gripper:
            self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)


class FrankaPickApp:
    def __init__(self, usd_path):
        self._world = World()
        self._task = FrankaPickTask(usd_path=usd_path)
        self._world.add_task(self._task)

        self._franka = None
        self._controller = None
        self._targets, self._deg = self._load_targets()
        self._index = 0
        self._pick_in_progress = False
        self._reset_requested = False
        self._reset_counter = 0
        self._frame_counter = 0

        self._camera = None
        os.makedirs("camera_images", exist_ok=True)

    def _load_targets(self):
        targets = []
        d=[]
        with open("demo_targets.txt", "r") as f:
            for line in f:
                x, y, z, deg = map(float, line.strip().split())
                targets.append(np.array([x, y, z]))
                d.append(np.array([deg]))
        print(f"🎯 Loaded {len(targets)} targets from file")
        return targets, d

    def setup_scene(self):
        self._world.reset()
        self._franka = self._world.scene.get_object("franka")
        if self._franka is None:
            raise RuntimeError("❌ Franka not found in scene")
        fast_events_dt = [0.0095, 0.005, 0.5, 0.1, 0.01, 0.07, 0.0045, 0.1, 1, 0.8]
                        #[0.008, 0.005, 0.1, 0.1, 0.0025, 0.001, 0.0025, 1, 0.008, 0.08]
        
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
            events_dt=fast_events_dt
        )

        self._world.add_physics_callback("sim_step", self.physics_step)

        # Add camera (optional)
        try:
            self._camera = Camera(
                prim_path="/Camera",
                position=(0.5, 0.0, 3.0),
                orientation=(0.0, -0.7071, 0.0, 0.7071),
                frequency=10,
                resolution=(640, 480),
            )
            self._camera.initialize()
            print(self._camera.get_intrinsics_matrix())

            print("✅ Camera initialized")
        except Exception as e:
            print(f"⚠️ Camera setup failed: {e}")
            self._camera = None

        return True

    def physics_step(self, step_size):
        try:
            observations = self._world.get_observations()

            if self._index >= len(self._targets):
                if not self._reset_requested:
                    print("🏁 All pick targets complete. Resetting...")
                    self._reset_requested = True
                    self._reset_counter = 0
                else:
                    self._reset_counter += 1
                    if self._reset_counter > 300:
                        self._index = 0
                        self._reset_requested = False
                        self._pick_in_progress = False
                        self._world.reset()
                        print("🔄 Simulation reset complete")
                return
            
            pick_pos = self._targets[self._index] + np.array([0, 0, 0.0055])
            import random
            place_pos = np.array([-0.25, -0.3, 0.55])
            
            if not self._pick_in_progress:
                print(f"🎯 Starting pick {self._index} at {pick_pos}")
                self._pick_in_progress = True
            
            import random
            action = self._controller.forward(
                picking_position=pick_pos,
                placing_position=place_pos,
                current_joint_positions=observations["franka"]["joint_positions"],
                end_effector_orientation=euler_angles_to_quat(np.array([0, np.pi, self._deg[self._index][0]*np.pi/180])) #0, np.pi, -np.pi/2
            )

            if action is not None:
                self._franka.apply_action(action)

            if self._controller.is_done():
                print(f"✅ Pick-place {self._index} done")
                self._controller.reset()
                self._index += 1
                self._pick_in_progress = False

            if self._camera and self._frame_counter%10==0:
                try:
                    rgb = self._camera.get_rgba()
                    if rgb is not None:
                        PIL.Image.fromarray(rgb).save(f"camera_images/frame_{self._frame_counter:04d}.png")
                        print(f"📸 Saved frame {self._frame_counter}")
                except Exception as e:
                    print(f"⚠️ Camera capture failed: {e}")
            self._frame_counter += 1

        except Exception as e:
            print(f"❌ Error in physics step: {e}")

    def run_simulation(self):
        if not self.setup_scene():
            print("❌ Scene setup failed.")
            return

        print("🚀 Running simulation... Press Ctrl+C to stop.")
        step = 0
        try:
            while simulation_app.is_running():
                self._world.step(render=True)
                step += 1
                if step % 1000 == 0:
                    print(f"⏱️ Step {step}, Pick {self._index + 1}")
        except KeyboardInterrupt:
            print("🛑 Stopped by user.")
        except Exception as e:
            print(f"❌ Simulation error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        print("🧹 Cleaning up...")
        try:
            if self._world:
                self._world.stop()
        except:
            pass
        try:
            simulation_app.close()
        except:
            pass


def main():
    USD_FILE_PATH = "999.usd"

    print("🎮 Isaac Sim Franka Pick-and-Place")
    print("==================================")

    if not os.path.exists(USD_FILE_PATH):
        print(f"❌ USD file not found: {USD_FILE_PATH}")
        return

    app = FrankaPickApp(usd_path=USD_FILE_PATH)
    app.run_simulation()
    print("👋 Done!")


if __name__ == "__main__":
    main()

