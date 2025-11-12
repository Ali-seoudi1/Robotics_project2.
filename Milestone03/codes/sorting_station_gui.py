#!/usr/bin/env python3
"""
Sorting Station GUI - Interactive MuJoCo visualization
Features:
- Conveyor belt with white and black boxes
- Left bin for white boxes, right bin for black boxes
- UR5e robot arm for sorting
- Interactive visualization with mouse controls
"""

import mujoco
import mujoco.viewer
import numpy as np
import os
import sys

def main():
    # Get the scene file path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, '..', 'model', 'sorting_scene.xml')
    
    if not os.path.exists(model_path):
        print(f"Error: Scene file not found at {model_path}")
        print("Make sure sorting_scene.xml exists in the model directory")
        sys.exit(1)
    
    try:
        # Load the model
        print("Loading sorting station scene...")
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        
        print("✓ Scene loaded successfully!")
        print("\n" + "="*60)
        print("SORTING STATION SIMULATION")
        print("="*60)
        print("Environment components:")
        print("  • Conveyor belt with 6 boxes (3 white, 3 black)")
        print("  • White sorting bin (left)")
        print("  • Black sorting bin (right)")
        print("  • UR5e robotic arm")
        print("  • Work table and safety equipment")
        print("\nControls:")
        print("  • Left mouse: Rotate view")
        print("  • Right mouse: Pan view")
        print("  • Scroll: Zoom")
        print("  • Double-click: Select objects")
        print("  • Ctrl+R: Reset simulation")
        print("  • ESC: Exit")
        print("="*60 + "\n")
        
        # Set initial state to home keyframe
        mujoco.mj_resetDataKeyframe(model, data, 0)
        
        # Launch interactive viewer
        print("Launching interactive viewer...")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set camera position for good initial view
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            viewer.cam.distance = 3.0
            viewer.cam.lookat[:] = [0, 0, 0.5]
            
            # Main simulation loop
            step_count = 0
            while viewer.is_running():
                # Step the simulation
                mujoco.mj_step(model, data)
                step_count += 1
                
                # Synchronize viewer (this handles rendering)
                viewer.sync()
                
                # Optional: Add some simple conveyor motion simulation
                # (In real system, this would be controlled by actuators)
                if step_count % 100 == 0:
                    # You can add conveyor belt animation here
                    pass
        
        print("Viewer closed.")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
