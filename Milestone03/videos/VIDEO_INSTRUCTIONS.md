# Video Recording Instructions for Milestone 03

## Required Videos

You need to record **3 videos** for this milestone. Here are detailed instructions for each.

---

## Video 1: Kinematics Validation (`kinematics_validation.mp4`)

**Duration:** 2-3 minutes  
**Focus:** Demonstrate that kinematic equations match simulator output

### Steps to Record:

1. **Open a terminal** in the project root directory

2. **Run the validation test:**
   ```bash
   cd Milestone03/codes
   python3 test_sim_vs_kinematics.py
   ```

3. **What to capture:**
   - The terminal window showing all test output
   - Tolerances displayed at the top
   - Each test case (1-5) with PASS/FAIL status
   - Position, velocity, and acceleration errors
   - Summary showing all tests PASSED

4. **Commentary to include (voice-over or text):**
   - "This test validates our forward and inverse kinematics equations"
   - "We're comparing 5 different robot configurations"
   - "Position errors are under 1.2mm - excellent accuracy"
   - "Velocity errors are under 0.12mm/s"
   - "Acceleration errors are under 0.05mm/s²"
   - "Test 2 has higher inverse velocity error due to near-singularity at zero configuration"
   - "All tests pass within specified tolerances"

5. **Recording tips:**
   - Use a screen recorder (OBS Studio, SimpleScreenRecorder, or Kazam on Linux)
   - Zoom in on terminal text if needed
   - Pause briefly on the summary section

---

## Video 2: GUI Demonstration (`sorting_station_gui.mp4`)

**Duration:** 2-4 minutes  
**Focus:** Showcase the complete sorting station environment

### Steps to Record:

1. **Launch the GUI:**
   ```bash
   python3 launch_sorting_station.py
   ```

2. **What to demonstrate:**
   - **Initial view** (0:00-0:15): Show the complete scene
     - Conveyor belt in the center
     - 6 boxes (3 white, 3 black) on the conveyor
     - White sorting bin on the left
     - Black sorting bin on the right
     - UR5e robot arm
     - Work table and safety equipment

   - **Camera controls** (0:15-0:45):
     - Left-click drag to rotate view
     - Right-click drag to pan
     - Scroll to zoom
     - Show different angles: front, side, top, close-ups

   - **Component details** (0:45-1:30):
     - Zoom into conveyor belt texture
     - Show white boxes (highlight 3 locations)
     - Show black boxes (highlight 3 locations)
     - Show transparent bin walls
     - Show robot arm joints and end-effector

   - **Physics demonstration** (1:30-2:30):
     - Let simulation run to show boxes settling
     - Show gravity effects
     - Demonstrate box-conveyor interaction
     - (Optional) Click/drag a box to show physics response

   - **Environment features** (2:30-3:00):
     - Show work table
     - Show safety fence
     - Show tool rack
     - Show emergency stop button

3. **Commentary to include:**
   - "Here's our complete sorting station environment"
   - "The conveyor belt holds 6 boxes ready for sorting"
   - "White boxes will go to the left bin, black to the right"
   - "The UR5e robot is positioned for optimal reach"
   - "All components use realistic physics and materials"
   - "This environment has 21 bodies and 63 geometric shapes"

4. **Recording tips:**
   - Record at 1920×1080 or higher resolution
   - Keep movements smooth (don't rotate camera too fast)
   - Pause on important components for 2-3 seconds

---

## Video 3: Kinematics Testing (`kinematics_testing.mp4`)

**Duration:** 2-3 minutes  
**Focus:** Show kinematic equations being tested with different inputs

### Steps to Record:

1. **Option A - Run the test script with verbose output:**
   ```bash
   cd Milestone03/codes
   python3 test_sim_vs_kinematics.py
   ```

2. **Option B - Run the demo with example:**
   ```bash
   python3 kinematics_derivations.py
   ```

3. **What to capture:**
   - Joint angles (q) being input
   - Joint velocities (q_dot) values
   - Joint accelerations (q_ddot) values
   - Computed end-effector positions from equations
   - Simulator positions for comparison
   - Error values showing they're within tolerance
   - Both forward and inverse kinematics results

4. **Split screen option (advanced):**
   - Left side: Terminal running test
   - Right side: MuJoCo viewer showing robot at those positions
   - This requires two terminals and some video editing

5. **Commentary to include:**
   - "Testing kinematic equations with various joint angles"
   - "First, home position with non-zero velocities"
   - "The simulator and equations agree within 1mm"
   - "Second test at zero configuration (potential singularity)"
   - "Handled correctly with appropriate tolerances"
   - "Random configurations also pass successfully"
   - "Both position and derivatives match within tolerance"

6. **Highlight these key points:**
   - Show specific error values (e.g., "0.714mm error → PASS")
   - Point out when inverse kinematics recovers original joint states
   - Explain tolerance values and why they're reasonable

---

## General Recording Tips

### Software Recommendations:

**Linux:**
- OBS Studio (best quality, free)
- SimpleScreenRecorder (lightweight)
- Kazam (simple interface)
- RecordMyDesktop (basic)

**Installation:**
```bash
# OBS Studio
sudo apt install obs-studio

# SimpleScreenRecorder
sudo apt install simplescreenrecorder

# Kazam
sudo apt install kazam
```

### Recording Settings:
- **Resolution:** 1920×1080 (1080p)
- **Frame rate:** 30 fps minimum
- **Format:** MP4 (H.264 codec)
- **Audio:** Optional but recommended for commentary
- **Bitrate:** 5-10 Mbps for good quality

### Post-Processing (Optional):
- Use a video editor to:
  - Add title slides
  - Add text annotations
  - Zoom into specific areas
  - Add voice-over commentary
  - Trim unnecessary parts

**Recommended editors:**
- Kdenlive (free, powerful)
- OpenShot (free, simple)
- DaVinci Resolve (free, professional)

### File Organization:

Save videos with these names:
```
Milestone03/videos/
├── kinematics_validation.mp4
├── sorting_station_gui.mp4
└── kinematics_testing.mp4
```

---

## Checklist Before Recording

- [ ] MuJoCo is installed and working
- [ ] Test script runs successfully
- [ ] GUI launches without errors
- [ ] Screen recorder is set up and tested
- [ ] Audio recording works (if adding commentary)
- [ ] Adequate lighting if recording screen directly
- [ ] Background noise minimized
- [ ] Have a script/notes for commentary

---

## Common Issues and Solutions

**Issue:** Text in terminal is too small
- **Solution:** Increase terminal font size before recording (Ctrl+Shift++ in most terminals)

**Issue:** GUI window is too small
- **Solution:** Maximize the MuJoCo viewer window, or resize before recording

**Issue:** Video file is too large
- **Solution:** Use H.264 codec with bitrate 5-8 Mbps, or compress after recording

**Issue:** Screen recorder drops frames
- **Solution:** Close unnecessary applications, or reduce recording resolution temporarily

---

## Example Commentary Scripts

### For Kinematics Validation:
> "Welcome to the kinematics validation for the UR5e robotic arm. We've implemented forward and inverse velocity and acceleration kinematics using the Denavit-Hartenberg method. This test compares our mathematical equations against the MuJoCo physics simulator. We test 5 different robot configurations, including the home position, zero configuration, and three random poses. As you can see, all position errors are under 1.2 millimeters, velocity errors under 0.12 millimeters per second, and acceleration errors under 0.05 millimeters per second squared. Test 2 shows higher inverse velocity error because the robot is near a singularity at the zero configuration, which is expected and handled correctly by our pseudoinverse approach. The summary confirms all tests pass within the specified tolerances. Our kinematic equations are validated and ready for motion control applications."

### For GUI Demonstration:
> "This is our complete sorting station environment built in MuJoCo. The scene includes an industrial conveyor belt in the center, with six boxes placed on it - three white boxes and three black boxes. On the left side, we have a transparent white sorting bin, and on the right, a black sorting bin. The UR5e robotic arm is mounted at the optimal position for picking boxes from the conveyor and placing them in the appropriate bins. The environment also includes a work table, safety fence, tool rack, and emergency stop button. You can see the realistic physics simulation with gravity affecting the boxes. The scene contains 21 rigid bodies with 63 geometric shapes, providing a complete testing environment for automated sorting operations. The camera can be controlled with the mouse to view the station from any angle."

---

**Good luck with your recordings!** If you have questions, refer to the main README or MuJoCo documentation.
