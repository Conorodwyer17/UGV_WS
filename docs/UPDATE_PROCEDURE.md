# Update Procedure

How to pull, build, and deploy updates to the UGV tyre inspection system.

## 1. Pull Latest Code

```bash
cd ~/ugv_ws
git pull origin main
```

## 2. Install Dependencies

If `requirements.txt` or `package.xml` changed:

```bash
pip3 install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
```

## 3. Rebuild

```bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Test Before Deployment

**Simulation:**
```bash
ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true
```

Let a full mission complete. Verify no regressions.

**Unit tests (if available):**
```bash
colcon test --packages-select inspection_manager
colcon test-result --all
```

## 5. Deploy

If using systemd:
```bash
sudo systemctl stop ugv_mission
# Pull, build (steps 1–3)
sudo systemctl start ugv_mission
```

Or run manually from terminal.

## 6. Rollback

If issues occur:
```bash
git log -1  # note current commit
git checkout <previous-working-commit>
colcon build --symlink-install
```
