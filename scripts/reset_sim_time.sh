#!/bin/bash
# Reset ROS simulation time after bag playback stops

echo "🕰️ Resetting simulation time..."
rosparam set /use_sim_time false
echo "✅ Simulation time reset to wall clock time"
echo "🔄 You may need to restart AirSLAM for clean state"


