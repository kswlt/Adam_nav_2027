#!/usr/bin/env bash
# TEST_ONLY: isolated DDS domains and PTY/ideal-plant fixtures; no physical drivers.
set -eo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project/tools/workspace_env.bash"
report="$(mktemp -d "${TMPDIR:-/tmp}/adam_nav_acceptance.XXXXXX")"
trap 'echo "Acceptance evidence: $report"' EXIT
cd "$project"
python3 tools/test_publish_map_bundle.py > "$report/map_publish.log" 2>&1
python3 tools/test_map_bundle_contract.py > "$report/map_contract.log" 2>&1
python3 tools/test_official_alignment.py > "$report/alignment.log" 2>&1
domain=160
for test in smoke_local_state smoke_observation_submap smoke_frozen_map_localization smoke_initial_pose \
  smoke_motion_gate smoke_kiss_recovery smoke_auto_recovery smoke_recovery_transaction \
  smoke_recovery_transaction_faults smoke_task_resume smoke_mapping_capture smoke_ros2_control smoke_serial_pty; do
  domain=$((domain+1))
  echo "RUN $test domain=$domain"
  ROS_DOMAIN_ID="$domain" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    python3 "tools/$test.py" > "$report/$test.log" 2>&1
  printf '%s PASS\n' "$test" >> "$report/results.txt"
done
ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tools/smoke_fixed_mount.py > "$report/fixed_mount.log" 2>&1
ROS_DOMAIN_ID=177 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tools/smoke_fixed_mount.py --legacy > "$report/legacy_projection.log" 2>&1
for profile in mppi tdt; do
  domain=$((domain+1))
  ROS_DOMAIN_ID="$domain" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    python3 tools/smoke_mppi_baseline.py --controller-profile "$profile" > "$report/nav2_$profile.log" 2>&1
  printf 'nav2_%s PASS\n' "$profile" >> "$report/results.txt"
done
ROS_DOMAIN_ID=179 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 tools/smoke_small_point_lio.py --with-state --with-mapping > "$report/lio_mapping.log" 2>&1
python3 tools/smoke_offline_mapping.py > "$report/offline_mapping.log" 2>&1
python3 tools/smoke_loop_enabled_mapping.py > "$report/loop_mapping.log" 2>&1
python3 tools/smoke_online_mapping.py > "$report/online_mapping.log" 2>&1
echo 'PASS software integration suite; hardware and field accuracy remain unverified'
