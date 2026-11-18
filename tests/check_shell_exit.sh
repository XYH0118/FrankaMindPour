#!/bin/bash
# Replace every external/ROS operation before invoking the actual runner.
# No ROS installation, log mutation, or robot access occurs in this check.
source() { :; }
roslaunch() { return "${FMP_TEST_ROS_EXIT}"; }
find() { return "${FMP_TEST_FIND_EXIT}"; }
export -f source roslaunch find
for scenario in "7 0 7" "0 9 0"; do
  read -r FMP_TEST_ROS_EXIT FMP_TEST_FIND_EXIT expected <<< "$scenario"
  export FMP_TEST_ROS_EXIT FMP_TEST_FIND_EXIT
  bash scripts/run_session.sh participant:=P01 session:=0 > /dev/null
  actual=$?
  if [ "$actual" -ne "$expected" ]; then
    echo "Expected roslaunch exit $expected; got $actual" >&2
    exit 1
  fi
done
echo "Shell runner preserves roslaunch status across successful/failed log cleanup."
