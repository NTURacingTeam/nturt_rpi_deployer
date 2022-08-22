while true; do
    eval "$(cat $1/nturt_ros_pipe)"
    sleep 1
done
