while true; do
    COMMAND=$(cat $1/nturt_ros_pipe)
    if [ ${COMMAND} == "stop" ];then
        exit 0
    else
        eval ${COMMAND}
    fi
done
