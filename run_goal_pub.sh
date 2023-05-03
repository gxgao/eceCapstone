if [[ $# -eq 1 ]]; then
    python goal_pub.py $1;
    python lift.py
fi

if [[ $# -eq 0 ]]; then
    python goal_pub.py;
    python lift.py
fi