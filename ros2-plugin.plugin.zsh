# Check if fzf is installed
if ! command -v fzf &> /dev/null; then
    echo "The command 'fzf' is not installed. Please install it to use all features of the ros2-plugin."
    return
fi

alias rtlist="ros2 topic list | fzf"

_ros2_select_topic() {
    local prompt=$1
    ros2 topic list | fzf --height 40% --prompt="$prompt"
}

rtecho() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Echo > ")
    if [[ -n "$selected_topic" ]]; then
        ros2 topic echo "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

rthz() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Measure Hz > ")
    if [[ -n "$selected_topic" ]]; then
        ros2 topic hz "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

rtbw() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Measure Bw > ")
    if [[ -n "$selected_topic" ]]; then
        ros2 topic bw "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

rtinfo() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Check > ")
    if [[ -n "$selected_topic" ]]; then
        echo "$selected_topic"
        ros2 topic info -v "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

rtdelay() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Measure Delay > ")
    if [[ -n "$selected_topic" ]]; then
        ros2 topic delay "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

alias rnlist="ros2 node list | fzf"

_ros2_select_node() {
    local prompt=$1
    ros2 node list | fzf --prompt="$prompt"
}

rninfo() {
	local selected_node=$(_ros2_select_node "Select Node to Check > ")
	if [[ -n "$selected_node" ]]; then
        echo "$selected_node"
        ros2 node info "$selected_node"
    else
        echo "Node selection cancelled."
    fi
}

