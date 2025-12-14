# Check if fzf is installed
if ! command -v fzf &> /dev/null; then
    echo "The command 'fzf' is not installed or found. Please install it to use the OMZSH ros2 plugin.
If fzf is already installed, make sure it is sourced before oh-my-zsh at .zshrc : \"[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh\" "
    return
fi

alias rtlist="ros2 topic list | fzf --preview 'ros2 topic info {1}'"

_ros2_select_topic() {
    local prompt=$1
    local command=$2
    ros2 topic list | fzf --prompt="$prompt" --preview "ros2 topic $command {1}"
}

rtecho() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Echo > " "echo")
    if [[ -n "$selected_topic" ]]; then
        ros2 topic echo "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

rthz() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Measure Hz > " "hz")
    if [[ -n "$selected_topic" ]]; then
        ros2 topic hz "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

rtbw() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Measure Bw > " "bw")
    if [[ -n "$selected_topic" ]]; then
        ros2 topic bw "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

rtinfo() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Check > " "info -v")
    if [[ -n "$selected_topic" ]]; then
        echo "$selected_topic"
        ros2 topic info -v "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

rtdelay() {
    local selected_topic=$(_ros2_select_topic "Select Topic to Measure Delay > " "delay")
    if [[ -n "$selected_topic" ]]; then
        ros2 topic delay "$selected_topic"
    else
        echo "Topic selection cancelled."
    fi
}

alias rnlist="ros2 node list | fzf --preview 'ros2 node info {1}'"

_ros2_select_node() {
    local prompt=$1
    ros2 node list | fzf --prompt="$prompt" --preview 'ros2 node info {1}'
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

