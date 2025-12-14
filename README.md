# ROS2 Plugin for Oh My Zsh

## Overview

This plugin provides a set of convenient aliases and functions to speed up your workflow with ROS2 (Robot Operating System 2). It leverages `fzf` to offer an interactive and fuzzy-searchable menu for selecting ROS2 topics and nodes, making common tasks faster and more efficient.

## Prerequisites

Before installing this plugin, please ensure you have the following dependencies installed on your system:

*   [Oh My Zsh](https://ohmyz.sh/)
*   [ROS2](https://docs.ros.org/en/rolling/Installation.html) (and a sourced workspace)
*   [fzf](https://github.com/junegunn/fzf) (a command-line fuzzy finder)

## Installation

1.  **Clone the repository:**
    Clone this repository into your Oh My Zsh custom plugins directory.

    ```sh
    git clone https://github.com/farzadsw/oh-my-zsh-ros2-plugin.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/ros2
    ```

2.  **Activate the plugin:**
    Add `ros2` to the `plugins` array in your `~/.zshrc` file.

    ```zsh
    plugins=(
        # other plugins...
        ros2
    )
    ```

3.  **Restart your shell:**
    Either restart your terminal or source your `.zshrc` file for the changes to take effect.

    ```sh
    source ~/.zshrc
    ```

## Features

### Aliases

*   `rtlist`: Interactively list all available ROS2 topics.
*   `rnlist`: Interactively list all running ROS2 nodes.

### Functions

All functions that require a topic or node selection will open an `fzf`-powered interactive menu.

*   `rtecho`: Select a topic to echo its messages to the console (`ros2 topic echo`).
*   `rthz`: Select a topic to measure its publishing rate (`ros2 topic hz`).
*   `rtbw`: Select a topic to measure its bandwidth usage (`ros2 topic bw`).
*   `rtdelay`: Select a topic to measure the delay of its messages (`ros2 topic delay`).
*   `rtinfo`: Select a topic to view its detailed information (`ros2 topic info -v`).
*   `rninfo`: Select a node to view its detailed information (`ros2 node info`).

## Usage Examples

Using the plugin is as simple as calling the functions from your terminal.

**Echo a topic:**

```sh
$ rtecho
```
This will open a fuzzy-searchable list of all your current ROS2 topics. Select one to start echoing its content.

**Get information on a node:**

```sh
$ rninfo
```
This will open a list of active nodes. Select one to see its publishers, subscribers, services, and actions.

---

