# Autoware CARLA Bridge - workspace setup
#
# Source this after your Autoware environment:
#   source /path/to/autoware/install/setup.bash
#   source /path/to/this/setup.bash
#
# This script is position-independent. It locates itself via BASH_SOURCE
# and sources local_setup.bash from the same directory.

_ACB_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"
COLCON_CURRENT_PREFIX="$_ACB_DIR" source "$_ACB_DIR/local_setup.bash"
unset _ACB_DIR
