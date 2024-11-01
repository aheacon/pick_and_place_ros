# MoveIt2
- Based on [manual setup](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/)
- [MoveIt setup assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

Pipeline:
1. Create `config` files
  - Added `srdf` (`Semantic Robot Description Format`) file (not known how is created, usually with `gz sim -p .urdf > .sdf`?) that adds semantic,
  meaning information to the robot's description that is not included inthe URDF (Unified Robot Description Format),
  like how robot prefers to move and interact with environment. It includes:
  - naming groups of joints or links
  - defining default robot poses
  - which parts of the robot should be checked for colliding with each other during movement.

- Action servers `ros2 action list`
