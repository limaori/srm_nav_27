# srm27_nav_plugins

Compatibility metapackage retained for existing workspace and deployment dependencies.
New dependencies should name the owning plugin package directly:

- `srm27_costmap_plugins`: costmap layers.
- `srm27_nav_behavior_plugins`: Nav2 behaviors, goal checkers, and behavior tree nodes.

The plugin libraries and pluginlib descriptions are installed by those packages.
