# Recoveries

The `nav2_recoveries` package implements a task server for executing simple controlled robot movements such as rotating on its own axis, assisted teleop, or moving linearly.

The package defines:
- A `Recovery` template which is used as a base class to implement specific recovery timed action server - but not required.
- The `BackUp`, `Spin` and `Wait` recoveries.

The only required class a recovery must derive from is the `nav2_core/recovery.hpp` class, which implements the pluginlib interface the recovery server will use to dynamically load your behavior. The `nav2_recoveries/recovery.hpp` derives from this class and implements a generic action server for a timed recovery behavior (e.g. calls an implmentation function on a regular time interval to compute a value) but **this is not required** if it is not helpful. A behavior does not even need to be an action if you do not wish, it may be a service or other interface. However, most motion and behavior primitives are probably long-running and make sense to be modeled as actions, so the provided `recovery.hpp` helps in managing the complexity to simplify new behavior development, described more below. 

The value of the centralized recovery server is to **share resources** amongst several behaviors that would otherwise be independent nodes. Subscriptions to TF, costmaps, and more can be quite heavy and add non-trivial compute costs to a robot system. By combining these independent behaviors into a single server, they may share these resources while retaining complete independence in execution and interface.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-recovery-server.html) for additional parameter descriptions and a [tutorial about writing recovery behaviors](https://navigation.ros.org/plugin_tutorials/docs/writing_new_recovery_plugin.html).

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 
