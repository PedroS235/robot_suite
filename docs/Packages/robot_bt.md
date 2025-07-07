# Robot Behavior Tree

The `robot_bt` package implements a behavior tree (BT) for managing and executing plugins that define various behaviors for the robot. This package uses the `py-trees` library to structure and run the behavior tree.

## What is a Behavior Tree?

A behavior tree is a model that organizes defined behaviors into a tree structure, allowing them to be executed (or "ticked") in each iteration. While similar to a Finite State Machine (FSM), a behavior tree supports more complex scenarios with greater flexibility and simplicity.

In `py_trees`, a behavior tree is built using nodes, decorators, and composites, each with specific functionality to control the flow of execution. For more information, visit [py_trees documentation](https://py-trees.readthedocs.io/en/devel/)

---

### Common Nodes Used

#### Action Nodes

Action nodes perform specific tasks when ticked. They execute actions and return a status (`SUCCESS`, `FAILURE`, or `RUNNING`).

- `py_trees.behaviour.Behaviour`: The base class for any node you desire to create which does not require ROS.
- `robot_bt.nodes.actions.Action`: The base class for any node you desire to create which relies on ROS.
- `robot_bt.nodes.PluginClient`: A node which is used to execute a plugin.

#### Condition Nodes

- `robot_bt.nodes.conditionals.CanRunPlugin`: A conditional node which can be used to check if a plugin is currently selected to run.

Condition nodes check a condition and return `SUCCESS` if it is met, otherwise `FAILURE`.

---

### Common Decorators Used

Decorators modify the behavior of their child node based on specific rules or conditions.

- **`py_trees.decorators.Inverter`**: Inverts the result of the child node (e.g., `SUCCESS` becomes `FAILURE`).

---

### Common Composites Used

Composite nodes are used to group multiple child nodes and control their execution flow.

- **`py_trees.composites.Sequence`**: Executes its children in order until one fails. Returns:

    - `SUCCESS` if all children succeed.
    - `FAILURE` if any child fails.
    - `RUNNING` if a child is still running.

- **`py_trees.composites.Selector`**: Executes its children in order until one succeeds. Returns:

    - `SUCCESS` if any child succeeds.
    - `FAILURE` if all children fail.
    - `RUNNING` if a child is still running.

- **`py_trees.composites.Parallel`**: Executes all children simultaneously. Returns:
    - `SUCCESS` if the required number of children succeed.
    - `FAILURE` if too many children fail or if required conditions are not met.

---

## Creating a Behavior Tree

By default, the behavior tree used is `robot_bt.bt.DefaultBT`. To create your own custom behavior tree, follow these steps:

1. **Create a New File**: Add a new Python file in the `robot_bt.bt` directory and name it appropriately.
2. **Define the `bootstrap` Function**:
    - Your file must define a function named `bootstrap(ros_node: rclpy.Node) -> py_trees.behaviour.Behaviour`.
        - This function will be called by the BT ROS node to load your behavior tree.
        - It should return the root of your tree.
3. **Update Parameters**: Modify the parameter file to set the `bt_name` field to your file name (without the `.py` extension).

### Example

!!! example

    **Python Code**

    ```python title="robot_bt/bt/simple_bt.py"
    import py_trees
    from rclpy.node import Node

    def create_tree(): # Leaf nodes
        success_node = py_trees.behaviours.Success(name="Always Succeed")
        failure_node = py_trees.behaviours.Failure(name="Always Fail")

        # Decorator
        invert_failure = py_trees.decorators.Inverter(child=failure_node)

        # Composite
        sequence = py_trees.composites.Sequence(name="Simple Sequence")
        sequence.add_children([success_node, invert_failure])

        return sequence

    def bootstrap(ros_node: Node) -> py_trees.behavior.Behaviour:
        tree = create_tree()
        return tree
    ```

    **Parameters File**

    ```yaml title="config/params.yaml"
    bt_server:
        ros__parameters:
            bt_name: simple_bt
    ```

## Explaining Default BT

Below is the graphical representation of the `DefaultBT` behavior tree.

!!! note

    - **Reactive sequence** is a sequence without memory, meaning that it will always start from the first child.
    - **Sequence** has memory, meaning if one of it's children returns `RUNNING`, the next time it ticks, it will tick directly that child.

![Default BT Schematic](../assets/default_bt.webp){ align=center }

The structure and purpose of each part of the tree are as follows:

---

### Root: `DefaultBT`

- **Type**: Sequence (no memory)
- **Purpose**: Ensures all the children (subsystems) are checked or executed in order.

### 1. **Drone Connection Checker**

- **Type**: Selector (no memory)
- **Child**: `IsDroneConnected`
- **Purpose**: Verifies if the drone is connected.
    - If the connection is successful (`SUCCESS`), it moves to the next step.
    - If the connection is not established (`FAILURE`), the tree stops ticking and the sequence fails.

### 2. **Battery Checker**

- **Type**: Selector (no memory)
- **Children**:
    - **`IsBatteryLow`**: Checks if the battery is low. If it is, returns `SUCCESS`.
    - **Inverter** (decorator) → **`LandAction`**: If the battery is not low, the decorator inverts the result of the `LandAction`.
        - The `LandAction` would normally return `FAILURE` (since landing is not required when the battery is okay), but the `Inverter` converts it to `SUCCESS`.
- **Purpose**: Ensures that if the battery is low, the drone lands safely. Otherwise, it allows the behavior tree to continue execution.

### 3. **Remote Operator**

- **Type**: Action Node
- **Name**: `RemoteOperator`
- **Purpose**: Checks keyboard input from `robot_control_station` and change selected plugin from blackboard to hand gestures plugin if selected.
    - Alawas returns `SUCCESS`

### 4. **Plugins Selector**

- **Type**: Selector (no memory)
- **Children**:

    1. **Hand Gestures Control** (no memory):
        - **`CanRunPlugin`**: Checks if the "landmark_detector_node" plugin is enabled and can run.
        - **`PluginClient`**: Executes the plugin responsible for hand gesture control, enabling drone interaction using visual gestures.

- **Purpose**: Extends the drone's capabilities by enabling additional plugins, such as gesture control. If no plugin is active or available, this branch fails, but the rest of the tree can continue.

---

!!! example

    ```python title="robot_bt/bt/default_bt.py"
    # ...
    class DefaultBT(py_trees.composites.Sequence):
        # ...
        def build_tree(self):
            drone_connection = py_trees.composites.Selector(
                "DroneConnection",
                memory=False,
                children=[
                    IsDroneConnected("IsDroneConnected"),
                ],
            )

            battery_checker = py_trees.composites.Selector(
                "BatteryChecker",
                memory=False,
                children=[
                    IsBatteryLow("IsBatteryLow", self.node),
                    py_trees.decorators.Inverter(
                        "LandActionInverter", LandAction("LandAction", self.node)
                    ),
                ],
            )

            remote_operator = RemoteOperator("RemoteOperator", self.node)

            plugins = py_trees.composites.Selector(
                "Plugins",
                memory=False,
                children=[
                    py_trees.composites.Sequence(
                        "HandGesturesControl",
                        memory=False,
                        children=[
                            CanRunPlugin("CanRunHandGestures", "landmark_detector_node"),
                            PluginClient(
                                "HandGesturesPlugin", "landmark_detector_node", self.node
                            ),
                        ],
                    )
                ],
            )

            self.add_children([drone_connection, battery_checker, remote_operator, plugins])


    def bootstrap(ros_node: Node) -> py_trees.behaviour.Behaviour:
        return DefaultBT(ros_node)
    ```

## Behavior Tree Execution Flow - Example : Person tracking

The figure below shows the execution flow of the **Behavior Tree (BT)** when the **person tracking plugin** is active. Only branches highlighted in orange are executed.

![BT Flow Execution](./BT%20structure%20flow.png)

### Step-by-Step Flow

- **Connection Check**  
  → Branch 1 checks if the drone is connected.  
  ✅ If successful, continue.

- **Battery Check**  
  → Branch 2 checks if the battery level is sufficient.  
  ❌ If not, fallback to Branch 3 to land the drone.  
  ✅ If OK, continue.

- **Plugin Selection by Operator**  
  → Branch 4 allows the remote operator to select a plugin via a blackboard variable.  
  → In this example, the value is set to `"person_tracking"`.

- **Hand Gesture Plugin Check (Skipped)**  
  → Branch 5 checks if `"hand_gesture"` plugin was selected.  
  ❌ Fails (wrong plugin), so Branches 6 and 7 are not executed.

- **Person Tracking Plugin Check**  
  → Branch 8 checks if `"person_tracking"` plugin can be executed.  
  ✅ Success → proceed to Branch 9.

- **Object Detection Plugin**  
  → Branch 9 runs real-time object detection on drone video.  
  ✅ Always returns success.

- **Target Selection Mode**  
  → Branches 10–14 decide **how** the target person is selected:
  - **LLM interface** → executes Branches 10 and 11. ✅ *(used in this example)*
  - **Hand gesture** → would trigger Branches 12, 13, 14. ❌ *(not used)*

- **Tracking Execution**
  → Branch 15 identifies the person and publishes their position.  
  ✅ If tracking succeeds, go to Branch 16.

- **Tracking Control**
  → Branch 16 calculates velocity commands to follow the target with the drone camera.  
  ❌ If tracking fails (Branch 15 fails), fallback to:

- **Recovery and Fallback**
  → Branches 17 and 18 rotate the drone to search for the target.  
  → If still not found, Branch 19 triggers a landing.
