---
name: ros2-rclpy-testing
description: Isolate rclpy contexts per test class and use parameters= kwarg for node parameter overrides
source: auto-skill
extracted_at: '2026-06-26T09:14:22.386Z'
---

# ROS 2 Python (rclpy) Unit Testing Patterns

## Problem

When writing `unittest` tests that instantiate multiple rclpy `Node` subclasses in the same process, two common failures occur:

1. **`RuntimeError: Context.init() must only be called once`** — calling `rclpy.init()` in more than one test method on the global context raises this error on the second call.
2. **`TypeError: Node.__init__() got an unexpected keyword argument 'arguments'`** — passing CLI-style `arguments=["--ros-args", "-p", "key:=val"]` to a node constructor that doesn't forward them.

## Solution

### 1. Per-class isolated rclpy context

Create a fresh `rclpy.context.Context()` in each test class's `setUp`, and shut it down in `tearDown`. This isolates every class from the global context so multiple classes can coexist:

```python
class TestMyNode(unittest.TestCase):

    def setUp(self):
        import rclpy
        self._ctx = rclpy.context.Context()
        rclpy.init(context=self._ctx)

    def tearDown(self):
        import rclpy
        rclpy.shutdown(context=self._ctx)

    def test_something(self):
        node = MyNode()
        # ... assertions ...
        node.destroy_node()
```

**Why:** `rclpy.init()` on the default (global) context can only be called once per process. Using a dedicated `Context()` per test class sidesteps this entirely — each class gets its own lifecycle.

### 2. Use `parameters=` kwarg, not `arguments=`

To override node parameters in tests, pass a list of dicts via the `parameters` keyword:

```python
# Correct:
node = MyNode(parameters=[{"some_param": "value", "another": 42}])

# Wrong — raises TypeError if the node class doesn't forward arguments:
node = MyNode(arguments=["--ros-args", "-p", "some_param:=value"])
```

**Why:** The `arguments` kwarg is only forwarded to the underlying rclpy C++ layer when the node's `__init__` explicitly accepts and passes it. Most custom nodes don't. The `parameters=` kwarg is part of the standard `Node.__init__` signature and works everywhere.

### 3. Temp file cleanup with try/finally

When tests create temporary files (YAML configs, CSV logs), wrap node creation in `try/finally` to guarantee cleanup:

```python
with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as fh:
    fh.write("key: value\n")
    fh.flush()
    try:
        node = MyNode(parameters=[{"config_file": fh.name}])
        # ... assertions ...
        node.destroy_node()
    finally:
        os.unlink(fh.name)
```

## How to apply

Use this pattern whenever writing rclpy unit tests that:
- Instantiate more than one node class
- Need parameter overrides without launching via `ros2 run`
- Create temporary config or log files
