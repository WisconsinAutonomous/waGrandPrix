# Adding a Node

Most of the time you won't be creating new packages, but just adding nodes to existing packages. We are using a templating tool called [Hygen](https://www.hygen.io/) to help with boiler plate code. Hygen is already installed and configured in the development docker container, so that is the recommended avenue to use it. Otherwise, you are responsible for installing and setting it up (you need to set the `HYGEN_TMPLS` environment variable to the `_templates` directory).

This is a **4** step process

**1.**
To add a node to the `new_package` package, you can run the following:

```
$ cd src/new_package/new_package/
$ hygen ros2 python-node
```

You will be prompted to enter some basic information about the node, and a file for a new node will be written in the current directory. You should enter this information exactly as is documented the [WA Software Architecture Diagram](https://drive.google.com/file/d/1nBj6e1DiyWXzSxHhxgPGXwtTzqKDPyTg/view?usp=sharing)

```
✔ What is the name of the node ? · new_node
✔ What topics should be subscribed to (comma seperated)? · sub_topic1, sub_topic2, sub_topic3
✔ What topics should be published (comma seperated)? · pub_topic2, pub_topic2

Loaded templates: /root/ros2_workspace/_templates
       added: ./new_node.py
```

Resulting in:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class NewNode(Node):

    def __init__(self):
        super().__init__('new_node')


        # Create publisher handles
        self.publisher_handles = {}

        self.publisher_handles["pub_topic1"] = self.create_publisher(String, "pub_topic1", 10)

        self.publisher_handles["pub_topic2"] = self.create_publisher(String, "pub_topic2", 10)


        # Create subscriber handles
        self.subscriber_handles = {}

        self.subscriber_handles["sub_topic1"] = self.create_subscription(String, "sub_topic1", self.sub_topic1_callback, 10)

        self.subscriber_handles["sub_topic2"] = self.create_subscription(String, "sub_topic2", self.sub_topic2_callback, 10)

        self.subscriber_handles["sub_topic3"] = self.create_subscription(String, "sub_topic3", self.sub_topic3_callback, 10)


        # Periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        """
        Basic method for publishing. Get rid of or modify this method,
        but please include a description of the method as is done here.
        """
        msg = String()
        msg.data = 'Hello World: %d' % self.i

        self.publisher_handles["pub_topic1"].publish(msg)

        self.publisher_handles["pub_topic2"].publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def sub_topic1_callback(self, msg):
        """
        Callback for the sub_topic1 topic.
        """
        self.get_logger().info(f"Received {msg} on topic sub_topic1")


    def sub_topic2_callback(self, msg):
        """
        Callback for the sub_topic2 topic.
        """
        self.get_logger().info(f"Received {msg} on topic sub_topic2")


    def sub_topic3_callback(self, msg):
        """
        Callback for the sub_topic3 topic.
        """
        self.get_logger().info(f"Received {msg} on topic sub_topic3")



def main(args=None):
    rclpy.init(args=args)

    new_node = NewNode()

    rclpy.spin(new_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```


**2.**
Before you can run the node, you need to make sure the build system `colcon` will see it. 

- **2a)** First, make it executable by running: 
```
$ chmod +x new_node.py
```
- **2b)** Next, update the `setup.py` file in `src/my_package/`:

```python
entry_points={
  'console_scripts': [
    'new_node = new_package.new_node:main' # <--- New line
  ],
},
```

**3.**

- **3a)** Now, return to the `ros2_workspace` directory,
```
$ cd ~/ros2_workspace
```
- **3b)** and run `colcon build` or `make build`.

- **3c)** Make sure to source your shell so the build changes take effect by running `source ./install/setup.bash` OR `source ./install/setup.bash`

**4.**
Once that has completed, verify that the executable can be seen:
```
$ ros2 pkg executables new_package
```

