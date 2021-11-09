---
to: ./<%= h.inflection.underscore(nodeName) %>.py
---
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

<% publishers = publishers.split(",").map((name) => name.trim()).filter(function(el) { if(el) { return el; } }); -%>
<% subscriptions = subscriptions.split(",").map((name) => name.trim()).filter(function(el) { if(el) { return el; } }); -%>

class <%= h.inflection.camelize(nodeName) %>(Node):

    def __init__(self):
        super().__init__('<%= h.inflection.underscore( nodeName ) %>')
        
        <% if(publishers.length > 0) { %>
        # Create publisher handles
        self.publisher_handles = {}
        <% publishers.forEach(function(publisher) { %>
        self.publisher_handles["<%= publisher %>"] = self.create_publisher(String, "<%= publisher %>", 10)
        <% }); -%>
        <% } -%>

        <% if(subscriptions.length > 0) { %>
        # Create subscriber handles
        self.subscriber_handles = {}
        <% subscriptions.forEach(function(subscriber) { %>
        self.subscriber_handles["<%= subscriber %>"] = self.create_subscription(String, "<%= subscriber %>", self.<%= h.inflection.underscore(subscriber) %>_callback, 10)
        <% }); -%>
        <% } -%>

        <% if(publishers.length > 0) { %>
        # Periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        <% } -%>

    <% if(publishers.length > 0) { %>
    def timer_callback(self):
        """
        Basic method for publishing. Get rid of or modify this method,
        but please include a description of the method as is done here.
        """
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        <% publishers.forEach(function(publisher) { %>
        self.publisher_handles["<%= publisher %>"].publish(msg)
        <% }); %>
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    <% } -%>

    <% if(subscriptions.length > 0) { -%>
    <% subscriptions.forEach(function(subscriber) { -%>

    def <%= h.inflection.underscore(subscriber) %>_callback(self, msg):
        """
        Callback for the <%= subscriber %> topic.
        """
        self.get_logger().info(f"Received {msg} on topic <%= subscriber %>")

    <% }); -%>
    <% } -%>


def main(args=None):
    rclpy.init(args=args)

    <%= h.inflection.underscore( nodeName ) %> = <%= h.inflection.camelize(nodeName) %>()

    rclpy.spin(<%= h.inflection.underscore( nodeName ) %>)

    rclpy.shutdown()


if __name__ == '__main__':
    main() 
