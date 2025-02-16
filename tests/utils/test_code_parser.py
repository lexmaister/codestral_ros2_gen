import pytest
from codestral_ros2_gen.utils.code_parser import ROS2CodeParser

TEST_CASES = [
    # markdown_python
    pytest.param(
        'Here\'s the implementation:\n```python\ndef hello():\n    print("Hello")\n```\nHope this helps!',
        'def hello():\n    print("Hello")\n',
        id="markdown_python",
    ),
    # markdown_plain
    pytest.param(
        "Try this:\n```\nimport rclpy\nfrom rclpy.node import Node\n\nclass MyNode(Node):\n    def __init__(self):\n        super().__init__('test_node')\n```",
        'import rclpy\nfrom rclpy.node import Node\n\n\nclass MyNode(Node):\n    def __init__(self):\n        super().__init__("test_node")\n',
        id="markdown_plain",
    ),
    # python_tags
    pytest.param(
        "This should work:\n[PYTHON]\ndef main():\n    rclpy.init()\n    node = MyNode()\n    rclpy.spin(node)\n[/PYTHON]\nLet me know if you need any clarification.",
        "def main():\n    rclpy.init()\n    node = MyNode()\n    rclpy.spin(node)\n",
        id="python_tags",
    ),
    # direct_code
    pytest.param(
        "import rclpy\nfrom rclpy.node import Node\n\ndef main():\n    rclpy.init()",
        "import rclpy\nfrom rclpy.node import Node\n\n\ndef main():\n    rclpy.init()\n",
        id="direct_code",
    ),
    # multiple_blocks
    pytest.param(
        'Here\'s the main code:\n```python\ndef main():\n    print("main")\n```\nAnd here\'s another example:\n```python\ndef another():\n    print("another")\n```',
        'def main():\n    print("main")\n',
        id="multiple_blocks",
    ),
    # complex_ros2
    pytest.param(
        "```python\n#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\nclass PublisherNode(Node):\n    def __init__(self):\n        super().__init__('minimal_publisher')\n        self.publisher = self.create_publisher(String, 'topic', 10)\n        self.timer = self.create_timer(0.5, self.timer_callback)\n        self.i = 0\n\n    def timer_callback(self):\n        msg = String()\n        msg.data = f'Hello World: {self.i}'\n        self.publisher.publish(msg)\n        self.i += 1\n```",
        '#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\n\nclass PublisherNode(Node):\n    def __init__(self):\n        super().__init__("minimal_publisher")\n        self.publisher = self.create_publisher(String, "topic", 10)\n        self.timer = self.create_timer(0.5, self.timer_callback)\n        self.i = 0\n\n    def timer_callback(self):\n        msg = String()\n        msg.data = f"Hello World: {self.i}"\n        self.publisher.publish(msg)\n        self.i += 1\n',
        id="complex_ros2",
    ),
    # no_code
    pytest.param("Here is some text without any code.", None, id="no_code"),
    # empty_block
    pytest.param("```python\n```", None, id="empty_block"),
    # invalid_syntax
    pytest.param("```python\ndef invalid_syntax(\n```", None, id="invalid_syntax"),
    # mixed_markers
    pytest.param("[PYTHON]\ndef test():\n```", None, id="mixed_markers"),
    # extra_spaces
    pytest.param(
        '```python\n        def spaced():\n                print("spaced")   \n```',
        'def spaced():\n    print("spaced")\n',
        id="extra_spaces",
    ),
    # with_docstring
    pytest.param(
        '```python\n# This is a comment\ndef documented():\n    """This is a docstring."""\n    pass\n```',
        '# This is a comment\ndef documented():\n    """This is a docstring."""\n    pass\n',
        id="with_docstring",
    ),
    # real_world_ai
    pytest.param(
        "I've analyzed your requirements and here's a ROS2 node implementation that should work:\n\n[PYTHON]\n#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom example_interfaces.srv import AddTwoInts\n\nclass AdditionServer(Node):\n    def __init__(self):\n        super().__init__('addition_server')\n        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)\n    \n    def add_two_ints_callback(self, request, response):\n        response.sum = request.a + request.b\n        return response\n[/PYTHON]\n\nThis implementation provides a simple service server that adds two integers.",
        '#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom example_interfaces.srv import AddTwoInts\n\n\nclass AdditionServer(Node):\n    def __init__(self):\n        super().__init__("addition_server")\n        self.srv = self.create_service(\n            AddTwoInts, "add_two_ints", self.add_two_ints_callback\n        )\n\n    def add_two_ints_callback(self, request, response):\n        response.sum = request.a + request.b\n        return response\n',
        id="real_world_ai",
    ),
]


@pytest.mark.parametrize(["input_text", "expected"], TEST_CASES)
def test_code_parser(input_text, expected):
    result = ROS2CodeParser.parse(input_text)
    print("---")
    print("result:", result, sep="\n")
    print("---")
    print("expected:", expected, sep="\n")
    print("---")
    if expected is None:
        assert result is None
    else:
        assert result is not None
        assert result == expected
