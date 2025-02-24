import pytest
from codestral_ros2_gen.utils.code_parser import ROS2CodeParser

TEST_CASES = [
    # markdown_python: Uses proper ```python markers so should work.
    pytest.param(
        'Here is code:\n```python\ndef hello():\n    print("Hello")\n```',
        '#!/usr/bin/env python3\ndef hello():\n    print("Hello")\n',
        id="markdown_python",
    ),
    # markdown_plain: Missing language marker, so now expect None.
    pytest.param(
        "Try this:\n```\nimport rclpy\nfrom rclpy.node import Node\n\nclass MyNode(Node):\n    def __init__(self):\n        super().__init__('test_node')\n```",
        None,
        id="markdown_plain",
    ),
    # python_tags: Using [PYTHON] markers should return None.
    pytest.param(
        "[PYTHON]\ndef main():\n    rclpy.init()\n    node = MyNode()\n    rclpy.spin(node)\n[/PYTHON]",
        None,
        id="python_tags",
    ),
    # direct_code: No markers, so returns None.
    pytest.param(
        "import rclpy\nfrom rclpy.node import Node\n\ndef main():\n    rclpy.init()",
        None,
        id="direct_code",
    ),
    # multiple_blocks: Only the first valid block will be extracted.
    pytest.param(
        'Here is code:\n```python\ndef main():\n    print("main")\n```\nAnd extra:\n```python\ndef extra():\n    print("extra")\n```',
        '#!/usr/bin/env python3\ndef main():\n    print("main")\n',
        id="multiple_blocks",
    ),
    # complex_ros2: Valid markers.
    pytest.param(
        '```python\n#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\nclass PublisherNode(Node):\n    def __init__(self):\n        super().__init__("minimal_publisher")\n        self.publisher = self.create_publisher(String, "topic", 10)\n        self.timer = self.create_timer(0.5, self.timer_callback)\n        self.i = 0\n\n    def timer_callback(self):\n        msg = String()\n        msg.data = f"Hello World: {self.i}"\n        self.publisher.publish(msg)\n        self.i += 1\n```',
        '#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\n\nclass PublisherNode(Node):\n    def __init__(self):\n        super().__init__("minimal_publisher")\n        self.publisher = self.create_publisher(String, "topic", 10)\n        self.timer = self.create_timer(0.5, self.timer_callback)\n        self.i = 0\n\n    def timer_callback(self):\n        msg = String()\n        msg.data = f"Hello World: {self.i}"\n        self.publisher.publish(msg)\n        self.i += 1\n',
        id="complex_ros2",
    ),
    # no_code: no markers so returns None.
    pytest.param("Here is some text without code.", None, id="no_code"),
    # empty_block: empty code block should return None.
    pytest.param("```python\n```", None, id="empty_block"),
    # invalid_syntax: will likely return None.
    pytest.param("```python\ndef invalid_syntax(\n```", None, id="invalid_syntax"),
    # mixed_markers: unsupported markers now yield None.
    pytest.param("[PYTHON]\ndef test():\n```", None, id="mixed_markers"),
    # extra_spaces: valid block using python markers.
    pytest.param(
        '```python\n        def spaced():\n                print("spaced")   \n```',
        '#!/usr/bin/env python3\ndef spaced():\n    print("spaced")\n',
        id="extra_spaces",
    ),
    # with_docstring: valid block.
    pytest.param(
        '```python\n# This is a comment\ndef documented():\n    """This is a docstring."""\n    pass\n```',
        '#!/usr/bin/env python3\n# This is a comment\ndef documented():\n    """This is a docstring."""\n    pass\n',
        id="with_docstring",
    ),
    # real_world_ai: Using [PYTHON] markers returns None.
    pytest.param(
        "I've analyzed your requirements:\n\n[PYTHON]\n#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom example_interfaces.srv import AddTwoInts\n\nclass AdditionServer(Node):\n    def __init__(self):\n        super().__init__('addition_server')\n        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)\n    def add_two_ints_callback(self, request, response):\n        response.sum = request.a + request.b\n        return response\n[/PYTHON]",
        None,
        id="real_world_ai",
    ),
]


@pytest.mark.parametrize(["input_text", "expected"], TEST_CASES)
def test_code_parser(input_text, expected):
    result = ROS2CodeParser.parse(input_text)
    print("---")
    print("Result:", result)
    print("---")
    print("Expected:", expected)
    print("---")
    assert result == expected
