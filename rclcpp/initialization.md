# rclcpp: Initialization

## shared_from_this Cannot Be Used in Constructor
Sometime, you need to initialize some components outside of the constructor of your node.
For example, you may want to initialize an object by passing a shared pointer to the node to it.
This is not possible in the constructor, because the node is not fully initialized yet.

The following pattern can be used to initialize components outside of the constructor of your node.
This pattern was found through [this comment](https://github.com/ros2/rclcpp/issues/2110#issuecomment-1454228192).


```cpp
// Class declaration

class InitializedComponent
{
public:
  COMPOSITION_PUBLIC
  explicit InitializedComponent(const rclcpp::NodeOptions & options);

  COMPOSITION_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  rclcpp::Node::SharedPtr node_;
};

// Class implementation

InitializedComponent::InitializedComponent(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("node", options))
{
  /* Do some initialization here */
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
InitializedComponent::get_node_base_interface() const
{
  return this->node_->get_node_base_interface();
}
```

## Overriding Default Parameters
This is a pattern I find especially helpful when writing tests, where I want to create
several tests with different parameters but would rather not maintain separate YAML
or launch files with parameters:
```cpp
rclcpp::NodeOptions options;
options.parameter_overrides(
  {
    {"my_namespace.my_parameter", 5.2},
    {"another_parameter", "new value"},
  }
);
auto node = std::make_shared<rclcpp::Node>("node_name", options);
```
