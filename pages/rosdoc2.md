# rosdoc2

Documentation for ROS 2 packages can be built with
[rosdoc2](https://github.com/ros-infrastructure/rosdoc2).

When a ``doc`` job is configured in the ``rosdistro``, the resulting
documentation is uploaded to ``docs.ros.org``.
The design document for
[per package documentation](http://design.ros2.org/articles/per_package_documentation.html)
lays out the directory structure.

## Linking to Other Packages


This is a dirty hack, but appears to be the only way to have the table of contents
link to another package's documentation without hard-coding the distro. The trailing
``http://`` changes how Sphinx processes the link:

```rst
.. toctree::
   :maxdepth: 2

   other_package <../other_package/index.html#http://>
```

An example of this can be seen in the documentation for
[image_pipeline](http://docs.ros.org/en/rolling/p/image_pipeline/), where we want
to link to the documentation for each of the packages within the metapackage.
