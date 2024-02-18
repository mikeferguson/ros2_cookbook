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

## Documenting Metapackages

A metapackage is one that contains no code and exists basically to bundle up a set of
dependencies. For instance ``image_pipeline`` is a repository containing several packages
for image processing - and the ``image_pipeline`` metapackage depends on every package
in the repository to make it easier to install with ``apt install ros-<distro>-image-pipeline``
rather than specifying each package individually. This does lead to an issue with
``rosdoc2``, which will fail if there is no code to document. If you want to add
tutorials or documentation to a metapackage, you need to use a ``rosdoc2.yaml`` file
to properly build your documentation (which we assume is located in the ``doc``
folder:

```yaml
type: 'rosdoc2 config'
version: 1

---

settings:
    # Not generating any documentation of code
    generate_package_index: false
    always_run_doxygen: false
    enable_breathe: false
    enable_exhale: false
    always_run_sphinx_apidoc: false
    # Lie to rosdoc2, claim to be a python package
    override_build_type: 'ament_python'
    # Lie to rosdoc2 again, say your source is in doc folder
    python_source: 'doc'

builders:
   # Configure Sphinx with the location of the docs:
    - sphinx: {
        name: 'image_pipeline',
        sphinx_sourcedir: 'doc',
        output_dir: ''
    }
```

Don't forget to add your yaml file to the ``package.xml``:

```xml
<export>
  <rosdoc2>rosdoc2.yaml</rosdoc2>
</export>
```
