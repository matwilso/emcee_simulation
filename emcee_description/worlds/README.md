# Using the .world files

Use the this one if you are running. You must be in the worlds directory.
```bash
gazebo nasa_minimal.world
```
```xml
<mesh><uri>file://../meshes/arena/nasa_minimal.dae</uri></mesh>
```

If you are roslaunching normally (see launch/gazebo.launch), use:
```xml
<mesh><uri>model://emcee_description/meshes/arena/nasa_minimal.dae</uri></mesh>
```

This is the hack I found for making the 2nd command work (for reference: http://answers.gazebosim.org/question/6568/uri-paths-to-packages-in-the-sdf-model-file/)
```xml
...

<export>
  <!-- gazebo_ros_paths_plugin automatically adds these to
      GAZEBO_PLUGIN_PATH and GAZEBO_MODEL_PATH when you do this export inside
      the package.xml file. You can than use URIs of type model://my_package/stuff. -->
  <gazebo_ros
      gazebo_plugin_path="${prefix}/lib"
      gazebo_model_path="${prefix}/.." />
</export>


</package>
```
