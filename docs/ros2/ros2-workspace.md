# ROS 2 Workspace

In the `workspace/` folder is the actual ROS 2 control stack we use. It is structured as seen below:

```
└── src
    ├── common/
    ├── perception/
    │   ├── perception_py/
    │   └── perception_cpp/
    ├── state_estimation/
    └── ...
```

Inside the `src/` directory is a directory for each package. Each sub-team has it's own package (perception, state estimation, driving functions, etc.). The `common_interfaces` directory contains packages for messages (`common_msgs`), services (`common_srvs`), and actions (`common_actions`) used by two or more packages. Any messages, services, or actions used by one package alone (ex. for a topic shared only by nodes in that package) should be contained inside the `msg/` directory of that package. 

