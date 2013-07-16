rosparam load cube_params.yaml
rosservice call /cube1/set_parameters {}
rosservice call /cube2/set_parameters {}
rosservice call /cube1/regenerate {}
rosservice call /cube2/regenerate {}
