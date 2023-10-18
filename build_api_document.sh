colcon list --paths-only --packages-ignore \
    openscenario_experimental_catalog \
    openscenario_interpreter_msgs \
    openscenario_preprocessor_msgs \
    openscenario_utility kashiwanoha_map \
    scenario_simulator_v2 \
    scenario_test_runner \
    traffic_simulator_msgs \
    | xargs -i poetry run rosdoc2 build --package-path {}
