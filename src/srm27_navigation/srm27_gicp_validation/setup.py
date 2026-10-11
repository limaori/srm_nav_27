from setuptools import setup

package_name = "srm27_gicp_validation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/validation.yaml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/gicp_reload.launch.py", "launch/validation_stack.launch.py"],
        ),
    ],
    scripts=[
        "scripts/gicp_static_tf_republisher",
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SRM",
    maintainer_email="srm@example.com",
    description="GICP 初始重定位仿真验证工具",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gicp_prior_map_builder = srm27_gicp_validation.gicp_prior_map_builder:main",
            "gicp_truth_evaluator = srm27_gicp_validation.gicp_truth_evaluator:main",
            "gicp_initialization_runner = srm27_gicp_validation.gicp_initialization_runner:main",
            "gicp_cmd_vel_script = srm27_gicp_validation.gicp_cmd_vel_script:main",
            "gicp_eval_selftest = srm27_gicp_validation.gicp_eval_selftest:main",
            "gicp_session_check = srm27_gicp_validation.gicp_session_check:main",
            "gicp_tf_buffer_probe = srm27_gicp_validation.gicp_tf_buffer_probe:main",
            "gicp_static_tf_republisher = srm27_gicp_validation.gicp_static_tf_republisher:main",
            "gicp_odom_base_tf_publisher = srm27_gicp_validation.gicp_odom_base_tf_publisher:main",
            "gicp_registered_scan_provider = srm27_gicp_validation.gicp_registered_scan_provider:main",
            "gicp_map_recorder = srm27_gicp_validation.gicp_map_recorder:main",
            "gicp_manual_initial_pose = srm27_gicp_validation.gicp_manual_initial_pose:main",
        ],
    },
)
