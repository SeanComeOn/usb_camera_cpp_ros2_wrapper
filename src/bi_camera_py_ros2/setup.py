from setuptools import find_packages, setup

package_name = "bi_camera_py_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/bi_camera.launch.py"]),
        (f"share/{package_name}/config", ["config/bi_camera.yaml"]),
        (f"share/{package_name}", ["README.md"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="good",
    maintainer_email="good@example.com",
    description="ROS 2 USB stereo camera package with MJPG capture and FPS benchmarking.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_config_wizard = bi_camera_py_ros2.config_helper:main",
            "stereo_camera_node = bi_camera_py_ros2.stereo_camera_node:main",
            "camera_fps_benchmark = bi_camera_py_ros2.fps_benchmark_node:main",
        ],
    },
)
