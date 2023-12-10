from setuptools import setup

package_name = "gpt_main"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "gpt_status"],
    zip_safe=True,
    maintainer="Herman Ye",
    maintainer_email="hermanye233@icloud.com",
    description="GPT-4 main package for ROS2 Humble",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gpt_ros2_server = gpt_main.gpt_ros2_server_demo:main",
            "gpt_ros2_client = gpt_main.gpt_ros2_client_demo:main",
            "gpt_service = gpt_main.gpt_service:main",
        ],
    },
)
