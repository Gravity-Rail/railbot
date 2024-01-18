from setuptools import setup

package_name = "railbot_main"

setup(
    name=package_name,
    version="0.0.2",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["railbot_status"],
    zip_safe=True,
    maintainer="Daniel Walmsley",
    maintainer_email="dan@gravityrail.com",
    description="Railbot Main package for ROS2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chat_llm_service = railbot_main.chat_llm_service:main",
        ],
    },
)
