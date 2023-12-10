from setuptools import setup

package_name = "gpt_status"

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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Herman Ye",
    maintainer_email="hermanye233@icloud.com",
    description="Status package for GPT",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gpt_param_server = gpt_status.gpt_param_server:main",
        ],
    },
)
