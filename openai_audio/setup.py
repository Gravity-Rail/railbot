from setuptools import setup

package_name = "openai_audio"

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
    install_requires=["railbot_status"],
    zip_safe=True,
    maintainer="Daniel Walmsley",
    maintainer_email="dan@gravityrail.com",
    description="OpenAI TTS/STT for Railbot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "audio_input = openai_audio.audio_input:main",
            "audio_output = openai_audio.audio_output:main",
        ],
    },
)
