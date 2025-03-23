from setuptools import setup
import os
import shutil

package_name = "ros2_webcam_stream"


def copy_recursive(src, dst):
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            shutil.copytree(s, d)
        else:
            shutil.copy2(s, d)


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["horse.jpg"]),
        ("share/" + package_name + "/web", ["web/index.html", "web/client.js"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Your Name",
    author_email="user@example.com",
    maintainer="Your Name",
    maintainer_email="user@example.com",
    description="ROS2 package for streaming ROS images and publishing disk images.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stream_video = ros2_webcam_stream.stream_video:main",
            "disk_image_publisher = ros2_webcam_stream.disk_image_publisher:main",
        ],
    },
)
