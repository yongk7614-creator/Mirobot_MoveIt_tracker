from setuptools import find_packages, setup


package_name = "wlkatapython"

setup(
    name=package_name,
    version="0.1.1",
    description="WLKATA Mirobot/E4/MT4/Haro380/MS4220 Python serial SDK.",
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    author="DSS",
    packages=find_packages(include=["*", "tests*"]),
    py_modules=["wlkatapython"],
    install_requires=["pyserial"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
