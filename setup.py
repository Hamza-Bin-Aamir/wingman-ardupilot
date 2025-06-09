from setuptools import setup, find_packages

setup(
    name="wingman-ardupilot",
    version="0.0.1",
    description="A high-level wrapper for pymavlink to simplify UAV programming.",
    long_description=open("readme.md").read(),
    long_description_content_type="text/markdown",
    author="Hamza Bin Aamir",
    license="BSD",
    packages=find_packages(),
    install_requires=[
        "pymavlink==2.4.47"
    ],
    python_requires=">=3.7",
    include_package_data=True,
    url="",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD License",
        "Operating System :: OS Independent",
    ],
)