from glob import glob
from setuptools import setup


setup(
    name="eva_suit",
    version="0.1.0",
    packages=["eva_suit"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/eva_suit"]),
        ("share/eva_suit", ["package.xml", "README.md"]),
        ("share/eva_suit/config", glob("config/*.yaml")),
        ("share/eva_suit/deploy", glob("deploy/*")),
        ("share/eva_suit/firmware/flex", glob("firmware/flex/*")),
    ],
    install_requires=["setuptools", "pyserial>=3.5,<4", "pynmea2>=1.15,<2", "PyYAML>=5.4"],
    zip_safe=True,
    maintainer="Batmobile Team",
    maintainer_email="team@example.invalid",
    description="Offline, non-actuating EVA suit sensors, voice, ROS backend and touchscreen GUI",
    url="https://github.com/Abiralsaba/Batmobile",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "eva_backend = eva_suit.backend:main",
        "eva_gui = eva_suit.gui:main",
    ]},
)
