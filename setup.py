"""Rqt GUI plugin."""

from setuptools import setup

PACKAGE_NAME = "alert_dashboard_rqt"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME, ["plugin.xml"]),
        ("share/" + PACKAGE_NAME + "/resource", ["resource/mainwindow.ui", "resource/dashboard_rqt.perspective", "resource/estop_rqt.perspective"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shubham Pawar",
    maintainer_email="skpawar1305@gmail.com",
    description="rqt GUI plugin.",
    license="",
    entry_points={
    },
)
