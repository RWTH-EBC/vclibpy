"""Setup.py script for the vclibpy-framework"""

import setuptools

# read the contents of your README file
from pathlib import Path
readme_path = Path(__file__).parent.joinpath("README.md")
long_description = readme_path.read_text()

INSTALL_REQUIRES = [
    'numpy',
    'coolprop',
    'matplotlib',
    'pandas',
    'sdf',
    'ctREFPROP'
]

with open(Path(__file__).parent.joinpath("vclibpy", "__init__.py"), "r") as file:
    for line in file.readlines():
        if line.startswith("__version__"):
            VERSION = line.replace("__version__", "").split("=")[1].strip().replace("'", "").replace('"', '')

setuptools.setup(
    name='vclibpy',
    version=VERSION,
    description="Package with functions for the vapor "
                "compression process design using steady state simulation models",
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/RWTH-EBC/vclibpy',
    download_url=f'https://github.com/RWTH-EBC/vclibpy/archive/refs/tags/{VERSION}.tar.gz',
    license='BSD 3-Clause',
    author='RWTH Aachen University, E.ON Energy Research Center, Institute '
           'of Energy Efficient Buildings and Indoor Climate',
    author_email='fabian.wuellhorst@eonerc.rwth-aachen.de',
    # Specify the Python versions you support here. In particular, ensure
    # that you indicate whether you support Python 2, Python 3 or both.
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
    ],
    keywords=[
        "vapor-compression", "heat pumps",
        "chillers", "steady-state simulation"
    ],
    packages=setuptools.find_packages(exclude=['tests', 'tests.*', 'img']),
    install_requires=INSTALL_REQUIRES,
)
