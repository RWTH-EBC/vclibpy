![E.ON EBC RWTH Aachen University](https://github.com/RWTH-EBC/ebcpy/blob/master/docs/EBC_Logo.png)

[![pylint](https://rwth-ebc.github.io/vclibpy/main/pylint/pylint.svg )](https://rwth-ebc.github.io/vclibpy/main/pylint/pylint.html)
[![documentation](https://rwth-ebc.github.io/vclibpy/main/docs/doc.svg)](https://rwth-ebc.github.io/vclibpy/main/docs/index.html)
[![coverage](https://rwth-ebc.github.io/vclibpy/main/coverage/badge.svg)](https://rwth-ebc.github.io/vclibpy/main/coverage)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![build](https://rwth-ebc.github.io/vclibpy/main/build/build.svg)](https://rwth-ebc.github.io/vclibpy/main/build/build.svg)
[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RWTH-EBC/vclibpy/main?labpath=docs%2Fjupyter_notebooks)

# VCLibPy

Repository with a **V**apor **C**ompression **Lib**rary in **Py**thon for steady state process design and simulation.
It enables use of RefProp and CoolProp as well as different compressors, heat exchangers and flowsheet configurations for heat pumps and chillers.

# Installation

To install, run
```
pip install vclibpy
```
To use RefProp, you have to buy the licence and obtain a valid .dll file.

In order to help development, install it as an egg:

```
git clone https://github.com/RWTH-EBC/vclibpy
pip install -e vclibpy
```

# How to get started?

We recommend running our jupyter-notebook to be guided through a **helpful tutorial**.  
For this, we prepared several examples, which we encourage to check one by one.
To use, you can either run the code locally or in a [browser using juypter-notebook](https://mybinder.org/v2/gh/RWTH-EBC/vclibpy/main?labpath=docs%2Fjupyter_notebooks):

If the web-hosting is not available, you can run the notebooks locally with the following code:
```
pip install jupyter
git clone https://github.com/RWTH-EBC/vclibpy
jupyter notebook vclibpy/docs/jupyter_notebooks
```

Or, clone this repo and look at the folder `examples`.
Those examples are the same as the jupyter notebooks.

# How to cite vclibpy

`vclibpy` is currently in the process of publication. A corresponding DOI will be added soon.

# Documentation
Visit our official [Documentation](https://rwth-ebc.github.io/vclibpy/main/docs/index.html).

# Problems?
Please [raise an issue here](https://github.com/RWTH-EBC/vclibpy/issues/new).

