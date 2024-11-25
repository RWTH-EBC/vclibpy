![E.ON EBC RWTH Aachen University](https://github.com/RWTH-EBC/ebcpy/blob/master/docs/EBC_Logo.png)

[![pylint](https://rwth-ebc.github.io/vclibpy/main/pylint/pylint.svg )](https://rwth-ebc.github.io/vclibpy/main/pylint/pylint.html)
[![documentation](https://rwth-ebc.github.io/vclibpy/main/docs/doc.svg)](https://rwth-ebc.github.io/vclibpy/main/docs/index.html)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![build](https://rwth-ebc.github.io/vclibpy/main/build/build.svg)](https://rwth-ebc.github.io/vclibpy/main/build/build.svg)
[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RWTH-EBC/vclibpy/main?labpath=docs%2Fjupyter_notebooks)
[![DOI](https://zenodo.org/badge/725581067.svg)](https://doi.org/10.5281/zenodo.14202185)

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

`VCLibPy` is currently in the process of publication. 
Until the paper is published, you may cite the following DOI: https://doi.org/10.5281/zenodo.14202185

Moreoever, the following list of related publications show `VCLibPy` in use.

## Related publications

- Vering, C., Wüllhorst, F., Mehrfeld, P., & Müller, D. (2021). Towards an integrated design of heat pump systems: Application of process intensification using two-stage optimization. Energy Conversion and Management, 250, 114888.  https://doi.org/10.1016/j.enconman.2021.114888
- Vering, Christian; Müller, Dirk (Thesis advisor); Elbel, Stefan (Thesis advisor) (2023). Optimal design of heat pump systems for existing buildings, PhD thesis. https://doi.org/10.18154/RWTH-2023-04070

# Documentation
Visit our official [Documentation](https://rwth-ebc.github.io/vclibpy/main/docs/index.html).

# Problems?
Please [raise an issue here](https://github.com/RWTH-EBC/vclibpy/issues/new).

# Acknowledgements

We gratefully acknowledge the financial support by the Federal Ministry for Economic Affairs and Climate Action (BMWK), promotional reference 03EN1022B, as well as the European Regional Development Fund (ERDF) (ERDF-0500029).

<img src="https://github.com/RWTH-EBC/BESMod/blob/main/BESMod/Resources/Images/BMWK_logo.png" alt="BMWK" width="200"/>

