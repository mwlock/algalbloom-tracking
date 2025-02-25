<h1 align="center">Adaptive Sampling of Algal Blooms
Using Autonomous Underwater Vehicle and Satellite Imagery:
Experimental Validation in the Baltic Sea</h1>

[<img src="/figures/banner.png" width="100%">]()

> This repo serves as a showcase for the final paper. See [Joana Fonseca's fork](https://github.com/JoanaFonsec/algalbloom-tracking) for implementation details.

> Paper aviailable at [arXiv](https://arxiv.org/abs/2305.00774)

## Abstract
This paper investigates using satellite data to improve adaptive sampling missions, particularly for front tracking
scenarios such as with algal blooms. Our proposed solution to find
and track algal bloom fronts uses an Autonomous Underwater
Vehicle (AUV) equipped with a sensor that measures the concentration of chlorophyll a and satellite data. The proposed method
learns the kernel parameters for a Gaussian process (GP) model
using satellite images of chlorophyll a from the previous days.
Then, using the data collected by the AUV, it models chlorophyll a
concentration online. We take the gradient of this model to obtain
the direction of the algal bloom front and feed it to our control
algorithm. The performance of this method is evaluated through
realistic simulations for an algal bloom front in the Baltic sea,
using the models of the AUV and the chlorophyll a sensor.
We compare the performance of different estimation methods,
from GP to curve interpolation using least squares. Sensitivity
analysis is performed to evaluate the impact of sensor noise on
the methods’ performance. We implement our method on an
AUV and run experiments in the Stockholm archipelago in the
summer of 2022.

## Citation

If you find this work useful, please consider citing it:

```bibtex
@article{fonseca2025adaptive,
  author = {Joana Fonseca, Sriharsha Bhat, Matthew Lock, Ivan Stenius, Karl H. Johansson},
  title = {Adaptive Sampling of Algal Blooms Using Autonomous Underwater Vehicles and Satellite Imagery: Experimental Validation in the Baltic Sea},
  journal = {Journal TBD},
  year = {2025}
}
```

