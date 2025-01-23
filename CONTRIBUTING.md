# Contribution Guide

Helios++ welcomes contributions from its community. If you want to contribute to read through this guide which aims to clarify the contribution process.

## Planning contribution

If you want to contribute to the project without a specific contribution in mind, please look at the [issue tracker](https://github.com/3dgeo-heidelberg/helios/issues) for an issue looking interesting to you. It is good practice to post on the issue that you intend to work on the issue. This avoids duplication of work and allows maintainers to give you valueable insight for your contribution.

If you plan to contribute a specific feature relevant to your work, please [open an issue](https://github.com/3dgeo-heidelberg/helios/issues/new/choose) before you start working on the feature. This allows maintainers to give you feedback on your ideas and allows upfront confirmation that such change would be merged into the code base when finished.

## Development model

The Helios++ git repository has currently two persisting branches: `main` and `alpha-dev`. `main` contains the latest development version of Helios++ and is used to create its stable releases. `alpha-dev` fulfills the same role for drastic changes which will only be part of a stable release in the next major release series. For your contributions, you should create a branch based on the current state of `main`. After development, you should push this branch to your fork of the repository and open a pull request against the `main` branch of the Helios++ main repository. Extensive Continuous Integration tests will run automatically on this PR. Please inspect the results of these tests and iterate your development until the tests pass. Then, please ping the maintainers for a code review. This workflow is widely known as [*GitHub Flow*](https://docs.github.com/en/get-started/using-github/github-flow).

## Development installation

A suitable setup for development is documented [the installation section of the README](https://github.com/3dgeo-heidelberg/helios?tab=readme-ov-file#development-installation)/

## Licensing and Copyright

Helios++ uses a shared copyright model. This means that you keep the rights to your contributions, but agree to publish it under the [LGPL v3 license](https://github.com/3dgeo-heidelberg/helios/blob/main/COPYING.LESSER). When you contribute for the first time, feel free to add your name to the `authors` section of [CITATION.cff](https://github.com/3dgeo-heidelberg/helios/blob/main/CITATION.cff).
