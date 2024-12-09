## Wiki
You can build the HELIOS++ Wiki locally as html files using `mkdocs`. For this, the wiki is included as a `git submodule`.

Make sure you have the latest copy of the submodule checked out with `git submodule update --init --recursive` or, if not checking out for the first time, `git submodule update --recursive --remote` (to update to latest tip of the submodule). 

Then change to the respective directory: `cd wiki-repo`

And run mkdocs (you may need to install it and any required packages using `pip` or other python package managers):

`python -m mkdocs build`

The local wiki build is then generated in the `site` subdirectory.