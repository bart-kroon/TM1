# Contributing

Contributions are expected to be in the form of merge requests to the [MPEG-internal repository](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git). The [public repository](https://gitlab.com/mpeg-i-visual/tmiv.git) is a mirror of the `master` branch.

## Semantic versioning of releases

Releases have semantic versioning x.y.z:

- x: Major releases, no compatibility required between them
- y: Minor releases, we require forward bitstream compatibility (e.g. 6.1 can read the bitstream produced by 6.0)
- z: Bug fixes and non-code improvements (e.g. license, manual)

## Branches

This repository has the following branch model:

- Branch `master` is always at the latest release and in sync with the [public mirror](https://gitlab.com/mpeg-i-visual/tmiv) where the test model and reference software are published. Only masters can push and nobody can merge.
- Branch `integration` is working on the next major/minor release out of the last MPEG meeting. Pushing is forbidden and only masters can merge. Developers need to do merge requests. If a master of one organization creates a merge request, another organization performs code review and merges.
- Branch `vX.0-dev` is an experiment to allow collaborate work on non-controversial topics in preparation of the TMIV X major release. For the latext `vX.0-dev` branch, masters can push and merge. Developers need to do merge requests.
- Branch `m12345` is the proponent branch to document m12345 and will be deleted after the MPEG meeting when integrated or rejected
- Issue branches as created by gitlab, will be deleted after the merge request

Masters are @fleureauj, @franck.thudor, @vinod_mv and @bartkroon.
