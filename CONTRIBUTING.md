# Contributing

Currently, merge requests are not accepted at the [public gitlab repository](https://gitlab.com/mpeg-i-visual/tmiv.git), only in the [internal MPEG gitlab repository](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git).

## Semantic versioning of releases

Releases have semantic versioning x.y.z:

- x: No compatibility between major releases
- y: Since TMIV 6, we require forward bitstream compatibility for minor releases (6.1 can read 6.0 bitstream)
- z: bug fixes and non-code improvements (e.g. license, manual)

## Branches

This repository has the following branch model:

- Branch `master` is always at the latest release and in sync with the [public mirror](https://gitlab.com/mpeg-i-visual/tmiv) where the test model and reference software are published. Only masters can push and nobody can merge.
- Branch `integration` is working on the next major/minor release out of the last MPEG meeting. Pushing is forbidden and only masters can merge. Developers need to do merge requests. If a master of one organization creates a merge request, another organization performs code review and merges.
- Branch `vX.0-dev` is an experiment to allow collaborate work on non-controversial topics in preparation of the TMIV X major release. For the latext `vX.0-dev` branch, masters can push and merge. Developers need to do merge requests.
- Branch `m12345` is the proponent branch to document m12345 and will be deleted after the MPEG meeting when integrated or rejected
- Issue branches as created by gitlab, will be deleted after the merge request

Currently, masters are @fleureauj, @franck.thudor, @vinod_mv and @bartkroon. In holiday season, we sometimes struggle with this and we cannot always keep this rule.
