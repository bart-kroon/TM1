1. [ ] **Only** for a patch release (e.g. 8.0.1), open a development branch (e.g. `v8.0-dev`) from the release tag on which the patch will be based, e.g. v8.0-dev
1. [ ] Bump the version in [CMakeLists.txt](CMakeLists.txt) (two occurrences)
1. (Open and handle more merge requests on the branch.)
1. [ ] Create a tag on the relevant branch (e.g. `main` or `v8.0-dev`) with release notes in the same style as for previous tags
1. [ ] Mirror that branch to https://gitlab.com/mpeg-i-visual/tmiv (using a `git push -f --tag`)
