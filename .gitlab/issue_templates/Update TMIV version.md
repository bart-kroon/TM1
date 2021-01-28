1. Create new branch
1. Create tag for previous version on new branch (this will create a [release](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/-/releases))
1. Bump version in `CMakeLists.txt`, lines 6 and 73
1. Bump version in `gitlab-ci.yml`, variable `TMIV_VERSION`. Ignore the third place of the version number.
1. Fix badge link in `Readme.md`
