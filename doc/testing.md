# Running the software tests

In general and especially when [contributing to TMIV](/doc/contributing.md), it is a good idea to run the software tests before starting any (large) experiments. TMIV has two types of software tests that are integrated into the project.

## Unit tests

The unit tests check aspects of the software in isolation and without I/O. Running the 100+ unit tests only takes a couple of seconds in total.

To run all unit tests, execute the following command inside your build directory:

```shell
cmake --build . --parallel --config Release --target test
```

For Visual Studio, please substitute `test` for `RUN_TESTS`.

## Integration tests

The integration test runs the TMIV executables on real data to make sure that there are no runtime errors in a range of conditions, and it reports if any bitstreams or YUV files have changed compared to the reference (when provided).

To run the test first _build_, _test_ and **install** the project. The integration test itself is a Python 3.5 script:

```shell
$ python /Workspace/tmiv/scripts/test/integration_test.py
Usage: integration_test.py TMIV_INSTALL_DIR TMIV_SOURCE_DIR CONTENT_DIR TEST_DIR [-g GIT_COMMAND] [-j MAX_WORKERS] [-r REFERENCE_MD5_FILE]
```

whereby:

* `python` needs to be a Python 3 interpreter, at least version 3.5. On some systems the command may be `python3` instead. (Try `python --version`.)
* In this manual, TMIV_INSTALL_DIR is set as `/Workspace/tmiv_install`
* In this manual, TMIV_SOURCE_DIR is set as `/Workspace/tmiv`
* In this manual, CONTENT_DIR is set as `/Content`
* TEST_DIR is the output directory of the configuration file. It is advised to delete and recreate the directory before each run.
* GIT_COMMAND is typically `git`. This option may be omitted but when provided the revision of TMIV is saved to the output directory for future reference.
* The MAX_WORKERS parameter controls the number of threads. This is a runtime/memory consumption tradeoff. The default value is typically good enough.
* The REFERENCE_MD5_FILE:
    * is omitted to create the reference or for manual comparison between different runs (using `diff` or `WinMerge`).
    * is set equal to an existing reference file containing output file MD5 sums to compare against it. This is highly recommended.
* Try `--help` for advanced options.
