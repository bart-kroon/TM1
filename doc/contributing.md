# Contributing

Contributions are expected to be in the form of merge requests to the [MPEG-internal repository](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git). The [public repository](https://gitlab.com/mpeg-i-visual/tmiv.git) is a mirror of the internal repository's `main` branch.

Maintainers of this project are @bartkroon (Philips) and @franck.thudor (InterDigital).

## Testing

The following levels of testing have been defined:
1. Continuous integration (CI)
1. Integration test
1. Compare against 3-frame CTC anchors
1. Compare against 17-frame CTC anchors
1. Compare against 97-frame CTC anchors

### Continuous integration (CI)

   - Complexity: approx. 6min @ 1 core, no I/O
   - Runs automatically on on the default branch and on each new commit of a merge request
   - Includes:
      - Multiplatform build
      - Static analysis
      - Code formatting check
      - Unit tests (target: `test`)
      - Code coverage report
   - Success criteria: fully automated

### Integration test (target: `integration_test`)

   - Complexity: approx. 8min @ 12 cores, writes 1 GB to disk
   - To be run by either the developer or the code viewer run the test on their system
   - Success criteria:
      - When the test stops with an error then the test has failed
      - Missing output files trigger an error
      - Differences in output files need to be expected/explainable

### Compare against further reduced frame (3) CTC anchors,

   1. Complexity: much higher
   1. TMIV encoder parameters include `-n 3 -p intraPeriod 2`
   1. Anchors that are provenly unchanged (e.g. by the integration test) may be skipped
   1. The developer uses his own scripts and system to run the test
   1. The developer fills in and shares the CTC reporting template
   1. The code reviewer does a small spot check to crosscheck the results

### Compare against reduced-frame (17) CTC anchors

   1. Complexity: much higher
   1. Anchors that are provenly unchanged (e.g. by the integration test) may be skipped
   1. The developer uses his own scripts and system to run the test
   1. The developer fills in and shares the CTC reporting template
   1. The code reviewer does a small spot check to crosscheck the results

### Compare against full-frame (97) CTC anchors with 300-frame pose traces

   1. Complexity: much higher
   1. The developer uses his own scripts and system to run the test
   1. The developer fills in and shares the CTC reporting template
   1. The code reviewer does a small spot check to crosscheck the results

### Appropriate level of testing

The appropriate level of testing (for testing instructions, see [testing.md](/doc/testing.md)) depends on the changes in the merge request, e.g.:
  * Non-code change like manual: code reviewer reads through the changes
  * Full code coverage of an isolated unit: CI is sufficient (no need for the code reviewer to clone the branch)
  * Changes across units (incl. HLS) w/o change to YUV files: integration test
  * Improved test model performance on a frame basis: 3 frames
  * Improved test model performance with a temporal aspect: 17 frames
  * Improved test model performance relating to intra periods or pose traces: 97 frames, incl. 300-frame pose traces
  * New modes not covered by CTC anchor or integration test: provide a suitable new test

### Dividing work into multiple merge requests

When code has different levels of testability, it is often possible to split work in multiple merge requests. For instance, a new functionality may:
1. include a new unit: CI is sufficient, 
1. make some changes to the encoder w/o changing outputs: integration test is sufficient,
1. change or add configurations to enable the new functionality: relevant CTC anchor or a new test

By splitting off the easy-to-test parts there is less uncertainty in the final MR. This will speed up the code review and improve the review quality.

### Limit on the number of open merge requests

To avoid duplicate work, merge conflicts and/or confusion, we impose a limit on the number of open merge requests:

- Open merge requests are counted regardless of their "WIP", "Draft" or "ready" status.
- Open merge requests that are labeled ~"Test Model" are not counted
- A developer that does not yet have an open merge request, is allowed to open one when there are less than six (6).
- A developer that already has at least one open merge request, is allowed to open one more when there are less than four (4).
- A maintainer that cannot progress on any open merge request, is allowed to have at most three (3).
- Merge requests are intended to have a short lifetime. Maintainers may close merge requests that are open for more than a week.

## Semantic versioning of releases

Releases have semantic versioning x.y.z:

- x: Major releases, no compatibility required between them
- y: Minor releases, non-CTC releases, e.g. implementing more proposals and general improvements
- z: Patch releases, bug fixes and non-code improvements (e.g. license, manual)

Releases are tagged like `vx.y` or `vx.y.z`. The first patch of `v8.0` is `v8.0.1`. 

NOTE: In general there is no bitstream compatibility between minor releases because it is an ongoing effort to make the test model conformant with the specification.

## Branches

This repository has the following branch model:

* The [public mirror](https://gitlab.com/mpeg-i-visual/tmiv) is updated whenever there is a new release
* Branch `main` is working on the next major/minor/patch release out of the last MPEG meeting. Pushing is forbidden and only maintainers can merge. Developers need to do merge requests. If a software coordinator of one organization creates a merge request then a software coordinator of another organization performs code review and merges.
* When an existing release needs to be patched, a `vx.y-dev` branch (e.g. `v8.0-dev`) is created. These branches are protected in the same way as the main branch and can also be the target of merge requests. These branches are removed as soon as the new release is tagged (e.g. `v8.0.2`).
* Branch `m12345` is the proponent branch to document m12345 and will be deleted after the MPEG meeting when integrated or rejected
* Issue branches as created by gitlab, will be deleted after the merge request

## Code guidelines

NOTE: This section may be expanded by the software coordinators based on what comes in on code reviews.

### Comments

- The code should be readable when all comments are stripped.
- Do not add meaningless comments.
- Do not add `// m12345 Proposal something` lines. (Here m12345 is placeholder for the MPEG document number.)
- Do not add `// TODO's` lines. Instead open new issues for aspects that need improvement but are outside of the scope of the current merge request.
- Use `// NOTE(initials/m12345): Something` to make mention of something important (e.g. a property of the code) that may go unnoticed otherwise (and accidentall destroyed later):
    - `// NOTE(BK): The class interface deliberately disallows integer computations`
    - `// NOTE(BK): Stable ordering`

### Static analysis tools

- There is a zero warning policy to avoid the simple bugs
- Use Clang Tidy with the provided `.clang-tidy` file
- Preferably use `-Werror -Wall -Wextra -Wpedantic` on GCC and Clang
- Without access to these tools, you may ask for a build log from the software coordinators

### Formatting

- Preferably use Clang Format with the provided `.clang-format` file
- When not using Clang Format, at least try to follow the style to keeps diffs small
- The software coordinators may format contributions in code review

#### Integer casting

- Avoid casting integers when possible. This is not always possible, for instance `vps_atlas_count_minus1()` returns a `std::uint8_t` to match with the specification but `std::vector<>::size()` returns a `std::size_t`.
- Use curly braces (unified constructor syntax) for implicit casts, e.g. `int{vps_atlas_count_minus1()}`
- Use `static_cast<>` for explicit casts, e.g. `static_cast<int>(vector.size())`
- Do not use C++ explicit casts `int()` for readability, use `static_cast<>` instead.
- Using C-style cast was deprecated before most of us are born

### Naming of identifiers

- CMake modules, C++ namespaces and C++ classes are in `UpperCamelCase` notation
- C++ variables are in `lowerCamelCase` notation, with the following exception:
  - Syntax elements are named exactly like in the specification, e.g. `vps_frame_width`
  - No such exception is made for parser/formatter of a syntax structure (see [below](#syntax-structures)), e.g. `v3c_parameter_set()` --> `V3cParameterSet`
- Avoid unnecessary abbreviations
  - Abbreviations that are defined in ISO/IEC 23090-12 Clause 3 _Terms and Definitions_ are allowed
  - Some commonly-used TMIV-specific classes are also abbreviated, e.g. `ViewParamsList` --> `vpl`
  - Avoid non-standard abbreviations

### Implementing proposals

- When writing software for a proposal, assume that your proposal will be adopted in the specification. (If not already known.)
- When you already know your syntax is adopted, preferably work on the specification first and implement it *exactly* like edited, thus including any editorial changes by the editors.
- This is a test model: write for readability and algorithmic complexity, but do not optimize

### Syntax structures

Syntax structures are in this context defined by the MIV and V-PCC/V3C specification and don't refer to C++ syntax. When you parse a syntax structure, you obtain the syntax element values and any variables that are defined as part of the semantics. When you format a syntax structure, you take the syntax element values and variables from the semantics and into a syntax structure.

### Implementing a new syntax structure

- Add a parser/formatter to the MivBitstreamLib, named exactly like the syntax structure but in `uppperCamelCase` notation, e.g. `v3c_parameter_set()` --> `V3cParameterSet`
- Add a comment box on top of the class definition that lists all the limitations.
  - Indicate if the limitation is due to MIV e.g. `asps_long_term_ref_atlas_frames_flag == 0` in `RefListStruct` (in TMIV 6.1)
  - or due to the implementation, e.g. `vui_hrd_parameters_present_flag = 0` in `VuiParameters` (in TMIV 6.1)
- Although most current modules (=.cpp/.hpp/.h tuple) are at RBSP level, containing all syntax structures carried within, it is allowed to have a new module for a new syntax structure however small or big.
- The public interface has to match exactly with the syntax structure.
- The implementation of the getters and setters shall check all semantics that can be checked in that context. For those checks, use
  - `VERIFY_V3CBITSTREAM()`and  `VERIFY_MIVBITSTREAM()`, which are almost the same but the messages are different. You can find examples where in the same code both are used: the first to check the parsing and the second to check if a MIV restriction was applied.
  - `LIMITATION()` for when TMIV does not implement everything it should do. Again, that is just a different message.
- The parser/formatter (decodeFrom/encodeTo) shall check all semantics that can be checked in that context.

## Best practices for development

### Break on exceptions

If you want to debug bitstream related errors, e.g. if `VERIFY_*BITSTREAM` is throwing an exception, it is a good idea to set your IDE to break on unhandled exceptions ([Visual Studio instructions](https://docs.microsoft.com/en-us/visualstudio/debugger/managing-exceptions-with-the-debugger), [CLion instructions](https://www.jetbrains.com/help/clion/using-breakpoints.html#exception-breakpoints)) and start a debugging session.
