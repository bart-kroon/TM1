# Contributing

Contributions are expected to be in the form of merge requests to the [MPEG-internal repository](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git). The [public repository](https://gitlab.com/mpeg-i-visual/tmiv.git) is a mirror of the internal repository's `master` branch.

## Semantic versioning of releases

Releases have semantic versioning x.y.z:

- x: Major releases, no compatibility required between them
- y: Minor releases, we require forward bitstream compatibility (e.g. 6.1 can read the bitstream produced by 6.0)
- z: Bug fixes and non-code improvements (e.g. license, manual)

## Branches

This repository has the following branch model:

- Branch `master` is always at the latest release and in sync with the [public mirror](https://gitlab.com/mpeg-i-visual/tmiv) where the test model and reference software are published. Only masters can push and nobody can merge.
- Branch `integration` is working on the next major/minor release out of the last MPEG meeting. Pushing is forbidden and only masters can merge. Developers need to do merge requests. If a master of one organization creates a merge request, another organization performs code review and merges.
- Branch `vX.0-dev` is an experiment to allow collaborate work on non-controversial topics in preparation of the TMIV X major release. For the latest `vX.0-dev` branch, masters can push and merge. Developers need to do merge requests.
- Branch `m12345` is the proponent branch to document m12345 and will be deleted after the MPEG meeting when integrated or rejected
- Issue branches as created by gitlab, will be deleted after the merge request

Masters are @fleureauj, @franck.thudor, @vinod_mv and @bartkroon.

## Code guidelines

NOTE: This section may be expanded by the software coordinators based on what comes in on code reviews.

### Comments

- The code should be readable when all comments are stripped.
- Do not add meaningless comments.
- Do not add `// m12345 Proposal something` lines. (Here m12345 is placeholder for the MPEG document number.)
- use `// TODO(initials/m12345): Something` when there is something that needs improvement that is noted but out-of-scope of the work at hand.
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
- Do not use C++ explicit casts `int()` for readablity, use `static_cast<>` instead.
- Using C-style cast was depricated before most of us are born

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
- When you already know your syntax is adopted, preferably work on the specifiation first and implement it *exactly* like edited, thus including any editorial changes by the editors.
- This is a test model: write for readability and algorithmic complexity, but do not optimize

### Syntax structures

Syntax structures are in this context defined by the MIV and V-PCC/V3C specification and don't refer to C++ syntax. When you parse a syntax structure, you obtain the syntax element values and any variables that are defined as part of the semantics. When you format a syntax structure, you take the syntax element values and variables from the semantics and into a syntax structure.

### Implementing a new syntax structure

- Add a parser/formatter to the MivBitstreamLib, named exactly like the syntax structure but in `uppperCamelCase` notation, e.g. `v3c_parameter_set()` --> `V3cParameterSet`
- Add a comment box on top of the class definition that lists all the limitations.
    - Indicate if the limitation is due to MIV e.g. `asps_long_term_ref_atlas_frames_flag == 0` in `RefListStruct` (in TMIV 6.1)
    - or due to the implementation, e.g. `vui_hrd_parameters_present_flag = 0` in `VuiParameters` (in TMIV 6.1)
- Although most current modules (=.cpp/.hpp/.h tuple) are at RBSP level, containing all syntax structures carried within, it is allowed to have a new module for a new syntax structure however small or big.
- The public interface has to match exactly with the syntax structure. TODO(bartkroon) can we offer addtional API?
- The implementation of the getters and setters shall check all semantics that can be checked in that context.
- The parser/formatter (decodeFrom/encodeTo) shall check all semantics that can be checked in that context.

