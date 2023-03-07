#!/usr/bin/env python3
#
# The copyright in this software is being made available under the BSD
# License, included below. This software may be subject to other third party
# and contributor rights, including patent rights, and no such rights are
# granted under this license.
#
# Copyright (c) 2010-2023, ISO/IEC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the ISO/IEC nor the names of its contributors may
#    be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.


import argparse
import json
import pathlib


class VtAnchorSelectCameras:
    def __init__(self, args):
        self.maxLumaSampleRate = args.max_luma_sample_rate
        self.leaveNOut = args.leave_n_out
        self.componentCount = args.component_count

        if isinstance(args.config, pathlib.Path):
            with open(args.config) as stream:
                config = json.load(stream)

            self.basename = args.config.name
            self.delimiter = args.delimiter
            self.prefixBasename = args.prefix_basename
        else:
            config = args.config

        self.cameras = config["cameras"]
        self.frameRate = config["Fps"]
        self.sourceCameraNames = config["sourceCameraNames"]

    def run(self):
        prefix = "{}: ".format(self.basename[0]) if self.prefixBasename else ""
        print("{}{}".format(prefix, self.delimiter.join(self.targetCameraNames())))

    def fullViewCount(self):
        return len(self.sourceCameraNames)

    def fullSequenceRate(self):
        result = 0.0
        lastResolution = None

        for camera in self.cameras:
            if camera["Name"] in self.sourceCameraNames:
                resolution = camera["Resolution"]
                if lastResolution and (resolution != lastResolution):
                    raise RuntimeError(
                        "This implementation is (currently) restricted to sequences that have views of equal resolution"
                    )
                lastResolution = resolution
                result += self.componentCount * resolution[0] * resolution[1] * self.frameRate

        return result

    def targetCameraNames(self):
        result = []

        targetViewCount = min(
            self.fullViewCount() - self.leaveNOut,
            int(self.fullViewCount() * self.maxLumaSampleRate / self.fullSequenceRate()),
        )
        for outputViewIdx in range(targetViewCount):
            inputViewIdx = int((outputViewIdx + 0.5) * self.fullViewCount() / targetViewCount)
            result.append(self.sourceCameraNames[inputViewIdx])

        return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        "config",
        help="The path of the sequence configuration (.json)",
        type=pathlib.Path,
    )
    parser.add_argument(
        "-p",
        "--max-luma-sample-rate",
        help="The maximum luma sample rate [Hz] over all video streams combined",
        type=float,
        default=1069547520.0,
    )
    parser.add_argument(
        "-n",
        "--leave-n-out",
        help="Leave at least n views out to enable objective evaluation on source view positions",
        type=int,
        default=0,
    )
    parser.add_argument(
        "-c",
        "--component-count",
        help="Component count, defaults to two (texture and geometry)",
        type=int,
        default=2,
    )
    parser.add_argument(
        "-d",
        "--delimiter",
        help="The delimiter used to to output the target camera names",
        type=str,
        default=", ",
    )
    parser.add_argument(
        "--prefix-basename",
        help="Prefix the output with the base name of the sequence configuration",
        action="store_true",
    )

    app = VtAnchorSelectCameras(parser.parse_args())
    app.run()
