#!/usr/bin/env python3

# The copyright in this software is being made available under the BSD
# License, included below. This software may be subject to other third party
# and contributor rights, including patent rights, and no such rights are
# granted under this license.
#
# Copyright (c) 2010-2022, ISO/IEC
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
import concurrent.futures
from difflib import Differ
import hashlib
from pathlib import Path
import subprocess
import sys
import time

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(SCRIPTS_DIR))

from common.video_types import Resolution


def dirPath(path_string: str) -> Path:
    if Path(path_string).is_dir():
        return Path(path_string)
    else:
        raise NotADirectoryError(path_string)


def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "tmiv_install_dir", type=dirPath, help="Directory with the TM1 binaries, includes, etc"
    )
    parser.add_argument("tmiv_source_dir", type=dirPath, help="Root of the TM1 repository")
    parser.add_argument("content_dir", type=dirPath)
    parser.add_argument("output_dir", type=Path, help="Output files will be stored here")
    parser.add_argument("-g", "--git-command", type=str)
    parser.add_argument("-j", "--max-workers", type=int)
    parser.add_argument("-r", "--reference-md5-file", type=Path)
    parser.add_argument(
        "--dry-run", action="store_true", help="Only print TMIV commands without executing them"
    )
    parser.add_argument(
        "--deps-install-dir",
        type=dirPath,
        help="Directory with the installed TM1 dependencies",
        default=None,
    )

    args = parser.parse_args()

    if not args.deps_install_dir:
        args.deps_install_dir = args.tmiv_install_dir

    return args


def computeMd5Sum(fileName: Path):
    hash_md5 = hashlib.md5()
    with open(fileName, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()


class IntegrationTest:
    def __init__(self):
        minVersion = (3, 5)
        if sys.version_info < minVersion:
            raise RuntimeError("This script requires Python {}.{} or newer".format(*minVersion))

        args = parseArguments()

        self.tmivInstallDir = args.tmiv_install_dir.absolute().resolve()
        self.depsInstallDir = args.deps_install_dir.absolute().resolve()
        self.tmivSourceDir = args.tmiv_source_dir.absolute().resolve()
        self.contentDir = args.content_dir.absolute().resolve()
        self.testDir = args.output_dir.absolute().resolve()
        self.gitCommand = args.git_command
        self.maxWorkers = args.max_workers
        self.referenceMd5File = (
            args.reference_md5_file.absolute().resolve() if args.reference_md5_file else None
        )
        self.dryRun = args.dry_run
        self.md5sums = []
        self.testDir.mkdir(parents=True, exist_ok=True)
        self.md5sumsFile = self.testDir / "integration_test.md5"

        self.stop = False

    def run(self):
        if not self.dryRun:
            print("{{0}} = {}".format(self.tmivInstallDir))
            print("{{1}} = {}".format(self.tmivSourceDir))
            print("{{2}} = {}".format(self.contentDir))
            print("{{3}} = {}".format(self.testDir), flush=True)
            print("{{4}} = {}".format(self.depsInstallDir))

        app.inspectEnvironment()

        with concurrent.futures.ThreadPoolExecutor(max_workers=self.maxWorkers) as executor:
            futures = self.testMivAnchor(executor)
            futures += self.testMivViewAnchor(executor)
            futures += self.testOneView(executor)
            futures += self.testMivDsdeAnchor(executor)
            futures += self.testBestReference(executor)
            futures += self.testAdditiveSynthesizer(executor)
            futures += self.testMivMpi(executor)
            futures += self.testFramePacking(executor)
            futures += self.testEntityCoding(executor)
            futures += self.testExplicitOccupancy(executor)
            self.sync(futures)

        if not self.dryRun:
            self.storeMd5Sums()

            if self.referenceMd5File:
                return self.compareMd5Files()

        return 0

    def inspectEnvironment(self):
        self.checkIfProbeExistsInDir(
            "TMIV installation",
            self.tmivInstallDir,
            Path("include") / "TMIV" / "Decoder" / "DecodeMiv.h",
        )
        self.checkIfProbeExistsInDir("TMIV source", self.tmivSourceDir, Path("README.md"))
        self.checkIfProbeExistsInDir(
            "content", self.contentDir, Path("E") / "v13_texture_480x270_yuv420p10le.yuv"
        )

        self.testDir.mkdir(exist_ok=True)

        if self.gitCommand and not self.dryRun:
            with open(self.testDir / "git.log", "w") as stream:
                for target in [None, stream]:
                    subprocess.run(
                        [self.gitCommand, "log", "-n", "10", "--decorate=short", "--oneline"],
                        shell=False,
                        cwd=self.tmivSourceDir,
                        check=True,
                        stdout=target,
                    )
                    subprocess.run(
                        [self.gitCommand, "status", "--short"],
                        shell=False,
                        cwd=self.tmivSourceDir,
                        check=True,
                        stdout=target,
                    )

    def testMivAnchor(self, executor):
        if not self.dryRun:
            (self.testDir / "A3" / "E" / "QP3").mkdir(parents=True, exist_ok=True)

        geometryResolution = Resolution(512, 512)
        textureResolution = Resolution(1024, 1024)
        renderResolution = Resolution(480, 270)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivEncoder", "-c", "{1}/config/ctc/miv_anchor/A_1_TMIV_encode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-s", "E", "-p", "intraPeriod", "2"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "maxLumaPictureSize", "1048576", "-f", "0"]
            + ["-V", "debug"],
            "{3}/A3/E/TMIV_A3_E.log",
            [
                "A3/E/TMIV_A3_E.bit",
                f"A3/E/TMIV_A3_E_geo_c00_{geometryResolution}_yuv420p10le.yuv",
                f"A3/E/TMIV_A3_E_geo_c01_{geometryResolution}_yuv420p10le.yuv",
                f"A3/E/TMIV_A3_E_tex_c00_{textureResolution}_yuv420p10le.yuv",
                f"A3/E/TMIV_A3_E_tex_c01_{textureResolution}_yuv420p10le.yuv",
            ],
        )

        f2_1 = self.launchCommand(
            executor,
            [f1],
            ["{4}/bin/vvencFFapp", "-c", "{1}/config/test/miv_anchor/A_2_VVenC_encode_geo.cfg"]
            + ["-i", f"{{3}}/A3/E/TMIV_A3_E_geo_c00_{geometryResolution}_yuv420p10le.yuv", "-b"]
            + ["{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c00.bit", "-s", str(geometryResolution), "-q", "20"]
            + ["-f", "3", "-fr", "30"],
            "{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c00_vvenc.log",
            ["A3/E/QP3/TMIV_A3_E_QP3_geo_c00.bit"],
        )

        f2_2 = self.launchCommand(
            executor,
            [f1],
            ["{4}/bin/vvencFFapp", "-c", "{1}/config/test/miv_anchor/A_2_VVenC_encode_geo.cfg"]
            + ["-i", f"{{3}}/A3/E/TMIV_A3_E_geo_c01_{geometryResolution}_yuv420p10le.yuv", "-b"]
            + ["{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c01.bit", "-s", str(geometryResolution), "-q", "20"]
            + ["-f", "3", "-fr", "30"],
            "{3}/A3/E/QP3/TMIV_A3_E_QP3_geo_c01_vvenc.log",
            ["A3/E/QP3/TMIV_A3_E_QP3_geo_c01.bit"],
        )

        f2_3 = self.launchCommand(
            executor,
            [f1],
            ["{4}/bin/vvencFFapp", "-c", "{1}/config/test/miv_anchor/A_2_VVenC_encode_tex.cfg"]
            + ["-i", f"{{3}}/A3/E/TMIV_A3_E_tex_c00_{textureResolution}_yuv420p10le.yuv", "-b"]
            + ["{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c00.bit", "-s", str(textureResolution), "-q", "43"]
            + ["-f", "3", "-fr", "30"],
            "{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c00_vvenc.log",
            ["A3/E/QP3/TMIV_A3_E_QP3_tex_c00.bit"],
        )

        f2_4 = self.launchCommand(
            executor,
            [f1],
            ["{4}/bin/vvencFFapp", "-c", "{1}/config/test/miv_anchor/A_2_VVenC_encode_tex.cfg"]
            + ["-i", f"{{3}}/A3/E/TMIV_A3_E_tex_c01_{textureResolution}_yuv420p10le.yuv", "-b"]
            + ["{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c01.bit", "-s", str(textureResolution), "-q", "43"]
            + ["-f", "3", "-fr", "30"],
            "{3}/A3/E/QP3/TMIV_A3_E_QP3_tex_c01_vvenc.log",
            ["A3/E/QP3/TMIV_A3_E_QP3_tex_c01.bit"],
        )

        f3 = self.launchCommand(
            executor,
            [f2_1, f2_2, f2_3, f2_4],
            ["{0}/bin/TmivMultiplexer", "-c", "{1}/config/test/miv_anchor/A_3_TMIV_mux.json"]
            + ["-p", "inputDirectory", "{3}", "-p", "outputDirectory", "{3}"]
            + ["-n", "3", "-s", "E", "-r", "QP3"]
            + ["-V", "debug"],
            "{3}/A3/E/QP3/TMIV_A3_E_QP3.log",
            ["A3/E/QP3/TMIV_A3_E_QP3.bit"],
        )

        f2_5 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivParser"]
            + ["-b", "{3}/A3/E/QP3/TMIV_A3_E_QP3.bit"]
            + ["-o", "{3}/A3/E/QP3/TMIV_A3_E_QP3.hls"],
            None,
            ["A3/E/QP3/TMIV_A3_E_QP3.hls"],
        )

        f2_6 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivBitrateReport"]
            + ["-b", "{3}/A3/E/QP3/TMIV_A3_E_QP3.bit"]
            + ["-o", "{3}/A3/E/QP3/TMIV_A3_E_QP3.csv"],
            None,
            ["A3/E/QP3/TMIV_A3_E_QP3.csv"],
        )

        f4_1 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivDecoder", "-c", "{1}/config/ctc/miv_anchor/A_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-N", "3", "-s", "E"]
            + ["-r", "QP3", "-v", "v11", "-p", "outputLogPath", "{3}/A3/E/QP3/TMIV_A3_E_QP3.dec2"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-V", "debug"],
            "{3}/A3/E/QP3/A3_E_QP3_v11.log",
            [
                "A3/E/QP3/TMIV_A3_E_QP3.dec2",
                f"A3/E/QP3/A3_E_QP3_v11_tex_{renderResolution}_yuv420p10le.yuv",
            ],
        )

        f4_2 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivDecoderLog"]
            + ["-b", "{3}/A3/E/QP3/TMIV_A3_E_QP3.bit"]
            + ["-o", "{3}/A3/E/QP3/TMIV_A3_E_QP3.dec"],
            "{3}/A3/E/QP3/TMIV_A3_E_QP3.dec.log",
            ["A3/E/QP3/TMIV_A3_E_QP3.dec"],
        )

        f4_3 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivNativeVideoDecoderTest", "{3}/A3/E/QP3/TMIV_A3_E_QP3.bit", "3", "2"],
            "{3}/A3/E/QP3/TMIV_A3_E_QP3.nvdt.log",
            [],
        )

        return [f4_1, f4_2, f4_3, f2_5, f2_6]

    def testMivViewAnchor(self, executor):
        if not self.dryRun:
            (self.testDir / "V1" / "D" / "R0").mkdir(parents=True, exist_ok=True)

        # maxLumaPictureSize is set to limit to 1:8 aspect ratio
        geometryResolution = Resolution(512, 4096)
        textureResolution = Resolution(1024, 8192)
        poseTraceRenderResolution = Resolution(480, 270)
        cameraRenderResolution = Resolution(512, 272)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivEncoder", "-c", "{1}/config/ctc/miv_view_anchor/V_1_TMIV_encode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}", "-p"]
            + ["outputDirectory", "{3}", "-n", "1", "-s", "D", "-p", "intraPeriod", "2"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "maxLumaPictureSize", "8388608", "-f", "0"]
            + ["-p", "levelIdc", '"8.5"', "-p", "oneV3cFrameOnly", "true"]
            + ["-V", "debug"],
            "{3}/V1/D/TMIV_V1_D.log",
            [
                "V1/D/TMIV_V1_D.bit",
                f"V1/D/TMIV_V1_D_geo_c00_{geometryResolution}_yuv420p10le.yuv",
                f"V1/D/TMIV_V1_D_geo_c01_{geometryResolution}_yuv420p10le.yuv",
                f"V1/D/TMIV_V1_D_tex_c00_{textureResolution}_yuv420p10le.yuv",
                f"V1/D/TMIV_V1_D_tex_c01_{textureResolution}_yuv420p10le.yuv",
            ],
        )

        f2_1 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivDecoder", "-c", "{1}/config/ctc/miv_view_anchor/V_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}", "-p"]
            + ["outputDirectory", "{3}", "-p", "inputGeometryVideoFramePathFmt"]
            + ["V{{0}}/{{1}}/TMIV_V{{0}}_{{1}}_geo_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputTextureVideoFramePathFmt"]
            + ["V{{0}}/{{1}}/TMIV_V{{0}}_{{1}}_tex_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputBitstreamPathFmt", "V1/D/TMIV_V1_D.bit"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-n", "1", "-N", "3", "-s", "D", "-r", "R0", "-v", "v14"]
            + ["-V", "debug"],
            "{3}/V1/D/R0/V1_D_R0_v14.log",
            [f"V1/D/R0/V1_D_R0_v14_tex_{cameraRenderResolution}_yuv420p10le.yuv"],
        )

        f2_2 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivParser"]
            + ["-b", "{3}/V1/D/TMIV_V1_D.bit"]
            + ["-o", "{3}/V1/D/TMIV_V1_D.hls"],
            None,
            ["V1/D/TMIV_V1_D.hls"],
        )

        f2_3 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivBitrateReport"]
            + ["-b", "{3}/V1/D/TMIV_V1_D.bit"]
            + ["-o", "{3}/V1/D/TMIV_V1_D.csv"],
            None,
            ["V1/D/TMIV_V1_D.csv"],
        )

        f2_4 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivDecoder", "-c", "{1}/config/ctc/miv_view_anchor/V_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}"]
            + ["-p", "outputDirectory", "{3}", "-p", "inputGeometryVideoFramePathFmt"]
            + ["V{{0}}/{{1}}/TMIV_V{{0}}_{{1}}_geo_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputTextureVideoFramePathFmt"]
            + ["V{{0}}/{{1}}/TMIV_V{{0}}_{{1}}_tex_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputBitstreamPathFmt", "V1/D/TMIV_V1_D.bit"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-n", "1", "-N", "3", "-s", "D", "-r", "R0", "-P", "p02"]
            + ["-V", "debug"],
            "{3}/V1/D/R0/V1_D_R0_p02.log",
            [f"V1/D/R0/V1_D_R0_p02_tex_{poseTraceRenderResolution}_yuv420p10le.yuv"],
        )

        return [f2_1, f2_2, f2_3, f2_4]

    def testOneView(self, executor):
        if not self.dryRun:
            (self.testDir / "W3" / "D" / "R0").mkdir(parents=True, exist_ok=True)

        geometryResolution = Resolution(256, 136)
        textureResolution = Resolution(512, 272)
        poseTraceRenderResolution = Resolution(480, 270)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivEncoder", "-c", "{1}/config/test/one_view/W_1_TMIV_encode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}", "-p"]
            + ["outputDirectory", "{3}", "-n", "3", "-s", "D", "-p", "intraPeriod", "2"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}_OneView.json"]
            + ["-f", "0"]
            + ["-V", "debug"],
            "{3}/W3/D/TMIV_W3_D.log",
            [
                "W3/D/TMIV_W3_D.bit",
                f"W3/D/TMIV_W3_D_geo_c00_{geometryResolution}_yuv420p.yuv",
                f"W3/D/TMIV_W3_D_tex_c00_{textureResolution}_yuv420p.yuv",
            ],
        )

        f2_1 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivParser"]
            + ["-b", "{3}/W3/D/TMIV_W3_D.bit"]
            + ["-o", "{3}/W3/D/TMIV_W3_D.hls"],
            None,
            ["W3/D/TMIV_W3_D.hls"],
        )

        f2_2 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivBitrateReport"]
            + ["-b", "{3}/W3/D/TMIV_W3_D.bit"]
            + ["-o", "{3}/W3/D/TMIV_W3_D.csv"],
            None,
            ["W3/D/TMIV_W3_D.csv"],
        )

        f2_3 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivDecoder", "-c", "{1}/config/test/one_view/W_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}"]
            + ["-p", "outputDirectory", "{3}", "-p", "inputGeometryVideoFramePathFmt"]
            + ["W{{0}}/{{1}}/TMIV_W{{0}}_{{1}}_geo_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputTextureVideoFramePathFmt"]
            + ["W{{0}}/{{1}}/TMIV_W{{0}}_{{1}}_tex_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputBitstreamPathFmt", "W3/D/TMIV_W3_D.bit"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-n", "3", "-N", "3", "-s", "D", "-r", "R0", "-P", "p02"]
            + ["-V", "debug"],
            "{3}/W3/D/R0/W3_D_R0_p02.log",
            [f"W3/D/R0/W3_D_R0_p02_tex_{poseTraceRenderResolution}_yuv420p10le.yuv"],
        )

        return [f2_1, f2_2, f2_3]

    def testMivDsdeAnchor(self, executor):
        if not self.dryRun:
            (self.testDir / "G3" / "N" / "R0").mkdir(parents=True, exist_ok=True)

        textureResolution = Resolution(512, 1024)
        renderResolution = Resolution(512, 512)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivEncoder", "-c", "{1}/config/ctc/miv_dsde_anchor/G_1_TMIV_encode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-s", "N", "-p", "intraPeriod", "2"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "maxLumaPictureSize", "524288", "-f", "0"]
            + ["-V", "debug"],
            "{3}/G3/N/TMIV_G3_N.log",
            [
                "G3/N/TMIV_G3_N.bit",
                f"G3/N/TMIV_G3_N_tex_c00_{textureResolution}_yuv420p10le.yuv",
                f"G3/N/TMIV_G3_N_tex_c01_{textureResolution}_yuv420p10le.yuv",
                f"G3/N/TMIV_G3_N_tex_c02_{textureResolution}_yuv420p10le.yuv",
                f"G3/N/TMIV_G3_N_tex_c03_{textureResolution}_yuv420p10le.yuv",
            ],
        )

        f2_1 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivParser"]
            + ["-b", "{3}/G3/N/TMIV_G3_N.bit"]
            + ["-o", "{3}/G3/N/TMIV_G3_N.hls"],
            None,
            ["G3/N/TMIV_G3_N.hls"],
        )

        f2_2 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivBitrateReport"]
            + ["-b", "{3}/G3/N/TMIV_G3_N.bit"]
            + ["-o", "{3}/G3/N/TMIV_G3_N.csv"],
            None,
            ["G3/N/TMIV_G3_N.csv"],
        )

        f2_3 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivDecoder", "-c", "{1}/config/ctc/miv_dsde_anchor/G_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}"]
            + ["-p", "outputDirectory", "{3}", "-p", "inputGeometryVideoFramePathFmt"]
            + ["G{{0}}/{{1}}/TMIV_G{{0}}_{{1}}_geo_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputTextureVideoFramePathFmt"]
            + ["G{{0}}/{{1}}/TMIV_G{{0}}_{{1}}_tex_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputBitstreamPathFmt", "G3/N/TMIV_G3_N.bit"]
            + ["-n", "3", "-N", "3", "-s", "N", "-r", "R0"]
            + ["-V", "debug"],
            "{3}/G3/N/R0/G3_N_R0_none.log",
            [
                "G3/N/R0/TMIV_G3_N_R0_0000.json",
                f"G3/N/R0/TMIV_G3_N_R0_tex_pv00_{renderResolution}_yuv420p10le.yuv",
                f"G3/N/R0/TMIV_G3_N_R0_tex_pv01_{renderResolution}_yuv420p10le.yuv",
                f"G3/N/R0/TMIV_G3_N_R0_tex_pv02_{renderResolution}_yuv420p10le.yuv",
                f"G3/N/R0/TMIV_G3_N_R0_tex_pv03_{renderResolution}_yuv420p10le.yuv",
                f"G3/N/R0/TMIV_G3_N_R0_tex_pv04_{renderResolution}_yuv420p10le.yuv",
                f"G3/N/R0/TMIV_G3_N_R0_tex_pv05_{renderResolution}_yuv420p10le.yuv",
                f"G3/N/R0/TMIV_G3_N_R0_tex_pv06_{renderResolution}_yuv420p10le.yuv",
            ],
        )

        return [f2_1, f2_2, f2_3]

    def testBestReference(self, executor):
        if not self.dryRun:
            (self.testDir / "R3" / "O" / "R0").mkdir(parents=True, exist_ok=True)

        resolution = Resolution(480, 270)

        f1_1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivRenderer", "-c", "{1}/config/ctc/best_reference/R_1_TMIV_render.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-N", "3", "-s", "O", "-r", "R0"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-v", "v01", "-f", "0"]
            + ["-V", "debug"],
            "{3}/R3/O/R0/R3_O_R0_v01.log",
            [
                f"R3/O/R0/R3_O_R0_v01_geo_{resolution}_yuv420p16le.yuv",
                f"R3/O/R0/R3_O_R0_v01_tex_{resolution}_yuv420p10le.yuv",
            ],
        )

        f1_2 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivRenderer", "-c", "{1}/config/ctc/best_reference/R_1_TMIV_render.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-N", "3", "-s", "O"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-r", "R0", "-P", "p02", "-f", "0"]
            + ["-V", "debug"],
            "{3}/R3/O/R0/R3_O_R0_p02.log",
            [
                f"R3/O/R0/R3_O_R0_p02_geo_{resolution}_yuv420p16le.yuv",
                f"R3/O/R0/R3_O_R0_p02_tex_{resolution}_yuv420p10le.yuv",
            ],
        )

        return [f1_1, f1_2]

    def testMivMpi(self, executor):
        if not self.dryRun:
            (self.testDir / "M3" / "M" / "QP3").mkdir(parents=True, exist_ok=True)

        atlasResolution = Resolution(1280, 1280)
        renderResolution = Resolution(480, 270)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivMpiEncoder", "-c", "{1}/config/test/miv_mpi/M_1_TMIV_encode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-s", "M", "-p", "intraPeriod", "2"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "overrideAtlasFrameSizes", "[[ 1280, 1280 ]]"]
            + ["-V", "debug"],
            "{3}/M3/M/TMIV_M3_M.log",
            [
                "M3/M/TMIV_M3_M.bit",
                f"M3/M/TMIV_M3_M_tra_c00_{atlasResolution}_yuv420p10le.yuv",
                f"M3/M/TMIV_M3_M_tex_c00_{atlasResolution}_yuv420p10le.yuv",
            ],
        )

        f2_1 = self.launchCommand(
            executor,
            [f1],
            ["{4}/bin/TAppEncoder", "-c", "{1}/config/test/miv_mpi/M_2_HM_encode_tra.cfg"]
            + ["-i", f"{{3}}/M3/M/TMIV_M3_M_tra_c00_{atlasResolution}_yuv420p10le.yuv"]
            + ["-b", "{3}/M3/M/QP3/TMIV_M3_M_QP3_tra_c00.bit"]
            + ["-wdt", str(atlasResolution.width), "-hgt", str(atlasResolution.height)]
            + ["-q", "35", "-f", "3", "-fr", "30"],
            "{3}/M3/M/QP3/TMIV_M3_M_QP3_tra_c00.log",
            ["M3/M/QP3/TMIV_M3_M_QP3_tra_c00.bit"],
        )

        f2_2 = self.launchCommand(
            executor,
            [f1],
            ["{4}/bin/TAppEncoder", "-c", "{1}/config/test/miv_mpi/M_2_HM_encode_tex.cfg"]
            + ["-i", f"{{3}}/M3/M/TMIV_M3_M_tex_c00_{atlasResolution}_yuv420p10le.yuv"]
            + ["-b", "{3}/M3/M/QP3/TMIV_M3_M_QP3_tex_c00.bit"]
            + ["-wdt", str(atlasResolution.width), "-hgt", str(atlasResolution.height)]
            + ["-q", "30", "-f", "3", "-fr", "30"],
            "{3}/M3/M/QP3/TMIV_M3_M_QP3_tex_c00.log",
            ["M3/M/QP3/TMIV_M3_M_QP3_tex_c00.bit"],
        )

        f3 = self.launchCommand(
            executor,
            [f2_1, f2_2],
            ["{0}/bin/TmivMultiplexer", "-c", "{1}/config/test/miv_mpi/M_3_TMIV_mux.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-s", "M", "-r", "QP3"]
            + ["-V", "debug"],
            "{3}/M3/M/QP3/TMIV_M3_M_QP3.log",
            ["M3/M/QP3/TMIV_M3_M_QP3.bit"],
        )

        f4_1 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivParser"]
            + ["-b", "{3}/M3/M/QP3/TMIV_M3_M_QP3.bit"]
            + ["-o", "{3}/M3/M/QP3/TMIV_M3_M_QP3.hls"],
            None,
            ["M3/M/QP3/TMIV_M3_M_QP3.hls"],
        )

        f4_2 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivBitrateReport"]
            + ["-b", "{3}/M3/M/QP3/TMIV_M3_M_QP3.bit"]
            + ["-o", "{3}/M3/M/QP3/TMIV_M3_M_QP3.csv"],
            None,
            ["M3/M/QP3/TMIV_M3_M_QP3.csv"],
        )

        f4_3 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivDecoder", "-c", "{1}/config/test/miv_mpi/M_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-N", "3", "-s", "M", "-r", "QP3"]
            + ["-v", "viewport"]
            + ["-V", "debug"],
            "{3}/M3/M/QP3/M3_M_QP3_viewport.log",
            [f"M3/M/QP3/M3_M_QP3_viewport_tex_{renderResolution}_yuv420p10le.yuv"],
        )

        f4_4 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivDecoderLog"]
            + ["-b", "{3}/M3/M/QP3/TMIV_M3_M_QP3.bit"]
            + ["-o", "{3}/M3/M/QP3/TMIV_M3_M_QP3.dec"],
            "{3}/M3/M/QP3/TMIV_M3_M_QP3.dec.log",
            ["M3/M/QP3/TMIV_M3_M_QP3.dec"],
        )

        f4_5 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivNativeVideoDecoderTest", "{3}/M3/M/QP3/TMIV_M3_M_QP3.bit", "3", "2"],
            "{3}/M3/M/QP3/TMIV_M3_M_QP3.nvdt.log",
            [],
        )

        return [f4_1, f4_2, f4_3, f4_4, f4_5]

    def testFramePacking(self, executor):
        if not self.dryRun:
            (self.testDir / "P3" / "E" / "R0").mkdir(parents=True, exist_ok=True)

        packedResolution = Resolution(1024, 1280)
        renderResolution = Resolution(480, 270)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivEncoder", "-c", "{1}/config/test/frame_packing/P_1_TMIV_encode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-s", "E", "-p", "intraPeriod", "2"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "maxLumaPictureSize", "1048576", "-f", "0"]
            + ["-p", "rewriteParameterSets", "true"]
            + ["-V", "debug"],
            "{3}/P3/E/TMIV_P3_E.log",
            [
                "P3/E/TMIV_P3_E.bit",
                f"P3/E/TMIV_P3_E_pac_c00_{packedResolution}_yuv420p10le.yuv",
                f"P3/E/TMIV_P3_E_pac_c01_{packedResolution}_yuv420p10le.yuv",
            ],
        )

        f2_1 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivDecoder", "-c", "{1}/config/test/frame_packing/P_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}", "-p"]
            + ["outputDirectory", "{3}", "-p", "inputPackedVideoFramePathFmt"]
            + ["P{{0}}/{{1}}/TMIV_P{{0}}_{{1}}_pac_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputBitstreamPathFmt", "P3/E/TMIV_P3_E.bit"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-n", "3", "-N", "3", "-s", "E", "-r", "R0", "-v", "v11"]
            + ["-V", "debug"],
            "{3}/P3/E/R0/P3_E_R0_v11.log",
            [f"P3/E/R0/P3_E_R0_v11_tex_{renderResolution}_yuv420p10le.yuv"],
        )

        f2_2 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivParser"]
            + ["-b", "{3}/P3/E/TMIV_P3_E.bit"]
            + ["-o", "{3}/P3/E/TMIV_P3_E.hls"],
            None,
            ["P3/E/TMIV_P3_E.hls"],
        )

        f2_3 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivBitrateReport"]
            + ["-b", "{3}/P3/E/TMIV_P3_E.bit"]
            + ["-o", "{3}/P3/E/TMIV_P3_E.csv"],
            None,
            ["P3/E/TMIV_P3_E.csv"],
        )

        return [f2_1, f2_2, f2_3]

    def testAdditiveSynthesizer(self, executor):
        if not self.dryRun:
            (self.testDir / "S1" / "C" / "R0").mkdir(parents=True, exist_ok=True)

        renderResolution = Resolution(512, 512)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivRenderer", "-c"]
            + ["{1}/config/test/additive_synthesizer/S_1_TMIV_render.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}", "-n", "1", "-N", "1", "-s", "C", "-r", "R0"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-P", "p03", "-f", "0"]
            + ["-V", "debug"],
            "{3}/S1/C/R0/S1_C_R0_p03.log",
            [
                f"S1/C/R0/S1_C_R0_p03_geo_{renderResolution}_yuv420p16le.yuv",
                f"S1/C/R0/S1_C_R0_p03_tex_{renderResolution}_yuv420p10le.yuv",
            ],
        )

        return [f1]

    def testEntityCoding(self, executor):
        if not self.dryRun:
            (self.testDir / "E3" / "B" / "QP3").mkdir(parents=True, exist_ok=True)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivEncoder", "-c"]
            + ["{1}/config/test/entity_based_coding/E_1_TMIV_encode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}"]
            + ["-n", "3", "-s", "B", "-f", "0"]
            + ["-p", "intraPeriod", "2"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/TB.json"]
            + ["-p", "maxLumaPictureSize", "1048576"]
            + ["-p", "maxAtlases", "1"]
            + ["-p", "entityEncodeRange", "[10, 13]"]
            + ["-p", "inputCameraNames", '[ "v3", "v5", "v9"]']
            + ["-p", "rewriteParameterSets", "true"]
            + ["-V", "debug"],
            "{3}/E3/B/TMIV_E3_B.log",
            [
                "E3/B/TMIV_E3_B.bit",
                "E3/B/TMIV_E3_B_geo_c00_256x1024_yuv420p10le.yuv",
                "E3/B/TMIV_E3_B_tex_c00_512x2048_yuv420p10le.yuv",
            ],
        )

        f2_1 = self.launchCommand(
            executor,
            [f1],
            ["{4}/bin/TAppEncoder"]
            + ["-c", "{1}/config/test/entity_based_coding/E_2_HM_encode_geo.cfg"]
            + ["-i", "{3}/E3/B/TMIV_E3_B_geo_c00_256x1024_yuv420p10le.yuv"]
            + ["-b", "{3}/E3/B/QP3/TMIV_E3_B_QP3_geo_c00.bit"],
            "{3}/E3/B/QP3/TMIV_E3_B_QP3_geo_c00.log",
            ["E3/B/QP3/TMIV_E3_B_QP3_geo_c00.bit"],
        )

        f2_2 = self.launchCommand(
            executor,
            [f1],
            ["{4}/bin/TAppEncoder"]
            + ["-c", "{1}/config/test/entity_based_coding/E_2_HM_encode_tex.cfg"]
            + ["-i", "{3}/E3/B/TMIV_E3_B_tex_c00_512x2048_yuv420p10le.yuv"]
            + ["-b", "{3}/E3/B/QP3/TMIV_E3_B_QP3_tex_c00.bit"],
            "{3}/E3/B/QP3/TMIV_E3_B_QP3_tex_c00.log",
            ["E3/B/QP3/TMIV_E3_B_QP3_tex_c00.bit"],
        )

        f3 = self.launchCommand(
            executor,
            [f2_1, f2_2],
            ["{0}/bin/TmivMultiplexer", "-c"]
            + ["{1}/config/test/entity_based_coding/E_3_TMIV_mux.json"]
            + ["-p", "configDirectory", "{1}/config"]
            + ["-p", "inputDirectory", "{3}"]
            + ["-p", "outputDirectory", "{3}"]
            + ["-n", "3", "-s", "B", "-r", "QP3"]
            + ["-V", "debug"],
            "{3}/E3/B/QP3/TMIV_E3_B_QP3.log",
            ["E3/B/QP3/TMIV_E3_B_QP3.bit"],
        )

        f4_1 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivParser"]
            + ["-b", "{3}/E3/B/QP3/TMIV_E3_B_QP3.bit"]
            + ["-o", "{3}/E3/B/QP3/TMIV_E3_B_QP3.hls"],
            None,
            ["E3/B/QP3/TMIV_E3_B_QP3.hls"],
        )

        f4_2 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivBitrateReport"]
            + ["-b", "{3}/E3/B/QP3/TMIV_E3_B_QP3.bit"]
            + ["-o", "{3}/E3/B/QP3/TMIV_E3_B_QP3.csv"],
            None,
            ["E3/B/QP3/TMIV_E3_B_QP3.csv"],
        )

        f4_3 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivDecoder"]
            + ["-c", "{1}/config/test/entity_based_coding/E_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config"]
            + ["-p", "inputDirectory", "{3}"]
            + ["-p", "outputDirectory", "{3}"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-n", "3", "-N", "3", "-s", "B", "-r", "QP3", "-P", "p01"]
            + ["-p", "entityDecodeRange", "[10, 11]"]
            + ["-V", "debug"],
            "{3}/E3/B/QP3/E3_B_QP3_p01.log",
            ["E3/B/QP3/E3_B_QP3_p01_tex_512x512_yuv420p10le.yuv"],
        )

        f4_4 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivDecoderLog"]
            + ["-b", "{3}/E3/B/QP3/TMIV_E3_B_QP3.bit"]
            + ["-o", "{3}/E3/B/QP3/TMIV_E3_B_QP3.dec"],
            "{3}/E3/B/QP3/TMIV_E3_B_QP3.dec.log",
            ["E3/B/QP3/TMIV_E3_B_QP3.dec"],
        )

        f4_5 = self.launchCommand(
            executor,
            [f3],
            ["{0}/bin/TmivNativeVideoDecoderTest", "{3}/E3/B/QP3/TMIV_E3_B_QP3.bit", "3", "2"],
            "{3}/E3/B/QP3/TMIV_E3_B_QP3.nvdt.log",
            [],
        )

        return [f4_1, f4_2, f4_3, f4_4, f4_5]

    def testExplicitOccupancy(self, executor):
        if not self.dryRun:
            (self.testDir / "O3" / "N" / "R0").mkdir(parents=True, exist_ok=True)

        occupancyResolution = Resolution(32, 40)
        geometryResolution = Resolution(512, 640)
        textureResolution = Resolution(1024, 1280)
        renderResolution = Resolution(512, 512)

        f1 = self.launchCommand(
            executor,
            [],
            ["{0}/bin/TmivEncoder", "-c", "{1}/config/test/explicit_occupancy/O_1_TMIV_encode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{2}"]
            + ["-p", "outputDirectory", "{3}", "-n", "3", "-s", "N", "-p", "intraPeriod", "2"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/T{{1}}.json"]
            + ["-p", "maxLumaPictureSize", "1310720", "-f", "0"]
            + ["-p", "inputBitstreamPathFmt", "O{0}/{1}/TMIV_O{0}_{1}.bit"]
            + ["-V", "debug"],
            "{3}/O3/N/TMIV_O3_N.log",
            [
                "O3/N/TMIV_O3_N.bit",
                f"O3/N/TMIV_O3_N_occ_c00_{occupancyResolution}_yuv420p10le.yuv",
                f"O3/N/TMIV_O3_N_occ_c01_{occupancyResolution}_yuv420p10le.yuv",
                f"O3/N/TMIV_O3_N_geo_c00_{geometryResolution}_yuv420p10le.yuv",
                f"O3/N/TMIV_O3_N_geo_c01_{geometryResolution}_yuv420p10le.yuv",
                f"O3/N/TMIV_O3_N_tex_c00_{textureResolution}_yuv420p10le.yuv",
                f"O3/N/TMIV_O3_N_tex_c01_{textureResolution}_yuv420p10le.yuv",
            ],
        )

        f2_1 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivDecoder", "-c", "{1}/config/test/explicit_occupancy/O_4_TMIV_decode.json"]
            + ["-p", "configDirectory", "{1}/config", "-p", "inputDirectory", "{3}", "-p"]
            + ["outputDirectory", "{3}", "-p", "inputOccupancyVideoFramePathFmt"]
            + ["O{{0}}/{{1}}/TMIV_O{{0}}_{{1}}_occ_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputGeometryVideoFramePathFmt"]
            + ["O{{0}}/{{1}}/TMIV_O{{0}}_{{1}}_geo_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputTextureVideoFramePathFmt"]
            + ["O{{0}}/{{1}}/TMIV_O{{0}}_{{1}}_tex_c{{3:02}}_{{4}}x{{5}}_{{6}}.yuv"]
            + ["-p", "inputBitstreamPathFmt", "O3/N/TMIV_O3_N.bit"]
            + ["-p", "inputViewportParamsPathFmt", "test/sequences/T{{1}}.json"]
            + ["-n", "3", "-N", "3", "-s", "N", "-r", "R0", "-P", "p01"]
            + ["-V", "debug"],
            "{3}/O3/N/R0/O3_N_R0_p01.log",
            [f"O3/N/R0/O3_N_R0_p01_tex_{renderResolution}_yuv420p10le.yuv"],
        )

        f2_2 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivParser"]
            + ["-b", "{3}/O3/N/TMIV_O3_N.bit"]
            + ["-o", "{3}/O3/N/TMIV_O3_N.hls"],
            None,
            ["O3/N/TMIV_O3_N.hls"],
        )

        f2_3 = self.launchCommand(
            executor,
            [f1],
            ["{0}/bin/TmivBitrateReport"]
            + ["-b", "{3}/O3/N/TMIV_O3_N.bit"]
            + ["-o", "{3}/O3/N/TMIV_O3_N.csv"],
            None,
            ["O3/N/TMIV_O3_N.csv"],
        )

        return [f2_1, f2_2, f2_3]

    def launchCommand(self, executor, futures, args, logFile, outputFiles):
        return executor.submit(self.syncAndRunCommand, futures, args, logFile, outputFiles)

    def syncAndRunCommand(self, futures, args, logFile, outputFiles):
        try:
            for future in concurrent.futures.as_completed(futures):
                future.result()

            self.runCommand(args, logFile)

            if not self.dryRun:
                self.computeMd5Sums(outputFiles)
        except Exception as e:
            self.stop = True
            print(
                f"EXCEPTION of type {type(e).__name__}, {e}, with args = {args}, and logFile = {logFile}."
            )
            sys.exit(1)

    def sync(self, futures):
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as exception:
                self.stop = True
                raise exception

    def runCommand(self, args, logFile):
        if not self.dryRun:
            print("+ {}".format(" ".join(args)), flush=True)

        def fillPlaceholders(arg):
            return arg.format(
                self.tmivInstallDir,
                self.tmivSourceDir,
                self.contentDir,
                self.testDir,
                self.depsInstallDir,
            )

        args = list(map(fillPlaceholders, args))
        if logFile:
            logFile = fillPlaceholders(logFile)

        if self.dryRun:
            print(" ".join(args))
        elif not self.stop:
            # Execute the command in an interruptable way
            popen = subprocess.Popen(
                args,
                shell=False,
                cwd=self.testDir,
                stdout=(open(logFile, "w") if logFile else None),
                stderr=subprocess.STDOUT,
            )

            while True:
                time.sleep(1)
                returncode = popen.poll()
                if self.stop:
                    print("Killing process {}".format(args[0]))
                    popen.kill()
                    sys.exit(1)
                elif returncode is None:
                    continue
                elif returncode == 0:
                    return
                else:
                    raise RuntimeError(
                        "EXECUTION FAILED!\n  * Log-file    : {}\n  * Command     : {}".format(
                            logFile, " ".join(args)
                        )
                    )

    def checkIfProbeExistsInDir(self, what: str, folder: Path, probeFile: Path) -> None:
        if not (folder / probeFile).exists():
            raise RuntimeError(
                f"{folder} does not appear to be a {what} directory because {probeFile} was not found."
            )

    def computeMd5Sums(self, outputFiles):
        for f in outputFiles:
            self.md5sums.append((computeMd5Sum(self.testDir / f), f))

    def storeMd5Sums(self):
        with open(self.md5sumsFile, "w") as stream:
            for md5sum in sorted(self.md5sums, key=lambda pair: pair[1]):
                stream.write(f"{md5sum[0]} *{md5sum[1]}\n")

    def compareMd5Files(self):
        assert self.referenceMd5File
        reference = self.referenceMd5File.read_text().splitlines()
        actual = self.md5sumsFile.read_text().splitlines()

        haveDifferences = False
        for line in Differ().compare(reference, actual):
            if not line.startswith(" "):
                if not haveDifferences:
                    haveDifferences = True
                    print("Different MD5 sums found:")
                print(line)

        if not haveDifferences:
            print("All MD5 sums are equal to the reference.")

        return int(haveDifferences)


if __name__ == "__main__":
    try:
        app = IntegrationTest()
        exit(app.run())
    except RuntimeError as e:
        print(e)
        exit(1)
