#!/usr/bin/env python3

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
from pathlib import Path
import subprocess
import sys


class ConformanceBitstreamEncoder:
    def __init__(
        self,
        id: str,
        work_dir: Path,
        output_dir: Path,
        content_dir: Path,
        tmiv_url: str,
        tmiv_ref: str,
        thread_count: int,
    ):
        self.id = id
        self.work_dir = sub_work_dir(self.id, work_dir)
        self.output_dir = sub_work_dir(self.id, output_dir)
        self.content_dir = content_dir
        self.tmiv_deps_source_dir = project_dir() / ".deps" / "source"

        self.tmiv_url = tmiv_url
        self.tmiv_ref = tmiv_ref
        self.custom_tmiv_ref = self.id.lower().replace(".", "-")

        self.thread_count = thread_count

        id_ = self.id.replace(".", "_")
        self.intermediate_file = self.work_dir / "intermediate.bit"
        self.bitstream_file = self.output_dir / f"{id_}.bit"
        self.parser_file = self.bitstream_file.with_suffix(".hls")
        self.report_file = self.bitstream_file.with_suffix(".csv")
        self.decoder_file = self.bitstream_file.with_suffix(".dec")
        self.diff_file = self.bitstream_file.with_suffix(".diff")
        self.doc_file = self.bitstream_file.with_suffix(".txt")

    def encode(self):
        with open(
            project_dir() / "doc" / "conformance" / self.doc_file.name, encoding="utf8"
        ) as istream:
            with open(self.doc_file, mode="w", encoding="utf8") as ostream:
                ostream.write(istream.read())

        {
            "CB01": self.encode_cb01,
            "CB02": self.encode_cb02,
            "CB03": self.encode_cb03,
            "CB04": self.encode_cb04,
            "CB05.1": self.encode_cb05_1,
            "CB05.2": self.encode_cb05_2,
            "CB05.3": self.encode_cb05_3,
            "CB06": self.encode_cb06,
            "CB07.1": self.encode_cb07_1,
            "CB07.2": self.encode_cb07_2,
            "CB07.3": self.encode_cb07_3,
            "CB08": self.encode_cb08,
            "CB09": self.encode_cb09,
            "CB10": self.encode_cb10,
            "CB11": self.encode_cb11,
            "CB12": self.encode_cb12,
            "CB14": self.encode_cb14,
            "CB15": self.encode_cb15,
            "CB16": self.encode_cb16,
            "CB17": self.encode_cb17,
            "CB18": self.encode_cb18,
            "CB19": self.encode_cb19,
            "CB20": self.encode_cb20,
        }[self.id]()

        run(f"bin/TmivParser -b {self.bitstream_file} -o {self.parser_file}", cwd=self.work_dir)
        run(
            f"bin/TmivBitrateReport -b {self.bitstream_file} -o {self.report_file}",
            cwd=self.work_dir,
        )
        run(
            f"bin/TmivDecoderLog -b {self.bitstream_file} -o {self.decoder_file}", cwd=self.work_dir
        )

    def encode_cb01(self):
        self.encoder_config_file = "test/entity_based_coding/E_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/entity_based_coding/E_3_TMIV_mux.json"
        self.condition_id = "E"
        self.frame_count = 1
        self.content_id = "B01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb02(self):
        self.encoder_config_file = "test/explicit_occupancy/O_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/explicit_occupancy/O_3_TMIV_mux.json"
        self.condition_id = "O"
        self.frame_count = 5
        self.content_id = "B02"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb03(self):
        self.encoder_config_file = "test/full_views/V_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/full_views/V_3_TMIV_mux.json"
        self.condition_id = "V"
        self.frame_count = 5
        self.content_id = "E01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb04(self):
        self.encoder_config_file = "test/full_views/V_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/full_views/V_3_TMIV_mux.json"
        self.condition_id = "V"
        self.frame_count = 37
        self.content_id = "D01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb05_1(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 3
        self.content_id = "C01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb05_2(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 3
        self.content_id = "C01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb05_3(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 3
        self.content_id = "C01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb06(self):
        self.encoder_config_file = "test/full_views/V_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/full_views/V_3_TMIV_mux.json"
        self.condition_id = "V"
        self.frame_count = 5
        self.content_id = "D01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb07_1(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 3
        self.content_id = "E01"

        self.build_tmiv()
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb07_2(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 3
        self.content_id = "E01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb07_3(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 3
        self.content_id = "E01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb08(self):
        self.encoder_config_file = "test/miv_mpi/M_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/miv_mpi/M_3_TMIV_mux.json"
        self.condition_id = "M"
        self.frame_count = 3
        self.content_id = "M"

        self.build_tmiv()
        self.tmiv_mpi_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb09(self):
        self.encoder_config_file = "test/full_views/V_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/full_views/V_3_TMIV_mux.json"
        self.condition_id = "V"
        self.frame_count = 3
        self.content_id = "E02"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb10(self):
        self.encoder_config_file = "test/non_irap_frames/I_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/non_irap_frames/I_3_TMIV_mux.json"
        self.condition_id = "I"
        self.frame_count = 5
        self.content_id = "C01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb11(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 3
        self.content_id = "W01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb12(self):
        self.encoder_config_file = "test/full_views/V_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/full_views/V_3_TMIV_mux.json"
        self.condition_id = "V"
        self.frame_count = 1
        self.content_id = "J01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb14(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 17
        self.content_id = "A01"

        self.build_tmiv()
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb15(self):
        self.encoder_config_file = "ctc/miv_main_anchor/A_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_main_anchor/A_3_TMIV_mux.json"
        self.condition_id = "A"
        self.frame_count = 97
        self.content_id = "D01"

        self.build_tmiv()
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb16(self):
        self.encoder_config_file = "test/full_views/V_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/full_views/V_3_TMIV_mux.json"
        self.condition_id = "V"
        self.frame_count = 17
        self.content_id = "B01"

        self.build_tmiv()
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb17(self):
        self.encoder_config_file = "test/full_views/V_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/full_views/V_3_TMIV_mux.json"
        self.condition_id = "V"
        self.frame_count = 17
        self.content_id = "E01"

        self.build_tmiv()
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb18(self):
        self.encoder_config_file = "ctc/miv_dsde_anchor/G_1_TMIV_encode.json"
        self.multiplexer_config_file = "ctc/miv_dsde_anchor/G_3_TMIV_mux.json"
        self.condition_id = "G"
        self.frame_count = 17
        self.content_id = "J02"

        self.build_tmiv()
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb19(self):
        self.encoder_config_file = "test/non_irap_frames/I_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/non_irap_frames/I_3_TMIV_mux.json"
        self.condition_id = "I"
        self.frame_count = 3
        self.content_id = "E01"

        self.build_tmiv()
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def encode_cb20(self):
        self.encoder_config_file = "test/multi-tile/T_1_TMIV_encode.json"
        self.multiplexer_config_file = "test/multi-tile/T_3_TMIV_mux.json"
        self.condition_id = "T"
        self.frame_count = 17
        self.content_id = "E01"

        self.build_tmiv(patch=True)
        self.tmiv_encoder()
        self.encode_video_sub_bitstreams()
        self.tmiv_multiplexer()

    def build_tmiv(self, patch: bool = False):
        self.tmiv_source_dir = self.work_dir / "tmiv-src"
        self.tmiv_build_dir = self.work_dir / "tmiv-build"
        self.install_prefix = self.work_dir

        tmiv_ref = self.custom_tmiv_ref if patch else self.tmiv_ref

        if (self.tmiv_source_dir / ".git").is_dir():
            run(["git", "fetch", "--all", "-p"], cwd=self.tmiv_source_dir)
            run(["git", "checkout", tmiv_ref], cwd=self.tmiv_source_dir)
            run(["git", "pull"], cwd=self.tmiv_source_dir)
        else:
            run(["git", "clone", self.tmiv_url, "-b", tmiv_ref, self.tmiv_source_dir])

        if patch:
            print(f"> cd {self.tmiv_source_dir} && " f"git diff {self.tmiv_ref} > {self.diff_file}")
            with open(self.diff_file, "w") as stream:
                subprocess.run(
                    ["git", "diff", f"{self.tmiv_ref}"],
                    check=True,
                    stdout=stream,
                    cwd=self.tmiv_source_dir,
                )

        run(
            [sys.executable, self.tmiv_source_dir / "scripts" / "build" / "build_dependencies.py"]
            + ["--source-dir", self.tmiv_deps_source_dir]
            + ["--build-dir", self.tmiv_build_dir]
            + ["--install-dir", self.install_prefix]
            + ["--build-type", "Release"]
            + ["--thread-count", self.thread_count]
        )
        run(
            ["cmake"]
            + ["-G", "Ninja"]
            + ["-DCMAKE_BUILD_TYPE=Release"]
            + ["-S", self.tmiv_source_dir]
            + ["-B", self.tmiv_build_dir]
            + ["--install-prefix", self.install_prefix]
        )
        run(["ninja", "-C", self.tmiv_build_dir, "-j", self.thread_count])
        run(["ninja", "-C", self.tmiv_build_dir, "install"])

    def tmiv_encoder(self):
        run(
            ["bin/TmivEncoder"]
            + ["-c", f"share/config/{self.encoder_config_file}", "-V", "verbose", "-j", 1]
            + ["-s", self.content_id]
            + ["-n", self.frame_count, "-f", 0]
            + ["-p", "inputDirectory", self.content_dir]
            + ["-p", "configDirectory", "share/config"]
            + ["-p", "outputBitstreamPathFmt", self.intermediate_file],
            cwd=self.work_dir,
        )

    def tmiv_mpi_encoder(self):
        if self.intermediate_file.is_file() and 1 < self.intermediate_file.stat().st_size:
            print(
                f"WARNING: Skipping TMIV MPI encoder because {self.intermediate_file} already exists"
            )
            return

        run(
            ["bin/TmivMpiEncoder"]
            + ["-c", f"share/config/{self.encoder_config_file}", "-V", "verbose", "-j", 1]
            + ["-s", self.content_id]
            + ["-n", self.frame_count, "-f", 0]
            + ["-p", "inputDirectory", self.content_dir]
            + ["-p", "configDirectory", "share/config"]
            + ["-p", "inputSequenceConfigPathFmt", "test/sequences/{1}.json"]
            + ["-p", "outputBitstreamPathFmt", self.intermediate_file],
            cwd=self.work_dir,
        )

    def encode_video_sub_bitstreams(self, frame_rate: int = 30):
        with open(self.work_dir / "intermediate.json", encoding="utf8") as stream:
            vsbs = json.load(stream)

        config_dir = self.work_dir / "share" / "config"

        with open(config_dir / self.encoder_config_file, encoding="utf8") as stream:
            encoder_config = json.load(stream)

        with open(config_dir / self.multiplexer_config_file, encoding="utf8") as stream:
            multiplexer_config = json.load(stream)

        codec_group_idc = encoder_config.get("codecGroupIdc", "HEVC Main10")
        codec = {"HEVC Main10": "HM", "VVC Main10": "VVenC"}[codec_group_idc]

        for vsb in vsbs:
            vuh_atlas_id = vsb["vuh_atlas_id"]
            width = vsb["frame_size"][0]
            height = vsb["frame_size"][1]
            bit_depth = vsb["bit_depth"]
            vuh_unit_type = vsb["vuh_unit_type"]

            if vuh_unit_type == 4:
                ai_attribute_type_id = vsb["ai_attribute_type_id"]
                component_id = {0: "Texture", 2: "Transparency"}[ai_attribute_type_id]
            else:
                component_id = {2: "Occupancy", 3: "Geometry", 5: "Packed"}[vuh_unit_type]

            video_format = "yuv420p" if bit_depth == 8 else f"yuv420p{bit_depth}le"

            input_file = self.work_dir / encoder_config[
                f"output{component_id}VideoDataPathFmt"
            ].format(
                self.frame_count, self.content_id, "R0", vuh_atlas_id, width, height, video_format
            )

            video_bitstream_file = self.work_dir / multiplexer_config[
                f"input{component_id}VideoSubBitstreamPathFmt"
            ].format(self.frame_count, self.content_id, "R1", vuh_atlas_id)
            video_bitstream_file.parent.mkdir(exist_ok=True, parents=True)

            {"HM": self.hm_encoder, "VVenC": self.vvenc}[codec](
                bit_depth=bit_depth,
                width=width,
                height=height,
                frame_rate=frame_rate,
                intra_period=encoder_config["intraPeriod"],
                full_range=int(component_id != "tex"),
                input_file=input_file,
                video_bitstream_file=video_bitstream_file,
                recon_file=video_bitstream_file.parent / input_file.name,
            )

    def hm_encoder(
        self,
        bit_depth: int,
        width: int,
        height: int,
        frame_rate: int,
        intra_period: int,
        full_range: bool,
        input_file: Path,
        video_bitstream_file: Path,
        recon_file: Path,
    ):
        gop_params = {
            2: [
                "--IntraPeriod=2",
                "--GOPSize=2",
                "--Frame1=B 2 1  0.0    0.0    0 0 1.0 0 0 0 2 3 -2 -3 -4 0",
                "--Frame2=B 1 1 -4.8848 0.2061 0 0 1.0 0 0 1 2 3 -1 -2  1 1 1 4 1 1 0 1",
            ],
            32: [],
        }[intra_period]

        run(
            ["bin/TAppEncoder"]
            + ["-c", "share/config/hm/encoder_randomaccess_main10.cfg"]
            + ["-wdt", width, "-hgt", height]
            + ["-f", self.frame_count, "-fr", frame_rate, "-q", 30]
            + [f"--InputBitDepth={bit_depth}"]
            + [f"--InternalBitDepth={bit_depth}"]
            + gop_params
            + ["--SEIDecodedPictureHash=1"]
            + ["--VuiParametersPresent=1", "--VideoSignalTypePresent=1"]
            + [f"--VideoFullRange={int(full_range)}"]
            + ["-i", input_file, "-b", video_bitstream_file, "-o", recon_file],
            cwd=self.work_dir,
        )

    def vvenc(
        self,
        bit_depth: int,
        width: int,
        height: int,
        frame_rate: int,
        intra_period: int,
        full_range: bool,
        input_file: Path,
        video_bitstream_file: Path,
        recon_file: Path,
    ):
        gop_size = intra_period

        vvenc_config_dir = project_dir() / ".deps" / "source" / "vvenc-v1.7.0" / "cfg"
        vvenc_config_file = vvenc_config_dir / "randomaccess_faster.cfg"
        run(
            ["bin/vvencFFapp"]
            + ["-c", vvenc_config_file]
            + ["-s", f"{width}x{height}"]
            + ["-f", self.frame_count, "-fr", frame_rate, "-q", 30]
            + [f"--InputBitDepth={bit_depth}"]
            + [f"--InternalBitDepth={bit_depth}"]
            + [f"--IntraPeriod={intra_period}"]
            + [f"--GOPSize={gop_size}"]
            + ["--SEIDecodedPictureHash=1"]
            + ["--VuiParametersPresent=1"]
            + [f"--VideoFullRange={int(full_range)}"]
            + ["-i", input_file, "-b", video_bitstream_file, "-o", recon_file],
            cwd=self.work_dir,
        )

    def tmiv_multiplexer(self):
        run(
            ["bin/TmivMultiplexer"]
            + ["-c", f"share/config/{self.multiplexer_config_file}"]
            + ["-V", "verbose"]
            + ["-s", self.content_id]
            + ["-n", self.frame_count]
            + ["-r", "R1"]
            + ["-p", "inputBitstreamPathFmt", self.intermediate_file]
            + ["-p", "configDirectory", "share/config"]
            + ["-p", "outputBitstreamPathFmt", self.bitstream_file],
            cwd=self.work_dir,
        )


def parse_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-i", "--id", type=str, help="ID of the conformance bitstream as in Table 1", required=True
    )
    parser.add_argument(
        "-u",
        "--tmiv-url",
        type=str,
        help="Git URL of TMIV",
        default="https://git.mpeg.expert/MPEG/MIV/RS/TM1.git",
    )
    parser.add_argument(
        "-r",
        "--tmiv-ref",
        type=str,
        help="Default Git reference (tag or branch) of TMIV",
        default="main",
    )
    parser.add_argument(
        "-w",
        "--work-dir",
        type=Path,
        help="Directory for intermediate results including TMIV builds",
        default=project_dir() / "out" / Path(__file__).stem,
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="Directory to output the results",
        default=project_dir() / "out" / "conformance",
    )
    parser.add_argument(
        "-c", "--content-dir", type=Path, help="CTC content directory", required=True
    )
    parser.add_argument(
        "-j",
        "--thread-count",
        type=int,
        help="Number of threads that sub processes may use",
        default=1,
    )
    parser.add_argument(
        "--sbatch", action="store_true", help="Launch a batch job on a slurm cluster"
    )
    parser.add_argument(
        "--mail-user",
        type=str,
        help="E-mail address for slurm to communicate about the batch job",
    )
    parser.add_argument(
        "--venv-file",
        type=Path,
        nargs="+",
        help="Script file that is sourced to set-up the Python environment",
        default=project_dir() / "venv" / "bin" / "activate",
    )
    parser.add_argument(
        "--env-files",
        type=Path,
        nargs="+",
        help="Script files that are sourced to set-up the environment",
    )
    return parser.parse_args()


def main(args: argparse.Namespace):
    encoder = ConformanceBitstreamEncoder(
        id=args.id,
        work_dir=args.work_dir,
        output_dir=args.output_dir,
        content_dir=args.content_dir,
        tmiv_url=args.tmiv_url,
        tmiv_ref=args.tmiv_ref,
        thread_count=args.thread_count,
    )
    encoder.encode()


def sbatch(args: argparse.Namespace):
    if not args.mail_user:
        raise RuntimeError("--mail-user is required in combination with --sbatch")

    work_dir = sub_work_dir(args.id, args.work_dir)
    slurm_script_file = work_dir / "sbatch.sh"
    slurm_output_file_fmt = work_dir / "slurm-%j.out"

    env_files = [args.venv_file]

    if "env_files" in args:
        env_files += args.env_files

    with open(slurm_script_file, "w") as stream:
        stream.write("#!/bin/bash\n\n")

        stream.write(f"# This file was generated by {__file__}\n\n")

        stream.write(f"#SBATCH --job-name={args.id}\n")
        stream.write(f"#SBATCH --output={slurm_output_file_fmt}\n")
        stream.write(f"#SBATCH --error={slurm_output_file_fmt}\n")
        stream.write(f"#SBATCH --mail-type=END,FAIL\n")
        stream.write(f"#SBATCH --mail-user={args.mail_user}\n")
        stream.write(f"#SBATCH --ntasks={args.thread_count}\n\n")

        for env_file in env_files:
            stream.write(f". {env_file}\n")

        stream.write(f"\n{sys.executable} {__file__} \\\n")
        stream.write(f"  --id {args.id} \\\n")
        stream.write(f"  --tmiv-url {args.tmiv_url} \\\n")
        stream.write(f"  --tmiv-ref {args.tmiv_ref} \\\n")
        stream.write(f"  --work-dir {args.work_dir} \\\n")
        stream.write(f"  --output-dir {args.output_dir} \\\n")
        stream.write(f"  --content-dir {args.content_dir} \\\n")
        stream.write(f"  --thread-count {args.thread_count}\n")

    run(["sbatch", slurm_script_file])


def project_dir():
    return Path(__file__).parent.parent.parent


def sub_work_dir(id: str, work_dir: Path) -> Path:
    result = work_dir / id.replace(".", "_")
    result.mkdir(exist_ok=True, parents=True)
    return result


def sub_output_dir(id: str, output_dir: Path) -> Path:
    result = output_dir / id.replace(".", "_")
    result.mkdir(exist_ok=True, parents=True)
    return result


def run(args: list, cwd=None):
    if isinstance(args, str):
        args = args.split(" ")

    args_s = [str(arg) for arg in args]
    cwd_s = f"cd {cwd} && " if cwd else ""
    print(f"> {cwd_s}{' '.join(args_s)}", flush=True)

    with subprocess.Popen(
        args_s, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    ) as process:
        print(process.stdout.read(), flush=True)
        returncode = process.wait()

        if returncode != 0:
            raise RuntimeError(f"Sub-process exited with return code {returncode}")


if __name__ == "__main__":
    try:
        args = parse_arguments()

        if not args.content_dir.is_dir():
            raise RuntimeError("Invalid content directory")

        if args.sbatch:
            sbatch(args)
        else:
            main(args)
    except (RuntimeError, subprocess.CalledProcessError, FileNotFoundError) as e:
        print(e)
        exit(1)
