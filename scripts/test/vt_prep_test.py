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
import json
import ninja
from pathlib import Path
import subprocess
import sys


STEP_TMIV_ENCODE = "--tmiv-encode"
STEP_HM = "--hm"
STEP_TMIV_MULTIPLEX = "--tmiv-multiplex"
STEP_TMIV_DECODE = "--tmiv-decode"
STEP_MP4 = "--mp4"


def main():
    try:
        args = parse_arguments()

        if args.tmiv_encode:
            run_step(tmiv_encode, STEP_TMIV_ENCODE, args.tmiv_encode)
        elif args.hm:
            run_step(hm, STEP_HM, args.hm)
        elif args.tmiv_multiplex:
            run_step(tmiv_multiplex, STEP_TMIV_MULTIPLEX, args.tmiv_multiplex)
        elif args.tmiv_decode:
            run_step(tmiv_decode, STEP_TMIV_DECODE, args.tmiv_decode)
        elif args.mp4:
            run_step(mp4, STEP_MP4, args.mp4)
        else:
            save_params(args)
            prepare_experiment(args)
            run_experiment(args)
    except subprocess.CalledProcessError as e:
        print(e)
        exit(1)


def parse_arguments():
    project_dir = relative_to(Path(__file__).parent.parent.parent, Path.cwd())
    default_tmiv_dir = project_dir / "out" / "install" / "x64-Release"
    default_input_dir = project_dir / "in"
    default_output_dir = project_dir / "out" / "vt_prep_test"

    parser = argparse.ArgumentParser()

    parser.add_argument("-j", "--parallel-process-count", type=int, default=6)
    parser.add_argument("-t", "--tmiv-dir", type=str, default=str(default_tmiv_dir))
    parser.add_argument("-i", "--input-dir", type=str, default=str(default_input_dir))
    parser.add_argument("-o", "--output-dir", type=str, default=str(default_output_dir))
    parser.add_argument("--ffmpeg-path", type=str, default="ffmpeg")
    parser.add_argument("-e", "--encoder-config-ids", type=str, nargs="*", default=["ctc", "vt1"])
    parser.add_argument("-d", "--decoder-config-ids", type=str, nargs="*", default=["ctc", "vt1"])
    parser.add_argument("-n", "--coded-frame-count", type=int, default=97)
    parser.add_argument("-N", "--rendered-frame-count", type=int, default=300)
    parser.add_argument("-s", "--seq-ids", type=str, nargs="*", default=["F", "S", "W", "X"])
    parser.add_argument("-r", "--rate-ids", type=str, nargs="*", default=["QP1", "QP3", "QP5"])
    parser.add_argument(
        "-v",
        "--viewport-ids",
        type=str,
        nargs="*",
        default=["p01", "p02", "p03", "p04", "p05", "p06"],
    )

    group = parser.add_mutually_exclusive_group()
    group.add_argument(STEP_TMIV_ENCODE, type=str, nargs=2)
    group.add_argument(STEP_HM, type=str, nargs=5)
    group.add_argument(STEP_TMIV_MULTIPLEX, type=str, nargs=3)
    group.add_argument(STEP_TMIV_DECODE, type=str, nargs=5)
    group.add_argument(STEP_MP4, type=str, nargs=5)

    return parser.parse_args()


def run_step(function, step: str, point: list):
    params = load_params()
    function(params, point)
    write_token(Path(params["output_dir"]), step, point)


def load_params() -> dict:
    with open("build.json") as stream:
        return json.load(stream)


def save_params(args: argparse.Namespace):
    with open("build.json", mode="w") as stream:
        json.dump(vars(args), stream, indent=4, sort_keys=True)


def write_token(output_dir: Path, step: str, point: list):
    file = token_file(output_dir, step, point)

    with open(file, mode="w", encoding="utf8") as stream:
        stream.write(f"point = {repr(point)}\n")


def token_file(output_dir: Path, step: str, point: list):
    dir = Path(output_dir) / "tokens" / step
    dir.mkdir(parents=True, exist_ok=True)
    return dir / f"{step}_{'_'.join(point)}.token"


def prepare_experiment(args: argparse.Namespace):
    with open("build.ninja", "w") as stream:
        writer = ninja.ninja_syntax.Writer(stream, 100)
        write_preamble(writer, args)

        for e in args.encoder_config_ids:
            for s in args.seq_ids:
                t_1 = write_step(writer, args, STEP_TMIV_ENCODE, [e, s], inputs=[])

                for r in args.rate_ids:
                    ts_2 = []
                    for c, a in [["tex", "c00"], ["geo", "c00"], ["tex", "c01"], ["geo", "c01"]]:
                        ts_2 += [write_step(writer, args, STEP_HM, [e, s, r, c, a], inputs=[t_1])]

                    t_3 = write_step(writer, args, STEP_TMIV_MULTIPLEX, [e, s, r], inputs=ts_2)

                    for d in args.decoder_config_ids:
                        for v in args.viewport_ids:
                            if not pose_trace_exists(args, s, v):
                                continue

                            point = [e, s, r, d, v]
                            t_4 = write_step(writer, args, STEP_TMIV_DECODE, point, inputs=[t_3])
                            write_step(writer, args, STEP_MP4, point, inputs=[t_4])


def write_preamble(writer: ninja.ninja_syntax.Writer, args: argparse.Namespace):
    script_file = relative_to(Path(__file__), Path.cwd())

    writer.variable("ninja_required_version", "1.10")
    writer.variable("run", f"{sys.executable} {script_file}")

    for step in [STEP_TMIV_ENCODE, STEP_HM, STEP_TMIV_MULTIPLEX, STEP_TMIV_DECODE, STEP_MP4]:
        writer.newline()
        writer.rule(step[2:], f"$run {step} $args", description=f"{step} $args")


def write_step(
    writer: ninja.ninja_syntax.Writer, args: argparse.Namespace, step: str, point: list, inputs=list
) -> Path:
    token_file_ = token_file(args.output_dir, step, point)

    writer.newline()
    writer.build(
        outputs=[str(token_file_)],
        rule=step[2:],
        inputs=list(map(str, inputs)),
        variables={"args": list(map(str, point))},
    )

    return token_file_


def tmiv_encode(params: dict, point: list):
    n = params["coded_frame_count"]
    tmiv_dir = Path(params["tmiv_dir"])

    config_file = {
        "ctc": tmiv_dir / "share/config/ctc/miv_anchor/A_1_TMIV_encode.json",
        "vt1": tmiv_dir / "share/config/vt/miv_main/A_1_TMIV_encode.json",
    }[point[0]]

    output_dir = Path(params["output_dir"]) / point[0]
    start_frame = {"F": 0, "S": 0, "W": 0, "X": 0}[point[1]]

    run_command(
        [tmiv_dir / "bin/TmivEncoder"]
        + ["-j", 1, "-c", config_file]
        + ["-s", point[1], "-n", n, "-f", start_frame]
        + ["-p", "inputDirectory", params["input_dir"]]
        + ["-p", "outputDirectory", output_dir]
        + ["-p", "configDirectory", tmiv_dir / "share/config"]
        + ["-p", "inputSequenceConfigPathFmt", "vt/sequences/{1}.json"]
        + ["-p", "codecGroupIdc", "HEVC Main10"],
        output_dir / f"A{n}" / point[1] / f"TMIV_A{n}_{point[1]}.log",
    )


def hm(params: dict, point: list):
    n = params["coded_frame_count"]

    tmiv_dir = Path(params["tmiv_dir"])
    output_dir = Path(params["output_dir"])
    input_dir = output_dir / point[0] / f"A{n}" / point[1]
    output_sub_dir = input_dir / point[2]
    output_sub_dir.mkdir(parents=True, exist_ok=True)

    vsb = vsb_parameters(params, point)
    width, height = vsb["frame_size"]

    input_file = (
        input_dir / f"TMIV_A{n}_{point[1]}_{point[3]}_{point[4]}_{width}x{height}_yuv420p10le.yuv"
    )
    bitstream_file = output_sub_dir / f"TMIV_A{n}_{'_'.join(point[1:])}.bit"
    recon_file = output_sub_dir / f"recon_A{n}_{'_'.join(point)}_{width}x{height}_yuv420p10le.yuv"

    run_command(
        [f"{params['tmiv_dir']}/bin/TAppEncoder"]
        + ["-c", tmiv_dir / "share/config/hm/encoder_randomaccess_main10.cfg"]
        + ["-i", input_file, "-b", bitstream_file, "-o", recon_file]
        + ["-wdt", width, "-hgt", height, "-f", n, "-fr", 30]
        + ["-q", qp(point)]
        + ["--InputBitDepth=10"],
        bitstream_file.with_suffix(".log"),
    )

    # HM 16.16 returns exit code 0 also on failure
    assert bitstream_file.is_file()
    assert recon_file.is_file()


def vsb_parameters(params: dict, point: list):
    n = params["coded_frame_count"]
    output_dir = Path(params["output_dir"])
    input_dir = output_dir / point[0] / f"A{n}" / point[1]
    vsb_params_file = input_dir / f"TMIV_A{n}_{point[1]}.json"

    with open(vsb_params_file) as stream:
        vsb_params = json.load(stream)

    for vsb in vsb_params:
        component_id = {3: "geo", 4: "tex"}[vsb["vuh_unit_type"]]
        atlas_id = f"c{vsb['vuh_atlas_id']:02}"

        if point[3] == component_id and point[4] == atlas_id:
            return vsb

    assert False


def qp(point: list) -> int:
    return tex_qp(point) if point[3] == "tex" else geo_qp(point)


def tex_qp(point: list) -> int:
    # The same QP's are used for CTC and VT encoder condition (no rate matching that way).
    # The CTC encoder config is only added to study the difference with the VT config.
    return {
        "F": {"QP1": 28, "QP2": 35, "QP3": 40, "QP4": 43, "QP5": 45},
        "S": {"QP1": 20, "QP2": 27, "QP3": 32, "QP4": 38, "QP5": 44},
        "W": {"QP1": 23, "QP2": 29, "QP3": 36, "QP4": 42, "QP5": 49},
        "X": {"QP1": 21, "QP2": 26, "QP3": 32, "QP4": 36, "QP5": 44},
    }[point[1]][point[2]]


def geo_qp(point: list) -> int:
    return max(1, round(-14.2 + 0.8 * tex_qp(point)))


def tmiv_multiplex(params: dict, point: list):
    n = params["coded_frame_count"]
    tmiv_dir = Path(params["tmiv_dir"])

    config_file = {
        "ctc": tmiv_dir / "share/config/ctc/miv_anchor/A_3_TMIV_mux.json",
        "vt1": tmiv_dir / "share/config/vt/miv_main/A_3_TMIV_mux.json",
    }[point[0]]

    output_dir = Path(params["output_dir"]) / point[0]

    run_command(
        [tmiv_dir / "bin/TmivMultiplexer"]
        + ["-j", 1, "-c", config_file]
        + ["-s", point[1], "-n", n, "-r", point[2]]
        + ["-p", "inputDirectory", output_dir]
        + ["-p", "outputDirectory", output_dir]
        + ["-p", "configDirectory", tmiv_dir / "share/config"],
        output_dir / f"A{n}" / point[1] / point[2] / f"TMIV_A{n}_{point[1]}_{point[2]}.log",
    )


def tmiv_decode(params: dict, point: list):
    n = params["coded_frame_count"]
    tmiv_dir = Path(params["tmiv_dir"])

    config_file = {
        "ctc": tmiv_dir / "share/config/ctc/miv_anchor/A_4_TMIV_decode.json",
        "vt1": tmiv_dir / "share/config/vt/miv_main/A_4_TMIV_decode.json",
    }[point[3]]

    input_dir = Path(params["output_dir"]) / point[0]
    output_dir = Path(params["output_dir"]) / point[0] / point[3]
    log_dir = output_dir / f"A{n}" / point[1] / point[2]
    log_file = log_dir / f"TMIV_A{n}_{point[1]}_{point[2]}_{point[4]}.log"

    assert point[4][0] == "p"
    render_args = ["-N", params["rendered_frame_count"], "-P", point[4]]

    if False:  # point[1] in ["F"]: -- support for pose trace-specific viewports
        inputViewportParamsPathFmt = f"vt/pose_traces/{point[1]}{point[4]}.json"
    else:
        inputViewportParamsPathFmt = f"vt/sequences/{point[1]}.json"

    run_command(
        [tmiv_dir / "bin/TmivDecoder"]
        + ["-j", 1, "-c", config_file]
        + ["-s", point[1], "-n", n, "-r", point[2]]
        + render_args
        + ["-p", "inputDirectory", input_dir]
        + ["-p", "outputDirectory", output_dir]
        + ["-p", "configDirectory", tmiv_dir / "share/config"]
        + ["-p", "inputPoseTracePathFmt", "vt/pose_traces/{1}{3}.csv"]
        + ["-p", "inputViewportParamsPathFmt", inputViewportParamsPathFmt],
        log_file,
    )


def pose_trace_exists(args: argparse.Namespace, seq_id: str, viewport_id: str) -> bool:
    pose_traces_dir = Path(args.tmiv_dir) / "share" / "config" / "vt" / "pose_traces"
    return (pose_traces_dir / f"{seq_id}{viewport_id}.csv").is_file()


def mp4(params: dict, point: list):
    n = params["coded_frame_count"]
    output_dir = Path(params["output_dir"]) / point[0] / point[3]
    output_sub_dir = output_dir / f"A{n}" / point[1] / point[2]
    input_file = (
        output_sub_dir / f"A{n}_{point[1]}_{point[2]}_{point[4]}_tex_1920x1080_yuv420p10le.yuv"
    )
    output_file = output_sub_dir / f"A{n}_{point[1]}_{point[2]}_{point[4]}.mp4"
    log_file = output_file.with_suffix(".ffmpeg")

    run_command(
        [params["ffmpeg_path"]]
        + ["-v", "warning", "-y"]
        + ["-f", "rawvideo", "-pix_fmt", "yuv420p10le"]
        + ["-s:v", f"1920x1080", "-r", 30]
        + ["-i", input_file]
        + ["-c:v", "libx264", "-crf", "10"]
        + ["-pix_fmt", "yuv420p", output_file],
        log_file,
    )


def relative_to(a: Path, b: Path):
    try:
        return a.relative_to(b)
    except ValueError:
        return a.resolve()


def run_command(cmd: list, log_file: Path):
    cmd = list(map(str, cmd))

    try:
        log_file.parent.mkdir(parents=True, exist_ok=True)

        with open(log_file, mode="w", encoding="utf8") as stream:
            print_command(cmd, stream)
            stream.flush()
            subprocess.run(cmd, check=True, stdout=stream, stderr=subprocess.STDOUT)
    except Exception as e:
        print(f"Log-file {log_file}")
        raise


def print_command(cmd: list, stream):
    line = "> "
    sep = ""

    for arg in cmd:
        text = str(arg)

        if len(line) + len(sep) + len(text) > 98:
            line += " \\"
            print(line, file=stream)
            line = "    "
            sep = ""

        line = f"{line}{sep}{text}"
        sep = " "

    print(line, file=stream)


def run_experiment(args: ninja.ninja_syntax.Writer):
    subprocess.run(["ninja", "-j", str(args.parallel_process_count)], check=True)


if __name__ == "__main__":
    main()
