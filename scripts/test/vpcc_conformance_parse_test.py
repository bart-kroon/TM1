import argparse
import os
from pathlib import Path
import subprocess

# Text of ISO/IEC FDIS 23090-20 Conformance for V-PCC [ISO/IEC JTC 1/SC 29/WG 07 N 00465]

CONFORMANCE_BITSTREAMS = [
    "HEVCMain10_Basic_Rec0_CCMSEIJMHHVTM_MC2_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_LOSSYOM_SAMSUNG_v1.bit",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M2P21MC1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M2P21MC2_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P11MC1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P11MC2_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P21MC1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P21MC2_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_MTLLRA_tileT2M2P21MC1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_MTLLRA_tileT2M2P21MC2_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_MTLLRA_tileT2M3P21MC1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_SEICCM_MC1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_SEICCM_MC2_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_STLINTRA_MC1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_STLINTRA_MC2_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_STLINTRA_SONY.bit",
    "HEVCMain10_Basic_Rec0_STLLRA_MC1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec0_STLLRA_MC2_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec1_ATTRSM_SAMSUNG_v1.bit",
    "HEVCMain10_Basic_Rec1_GEOSM_SONY.bit",
    "HEVCMain10_Basic_Rec1_PDI_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec2_ATTRSM_SAMSUNG_v1.bit",
    "HEVCMain10_Basic_Rec2_OCCSY_PBF_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec2_PLR_MC1PLR1_INTERDIGITAL.bit",
    "HEVCMain10_Basic_Rec2_PTRAW_INTERDIGITAL.bit",
    "HEVCMain10_Extended_Rec0_PEXT_SONY.bit",
    "HEVCMain10_Extended_Rec1_PTEOM_LOSGEO_INTERDIGITAL.bit",
    "HEVCMain10_Extended_Rec1_PTRAX_LOSGEO_INTERDIGITAL.bit",
]

KNOWN_REASONS = {
    "HEVCMain10_Basic_Rec0_CCMSEIJMHHVTM_MC2_INTERDIGITAL.bit": "ath_type != AthType::I_TILE && ath_type != SKIP_TILE",
    "HEVCMain10_Basic_Rec0_LOSSYOM_SAMSUNG_v1.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M2P21MC2_INTERDIGITAL.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P11MC2_INTERDIGITAL.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P21MC2_INTERDIGITAL.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec0_MTLLRA_tileT2M2P21MC2_INTERDIGITAL.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec0_SEICCM_MC1_INTERDIGITAL.bit": "ath_type != AthType::I_TILE && ath_type != SKIP_TILE",
    "HEVCMain10_Basic_Rec0_SEICCM_MC2_INTERDIGITAL.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec0_STLINTRA_MC2_INTERDIGITAL.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec0_STLLRA_MC1_INTERDIGITAL.bit": "ath_type != AthType::I_TILE && ath_type != SKIP_TILE",
    "HEVCMain10_Basic_Rec0_STLLRA_MC2_INTERDIGITAL.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec1_ATTRSM_SAMSUNG_v1.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec1_PDI_INTERDIGITAL.bit": "asps_pixel_deinterleaving_enabled_flag",
    "HEVCMain10_Basic_Rec2_ATTRSM_SAMSUNG_v1.bit": "vps_multiple_map_streams_present_flag",
    "HEVCMain10_Basic_Rec2_PLR_MC1PLR1_INTERDIGITAL.bit": "asps_plr_enabled_flag",
    "HEVCMain10_Basic_Rec2_PTRAW_INTERDIGITAL.bit": "asps_raw_patch_enabled_flag",
    "HEVCMain10_Extended_Rec1_PTEOM_LOSGEO_INTERDIGITAL.bit": "asps_raw_patch_enabled_flag",
    "HEVCMain10_Extended_Rec1_PTRAX_LOSGEO_INTERDIGITAL.bit": "vps_auxiliary_video_present_flag[ atlasId ]",
}


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-b",
        "--bitstreams-dir",
        type=Path,
        required=True,
        help="The bitstreams directory with the V-PCC conformance bitstreams",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        required=True,
        help="The directory to which to write the log files",
    )
    parser.add_argument(
        "-t", "--tmiv-dir", type=Path, required=True, help="TMIV installation prefix"
    )
    return parser.parse_args()


def tmiv_parser(args):
    file = args.tmiv_dir / "bin" / "TmivParser"
    return file.with_suffix(".exe") if os.name == "nt" else file


def main(args):
    success = True

    for name in CONFORMANCE_BITSTREAMS:
        error_file = (args.output_dir / name).with_suffix(".err")
        error_file.unlink(missing_ok=True)

        result = subprocess.run(
            [tmiv_parser(args)]
            + ["-b", args.bitstreams_dir / name]
            + ["-o", (args.output_dir / name).with_suffix(".hls")],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            encoding="utf8",
        )
        if result.returncode == 0:
            print(f"{name}: OK.")
        elif name in KNOWN_REASONS:
            print(f"{name}: known TMIV limitation: {KNOWN_REASONS[name]}")
        else:
            print(f"{name}: ERROR, see .err file for details.")
            success = False

            with open(error_file, mode="w") as stream:
                stream.write(result.stdout)

    if not success:
        print("There are one or more errors.")

    return success


if __name__ == "__main__":
    exit(0 if main(parse_arguments()) else 1)
