import argparse
import os
from pathlib import Path
import subprocess

# Draft V-PCC conformance bitstreams obtained from https://git.mpeg.expert/MPEG/PCC/Specs/23090-20/-/branches

# Excluding older ones that are known to be invalid:
#
#   * CCMSEIJMHHVTM-MC2_loot
#   * CCMSEIHM-MC1_loot
#   * CCMSEIHM-MC2_loot
#   * PLR-MC1PLR1_redandblack
#   * PTRAW-ECM1_redandblack
#   * PTEOM-ECM1LOSGEO1_redandblack
#   * PTRAX-ECM1LOSGEO1_redandblack

CONFORMANCE_BITSTREAMS = [
    "HEVC444_Basic_Rec2_PTRAW_INTERDIGITAL.bin",
    "HEVC444_Extended_Rec1_PTEOM_LOSGEO_INTERDIGITAL.bin",
    "HEVC444_Extended_Rec1_PTRAX_LOSGEO_INTERDIGITAL.bin",
    "HEVCMAIN10BASIC_MTLNONUNI_1MAP_SS_basketball_player.bit",
    "HEVCMAIN10BASIC_MTLNONUNI_2MAP_MS_basketball_player.bit",
    "HEVCMAIN10BASIC_MTL_1MAP_SS_basketball_player.bit",
    "HEVCMAIN10BASIC_MTL_2MAP_MS_basketball_player.bit",
    "HEVCMAIN10BASIC_STL_1MAP_SS_basketball_player.bit",
    "HEVCMAIN10BASIC_STL_1MAP_SS_longdress.bit",
    "HEVCMAIN10BASIC_STL_2MAP_MS_basketball_player.bit",
    "HEVCMain10_Basic_Rec0_LOSSYOM_SAMSUNG_v1.bit",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M2P21MC1_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M2P21MC2_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P11MC1_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P11MC2_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P21MC1_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLINTRA_tileT2M3P21MC2_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLLRA_tileT2M2P21MC1_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLLRA_tileT2M2P21MC2_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLLRA_tileT2M3P21MC1_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_MTLLRA_tileT2M3P21MC2_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_STLINTRA_MC1_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_STLINTRA_MC2_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_STLINTRA_SONY.bin",
    "HEVCMain10_Basic_Rec0_STLLRA_MC1_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec0_STLLRA_MC2_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec1_ATTRSM_SAMSUNG_v1.bit",
    "HEVCMain10_Basic_Rec1_GEOSM_SONY.bin",
    "HEVCMain10_Basic_Rec1_PDI_INTERDIGITAL.bin",
    "HEVCMain10_Basic_Rec2_ATTRSM_SAMSUNG_v1.bit",
    "HEVCMain10_Basic_Rec2_OCCSY_PBF_INTERDIGITAL.bin",
    "HEVCMAIN10_BASIC_STILL_MTL_2MAP_MS_basketball_player.bit",
    "HEVCMAIN10_BASIC_STILL_MTL_2MAP_MS_longdress.bit",
    "HEVCMAIN10_BASIC_STILL_STL_1MAP_SS_basketball_player.bit",
    "HEVCMAIN10_BASIC_STILL_STL_1MAP_SS_longdress.bit",
    "HEVCMain10_Extended_Rec0_PEXT_SONY.bin",
    "HEVCMain10_Extended_Rec1_PLR_MC1PLR1_INTERDIGITAL.bin",
    "HEVCMain10_Extended_Rec2_PLR_MC1PLR1_OCCSY_PBF_INTERDIGITAL.bin",
    "MP4RA_Basic_Rec0_SEICCM_MC1_INTERDIGITAL.bin",
    "MP4RA_Basic_Rec0_SEICCM_MC2_INTERDIGITAL.bin",
    "redandblack_vox10_GOF0.bin",
]

KNOWN_REASONS = {
    "HEVC444_Basic_Rec2_PTRAW_INTERDIGITAL.bin": "asps_raw_patch_enabled_flag",
    "HEVC444_Extended_Rec1_PTEOM_LOSGEO_INTERDIGITAL.bin": "asps_raw_patch_enabled_flag",
    "HEVC444_Extended_Rec1_PTRAX_LOSGEO_INTERDIGITAL.bin": "asps_raw_patch_enabled_flag",
    "HEVCMain10_Basic_Rec1_PDI_INTERDIGITAL.bin": "asps_pixel_deinterleaving_enabled_flag",
    "HEVCMain10_Extended_Rec1_PLR_MC1PLR1_INTERDIGITAL.bin": "asps_plr_enabled_flag",
    "HEVCMain10_Extended_Rec2_PLR_MC1PLR1_OCCSY_PBF_INTERDIGITAL.bin": "asps_plr_enabled_flag",
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

    args.output_dir.mkdir(parents=True, exist_ok=True)

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
        else:
            if name in KNOWN_REASONS:
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
