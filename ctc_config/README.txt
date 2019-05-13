CTC configuration
-----------------

In reference to MPEG/N18443 Common Test Conditions for Immersive Video

1) Download the sequences for A...E into separate directories
2) Copy sequences/*.json to the respective directories
3) Copy pose_traces/*.csv to the respective directories

For TMIV anchor (encoding/decoding) use the TMIV_anchor_*.json configuration 
files. For TMIV view anchor (encoding/decoding) use the TMIV_view_anchor_*.json
configuration files. The output files have "R0" in the name for "uncompressed".
The make_decoder_configs script generates variants with QP1...QP5 in the name.

For example:

mkdir output_SA
Encoder -c ctc_config/TMIV_anchor_CG1_A.json \
        -p SourceDirectory //fileserver/content/SA \
		-p OutputDirectory output_SA
# Run HEVC to make the QP4 streams
Decoder -c ctc_config/QP4/TMIV_anchor_CG1_A.json \
        -p SourceDirectory //fileserver/content/SA \
		-p OutputDirectory output_SA \
		-p OutputCameraName v7
...
