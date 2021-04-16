# TMIV tools

## Scale yuv sequences

Script [scale_sequence_yuvs.py](scale_sequence_yuvs.py) scales yuv sequences by a (float) scaling factor.
It relies on ffmpeg for scaling, so make sure you have that installed.
We use ffmpeg's [default `bicubic` scaling method](https://trac.ffmpeg.org/wiki/Scaling), which might not be optimal for depth views.
When called with

```shell
./scale_sequence_yuvs.py /ctc_content/E 0.25 /scaled_ctc_content/E
```

the script will
- find all `.yuv` files in folder `/ctc_content/E`
- for each file, parse color space and resolution from the file name
- scale both their width and height with a factor 0.25
- write the output file with corrected resolution in the filename to the output folder `/scaled_ctc_content/E`

## Scale sequence configuration files

Script [scale_sequence_json.py](scale_sequence_json.py), for each camera, scales entry `Resolution` by a scaling factor.
If the camera projection is `Perspective`, it also scales fields `Focal` and `Principle_point`
Call e.g.

```shell
./scale_sequence_json.py /sequences/E.json 0.25 /scaled_sequences/E.json
```
