### date: 
2022, July 15th

### author: 
franck.thudor@interdigital.com

### aim:
Some parameters are hard-coded in current version of miv_mpi encoder, hence there is no need to have them in the encoder configuration file _M_1_TMIV_encode.json_. Moreover, some of them were not correctly set. Below is the list of parameters removed from the configuration file, with their hard-coded value:
```
"codecGroupIdc": "HEVC Main10",
"levelIdc": "3.5",
"reconstructionIdc": "Rec Unconstrained",
"toolsetIdc": "MIV Extended",
"oneV3cFrameOnly": false,
```
    