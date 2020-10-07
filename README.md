# eswContest

### Requirement(Installment)
- Ubuntu 16.04, 18.04 (WSL2 is available)
- Hyundai Autorun's Embedded Car

### How to Download Requirement

#### Prerequisites
1. Download all the required package
```shell scripts
    sudo apt-get update
    sudo apt-get install build-essential
    sudo apt-get install libc6:i386 libncurses5:i386 libstdc++6:i386 libz1:i386
    sudo apt-get install ssh
    sudo apt-get install gparted

```
2. Download a Setup file 
```shell scripts
    git clone https://github.com/fabiano77/eswContest.git
    cd eswContest
```
3. Download crosscompiler
It is classified. (You can't get it in ordinary way)<br/>
If you have the crosscompiler, `tar -xzvf CarSDK.tar.gz` and move the `crossCompiler` dir to `eswContest/CarSDK/`.

4. Compile the file
```shell scripts
    cd CarSDK/Application/
```
- 'tracking red'
`cd 1st_mission_code` </br>
After changing directory, `make`.
- 'main autonomous driving system'
`cd self_driving_program`</br>
After changing directory, `make`.

5. Transfer to Embedded Car


#### Setting Communication with Car and PC


### Contributors
Daehee Kim (fabiano77)<br/>
Minsoo Kang (3neutronstar)<br/>
Sangmin Lee (smlee212)<br/>
Seokjun Lee (ykykyk112)<br/>
Soongil Univ.
