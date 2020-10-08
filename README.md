# eswContest
#### 2020년 제 18회 임베디드 소프트웨어 경진대회 - 자율주행모형자동차 부문 출품작
#### 시연 영상 URL : [https://youtu.be/-LHJQVNMprU](https://youtu.be/-LHJQVNMprU)
####    팀 이름 : 알아서가SSU
####    팀 구성 : 숭실대학교 김대희, 강민수, 이상민, 이석준
<br/>

# 주요 코드
프로그램 폴더 - [eswContest/CarSDK/Application/self_driving_program](https://github.com/fabiano77/eswContest/blob/master/CarSDK/Application/self_driving_program)

> 메인 - [main.c](https://github.com/fabiano77/eswContest/blob/master/CarSDK/Application/self_driving_program/main.c)
> 
> 영상 처리 - [imgProcess.h(주석 포함)](https://github.com/fabiano77/eswContest/blob/master/CarSDK/Application/self_driving_program/imgProcess.h), [imgProcess.cpp](https://github.com/fabiano77/eswContest/blob/master/CarSDK/Application/self_driving_program/imgProcess.cpp)
> 
> 차량 제어 - [control_mission.h(주석 포함)](https://github.com/fabiano77/eswContest/blob/master/CarSDK/Application/self_driving_program/control_mission.h), [control_mission.cpp](https://github.com/fabiano77/eswContest/blob/master/CarSDK/Application/self_driving_program/control_mission.cpp)
> 
> 미션 수행 - [mission.h(주석 포함)](https://github.com/fabiano77/eswContest/blob/master/CarSDK/Application/self_driving_program/mission.h), [mission.c](https://github.com/fabiano77/eswContest/blob/master/CarSDK/Application/self_driving_program/mission.c)


---
<br/>

### Requirement(Installment)
- Ubuntu 16.04, 18.04 (WSL2 is available)
- Hyundai Autron's Embedded Car

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
- Tracking Red</br>
`cd 1st_mission_code` 
After changing directory, make the run file by `make`.
- Main Autonomous Driving System </br>
`cd self_driving_program`</br>
After changing directory, make the run file by `make`.

5. Transfer to Embedded Car
Connect the Embedded Car with your PC by ethernet cable. (setup the ethernet environment part is needed#TODO)<br/>

```shell scripts
    scp self_driving_program root@10.10.70.4:/home/root/self_driving_program
```
Now, get the authority of a car by using ssh. You can test our code.
```shell scripts
    ssh root@10.10.70.4
    ./self_driving_program
```

### Contributors
- Daehee Kim (fabiano77)<br/>
- Minsoo Kang (3neutronstar)<br/>
- Sangmin Lee (smlee212)<br/>
- Seokjun Lee (ykykyk112)<br/>
<br/>
Soongil Univ.