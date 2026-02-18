# IWR6843-Read-Data-Python-MMWAVE-SDK
Read and plot IWR6843ISK sensor serial data using a Python program.

This is a modified version of the code created by ibaiGorordo (https://github.com/ibaiGorordo/IWR1443-Read-Data-Python-MMWAVE-SDK-1/blob/master/readData_IWR1443.py) to read data from the IWR1443. 

This code was also developed using the MMWAVE SDK version 3.05.00.04 parser demo scripts provided by Texas Instruments.


TO RUN:
---

### Install dependencies

1. Use Python 3.12 (download [here](https://www.python.org/downloads/release/python-3120/), need for Open3D version)
2. Create virtual environment
```bash
py -3.12 -m venv .venv
```
3. Activate virtual environment
```bash
.\.venv\Scripts\Activate.ps1
```
4. Install requirements
```bash
python -m pip install -r requirements.txt
```

### Edit script
Make sure the `COM` ports are selected to the correct ports in the `mmw_parse_script.py` file. Find these in the device manager. More information here: [https://dev.ti.com/tirex/explore/node?isTheia=false&node=A__AOJgwAYscJOUPEaB-c-y7A__radar_toolbox__1AslXXD__2.20.00.05](https://dev.ti.com/tirex/explore/node?isTheia=false&node=A__AOJgwAYscJOUPEaB-c-y7A__radar_toolbox__1AslXXD__2.20.00.05)

### Run
1. Make sure no other program is using the needed COM ports.
2. Run python script for 2D
```bash
python mmw_parse_script.py
```
3. Run python script for 3D
```bash
python 3D_mmw_parse_script.py
```
