# ****************************************************************************
# * (C) Copyright 2020, Texas Instruments Incorporated. - www.ti.com
# ****************************************************************************
# 3D visualization wrapper for TI mmWave demo UART point cloud
#
# Usage:
#   python 3D_mmw_parse_script.py
#
# Notes:
# - Requires parser_mmw_demo.py in the same folder (and its dependencies)
# - Uses Open3D to show an interactive 3D point cloud viewer
# - Close the Open3D window or press Ctrl+C to exit cleanly
# ****************************************************************************

import time
import signal
import numpy as np
import serial
import open3d as o3d

from parser_mmw_demo import parser_one_mmw_demo_output_packet

# -----------------------------
# User settings
# -----------------------------
CONFIG_FILE = "xwr68xxconfig.cfg"

CLI_PORT = "COM3"
DATA_PORT = "COM4"
CLI_BAUD = 115200
DATA_BAUD = 921600

DEBUG = False

# Buffering
MAX_BUFFER_SIZE = 2**15
magicWord = np.array([2, 1, 4, 3, 6, 5, 8, 7], dtype=np.uint8)
word = np.array([1, 2**8, 2**16, 2**24], dtype=np.uint32)

byteBuffer = np.zeros(MAX_BUFFER_SIZE, dtype=np.uint8)
byteBufferLength = 0

CLIport = None
Dataport = None


def serialConfig(configFileName: str):
    global CLIport, Dataport

    CLIport = serial.Serial(CLI_PORT, CLI_BAUD, timeout=0.1)
    Dataport = serial.Serial(DATA_PORT, DATA_BAUD, timeout=0.05)

    # Send config lines
    config = [line.rstrip("\r\n") for line in open(configFileName)]
    for line in config:
        CLIport.write((line + "\n").encode())
        print(line)
        time.sleep(0.01)

    return CLIport, Dataport


def parseConfigFile(configFileName: str):
    # (Kept from your script; used mainly for maxRange hints if needed)
    configParameters = {}

    config = [line.rstrip("\r\n") for line in open(configFileName)]

    numRxAnt = 4
    numTxAnt = 3

    startFreq = None
    idleTime = None
    rampEndTime = None
    freqSlopeConst = None
    numAdcSamples = None
    numAdcSamplesRoundTo2 = None
    digOutSampleRate = None

    chirpStartIdx = None
    chirpEndIdx = None
    numLoops = None
    numFrames = None
    framePeriodicity = None

    for line in config:
        splitWords = line.split(" ")
        if not splitWords:
            continue

        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])

            numAdcSamplesRoundTo2 = 1
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 *= 2

            digOutSampleRate = int(splitWords[11])

        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = int(splitWords[5])

    # If config is incomplete, return minimal defaults
    if any(v is None for v in [startFreq, idleTime, rampEndTime, freqSlopeConst, numAdcSamples,
                              numAdcSamplesRoundTo2, digOutSampleRate, chirpStartIdx, chirpEndIdx, numLoops]):
        configParameters["maxRange"] = 10.0
        return configParameters

    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * numAdcSamples
    )
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"]
    )
    configParameters["dopplerResolutionMps"] = 3e8 / (
        2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt
    )
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (
        4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt
    )
    return configParameters


def readAndParseData14xx():
    """
    Reads UART bytes, finds mmWave magic word packets, and parses one packet using TI parser.
    Returns: (dataOK, detObj)
      detObj: {"numObj": int, "x": np.array, "y": np.array, "z": np.array, "range": np.array}
    """
    global byteBuffer, byteBufferLength

    dataOK = 0
    detObj = {}

    # Read a decent chunk to reduce partial packets
    n = Dataport.in_waiting
    if n < 4096:
        n = 4096
    readBuffer = Dataport.read(n)
    if not readBuffer:
        return 0, detObj

    byteVec = np.frombuffer(readBuffer, dtype=np.uint8)
    byteCount = len(byteVec)

    if (byteBufferLength + byteCount) < MAX_BUFFER_SIZE:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength += byteCount

    # Need enough to search for magic + length
    if byteBufferLength <= 16:
        return 0, detObj

    # Find possible magic positions
    possibleLocs = np.where(byteBuffer[:byteBufferLength] == magicWord[0])[0]

    startIdx = []
    for loc in possibleLocs:
        if loc + 8 <= byteBufferLength and np.all(byteBuffer[loc:loc+8] == magicWord):
            startIdx.append(loc)

    if not startIdx:
        return 0, detObj

    # Shift buffer so first magic is at index 0
    if startIdx[0] > 0:
        shift = startIdx[0]
        byteBuffer[:byteBufferLength-shift] = byteBuffer[shift:byteBufferLength]
        byteBuffer[byteBufferLength-shift:byteBufferLength] = 0
        byteBufferLength -= shift

    if byteBufferLength < 16:
        return 0, detObj

    # total packet length at bytes 12..15
    totalPacketLen = int(np.matmul(byteBuffer[12:16].astype(np.uint32), word))

    if byteBufferLength < totalPacketLen or totalPacketLen <= 0:
        return 0, detObj

    # Parse exactly the valid bytes
    allBinData = byteBuffer[:byteBufferLength]
    readNumBytes = byteBufferLength

    parser_result, \
    headerStartIndex, \
    totalPacketNumBytes, \
    numDetObj, \
    numTlv, \
    subFrameNumber, \
    detectedX_array, \
    detectedY_array, \
    detectedZ_array, \
    detectedV_array, \
    detectedRange_array, \
    detectedAzimuth_array, \
    detectedElevation_array, \
    detectedSNR_array, \
    detectedNoise_array = parser_one_mmw_demo_output_packet(
        allBinData, readNumBytes, DEBUG
    )

    if parser_result == 0 and numDetObj > 0:
        detObj = {
            "numObj": int(numDetObj),
            "x": np.asarray(detectedX_array, dtype=np.float32),
            "y": np.asarray(detectedY_array, dtype=np.float32),
            "z": np.asarray(detectedZ_array, dtype=np.float32),
            "range": np.asarray(detectedRange_array, dtype=np.float32),
        }
        dataOK = 1
    else:
        # parser failed for this frame; keep going
        dataOK = 0

    # Shift out the parsed packet
    shiftSize = int(totalPacketNumBytes)
    if shiftSize > 0 and shiftSize <= byteBufferLength:
        byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
        byteBuffer[byteBufferLength - shiftSize:byteBufferLength] = 0
        byteBufferLength -= shiftSize
        if byteBufferLength < 0:
            byteBufferLength = 0

    return dataOK, detObj


def run_open3d_viewer(configParameters):
    """
    Open3D interactive 3D viewer.
    Controls:
      - Left mouse: rotate
      - Right mouse: pan
      - Wheel: zoom
      - Close window or Ctrl+C to exit
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="TI IWR6843 Point Cloud", width=1024, height=768)

    # Coordinate frame
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(frame)

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    # Optional: set a reasonable starting view distance
    R = float(configParameters.get("maxRange", 10.0))
    # Open3D camera is user-controlled; we just initialize some geometry.

    running = True

    def _sigint_handler(sig, frame_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _sigint_handler)

    # Simple "accumulation" option to look more LiDAR-like:
    # Keep last N frames of points
    ACCUM_FRAMES = 10
    buf_pts = []

    last_print = time.time()

    try:
        while running:
            # If the user closes the window, poll_events() returns False
            if not vis.poll_events():
                break

            dataOK, detObj = readAndParseData14xx()
            if dataOK and detObj.get("numObj", 0) > 0:
                x = detObj["x"]
                y = detObj["y"]
                z = detObj["z"]

                pts = np.stack([x, y, z], axis=1)  # Nx3 (meters)

                # accumulate a few frames to make it feel denser
                buf_pts.append(pts)
                if len(buf_pts) > ACCUM_FRAMES:
                    buf_pts.pop(0)

                pts_all = np.concatenate(buf_pts, axis=0)

                pcd.points = o3d.utility.Vector3dVector(pts_all.astype(np.float64))

                # Optional: color by height (z) or range; here we keep a fixed color
                colors = np.tile(np.array([[1.0, 0.0, 0.0]]), (pts_all.shape[0], 1))
                pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))

                vis.update_geometry(pcd)

                # Debug print once per second
                if time.time() - last_print > 1.0:
                    last_print = time.time()
                    r_xy = np.sqrt(x**2 + y**2)
                    print(
                        f"numObj={detObj['numObj']}  "
                        f"range[min,max]=({float(r_xy.min()):.2f},{float(r_xy.max()):.2f})  "
                        f"z[min,max]=({float(z.min()):.2f},{float(z.max()):.2f})"
                    )

            vis.update_renderer()
            time.sleep(0.01)

    finally:
        vis.destroy_window()


def shutdown():
    global CLIport, Dataport
    try:
        if CLIport is not None:
            CLIport.write(b"sensorStop\n")
            time.sleep(0.05)
    except Exception:
        pass

    for p in (CLIport, Dataport):
        try:
            if p is not None:
                p.close()
        except Exception:
            pass


def main():
    global CLIport, Dataport

    CLIport, Dataport = serialConfig(CONFIG_FILE)
    configParameters = parseConfigFile(CONFIG_FILE)

    try:
        run_open3d_viewer(configParameters)
    finally:
        shutdown()


if __name__ == "__main__":
    main()

