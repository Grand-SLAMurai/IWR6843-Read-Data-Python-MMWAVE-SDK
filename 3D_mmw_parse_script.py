# ****************************************************************************
# * (C) Copyright 2020, Texas Instruments Incorporated. - www.ti.com
# ****************************************************************************
# 3D visualization wrapper for TI mmWave demo UART point cloud (Open3D)
#
# IMPORTANT:
# - If detectedElevation_array is ~0 for all points, your current firmware/config is 2D.
#   You cannot recover real Z in software; you must run a 3D-capable demo/config.
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
CONFIG_FILE = "xwr68xxconfig3D.cfg"

CLI_PORT = "COM3"
DATA_PORT = "COM4"
CLI_BAUD = 115200
DATA_BAUD = 921600

DEBUG = False

# --- Visualization behavior switches ---
ACCUM_FRAMES = 10

# If your parser outputs x,y reliably, you can keep them and only reconstruct z.
# If your parser x,y are also "2D-only", set this False to compute x,y from range+azimuth.
PREFER_PARSER_XY = True

# This is ONLY a visualization hack (adds thickness). Not real depth.
FAKE_Z_IF_2D = False
FAKE_Z_SCALE_METERS = 0.30

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
    # Used only for maxRange hints; safe defaults if parse fails.
    cfg = {"maxRange": 10.0}
    try:
        lines = [line.rstrip("\r\n") for line in open(configFileName)]
    except Exception:
        return cfg

    startFreq = idleTime = rampEndTime = freqSlopeConst = None
    numAdcSamples = numAdcSamplesRoundTo2 = digOutSampleRate = None
    chirpStartIdx = chirpEndIdx = numLoops = None

    numTxAnt = 3

    for line in lines:
        w = line.split()
        if not w:
            continue
        if w[0] == "profileCfg":
            startFreq = int(float(w[2]))
            idleTime = int(w[3])
            rampEndTime = float(w[5])
            freqSlopeConst = float(w[8])
            numAdcSamples = int(w[10])
            numAdcSamplesRoundTo2 = 1
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 *= 2
            digOutSampleRate = int(w[11])
        elif w[0] == "frameCfg":
            chirpStartIdx = int(w[1])
            chirpEndIdx = int(w[2])
            numLoops = int(w[3])

    if any(v is None for v in [startFreq, idleTime, rampEndTime, freqSlopeConst,
                              numAdcSamples, numAdcSamplesRoundTo2,
                              digOutSampleRate, chirpStartIdx, chirpEndIdx, numLoops]):
        return cfg

    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    numDopplerBins = numChirpsPerFrame / numTxAnt

    maxRange = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    cfg["maxRange"] = float(maxRange)
    cfg["numDopplerBins"] = float(numDopplerBins)
    return cfg


def readAndParseData14xx():
    """
    Reads UART bytes, finds mmWave magic word packets, and parses one packet using TI parser.
    Returns:
      (dataOK, detObj)
    detObj keys:
      numObj, x,y,z, range, azimuth, elevation
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

    if byteBufferLength <= 16:
        return 0, detObj

    # Find magic
    possibleLocs = np.where(byteBuffer[:byteBufferLength] == magicWord[0])[0]
    startIdx = []
    for loc in possibleLocs:
        if loc + 8 <= byteBufferLength and np.all(byteBuffer[loc:loc+8] == magicWord):
            startIdx.append(loc)
    if not startIdx:
        return 0, detObj

    # Shift so first magic is at index 0
    if startIdx[0] > 0:
        shift = startIdx[0]
        byteBuffer[:byteBufferLength-shift] = byteBuffer[shift:byteBufferLength]
        byteBuffer[byteBufferLength-shift:byteBufferLength] = 0
        byteBufferLength -= shift

    if byteBufferLength < 16:
        return 0, detObj

    totalPacketLen = int(np.matmul(byteBuffer[12:16].astype(np.uint32), word))
    if byteBufferLength < totalPacketLen or totalPacketLen <= 0:
        return 0, detObj

    allBinData = byteBuffer[:byteBufferLength]
    readNumBytes = byteBufferLength

    (parser_result,
     headerStartIndex,
     totalPacketNumBytes,
     numDetObj,
     numTlv,
     subFrameNumber,
     detectedX_array,
     detectedY_array,
     detectedZ_array,
     detectedV_array,
     detectedRange_array,
     detectedAzimuth_array,
     detectedElevation_array,
     detectedSNR_array,
     detectedNoise_array) = parser_one_mmw_demo_output_packet(allBinData, readNumBytes, DEBUG)

    if parser_result == 0 and numDetObj > 0:
        detObj = {
            "numObj": int(numDetObj),
            "x": np.asarray(detectedX_array, dtype=np.float32),
            "y": np.asarray(detectedY_array, dtype=np.float32),
            "z": np.asarray(detectedZ_array, dtype=np.float32),
            "range": np.asarray(detectedRange_array, dtype=np.float32),
            "azimuth": np.asarray(detectedAzimuth_array, dtype=np.float32),
            "elevation": np.asarray(detectedElevation_array, dtype=np.float32),
        }
        dataOK = 1

    # Shift out the parsed packet
    shiftSize = int(totalPacketNumBytes)
    if 0 < shiftSize <= byteBufferLength:
        byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
        byteBuffer[byteBufferLength - shiftSize:byteBufferLength] = 0
        byteBufferLength -= shiftSize
        if byteBufferLength < 0:
            byteBufferLength = 0

    return dataOK, detObj


def compute_xyz(detObj):
    """
    Return Nx3 points in meters.
    Strategy:
      - If elevation has variation: compute z from range*sin(elev).
      - If elevation is ~0: warn and either keep z=0 or fake z (optional).
    """
    x = detObj["x"]
    y = detObj["y"]
    z = detObj["z"]
    r = detObj["range"]
    az = detObj["azimuth"]
    el = detObj["elevation"]

    # Heuristics for "no 3D"
    el_span = float(np.max(el) - np.min(el)) if el.size else 0.0
    z_span = float(np.max(z) - np.min(z)) if z.size else 0.0

    have_elev = (el.size > 0) and (el_span > 1e-4)  # radians-ish (depends on parser, but works for 0/nonnull)
    have_z = (z.size > 0) and (z_span > 1e-4)

    if have_z:
        # True 3D already present
        return np.stack([x, y, z], axis=1), True, el_span, z_span

    if have_elev:
        # Reconstruct z from range + elevation
        # Assume azimuth/elevation are in radians (common). If your parser outputs degrees,
        # z will look wildly wrong; see notes below.
        z_rec = r * np.sin(el)
        r_h = r * np.cos(el)

        if PREFER_PARSER_XY:
            # Keep x,y from parser; only replace z
            return np.stack([x, y, z_rec], axis=1), True, el_span, float(np.max(z_rec)-np.min(z_rec))

        # Compute x,y from azimuth + horizontal projection
        x_rec = r_h * np.sin(az)
        y_rec = r_h * np.cos(az)
        return np.stack([x_rec, y_rec, z_rec], axis=1), True, el_span, float(np.max(z_rec)-np.min(z_rec))

    # No elevation -> no real 3D exists
    if FAKE_Z_IF_2D and x.size:
        # purely for visualization; adds thickness based on angle/range
        z_fake = FAKE_Z_SCALE_METERS * (np.sin(az) * 0.5 + 0.5)
        return np.stack([x, y, z_fake], axis=1), False, el_span, float(np.max(z_fake)-np.min(z_fake))

    return np.stack([x, y, np.zeros_like(x)], axis=1), False, el_span, 0.0


def run_open3d_viewer(configParameters):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="TI IWR6843 Point Cloud", width=1024, height=768)

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(frame)

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    running = True

    def _sigint_handler(sig, frame_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _sigint_handler)

    buf_pts = []
    last_print = 0.0
    warned_2d = False

    try:
        while running:
            if not vis.poll_events():
                break

            dataOK, detObj = readAndParseData14xx()
            if dataOK and detObj.get("numObj", 0) > 0:
                pts, real3d, el_span, z_span = compute_xyz(detObj)

                buf_pts.append(pts)
                if len(buf_pts) > ACCUM_FRAMES:
                    buf_pts.pop(0)

                pts_all = np.concatenate(buf_pts, axis=0) if buf_pts else pts

                pcd.points = o3d.utility.Vector3dVector(pts_all.astype(np.float64))

                colors = np.tile(np.array([[1.0, 0.0, 0.0]]), (pts_all.shape[0], 1))
                pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))

                vis.update_geometry(pcd)

                now = time.time()
                if now - last_print > 1.0:
                    last_print = now

                    el = detObj["elevation"]
                    rxy = np.sqrt(detObj["x"]**2 + detObj["y"]**2)

                    print(
                        f"numObj={detObj['numObj']}  "
                        f"rangeXY[min,max]=({float(rxy.min()):.2f},{float(rxy.max()):.2f})  "
                        f"zSpan={z_span:.4f}  elevSpan={el_span:.6f}  "
                        f"elev[min,max]=({float(el.min()):.4f},{float(el.max()):.4f})"
                    )

                    if not real3d and not warned_2d:
                        warned_2d = True
                        print("\n[INFO] elevation is all ~0. Your current config/demo is effectively 2D.")
                        print("       You cannot recover real Z in software.")
                        print("       To get true 3D, flash/run a 3D-capable demo (e.g., people tracking/counting) + matching .cfg.\n")

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

