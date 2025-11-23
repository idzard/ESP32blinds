# ArtNet Multi-Device Upload Instructions

## Overview
The code now supports per-device DMX channel mapping through configuration files. You can upload the **same code** to all devices, but each device will use **different DMX channels** based on its config file.

## DMX Channel Mapping

All devices listen to **Universe 1**, but use different channel ranges:

| Device | IP Address   | mDNS Name | DMX Channels | Channel Offset |
|--------|-------------|-----------|--------------|----------------|
| 1      | 10.0.0.101  | window1   | 1-4          | 0              |
| 2      | 10.0.0.102  | window2   | 5-8          | 4              |
| 3      | 10.0.0.103  | window3   | 9-12         | 8              |
| 4      | 10.0.0.104  | window4   | 13-16        | 12             |

### Channel Breakdown per Device:
- **Channel 1-2** (Device 1) or **5-6** (Device 2) or **9-10** (Device 3) or **13-14** (Device 4): Stepper 0 (Bottom Screen)
- **Channel 3-4** (Device 1) or **7-8** (Device 2) or **11-12** (Device 3) or **15-16** (Device 4): Stepper 1 (Top Screen)

Each stepper uses 2 bytes (16-bit value, range 0-65535)

## Upload Process

### Step 1: Compile the Code Once
1. Build the project in PlatformIO (click the ✓ Build icon)
2. This compiles the code that will be uploaded to all devices

### Step 2: Upload to Each Device

For each device, you need to:

#### A. Copy the correct config file
Before uploading to each device, rename the appropriate config file:

**For Device 1 (10.0.0.101):**
- Copy `data/config-device1.json` to `data/config.json`

**For Device 2 (10.0.0.102):**
- Copy `data/config-device2.json` to `data/config.json`

**For Device 3 (10.0.0.103):**
- Copy `data/config-device3.json` to `data/config.json`

**For Device 4 (10.0.0.104):**
- Copy `data/config-device4.json` to `data/config.json`

#### B. Upload filesystem (SPIFFS)
1. In PlatformIO, click "Upload Filesystem Image" (or use task: `pio run -t uploadfs`)
2. This uploads the `config.json` file to the device

#### C. Upload code
1. In PlatformIO, select the appropriate environment:
   - `wt32-eth01-frame1` for 10.0.0.101
   - `wt32-eth01-frame2` for 10.0.0.102
   - `wt32-eth01-frame3` for 10.0.0.103
   - `wt32-eth01-frame4` for 10.0.0.104
2. Click Upload (→ arrow icon)

#### D. Repeat for each device
Repeat steps A, B, and C for all 4 devices.

## Configuration Files

All config files include:
- `mdnsName`: Unique name for each device (window1, window2, window3, window4)
- `ipadress`: Static IP address
- `gateway`: Network gateway (10.0.0.1)
- `subnet`: Subnet mask (255.255.255.0)
- `reverseStepper0`: Reverse direction for stepper 0 (false)
- `reverseStepper1`: Reverse direction for stepper 1 (true)
- `dmxChannelOffset`: DMX channel offset (0, 4, 8, or 12)

## Testing

After uploading to all devices:
1. Send ArtNet data to Universe 1
2. Channels 1-4 will control Device 1 (10.0.0.101)
3. Channels 5-8 will control Device 2 (10.0.0.102)
4. Channels 9-12 will control Device 3 (10.0.0.103)
5. Channels 13-16 will control Device 4 (10.0.0.104)

Each device operates independently based on its configured channel offset.

## Alternative: Pre-compiled SPIFFS Images

You can also create pre-compiled SPIFFS images for each device and keep them in the `config-images/` folder, similar to the existing `window1-spiffs.bin` and `window2-spiffs.bin` files. Then you can flash the appropriate image to each device without having to swap config files.
