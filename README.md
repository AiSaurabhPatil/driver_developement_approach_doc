# Robot Arm Hardware Interface — Development Approach


## Executive Summary

We are building a system where a **Vision-Language-Action (VLA) model** (e.g., π0.5) outputs motor commands that control physical robot arms equipped with **Feetech actuators**. Since the physical robot hardware is not yet available, we will use a **"Simulation First"** development strategy to build and validate the entire software stack.

This document outlines a phased approach that progressively increases hardware fidelity:

```
Phase 0: SIL (Software-in-the-Loop)  →  Pure software, virtual serial ports
Phase 1: HIL (Hardware-in-the-Loop)  →  ESP32 microcontroller as mock servo
```


## Phase 0: Software-in-the-Loop (SIL)

### Objective
Run the complete ROS2 driver stack on a single machine using **virtual serial ports**. The driver believes it is talking to real Feetech motors, but it's actually communicating with a Python-based motor simulator.

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        HOST MACHINE                             │
│                                                                 │
│  ┌──────────────┐      socat        ┌────────────────────────┐  │
│  │              │   /dev/pts/1  ←──→  /dev/pts/2             │  │
│  │  ROS2 Node   │        ↑                   ↑               │  │
│  │  (Driver)    │        │                   │               │  │
│  │              │   Opens this         Opens this            │  │
│  │  mock_serial │   as "hardware"      runs simulator        │  │
│  │  _system.cpp │                                            │  │
│  └──────────────┘                ┌────────────────────────┐  │  │
│         ↑                        │  feetech_simulator.py  │  │  │
│         │                        │                        │  │  │
│  ros2_control                    │  • Parses SCS protocol │  │  │
│  controllers                     │  • Simulates motor     │  │  │
│                                  │    physics (position,  │  │  │
│                                  │    velocity, load)     │  │  │
│                                  │  • Injects faults      │  │  │
│                                  └────────────────────────┘  │  │
└─────────────────────────────────────────────────────────────────┘
```

### Deliverables for Phase 0

| Item | Description |
|------|-------------|
| `feetech_simulator.py` | Python script that emulates Feetech SCS/STS protocol |
| Driver refactor | Modify `MockDriver` to open a real serial port path instead of in-memory buffers |
| `test_sil.launch.py` | Launch file that starts `socat`, simulator, and driver together |
| CI/CD integration | GitHub Actions workflow that runs SIL tests on every push |

### Feetech Protocol Reference

The simulator must implement the **SCS/STS serial protocol**:

```
┌────────┬────────┬────────┬────────┬─────────────┬──────────┐
│ Header │ Header │   ID   │ Length │ Instruction │ Checksum │
│  0xFF  │  0xFF  │ (1B)   │ (1B)   │  + Params   │   (1B)   │
└────────┴────────┴────────┴────────┴─────────────┴──────────┘
```

**Key Instructions:**

| Instruction | Code | Description |
|-------------|------|-------------|
| PING | 0x01 | Check if servo is alive |
| READ | 0x02 | Read data from servo memory table |
| WRITE | 0x03 | Write data to servo memory table |
| REG_WRITE | 0x04 | Buffer write (executed on ACTION) |
| ACTION | 0x05 | Execute all buffered REG_WRITEs |
| SYNC_WRITE | 0x83 | Write same data to multiple servos |

**Memory Table (Common Addresses):**

| Address | Name | R/W | Size |
|---------|------|-----|------|
| 0x38 | Goal Position | RW | 2 bytes |
| 0x39 | Goal Position (H) | RW | - |
| 0x3A | Running Time | RW | 2 bytes |
| 0x38 | Current Position | R | 2 bytes |
| 0x3C | Current Speed | R | 2 bytes |
| 0x3E | Current Load | R | 2 bytes |

### Fault Injection (Critical for Robustness)

The simulator should support these failure modes:

```python
# Example simulator config
FAULT_CONFIG = {
    "packet_drop_rate": 0.05,      # 5% of packets silently dropped
    "checksum_corruption": 0.02,   # 2% of responses have bad checksum
    "response_delay_ms": (1, 10),  # Random delay 1-10ms
    "timeout_simulation": False,   # Set True to test timeout recovery
}
```

**Why?** We need to verify the driver handles:
- Partial reads (packet split across two `read()` calls)
- Timeout recovery (servo doesn't respond)
- Checksum validation (corrupted data rejected)

---

## Phase 1: Hardware-in-the-Loop (HIL)

### Objective
Replace the Python simulator with a **physical ESP32 microcontroller** that behaves like a Feetech servo. This introduces real USB-serial latency and electrical characteristics.

### Architecture

```
┌──────────────────┐         USB Cable        ┌──────────────────┐
│                  │                          │                  │
│    HOST PC       │  /dev/ttyUSB0 ←────────→ │     ESP32        │
│                  │                          │                  │
│  ┌────────────┐  │                          │  ┌────────────┐  │
│  │ ROS2 Node  │  │                          │  │ Arduino    │  │
│  │ (Driver)   │  │                          │  │ Sketch     │  │
│  └────────────┘  │                          │  │            │  │
│        ↑         │                          │  │ Emulates   │  │
│        │         │                          │  │ Feetech    │  │
│  ros2_control    │                          │  │ Protocol   │  │
│                  │                          │  └────────────┘  │
└──────────────────┘                          └──────────────────┘
```

### Why HIL After SIL?

| SIL Misses | HIL Catches |
|------------|-------------|
| Real USB enumeration delays | ✓ |
| OS serial buffer behavior | ✓ |
| Electrical noise on RX/TX | ✓ |
| Actual round-trip latency (1-5ms) | ✓ |
| Permission issues (`/dev/ttyUSB0`) | ✓ |

### ESP32 Firmware Outline

```cpp
// feetech_emulator.ino (pseudocode)

#define SERVO_ID 1
float current_position = 0.0;
float target_position = 0.0;
float max_speed = 100.0;  // units per second

void setup() {
    Serial.begin(1000000);  // Feetech default baud rate
}

void loop() {
    // 1. Check for incoming packets
    if (Serial.available() >= 5) {
        if (parse_feetech_packet()) {
            handle_command();
        }
    }
    
    // 2. Simulate motor physics
    float dt = 0.001;  // 1ms loop
    float diff = target_position - current_position;
    float step = max_speed * dt;
    
    if (abs(diff) > step) {
        current_position += (diff > 0) ? step : -step;
    } else {
        current_position = target_position;
    }
    
    delay(1);
}

void handle_command() {
    switch (instruction) {
        case 0x03:  // WRITE
            if (address == 0x38) {
                target_position = (param_high << 8) | param_low;
            }
            send_ack();
            break;
            
        case 0x02:  // READ
            if (address == 0x38) {
                send_position_response(current_position);
            }
            break;
    }
}
```

### Deliverables for Phase 1

| Item | Description |
|------|-------------|
| `feetech_emulator.ino` | Arduino sketch for ESP32 |
| Wiring diagram | USB connection (typically just USB cable for ESP32 DevKit) |
| Latency benchmark | Measure round-trip time: command → response |
| Comparison report | SIL vs HIL timing differences |

---

