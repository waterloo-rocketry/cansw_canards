**rx from simulink packet format**
| byte | field             | datatype |
| ---- | ----------------- | -------- |
| 0    | header char1      | char     |
| 1    | header char2      | char     |
| 2    | header char3      | char     |
| 3    | header char4      | char     |
| 4-7  | board accel x     | float    |
| 8-11 | board accel y     | float    |
| ...  | all other sensors | floats   |
| N    | footer char       | char     |


```
typedef struct __attribute__((packed)) {
    char header[4];
    float32_t board_accel_x;
    etc...
    char footer[1];
} hil_reading_t;
```

notes:
- header must be a multiple of 4 bytes so that the payload starts on a word-aligned memory location. If payload is unaligned, then we can't access fields directly without memcpying.
- simulink passes floats not double (save bandwidth, and sensor readings arent that accurate irl anyway)
- simulink sends values little endian

**packet parser**
```
hil_parse_byte(byte) {
    if (!is_packet_in_progress) {
        // try to match full 4-byte header sequence
        if (byte == header[header_match_idx]) {
            header_match_idx++;
            if (header_match_idx == 4) {
                // full header found: start a new packet body
                is_packet_in_progress = true;
                wip_packet_idx = 0;
                header_match_idx = 0;
            }
        } else {
            // if any matching failed, restart matching
            header_match_idx = 0;
        }
        return;
    }

    // store data. WIP excludes header cuz that's used for start, but include footer for validity check 
    wip_packet_buf[wip_packet_idx++] = byte;

    // packet is done after N bytes are saved (N excludes header)
    if (wip_packet_idx == PACKET_BODY_SIZE_N) {
        enter critical section
        memcpy(&hil_ready_buf, &wip_packet_buf, PACKET_BODY_SIZE_N);
        exit critical section

        is_packet_in_progress = false;
        wip_packet_idx = 0;
    }
}
```

notes
- WIP buffer excludes header, includes footer.
- overflow: if wip_packet_idx exceeds PACKET_BODY_SIZE_N, reset parser state and log err.
- no explicit footer char validity check needed in parser (can check in debug if needed).

**get latest simulink sensor data**
```
get_latest_simulink_packet(all_sensor_readings_t *out) {
    enter critical
    hil_reading_t* packet = (hil_reading_t*)&hil_ready_buf;

    // set each field
    out->board_accel.x = packet->board_accel_x;
    etc...
    exit critical

    // also send flight phase events here if fake CAN was received. Later, real CAN HIL
    // will test the actual CAN path, so skip that and just do flight phase send event here.
    // Each `true` represents a new CAN msg, so send an event each time `true` happens
    if (packet->ignitor_msg == true) {
        flight_phase_send_event(ignitor event);
    }
    if (packet->inj_valve_open_msg == true) {
        flight_phase_send_event(inj valve open event);        
    }

}
```
notes
- this ALWAYS returns the latest rx packet, no matter how stale. So don't need to clear ready flag here

**tx to simulink packet format**
| byte  | field           | datatype |
| ----- | --------------- | -------- |
| 0     | header char     | char     |
| 1     | header char     | char     |
| 2     | header char     | char     |
| 3     | header char     | char     |
| 4-11  | canard cmd      | double   |
| 12-19 | other telem     | double   |
| ...   | all other telem | double   |
| M     | footer char     | char     |

```
typedef struct __attribute__((packed)) {
    char header[4];
    float64_t canard_cmd;
    etc...
    char footer[1];
} hil_cmd_t;
```

**send cmd to simulink**
```
send_simulink_packet(cmd, navigator_ctx) {
    hil_cmd_t out;
    out.canard_cmd = cmd;
    // etc for more output telem from navigator ctx

    out.header[0] = HEADER_CHAR;
    ...
    out.footer[0] = FOOTER_CHAR;

    // send packet via usb cdc transmit. busy wait is ok
}
```

**usb rx**
- override usb cdc receive callback isr
- for each usb packet received in usb ISR, loop bytes and run `hil_parse_byte`
- (note: 1 hil packet may span across many usb packets)

**usb tx**
- just do blocking transmit its fast enough

**timing**
- ideal soln: simulink is driven by canard board. Ie, simulink waits until it receives a cmd/packet/notification from canard board before advancing the sim step
- issue: how does canard board drive the first several seconds during idle / pad filter?
  - ideal soln: make canard board send a ping before sensor_get_all_readings(). Sketch:
    ```
        hil_get_sensor_readings() {
            tell_simulink_to_step();
            wait_for_sensor_reading_packet();
            read_simulink_sensor_packet();
        }
    ```
  - interim solution: run simulink async from canard board, and hope canard board usb can read every simulink packet on time
    - this worked last year though had to slow the sim down cuz uart issue. This shouldn't be an issue with usb now
