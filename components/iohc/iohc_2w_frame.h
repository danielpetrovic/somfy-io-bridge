/*
   Copyright (c) 2024. CRIDP https://github.com/cridp

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

           http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#ifndef IOHC_2W_FRAME_H
#define IOHC_2W_FRAME_H

// Shared 2W frame scaffolding, used by both this bridge's own 2W bonding
// (iohc_controller2w.cpp, controller role) and the "Recover System Key"
// responder (iohc_key_extraction.cpp, device role) - the two are opposite
// sides of the same handshake and build the same frame shapes, just with
// source/destination and a few framing bits swapped. Hoisted out of
// iohc_controller2w.cpp (where it originated) once a second file needed it.

#include <cstring>
#include "iohcPacket.h"

namespace IOHC {

// Header conventions (MsgLen = buffer_length-1, Protocol=0 for 2W,
// CtrlByte2=0) mirror IOHCRemote1W::forge_packet()'s pattern, adapted for
// 2W's unicast (not broadcast-group) addressing.
inline void forge_2w_packet(iohcPacket *packet, const IOHC::address &from, const IOHC::address &to) {
    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen = sizeof(_header) - 1;
    packet->payload.packet.header.CtrlByte1.asStruct.Protocol = 0;  // 2W
    packet->payload.packet.header.CtrlByte1.asStruct.StartFrame = 1;
    packet->payload.packet.header.CtrlByte1.asStruct.EndFrame = 1;
    packet->payload.packet.header.CtrlByte2.asByte = 0;
    memcpy(packet->payload.packet.header.source, from, sizeof(IOHC::address));
    memcpy(packet->payload.packet.header.target, to, sizeof(IOHC::address));
    packet->frequency = CHANNEL2;
    packet->repeatTime = 0;
    packet->repeat = 0;  // 2W is single-shot per direction, unlike 1W's repeat=4 broadcast redundancy.
    packet->lock = false;
}

// A 2W frame with explicit framing bits. Real controllers (and, in the
// reverse role, real devices) send the pairing frames in these shapes
// (laberning/home_io_control proto_commands.cpp):
//   0x28 discovery       start+end, low-power clear   (control bytes C8 00)
//   0x29 discovery resp  start+end, low-power clear   (device role)
//   0x2C confirm         start,     low-power = target class
//   0x2D confirm ack     end,       low-power clear    (device role)
//   0x31 key init        start,     low-power set      (48 20)
//   0x32 key transfer    continuation (neither),  low-power set (18 20)
//   0x33 key confirm     end,       low-power clear    (device role)
//   0x36 node verify req start,     low-power as sender decides
//   0x37 node verify resp continuation, low-power clear (device role)
//   0x3C challenge req   continuation, low-power as sender decides
//   0x3D challenge resp  continuation/end, low-power as sender decides
//   0x6F set config      start,     low-power set
inline void forge_2w_frame(iohcPacket *packet, const IOHC::address &from, const IOHC::address &to, bool start,
                          bool end, bool low_power) {
    forge_2w_packet(packet, from, to);
    packet->payload.packet.header.CtrlByte1.asStruct.StartFrame = start ? 1 : 0;
    packet->payload.packet.header.CtrlByte1.asStruct.EndFrame = end ? 1 : 0;
    packet->payload.packet.header.CtrlByte2.asStruct.LPM = low_power ? 1 : 0;
}

}  // namespace IOHC

#endif
