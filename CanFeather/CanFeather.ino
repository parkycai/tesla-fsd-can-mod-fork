/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <SPI.h>
#include <mcp2515.h>


#define LEGACY LegacyHandler
#define HW3 HW3Handler
#define HW4 HW4Handler  //HW4 since Version 2026.2.3 uses FSDV14, before that compile for HW3, even for HW4 vehicles.


#define HW LEGACY  //for what car to compile

bool enablePrint = true; //debug print through Serial
bool enableModify = false;  //是否修改数据帧，断网之后再改true！


#define LED_PIN 9  // 改用 D9，避免和 SPI SCK(D13) 冲突
// #define LED_PIN   13    // 原板载 LED，但 D13 同时是 SPI SCK，可能干扰通信
#define CAN_CS 10  // MCP2515 模块的 CS 引脚，接 D10
// 以下三个引脚在 Nano 上不存在板卡预定义，如不需要可删掉或手动指定
// #define CAN_INT_PIN  2   // 如需中断模式，接 D2
// #define CAN_STBY      -1 // 外部模块一般不需要
// #define CAN_RESET     -1 // 外部模块一般不需要

MCP2515 mcp(CAN_CS);

struct CarManagerBase {
  int speedProfile = 1;
  bool FSDEnabled = false;
  virtual void handelMessage(can_frame& frame);
  virtual const uint32_t* filterIds() const = 0;
  virtual uint8_t filterIdCount() const = 0;
};

inline uint8_t readMuxID(const can_frame& frame) {
  return frame.data[0] & 0x07;
}

inline bool isFSDSelectedInUI(const can_frame& frame) {
  return (frame.data[4] >> 6) & 0x01;
}

inline void setSpeedProfileV12V13(can_frame& frame, int profile) {
  frame.data[6] &= ~0x06;
  frame.data[6] |= (profile << 1);
}

inline void setBit(can_frame& frame, int bit, bool value) {
  // Determine which byte and which bit within that byte
  int byteIndex = bit / 8;
  int bitIndex = bit % 8;
  // Set the desired bit
  uint8_t mask = static_cast<uint8_t>(1U << bitIndex);
  if (value) {
    frame.data[byteIndex] |= mask;
  } else {
    frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
  }
}


struct LegacyHandler : public CarManagerBase {
  const uint32_t* filterIds() const override {
    static constexpr uint32_t ids[] = { 69, 1006 };
    return ids;
  }
  uint8_t filterIdCount() const override {
    return 2;
  }

  virtual void handelMessage(can_frame& frame) override {
    // STW_ACTN_RQ (0x045 = 69): Follow-Distance-Stalk as Source for Profile Mapping
    // byte[1]: 0x00=Pos1, 0x21=Pos2, 0x42=Pos3, 0x64=Pos4, 0x85=Pos5, 0xA6=Pos6, 0xC8=Pos7
    if (frame.can_id == 69) {
      uint8_t pos = frame.data[1] >> 5;
      if (pos <= 1)
        speedProfile = 2;
      else if (pos == 2)
        speedProfile = 1;
      else
        speedProfile = 0;
      return;
    }
    if (frame.can_id == 1006) {
      auto index = readMuxID(frame);
      if (index == 0)
        FSDEnabled = true;//isFSDSelectedInUI(frame);
      if (index == 0 && FSDEnabled) {
        setBit(frame, 46, true);
        setSpeedProfileV12V13(frame, speedProfile);
        debugSend(frame);
      }
      if (index == 1) {
        setBit(frame, 19, false);
        debugSend(frame);
      }
#ifndef NATIVE_BUILD
      if (index == 0 && enablePrint) {
        Serial.print("LegacyHandler: FSD: ");
        Serial.print(FSDEnabled);
        Serial.print(", Profile: ");
        Serial.println(speedProfile);
      }
#endif
    }
  }
};

struct HW3Handler : public CarManagerBase {
  const uint32_t* filterIds() const override {
    static constexpr uint32_t ids[] = { 1016, 1021 };
    return ids;
  }
  uint8_t filterIdCount() const override {
    return 2;
  }

  int speedOffset = 0;
  virtual void handelMessage(can_frame& frame) override {
    if (frame.can_id == 1016) {
      uint8_t followDistance = (frame.data[5] & 0b11100000) >> 5;
      switch (followDistance) {
        case 1:
          speedProfile = 2;
          break;
        case 2:
          speedProfile = 1;
          break;
        case 3:
          speedProfile = 0;
          break;
        default:
          break;
      }
      return;
    }
    if (frame.can_id == 1021) {
      auto index = readMuxID(frame);
      if (index == 0)
        FSDEnabled = true;//isFSDSelectedInUI(frame);
      if (index == 0 && FSDEnabled) {
        uint8_t raw = (frame.data[3] >> 1) & 0x3F;
        int val = (raw - 30) * 5;
        speedOffset = constrain(val, 0, 100);
        auto off = (uint8_t)((frame.data[3] >> 1) & 0x3F) - 30;
        switch (off) {
          case 2:
            speedProfile = 2;
            break;
          case 1:
            speedProfile = 1;
            break;
          case 0:
            speedProfile = 0;
            break;
          default:
            break;
        }
        setBit(frame, 46, true);
        setSpeedProfileV12V13(frame, speedProfile);
        debugSend(frame);
      }
      if (index == 1) {
        setBit(frame, 19, false);
        debugSend(frame);
      }
      if (index == 2 && FSDEnabled) {
        frame.data[0] &= ~(0b11000000);
        frame.data[1] &= ~(0b00111111);
        frame.data[0] |= (speedOffset & 0x03) << 6;
        frame.data[1] |= (speedOffset >> 2);
        debugSend(frame);
      }
    }
  }
};

struct HW4Handler : public CarManagerBase {
  const uint32_t* filterIds() const override {
#if defined(ISA_SPEED_CHIME_SUPPRESS)
    static constexpr uint32_t ids[] = { 921, 1016, 1021 };
    return ids;
  }
  uint8_t filterIdCount() const override {
    return 3;
  }
#else
    static constexpr uint32_t ids[] = { 1016, 1021 };
    return ids;
  }
  uint8_t filterIdCount() const override {
    return 2;
  }
#endif


  virtual void handelMessage(can_frame& frame) override {
#if defined(ISA_SPEED_CHIME_SUPPRESS)
    if (frame.can_id == 921) {
      frame.data[1] |= 0x20;
      uint8_t sum = 0;
      for (int i = 0; i < 7; i++)
        sum += frame.data[i];
      sum += (921 & 0xFF) + (921 >> 8);
      frame.data[7] = sum & 0xFF;
      debugSend(frame);
      return;
    }
#endif
    if (frame.can_id == 1016) {
      auto fd = (frame.data[5] & 0b11100000) >> 5;
      switch (fd) {
        case 1:
          speedProfile = 3;
          break;
        case 2:
          speedProfile = 2;
          break;
        case 3:
          speedProfile = 1;
          break;
        case 4:
          speedProfile = 0;
          break;
        case 5:
          speedProfile = 4;
          break;
      }
    }
    if (frame.can_id == 1021) {
      auto index = readMuxID(frame);
      if (index == 0)
        FSDEnabled = true;//isFSDSelectedInUI(frame);
      if (index == 0 && FSDEnabled) {
        setBit(frame, 46, true);
        setBit(frame, 60, true);
#if defined(EMERGENCY_VEHICLE_DETECTION)
        setBit(frame, 59, true);
#endif
        debugSend(frame);
      }
      if (index == 1) {
        setBit(frame, 19, false);
        setBit(frame, 47, true);
        debugSend(frame);
      }
      if (index == 2) {
        frame.data[7] &= ~(0x07 << 4);
        frame.data[7] |= (speedProfile & 0x07) << 4;
        debugSend(frame);
      }
    }
  }
};


CarManagerBase* handler;

void setup() {
  static HW hwHandler;
  handler = &hwHandler;
  pinMode(LED_PIN, OUTPUT);
  delay(1500);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1000) {}

  mcp.reset();
  MCP2515::ERROR e = mcp.setBitrate(CAN_500KBPS, MCP_8MHZ);
  if (e != MCP2515::ERROR_OK) Serial.println("setBitrate failed");

  const uint32_t* ids = handler->filterIds();
  const uint8_t count = handler->filterIdCount();
  mcp.setConfigMode();
  mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp.setFilter(MCP2515::RXF0, false, ids[0]);
  mcp.setFilter(MCP2515::RXF1, false, count > 1 ? ids[1] : ids[0]);
  mcp.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp.setFilter(MCP2515::RXF2, false, count > 2 ? ids[2] : ids[0]);
  mcp.setFilter(MCP2515::RXF3, false, count > 3 ? ids[3] : ids[0]);
  mcp.setFilter(MCP2515::RXF4, false, count > 4 ? ids[4] : ids[0]);
  mcp.setFilter(MCP2515::RXF5, false, count > 5 ? ids[5] : ids[0]);
  if(enableModify){
    mcp.setListenOnlyMode();
  }else{
    mcp.setListenOnlyMode();
  }

  Serial.println("MCP25625 ready @ 500k 1");
  Serial.print("Modify Enabled: ");
  Serial.println(enableModify ? "true" : "false");
}


void printFrame(const char* label, const can_frame& frame) {
  if(!enablePrint)return;
  // 用于调试是否收到CAN信号
  Serial.print(label);
  Serial.print(" ID:\t");
  Serial.print(frame.can_id);
  Serial.print("\t");
  // Serial.print(frame.can_dlc);
  Serial.print("Data:");
  for (int i = 0; i < frame.can_dlc; i++) {
    if (frame.data[i] < 0x10) Serial.print('0');
    Serial.print(frame.data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void debugSend(can_frame& frame) {
  if (enablePrint){
    if(!enableModify){
      Serial.print("[mock]");
    }else{
      Serial.print("\t");
    }
    printFrame("TX >>\t", frame);
  }
  mcp.sendMessage(&frame);
}

void handleSerialCommand() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'm' || c == 'M') {
      enableModify = false;
      mcp.setListenOnlyMode();
      Serial.println("Switched to monitoring mode.");
    } else if (c == 'n' || c == 'N') {
      enableModify = true;
      mcp.setNormalMode();
      Serial.println("Switched to normal mode");
    }
  }
}

void loop() {
  handleSerialCommand();
  can_frame frame;
  int r = mcp.readMessage(&frame);
  if (r != MCP2515::ERROR_OK) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  digitalWrite(LED_PIN, LOW);
  printFrame("RX <<\t\t", frame);

  handler->handelMessage(frame);
}
