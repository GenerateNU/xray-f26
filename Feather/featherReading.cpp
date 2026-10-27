#include <Arduino.h>
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "pico/util/queue.h"
 
Adafruit_USBH_Host USBHost;
Adafruit_USBH_CDC SerialHost;

// The board package defines these; fallbacks match the Feather USB Host pinout
#ifndef PIN_USB_HOST_DP
#define PIN_USB_HOST_DP 16   // D+ on GPIO16, D- is automatically GPIO17
#endif
#ifndef PIN_5V_EN
#define PIN_5V_EN 18         // GPIO 18 switches 5V power to the Type-A port
#endif

enum State {
  IDLE,
  INIT_SEND,
  INIT_READ,
  POLLING
};

volatile bool needsConfig = false;
volatile int cdcIdx = -1;

const uint8_t STX = 0x02;
const uint8_t ETX = 0x03;

volatile State currentState = POLLING;
uint32_t stateTime = 0;
const uint32_t TIMEOUT_MS = 5000;

struct Frame {
  uint8_t data[50];
  uint8_t len;
};

Frame frame0;
Frame frame1;
bool startFrame = false;

queue_t queue;


void setState(State state) {
  currentState = state;
  stateTime = millis();
}

uint8_t checksum(const char* data, int len) {
  uint8_t total = 0x02;                 // 0x02 + sum(data), & 0xFF via uint8_t wrap
  for (int i = 0; i < len; i++) total += (uint8_t)data[i];
  return total;
}

int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  c = toupper(c);
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Returns true and sets *result (uSv/h) if the frame is valid
bool parseFrame(const Frame& f, float* result) {
  if (f.len < 3) return false;

  char body[sizeof(f.data)];
  memcpy(body, f.data, f.len);

  int dataLen = f.len - 2;            
  int hi = hexDigit(body[dataLen]);
  int lo = hexDigit(body[dataLen + 1]);
  if (hi < 0 || lo < 0) return false;
  if (((hi << 4) | lo) != checksum(body, dataLen)) return false;

  body[dataLen] = '\0';                 
  char* p = body;
  while (*p == ' ') p++;               
  char* after;
  float value = strtof(p, &after);
  if (after == p) return false;

  *result = value * 0.01f;
  return true;
}

void setup() {
  Serial.begin(9600);
  queue_init(&queue, sizeof(Frame), 8); 
}
void loop() {
   while (queue_try_remove(&queue, &frame0)) {
    float uSv;
    if (parseFrame(frame0, &uSv)) {
      Serial.printf("%.2f uSv/h\r\n", uSv);
    } else {
      Serial.print("Bad frame: ");
    }
  }
}

void setup1() {
  delay(10);

  // power the USB A port
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, 1);

  SerialHost.begin(9600);

  // configure the USBHost
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);
  USBHost.begin(1); // one is the type A, 0 is the usb c
}

void loop1() {
  USBHost.task();

  if (needsConfig) {
  needsConfig = false;

  cdc_line_coding_t coding;
  coding.bit_rate  = 9600;
  coding.stop_bits = 2;   // 0 = 1 stop bit, 1 = 1.5, 2 = 2
  coding.parity    = 2;   // 0 = none, 1 = odd, 2 = even
  coding.data_bits = 7;

  bool ok1 = tuh_cdc_set_line_coding(cdcIdx, &coding, NULL, 0);
  bool ok2 = tuh_cdc_set_control_line_state(cdcIdx,
               CDC_CONTROL_LINE_STATE_DTR | CDC_CONTROL_LINE_STATE_RTS, NULL, 0);
  Serial.printf("line coding: %d, DTR/RTS: %d\r\n", ok1, ok2);

  startFrame = false;
  setState(INIT_SEND);
}
  
  switch (currentState) {
    case INIT_SEND:
      SerialHost.write(0x40);
      SerialHost.flush();
      setState(INIT_READ);
      break;

    case INIT_READ: {
      int response = SerialHost.read();

      if (millis() - stateTime > TIMEOUT_MS) {
          Serial.println("TIME OUT, resending");
          setState(INIT_SEND);
          break;
        }

      if ((char) response == '>') {
          SerialHost.write("*C1\r\n");
          SerialHost.flush();
          setState(INIT_SEND);
      } else if (response == '#') {
        Serial.println("Switched to framed mode");
        setState(INIT_SEND);
      } else if (response == 0x07) {
          Serial.println("OK");
          setState(POLLING);
      }
      break;
    }
    case POLLING: {
      while (SerialHost.available()) {
        uint8_t byte = SerialHost.read();
        //Serial.println(byte);
        if (startFrame && frame1.len >= sizeof(frame1.data)) {
          startFrame = false;
          continue;
        }

        if (byte == STX) {
          frame1.len = 0;
          startFrame = true;
        } else if (byte == ETX && startFrame) {
          queue_try_add(&queue, &frame1);
          startFrame = false;
        } else if (startFrame) {
          frame1.data[frame1.len++] = byte;
        }
      }
      break;
    }
    default:
      break;
  }
}

// TinyUSB auto calls these through USBHost.task()
extern "C" {
  void tuh_cdc_mount_cb(uint8_t idx) {
    SerialHost.mount(idx); // USBHost set the device to the usb port, so it knows which port to read/write from
    needsConfig = true; 
    cdcIdx = idx;
  }

  void tuh_cdc_umount_cb(uint8_t idx) {
    SerialHost.umount(idx); // disconnects.
    setState(IDLE);
  }
}
