# Modbus controller component

## A fork of [esphome](https://github.com/esphome/esphome) adding support for monitoring  modbus slave

Initially I created this component only for the EPEVER Trace solar controller (You can find that implementation in the epever branch here. )
Since alot of my code was already pretty generic I decided to create a general modbus component instead.

Modbus_controller suppors sensors, binary_sensors, text_sensors and switches
Custom command can be sent to the slave using lambdas.


Tested using an EPEVER Tracer2210AN MPPT controller and PZEM-017 




## Hardware setup

I'm using a cheap RS 485 module connected to an ESP32

![RS 485 Modul](https://i.stack.imgur.com/plH1X.jpg)

See [How is this RS485 Module Working?](https://electronics.stackexchange.com/questions/244425/how-is-this-rs485-module-working) on stackexchange for more details

To connect the RS 485 Module to the controller I cut off one side of an ethernet cable and connected PIN 3 (or 4)  to A+, PIN 5 (or 6) to B+ and 7 (or 8 to Ground).  Ground is also connected to GND.
The interface with ESP32 is GPIO PIN 25 to TXD PIN 27 to RXD . 3.3V to VCC and GND to GND.
The pins used on the ESP32 side can be changed there is no special reason I chose 25/27 except that most of my ESP32 boards have them available

## Software setup

```
# Clone repo
git clone https://github.com/martgras/esphome.git -b modbus_component

# Install ESPHome
cd esphome/
pip3 install -r requirements.txt -r requirements_test.txt
pip3 install -e .

esphome <path to your config.yaml> run

```


Modbus sensors can be directly defined (inline) under the modbus_controller hub or as standalone components

Technically there is no difference between the "inline" and the standard definitions approach.
Because the project started supporting only "inline" I'm keeping it in the code because it doesn't impact the code size and is a bit more convenient. The additional work to support both schemata is done in the python scripts generating the C++ code 

````
modbus:
  id: modbus_epsolar
  # ctrl_pin: 5    # if you need to set the driver enable (DE) pin high before transmitting data configure it here
  uart_id: mod_bus

modbus_controller:
  modbus_id: modbus_epsolar
  command_throttle: 0ms
  id: traceranx
  ## the Modbus device addr
  address: 0x1
  ## Any modbus registers not already implemented can be defined here
  ##
  sensors:
    - id: array_rated_voltage
      name: "array_rated_voltage"
      address: 0x3000
      offset: 0
      unit_of_measurement: "V"
      modbus_functioncode: "read_input_registers"
      value_type: U_WORD
      accuracy_decimals: 1
      skip_updates: 60
      filters:
      - multiply: 0.01
````


or define modbus_controller hub and sensors seperately

````
modbus_controller:
  modbus_id: modbus_epsolar
  command_throttle: 0ms
  id: traceranx
  ## the Modbus device addr
  address: 0x1

sensors:
  - platform: modbus_controller
    id: array_rated_voltage
    name: "array_rated_voltage"
    address: 0x3000
    offset: 0
    unit_of_measurement: "V"
    modbus_functioncode: "read_input_registers"
    value_type: U_WORD
    accuracy_decimals: 1
    skip_updates: 60
    filters:
    - multiply: 0.01
````



[Example config for the EPEVER controller](https://github.com/martgras/esphome/blob/modbus_component/esphome/components/modbus_controller/testconfig/epever.yaml)


### Format

Define an register in YAML
```yaml
    sensors:
      - id: length_of_night
        address: 0x9065
        offset: 0
        bitmask: default value is 0xFFFFF # some values are packed in a single response word. Bitmask can be used to extract the relevant parts
        name: 'Length of night'
        modbus_functioncode: read_holding_registers
        value_type: U_WORD
        scale_factor: 1.0
```



modbus_sensor_schema extends the sensors schema and adds these parameters:
  - modbus_functioncode: type of register
  - address: start address of the first register in a range
  - offset: offset from start address in bytes. If more than one register is read a modbus read registers command this value is used to find the start of this datapoint relative to start address. The component calculates the size of the range based on offset and size of the value type
  - value_type:
    - U_WORD (unsigned float from 1 register =16bit
    - S_WORD (signed float from one register)
    - U_DWORD (unsigned float from 2 registers = 32bit)
    - S_DWORD (unsigned float from 2 registers = 32bit)
    - U_DWORD_R (unsigend float from 2 registers low word first )
    - S_DWORD_R (sigend float from 2 registers low word first )    
    - U_QWORD (unsigned float from 4 registers = 64bit
    - S_QWORD (signed float from 4 registers = 64bit
    - U_QWORD_R (unsigend float from 4 registers low word first )
    - S_QWORD_R (sigend float from 4 registers low word first )    


#### modbus component:


  - platform: modbus_controller
  - cmodbus_id: id of the modbus hub
  - command_throttle:  milliseconds between 2 requests to the slave. Some slaves limit the rate of requests they can handle (e.g. only 1 request/s). 
  - id: component id
  - address: modbus slave address


#### sensor 
  - modbus_functioncode: type of register
  - address: start address of the first register in a range
  - offset: offset from start address in bytes. If more than one register is read a modbus read registers command this value is used to find the start of this datapoint relative to start address. 
  - bitmask: some values are packed in a response. The bitmask can be used to extract a value from the response.  For example, the high byte value register 0x9013 contains the minute value of the current time. To only extract this value use bitmask: 0xFF00.  The result will be automatically right shifted by the number of 0 before the first 1 in the bitmask.  For 0xFF00 (0b1111111100000000) the result is shifted 8 positions.  More than one sensor can use the same address/offset if the bitmask is different.

#### binarysensor
  - modbus_functioncode: type of register
  - address: start address of the first register in a range
  - offset: offset from start address in bytes. If more than one register is read a modbus read registers command this value is used to find the start of this datapoint relative to start address. 
  - bitmask: some values are packed in a response. The bitmask is used to determined if the result is true or false
  - create_switch: if this is a coil register setting this to true dynamically creates a modbus_switch component with the same name and sets the binarysensor to internal. Whenever the sensor reads a new value the state is synced with the switch component and vice versa (something like a binarysensorswitch)
  It is a shortcut for archiving this: 

  ````yaml
      binary_sensors:
      - id: force_load
        modbus_functioncode: read_coils
        address: 6
        offset: 0
        name: "Force Load on/off"
        internal: true 
        bitmask: 1
        on_state:
          if:
            condition:
              binary_sensor.is_on: force_load
            then: 
              - switch.turn_on: force_load_switch
            else:
              - switch.turn_off: force_load_switch

    switches:
      - id: force_load_switch
        modbus_functioncode: write_single_coil
        address: 6
        offset: 0
        name: "Force load on"
        bitmask: 1
  ````



#### text sensor:
   - modbus_functioncode: type of register
   - address: start address of the first register in a range
   - offset: offset from start address in bytes. If more than one register is read a modbus read registers command this value is used to find the start of this datapoint relative to start address. 
   - response_size: response number of bytes of the response
   - hex_encode: true | false     If the response is binary data it can't be published. Since a text sensor only publishes strings the hex_encode option encodes binary data as 2 byte hex string. 0x71 will be sent as "71". This allows you to process the data in a on_value lambda. See the example below how to convert the binary time data to a string and also how to set the time of the controller 
   - register_count: The number of registers this data point spans. Default is 1 
   - bitmask: some values are packed in a single response word. bitmask is used to convert to a bool value. For example, bit 8 of the register 0x3200 indicates an battery error. Therefore, if the bitmask is 256. the operation is `result = (raw value & bitmask != 0)`. More than one sensor can use the same address/offset if the bitmask is different
   
#### switch
  - modbus_functioncode: type of register
  - address: start address of the first register in a range
  - bitmask: applied before sending the value to the controller

See the MODBUS Specification: https://www.developpez.net/forums/attachments/p196506d1451307310/systemes/autres-systemes/automation/probleme-com-modbus-pl7-pro/controllerprotocolv2.3.pdf/ for details about the registers

## TIP
Write support is only implemented for switches.
However the C++ code already has the required methods to write to a modbus register

These methods can be called from a lambda. 
Here is an example how to set the rtc clock of a EPEVER Trace AN controller. 
The time is set by writing 12 bytes to register 0x9013. 
The code reads the current time of the controller using a text sensor and compares it with the time of the esp.
If they differ the time of the esp is sent to the EPEVER controller.

````yaml
    ...
    text_sensors:
      - name: "rtc_clock"
        id: rtc_clock
        internal: true
        modbus_functioncode: read_holding_registers
        address: 0x9013
        address: 0
        register_count: 3
        hex_encode: true
        response_size: 6
#                /*
#                E20 Real time clock 9013 D7-0 Sec, D15-8 Min
#                E21 Real time clock 9014 D7-0 Hour, D15-8 Day
#                E22 Real time clock 9015 D7-0 Month, D15-8 Year
#                */
        on_value:
          then:
            - lambda: |-
                ESP_LOGV("main", "decoding rtc hex encoded raw data: %s", x.c_str());
                uint8_t h=0,m=0,s=0,d=0,month_=0,y = 0 ;
                m = esphome::modbus_controller::byte_from_hex_str(x,0);
                s = esphome::modbus_controller::byte_from_hex_str(x,1);
                d = esphome::modbus_controller::byte_from_hex_str(x,2);
                h = esphome::modbus_controller::byte_from_hex_str(x,3);
                y = esphome::modbus_controller::byte_from_hex_str(x,4);
                month_ = esphome::modbus_controller::byte_from_hex_str(x,5);
                // Now check if the rtc time of the controller is ok and correct it
                time_t now = ::time(nullptr);
                struct tm *time_info = ::localtime(&now);
                int seconds = time_info->tm_sec;
                int minutes = time_info->tm_min;
                int hour = time_info->tm_hour;
                int day = time_info->tm_mday;
                int month = time_info->tm_mon + 1;
                int year = time_info->tm_year - 2000;
                // correct time if needed (ignore seconds)
                if (d != day || month_ != month || y != year || h != hour || m != minutes) {
                  // create the payload
                  std::vector<uint16_t> rtc_data = {uint16_t((minutes << 8) | seconds), uint16_t((day << 8) | hour),
                                                    uint16_t((year << 8) | month)};
                  // Create a modbus command item with the time information as the payload
                  esphome::modbus_controller::ModbusCommandItem set_rtc_command = esphome::modbus_controller::ModbusCommandItem::create_write_multiple_command(traceranx, 0x9013, 3, rtc_data);
                  // Submit the command to the send queue
                  traceranx->queue_command(set_rtc_command);
                  ESP_LOGI("ModbusLambda", "EPSOLAR RTC set to %02d:%02d:%02d %02d.%02d.%04d", hour, minutes, seconds, day, month, year + 2000);
                }
                char buffer[20];
                // format time as YYYY:mm:dd hh:mm:ss
                sprintf(buffer,"%04d:%02d:%02d %02d:%02d:%02d",y+2000,month_,d,h,m,s);
                id(template_rtc).publish_state(buffer);
````


## Protocol decoding example ## 


````
  sensors:
    - id: array_rated_voltage
      name: "array_rated_voltage"
      address: 0x3000
      offset: 0
      unit_of_measurement: "V"
      modbus_functioncode: "read_input_registers"
      value_type: U_WORD
      accuracy_decimals: 1
      skip_updates: 60
      filters:
        - multiply: 0.01

    - id: array_rated_current
      name: "array_rated_current"
      address: 0x3000
      offset: 2
      unit_of_measurement: "V"
      modbus_functioncode: "read_input_registers"
      value_type: U_WORD
      accuracy_decimals: 2
      filters:
        - multiply: 0.01

    - id: array_rated_power
      name: "array_rated_power"
      address: 0x3000
      register_count: 2
      offset: 4
      unit_of_measurement: "W"
      modbus_functioncode: "read_input_registers"
      value_type: U_DWORD_R
      accuracy_decimals: 1
      filters:
        - multiply: 0.01

    - id: battery_rated_voltage
      name: "battery_rated_voltage"
      address: 0x3000
      offset: 8
      unit_of_measurement: "V"
      modbus_functioncode: "read_input_registers"
      value_type: U_WORD
      accuracy_decimals: 1
      filters:
        - multiply: 0.01

    - id: battery_rated_current
      name: "battery_rated_current"
      address: 0x3000
      offset: 10
      unit_of_measurement: "A"
      modbus_functioncode: "read_input_registers"
      value_type: U_WORD
      accuracy_decimals: 1
      filters:
        - multiply: 0.01

    - id: battery_rated_power
      name: "battery_rated_power"
      address: 0x3000
      register_count: 2
      offset: 12
      unit_of_measurement: "W"
      modbus_functioncode: "read_input_registers"
      value_type: U_DWORD_R
      accuracy_decimals: 1
      filters:
        - multiply: 0.01

    - id: charging_mode
      name: "charging_mode"
      address: 0x3000
      offset: 16
      unit_of_measurement: ""
      modbus_functioncode: "read_input_registers"
      value_type: U_WORD
      accuracy_decimals: 0
````

To minimize the required transactions all registers with the same base address are read in one request. 
The response is mapped to the sensor based on register_count and offset in bytes. 

Request
````
0x1  (01)  : slave address
0x4  (04)  : function code 4 (Read Input Registers)
0x30 (48)  : start address high byte
0x0  (00)  : start address low byte
0x0  (00)  : number of registers to read (hi)
0x9  (09)  : number of registers to read (low)
0x3f (63)  : crc
0xc  (12)  : crc
````

Response:
````
H   0x1  (01)  : slave address
H   0x4  (04)  : function code
H   0x12 (18)  : byte count
---------------------------
0   0x27 (39)  : array_rated_voltage - hi
1   0x10 (16)  : array_rated_voltage - lo  => 0x2710 = 100000 - multiply: 0.01 = 100V
---------------------------
2   0x7  (7)   : next value is at offset 2 : array_rated_current - hi
3   0xd0 (208) : array_rated_current - lo  => 0x7d0 = 2000 - multiply: 0.01 = 20A
---------------------------
4   0xcb (203) : array_rated_power is a 32 bit value that spans 2 registers. high byte of low word
5   0x20 (32)  : low byte of low word
6   0x0  (0)   : high byte of high word
7   0x0  (0)   : low byte of low word => 0x0000CB20 = 52000 - multiply: 0.01 = 520W. Because the low word is sent first the data type is U_DWORD_R
---------------------------
8   0x9  (09)  : battery_rated_voltage hi
9   0x60 (96)  : battery_rated_voltage lo 0x960 = 2400- multiply: 0.01 = 12V
---------------------------
10  0x7  (07)  : battery_rated_current hi
11  0xd0 (208) : battery_rated_current lo = 2000 - multiply: 0.01 = 20A
---------------------------
12  0xcb (203) : battery_rated_power is a 32 bit value that spans 2 registers. high byte of low word
13  0x20 (32)  : low byte of low word
14  0x0  (0)   : high byte of high word
15  0x0  (0)   : low byte of low word => 0x0000CB20 = 52000 - multiply: 0.01 = 520W. Because the low word is sent first the data type is U_DWORD_R
---------------------------
16  0x0  (0)   : charging_mode high word
17  0x2  (02)  : charging_mode low word 0x2 (MPPT)
---------------------------
C   0x2f (47)  : crc
C   0x31 (49)  : crc
````