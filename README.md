# ESPHome [![Discord Chat](https://img.shields.io/discord/429907082951524364.svg)](https://discord.gg/KhAMKrd) [![GitHub release](https://img.shields.io/github/release/esphome/esphome.svg)](https://GitHub.com/esphome/esphome/releases/)

[![ESPHome Logo](https://esphome.io/_images/logo-text.png)](https://esphome.io/)

**This fork of esphome adds a modbus component that allows controlling devices via modbus/rs485**


The modbus_controller component is now part of esphome see https://github.com/esphome/esphome/pull/1779
If you are still using the modbus_controller code from the modbus_component branch please note that there are a couple of changes in the YAML definitions.
See [modbus documentation](https://github.com/martgras/esphome/blob/testing/esphome/components/modbus_controller/readme.md) for more details about the required changes.

Please switch to the esphome dev branch or if you are using external components change it to 
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/esphome/esphome
      ref: dev
    components: [ modbus, modbus_controller ]
```

There won't be new updates to the modbus_component branch (this branch). 
I will continue to use the ['testing'](https://github.com/martgras/esphome/tree/testing) branch for new code but generally recommend switch over to https://github.com/esphome/esphome

