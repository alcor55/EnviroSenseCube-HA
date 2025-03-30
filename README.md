# EnviroSenseCube HA v1.0 - Environment monitor HomeAssistant edition by Vincenzo Rivelli.

Features:
- Temperature, Relative humidity, Barometric pressure && Air Quality.
- Low power optimization && battery monitor.
- Data refresh threshold to save energy.
- E-Paper Display.
- Home Assistant update over WiFi to template sensors.

Hardware:
- Adafruit Huzzah ESP8266 ( 3.3volt logic ).
- Adafruit PowerBoost 500 Charger.
- LiPo battery 1800mAh ( EEMB Cell ).
- Adafruit BME688 ( I2C address: Default 0x77, 0x76 with closed bridge ).
- Adafruit LC709203F LiPoly/LiIon Fuel Gauge. ( I2C address fixed: 0x0B ). Need the same power as the logic level of your microcontroller ( 3.3v ).
- Adafruit On-Off Power Button ADA1683 ( Power Switch ).
- Aceinnolab Universal E-Paper driver board for 24-pin SPI.
- GoodDisplay GDEW0097T50 E-Paper Mini 0.97inc 184x88 24pin SPI. Min.FULL upd.: 180sec(3min.), Min.PARTIAL upd.: 5sec. https://www.good-display.com/product/434.html https://www.good-display.com/news/80.html#precaution
- Acrilic Case.

GPIO Pin map: #0, #2, #4, #5, #12, #13, #14, #15, #16.
- HUZZAH GPIO #16 Pin             -> HUZZAH GPIO Rst. Pin. ( For the DeepSleep awake ).
- HUZZAH TX Pin                   -> UART Cable WHITE.
- HUZZAH RX Pin                   -> UART Cable GREEN.
- HUZZAH V+ Pin                   -> UART Cable RED.
- HUZZAH GND Pin                  -> UART Cable BLACK.
- HUZZAH GPIO #13 Pin             -> E-Paper driver SPI SDI.
- HUZZAH GPIO #14 Pin             -> E-Paper driver SPI CLK.
- HUZZAH GPIO #2 Pin              -> E-Paper driver SPI CS.
- HUZZAH GPIO #15 Pin             -> E-Paper driver SPI DC.
- HUZZAH GPIO #0 Pin              -> E-Paper driver SPI RST.
- HUZZAH GPIO #12 Pin             -> E-Paper driver SPI BUSY.
- HUZZAH GPIO #4 Pin (i2c SDA)    -> BME688 SDI Pin    -> LC709203F SDA Pin (JST-SH Blue).
- HUZZAH GPIO #5 Pin (i2c SCL)    -> BME688 SCK Pin    -> LC709203F SCL Pin (JST-SH Yellow).

Power management:
- Charging time: 6 hours and 15 minutes. (Yellow LED (near pins): Charging; Green LED (near screw): Charge complete). 
- Autonomy: 11 days 4 hours and 40 minutes. ( SLEEP_TIME = 5m ).

![IMG_2259](https://github.com/user-attachments/assets/a2cb5548-ab53-4d6d-bf6d-7d8a5aafc315)
![IMG_1986](https://github.com/user-attachments/assets/8c7e6fed-1b8d-405b-a2c3-6f99d8559f20)

Home assistant configurations:

	---- configuration.yaml ----
	
	input_text:
	  envirosensecube_data:
	    name: "EnviroSenseCube Data"
	    initial: '{}'
	    max: 255
	
	template:
	  - sensor:
	      - name: "EnviroSenseCube Temperature"
	        unique_id: envirosensecube_temperature
	        state: >
	          {{ (states('input_text.envirosensecube_data') | from_json).temperature | float(0) }}
	        unit_of_measurement: "°C"
	        device_class: temperature
	        state_class: measurement
	      - name: "EnviroSenseCube Humidity"
	        unique_id: envirosensecube_humidity
	        state: >
	          {{ (states('input_text.envirosensecube_data') | from_json).humidity | float(0) }}
	        unit_of_measurement: "%"
	        device_class: humidity
	        state_class: measurement
	      - name: "EnviroSenseCube Pressure"
	        unique_id: envirosensecube_pressure
	        state: >
	          {{ (states('input_text.envirosensecube_data') | from_json).pressure | float(0) }}
	        unit_of_measurement: "hPa"
	        device_class: pressure
	        state_class: measurement
	      - name: "EnviroSenseCube Gas Resistance"
	        unique_id: envirosensecube_gas_resistance
	        state: >
	          {{ (states('input_text.envirosensecube_data') | from_json).gas_resistance | float(0) }}
	        unit_of_measurement: "Ω"
	        state_class: measurement
	      - name: "EnviroSenseCube Air Quality"
	        unique_id: envirosensecube_air_quality
	        state: >
	          {{ (states('input_text.envirosensecube_data') | from_json).air_quality | float(0) }}
	        state_class: measurement
	      - name: "EnviroSenseCube Battery"
	        unique_id: envirosensecube_battery
	        state: >
	          {{ (states('input_text.envirosensecube_data') | from_json).battery | float(0) }}
	        unit_of_measurement: "%"
	        device_class: battery
	        state_class: measurement
	
	---- automations.yaml ----
	
	- id: '[LONG RANDOM NUMBER]'
	  alias: Aggiornamento dati EnviroSenseCube
	  trigger:
	    - platform: webhook
	      webhook_id: envirocube_update_[HOME_ASSISTANT_KEY]
	  action:
	    - service: input_text.set_value
	      target:
	        entity_id: input_text.envirosensecube_data
	      data:
	        value: >
	          {
	            "temperature": {{ trigger.json.temperature | default(0) }},
	            "humidity": {{ trigger.json.humidity | default(0) }},
	            "pressure": {{ trigger.json.pressure | default(0) }},
	            "gas_resistance": {{ trigger.json.gas_resistance | default(0) }},
	            "air_quality": {{ trigger.json.air_quality | default(0) }},
	            "battery": {{ trigger.json.battery | default(0) }}
	          }