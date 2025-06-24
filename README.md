# ICM42688 Barsotion ESP32 driver
![image info](./docs/pic4.png)

[Methods descriptions](./methods_descriptions.md)

[ICM-42688-P Barsotion devboard](https://github.com/Barsy-Barsevich/Barsotion-AH1)

## Examples
### Specter diagram example
*Source code:* [spectre.c](./examples/spectre.c)
AAF (anti-alias filter) disabled, notch filter <ins>disabled</ins>, ODR=4kHz:
![](./docs/AAF_disabled_Notch_disabled_ODR_4kHz.png)
AAF disabled, notch filter <ins>enabled</ins>, ODR=4kHz:
![](./docs/AAF_disabled_Notch_enabled_ODR_4kHz.png)
## Contacts for Q&A
Email: barsotion@yandex.ru
Telegram: @barsybarsevich
## TODO

- [ ] Add library recursive calibration method
- [ ] Clean examples
- [ ] Create methods descriptions