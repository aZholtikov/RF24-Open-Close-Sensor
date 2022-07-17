# RF24 датчик открытия/закрытия
Датчик открытия/закрытия на ATmega168/328 + RF24.

## Функции:

1. Потребление энергии в спящем режиме около 10 мкА. До 4 лет работы на одной батарейке CR2450 (ориентировочно).
2. При срабатывании датчик передает уровень заряда батареи и состояние контакта (OPEN/CLOSE).
3. Может использоваться с нормально замкнутыми и нормально разомкнутыми контактами.
4. Автоматический перезапуск в случае зависания.

## Примечание:

1. Работает на основе протокола передачи данных [ZH RF24 Sensor Protocol](https://github.com/aZholtikov/ZH-RF24-Sensor-Protocol).
2. Для работы в сети необходимо наличие [RF24 - ESP-NOW Gateway](https://github.com/aZholtikov/RF24-ESP-NOW-Gateway).
3. Подробности "железа" см. в папке "hardware".

## Важно!

Для улучшения работы модуля RF24 настоятельно рекомендую воспользоваться советом умных людей и припаять на антенну дополнительный конденсатор. Подробности (там про другой модель, но это роли не играет) в посте на Хабре [Победа над nRF24L01: на три шага ближе](https://habr.com/ru/post/476716/).