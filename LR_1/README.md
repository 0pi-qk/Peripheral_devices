### Задание

- Создайте новое приложение в среде разработки CooCox® CoIDE, в соответствии с имеющейся отладочной платой. Приложение должно управлять светодиодами, подключенным к выводам PB6, PB7, PB8, PB9 платы STM32F103mini.
- Используйте режим «Debug in RAM» для отладки Вашего приложения.
- В режиме отладки с использованием пошагового режима и точек останова изучите поведение регистров GPIOB, сравните значения в регистрах с поведением светодиодов.
- Организуйте циклический опрос пользовательских кнопок WKUP(PA0) и USER(PA15), отображая на светодиодах состояние кнопок (нажата-отжата).