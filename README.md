## VRS_mouse

Projekt predstavuje autonómne robotické autíčko vybavené senzormi, ktoré sa dokáže samostatne pohybovať v priestore bez kolízií s prekážkami. Autíčko je navrhnuté tak, aby rozpoznalo prítomnosť človeka a aktívne sa od neho vzďaľovalo, čím simuluje správanie úniku.32. Môže sa pohybovať ľubovoľne, ale pri detekcii človeka sa musí vyhnúť zrážke.  

Riadiacou jednotkou systému bude mikrokontrolér STM (STM32), ktorý zabezpečuje spracovanie údajov zo senzorov, vyhodnocovanie okolitého prostredia a riadenie pohybu motorov v reálnom čase.

Autíčko bude pomocou senzorov nepretržite monitorovať vzdialenosť od prekážok a pohyb človeka. Na základe týchto informácií automaticky určuje smer a rýchlosť pohybu, pričom sa môže voľne pohybovať všetkými smermi – dopredu, dozadu aj do strán – tak, aby sa vyhlo nárazom a zároveň si udržiavalo bezpečný odstup od človeka.

## HW specifikacia
Auticko je riadene pomoсou MCU STM32 a obsahuje senzory na detekciu prekážok. 

## Rozdelenie uhol
Stanislav – konfigurácia hardvéru, nastavenia vstupov/výstupov, spracovanie údajov zo senzorov a pohyb motorov.

Roman - SW (pohyb auticka, utekanie od cloveka).
